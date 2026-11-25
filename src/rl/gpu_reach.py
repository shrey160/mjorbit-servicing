"""Batched scripted reach-and-close on the GPU (SPEC_7 section 24, G1): reach_bench.reach_close for N worlds at once in
MuJoCo Warp, with the bench's constants, so that a = 0 of the later residual policy is today's reach.

Per world, from the entry of reach_close (arm at the pre-reach pose, estimate E1 = (c, n), grasp azimuth a1, the bus
pinned at its station):
  cart     straight-line move to H_CLOSE above the estimated top (waypoints and their joint-speed timing as Bench.cart;
           IK on the torch arm model, rl/arm_torch.py), contact guard on (touch pads > TOUCH_N, wrist force step > FT_N),
           then 0.5 s settle
  descent  the commanded grip point closes on the tracked one at V_CATCH, descends at V_DESC / V_SLOW once caught up
           (CATCH_MM), stops on a guard trip; palm -> close, else a safe abort; arm tracking monitor (TRACK_MM)
  close    palm force follow (F_PALM, V_FOLLOW), both jaws ramp at V_JAW, then 2 s; lock: the jaws held where THIS
           world has them (per-world eq_data); 1 s hold; scored
  load     (hold_check) 3 N m about each client axis, 2 s + 0.5 s each; slip of the grip on the client
Control ticks every 5 physics steps (as the bench), asynchronous per world: a guard trip ends that world's tick at once.
The arm joint PD and the bus pin run every physics step. Contacts between the servicer and the client other than the
tool on the ring are collisions (Bench._contacts). Physics differences to the CPU bench: no noslip, float32, sensors
read from the step's own forward pass (one step older than the bench's), see section 24.1.
Hook for the policy (G3): `self.offset` (N, 3), an offset of the descent's tracked grip point in the estimated section
frame (radial, tangential, axial; m). The base tracks section(h) + offset (a = 0: today's reach). (A velocity on the
commanded point, the first hook, was undone by the catch-up every tick once caught up.)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
import _paths  # noqa: E402,F401
import reach_bench as RBN  # noqa: E402
import ring_fit  # noqa: E402
from arm_torch import ArmFK, frame, quat_to_mat, rot_from_rotvec, rotvec, section  # noqa: E402

PH_CART, PH_SETTLE, PH_DESC, PH_CLOSE, PH_HOLD, PH_LOAD, PH_DONE = range(7)
PH_NAME = ["to pre-close", "settle", "descent", "close", "hold", "load", "done"]
TOUCH = ("palm", "in", "hook", "lip")
LOCKS = ("lock_u", "lock_l")
DIAG = False
HOLD_AVG = 100              # hold-end pad loads: mean over the last 0.2 s (one sample: the bouncing client
                            # unloads palm and lip at an instant, 16 / 1024 copies 'no grip' with the clamp held)
TOUCH_PAD = 0.001            # touch-zone margin on the GPU (m)
W_MAX = 64                     # cart waypoints (Bench.cart uses max(10, dist / 4 mm, angle / 1 deg): ~20 here)


DUTY = [0.5]           # GPU duty cycle of run() (s25: the user asked for half load after three shutdowns)
T_PAUSE = [90]         # pause run() at this GPU temperature until 8 C cooler


class GpuReach:
    def __init__(self, B, starts, hold_check=True, nconmax=160, njmax=800, trim=None):
        import warp as wp
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.B, self.N, self.hold_check = B, len(starts), hold_check
        km = _copy(B.km)
        km.opt.noslip_iterations = 0
        # touch zones +1 mm (G1, 21 Nov): the ring lip lands on the palm zone's edge (z = its half height, 18.75 mm);
        # float64 on the CPU counts it, float32 on the GPU dropped it (palm 0 N, wrist 9-23 N: wrist F/T trips, 6/22)
        for k in TOUCH:
            km.site_size[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, f"touch_{k}")] += TOUCH_PAD
        if trim is not None:
            trim(km)
        self.km = km
        I = B.I
        N = self.N
        dev = "cuda"
        self.dev = dev
        f32, f64 = torch.float32, torch.float64
        self.dt = float(km.opt.timestep)
        # --- device model and data, per-world starts
        kd = mujoco.MjData(km)
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_setState(km, kd, starts[0]["state"], spec)
        mujoco.mj_forward(km, kd)
        self.wm = mjw.put_model(km)
        neq = km.neq
        self.wm.eq_data = wp.array(np.repeat(km.eq_data[None], N, 0).astype(np.float32), dtype=self.wm.eq_data.dtype)
        self.wd = mjw.put_data(km, kd, nworld=N, nconmax=nconmax, njmax=njmax)
        T = wp.to_torch
        wd = self.wd
        self.qpos, self.qvel, self.ctrl = T(wd.qpos), T(wd.qvel), T(wd.ctrl)
        self.xfrc, self.eqa, self.eqm = T(wd.xfrc_applied), T(wd.eq_active), T(self.wm.eq_data)
        self.sens = T(wd.sensordata)
        cols = dict(qpos=[], qvel=[], act=[], qacc_warmstart=[], ctrl=[])
        d1 = mujoco.MjData(km)
        for s in starts:
            mujoco.mj_setState(km, d1, s["state"], spec)
            for k in cols:
                cols[k].append(getattr(d1, k).copy())
        for k, v in cols.items():
            if getattr(d1, k).size:
                T(getattr(wd, k))[:] = torch.tensor(np.array(v), dtype=f32, device=dev)
        mjw.forward(self.wm, wd)
        mjw.step(self.wm, wd)                                  # compile, then restore the starts
        for k, v in cols.items():
            if getattr(d1, k).size:
                T(getattr(wd, k))[:] = torch.tensor(np.array(v), dtype=f32, device=dev)
        mjw.forward(self.wm, wd)
        wp.synchronize()
        with wp.ScopedCapture() as cap:
            mjw.step(self.wm, wd)
        self.graph = cap.graph
        # --- ids and constants
        t = lambda x, dt=f32: torch.tensor(np.asarray(x), dtype=dt, device=dev)  # noqa: E731
        self.aq, self.av, self.aa = t(I["arm_q"], torch.long), t(I["arm_v"], torch.long), t(I["act_arm"], torch.long)
        self.ag = t([I["act_u"], I["act_l"]], torch.long)
        self.gq = t(B.gq, torch.long)
        self.cq, self.cv, self.tq, self.tv = B.cq, B.cv, B.tq, B.tv
        self.bid_t = B.bid_t
        self.kp, self.kdv = t(B.kp), t(B.kdv)
        self.lim = t(km.actuator_ctrlrange[I["act_arm"], 1])
        self.ev = t([mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, n) for n in LOCKS], torch.long)
        sa = lambda n: (int(km.sensor_adr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SENSOR, n)]))  # noqa: E731
        self.s_touch = t([sa(f"touch_{k}") for k in TOUCH], torch.long)
        a = sa("wrist_force")
        self.s_wf = t([a, a + 1, a + 2], torch.long)
        self.q_open, self.sq = t(B.q_open), float(B.sq)
        # contact classes per geom: 1 servicer, 2 client; tool (tool bodies, not the camera unit), ring
        side = np.zeros(km.ngeom, np.int8)
        tool = np.zeros(km.ngeom, bool)
        ring = np.zeros(km.ngeom, bool)
        for g in range(km.ngeom):
            b = km.geom_bodyid[g]
            side[g] = 1 if b in B.srv_b else 2 if b in B.cli_b else 0
            tool[g] = b in B.tool_b and g != B.cam_g
            ring[g] = g in B.ring_g
        self.g_side, self.g_tool, self.g_ring = t(side, torch.int8), t(tool, torch.bool), t(ring, torch.bool)
        # --- per-world task data (float64 for the kinematics)
        self.pin = t(np.array([s["pin"] for s in starts]), f64)
        self.bp = self.pin[:, :3]
        self.bR = quat_to_mat(self.pin[:, 3:7])
        self.c = t(np.array([s["c"] for s in starts]), f64)
        self.n = t(np.array([s["n"] for s in starts]), f64)
        u, v = frame(self.n)
        a1 = t(np.array([s["a1"] for s in starts]), f64)
        self.rad = torch.cos(a1)[:, None] * u + torch.sin(a1)[:, None] * v
        self.A = ArmFK(km, arm_joints=I["arm_q"], jlo=B.jlo, jhi=B.jhi, device=dev)
        self.H_TOP, self.R_R = ring_fit.H_TOP, RBN.RING_R
        self.C_B, self.N_B = t(ring_fit.C_B, f64), t(ring_fit.N_B, f64)
        self.r_com = t(km.body_ipos[B.bid_t], f64)
        self.k_glob = 0
        # policy hooks (a = 0: the scripted reach)
        self.offset = torch.zeros(N, 3, dtype=f64, device=dev)
        self._init_state()

    # ------------------------------------------------------------------------------------------------------ state
    def _init_state(self):
        N, dev, f64 = self.N, self.dev, torch.float64
        z = lambda dt=f64: torch.zeros(N, dtype=dt, device=dev)  # noqa: E731
        self.phase = torch.full((N,), PH_CART, dtype=torch.long, device=dev)
        self.result = [None] * N
        self.info = [dict() for _ in range(N)]
        self.k_tick = torch.zeros(N, dtype=torch.long, device=dev)       # physics steps left in this tick
        self.q_tgt = self.qpos[:, self.aq].double().clone()
        self.qd_ff = torch.zeros(N, 6, dtype=f64, device=dev)          # joint velocity feed-forward (RBN.FF)
        self.vff_f = torch.zeros(N, 6, dtype=f64, device=dev)          # its low-passed value (RBN.TAU_FF)
        self.grip = self.q_open[None].double().repeat(N, 1)
        self.guard_on = torch.ones(N, dtype=torch.bool, device=dev)
        self.guard = self.sens[:, self.s_wf].double().clone()
        self.trip = torch.full((N,), -1, dtype=torch.long, device=dev)   # -1 none, 0..3 TOUCH, 4 wrist_ft
        self.bad = torch.zeros(N, dtype=torch.bool, device=dev)
        self.bad_phase = torch.full((N,), -1, dtype=torch.long, device=dev)
        self.peak = torch.zeros(N, 4, dtype=f64, device=dev)
        self.h, self.n_wait = z(), torch.zeros(N, dtype=torch.long, device=dev)
        self.at_top = torch.ones(N, dtype=torch.bool, device=dev)
        self.p_cmd = torch.zeros(N, 3, dtype=f64, device=dev)
        self.p_tick = torch.zeros(N, 3, dtype=f64, device=dev)
        self.k_close = torch.zeros(N, dtype=torch.long, device=dev)
        self.k_phase = torch.zeros(N, dtype=torch.long, device=dev)      # physics steps spent in the current phase
        self.load_ax = torch.zeros(N, dtype=torch.long, device=dev)
        self.rel0 = None
        self.first_tick = torch.zeros(N, dtype=torch.bool, device=dev)
        self.trk_max = torch.zeros(N, dtype=f64, device=dev)
        self.bad0 = torch.zeros(N, dtype=torch.bool, device=dev)
        self.slip = torch.zeros(N, 2, dtype=f64, device=dev)
        self.t_sum = torch.zeros(N, 4, dtype=f64, device=dev)
        self.t_n = torch.zeros(N, dtype=torch.long, device=dev)
        # s25.4: the descent catch-up's feed-forward of the tracked point's velocity, and the lost-catch count
        self.v_ff = torch.zeros(N, 3, dtype=f64, device=dev)
        self.n_lost = torch.zeros(N, dtype=torch.long, device=dev)
        self._plan_cart()

    def _site(self, q=None):
        q = self.qpos[:, self.aq].double() if q is None else q
        p, R, _, _ = self.A.fk(self.bp, self.bR, q)
        return p, R

    def sect(self, h, idx=None):
        if idx is None:
            return section(self.c, self.n, self.rad, h, self.H_TOP, self.R_R)
        return section(self.c[idx], self.n[idx], self.rad[idx], h, self.H_TOP, self.R_R)

    def _plan_cart(self):
        """Bench.cart: waypoints, their IK (60 iters, from the previous), the per-segment step counts."""
        N, dev = self.N, self.dev
        q_now = self.qpos[:, self.aq].double()
        p0, R0 = self._site(q_now)
        p1, R1 = self.sect(torch.full((N,), RBN.H_CLOSE, dtype=torch.float64, device=dev))
        dR = rotvec(R1 @ R0.transpose(1, 2))
        dist, ang = (p1 - p0).norm(dim=-1), dR.norm(dim=-1)
        Nw = torch.maximum(torch.maximum(torch.full_like(dist, 10.0), torch.floor(dist / 0.004)),
                           torch.floor(torch.rad2deg(ang) / 1.0)).long()
        assert int(Nw.max()) <= W_MAX, int(Nw.max())
        qs = torch.zeros(N, W_MAX, 6, dtype=torch.float64, device=dev)
        ok = torch.ones(N, dtype=torch.bool, device=dev)
        q = q_now.clone()
        for j in range(1, int(Nw.max()) + 1):
            act = j <= Nw
            tj = (j / Nw.double()).clamp(max=1.0)
            Rt = rot_from_rotvec(tj[:, None] * dR) @ R0
            qn, ep, er = self.A.ik(self.bp, self.bR, p0 + tj[:, None] * (p1 - p0), Rt, q, 60)
            ok = ok & ~(act & ((ep > 2e-3) | (er > 0.2)))
            q = torch.where(act[:, None], qn, q)
            qs[:, j - 1] = q
        dt = self.dt
        T_lin = torch.maximum(dist / 0.05, ang / math.radians(10.0))
        n_min = torch.clamp(torch.floor(T_lin / dt / Nw.double()), min=1).long()
        prev = torch.cat([q_now[:, None], qs[:, :-1]], 1)
        nseg = torch.ceil((qs - prev).abs().amax(-1) / (RBN.QD_MAX * dt)).long()
        nseg = torch.maximum(nseg, n_min[:, None])
        jj = torch.arange(W_MAX, device=dev)[None]
        nseg = torch.where(jj < Nw[:, None], nseg, torch.zeros_like(nseg))
        self.c_qs, self.c_prev, self.c_n, self.c_Nw = qs, prev, nseg, Nw
        self.c_seg = torch.zeros(N, dtype=torch.long, device=dev)
        self.c_k = torch.zeros(N, dtype=torch.long, device=dev)
        for i in torch.nonzero(~ok).flatten().tolist():
            self._finish(i, "abort: path")

    # ------------------------------------------------------------------------------------------------------ loop
    def _finish(self, i, res):
        if self.result[i] is None:
            self.result[i] = res
            self.phase[i] = PH_DONE

    def touches(self):
        return self.sens[:, self.s_touch].double()

    def seat(self):
        """Bench.seat on the truth (scoring only), batched: dict of (N,) tensors (mm, deg)."""
        tq = self.qpos[:, self.tq:self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        cc = tq[:, :3] + (Rc @ self.C_B)
        nn = Rc @ self.N_B
        p, Rg = self._site()
        v = p - cc
        h = (v * nn).sum(-1)
        rv = v - h[:, None] * nn
        r = rv.norm(dim=-1)
        ang = lambda a, b: torch.rad2deg(torch.acos(((a * b).sum(-1) / a.norm(dim=-1) / b.norm(dim=-1)).clamp(-1, 1)))  # noqa: E731
        return dict(rad_mm=(r - self.R_R) * 1e3, h_mm=(h - self.H_TOP) * 1e3,
                    tilt_deg=ang(Rg[:, :, 0], -nn), yaw_deg=ang(Rg[:, :, 2], rv))

    def _rel(self):
        tq = self.qpos[:, self.tq:self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        p, Rg = self._site()
        Rt = Rc.transpose(1, 2)
        return (Rt @ (p - tq[:, :3])[:, :, None]).squeeze(-1), Rt @ Rg

    def _contacts(self):
        wp = self.wp
        n = int(wp.to_torch(self.wd.nacon)[0])
        if n == 0:
            return
        g = wp.to_torch(self.wd.contact.geom)[:n].long()
        w = wp.to_torch(self.wd.contact.worldid)[:n].long()
        s1, s2 = self.g_side[g[:, 0]], self.g_side[g[:, 1]]
        sv = (s1 > 0) & (s2 > 0) & (s1 != s2)
        tool = (self.g_tool[g[:, 0]] & self.g_ring[g[:, 1]]) | (self.g_tool[g[:, 1]] & self.g_ring[g[:, 0]])
        badc = sv & ~tool
        if bool(badc.any()):
            nb = torch.zeros(self.N, dtype=torch.bool, device=self.dev)
            nb[w[badc]] = True
            new = nb & ~self.bad & (self.phase != PH_DONE)
            self.bad_phase = torch.where(new, self.phase, self.bad_phase)
            if bool(new.any()):                                   # the first bad contact per world (diagnostics)
                nm = lambda x: mujoco.mj_id2name(self.km, mujoco.mjtObj.mjOBJ_GEOM, int(x)) or f"#{int(x)}"  # noqa: E731
                pos = wp.to_torch(self.wd.contact.pos)[:n]
                dist = wp.to_torch(self.wd.contact.dist)[:n]
                st = self.seat()                                  # truth, scoring / diagnostics only
                for i in torch.nonzero(new).flatten().tolist():
                    self.info[i]["seat_bad"] = {k: round(float(v[i]), 2) for k, v in st.items()}
                    js = torch.nonzero(badc & (w == i)).flatten().tolist()
                    self.info[i]["bad_contact"] = dict(
                        ph=PH_NAME[int(self.phase[i])], k_phase=int(self.k_phase[i]), h_mm=round(float(self.h[i]) * 1e3, 1),
                        pairs=[(nm(g[j, 0]), nm(g[j, 1]), [round(float(x), 4) for x in pos[j]],
                                round(float(dist[j]) * 1e3, 2)) for j in js[:4]])
            self.bad |= new

    def physics_step(self):
        wp = self.wp
        q, qd = self.qpos[:, self.aq], self.qvel[:, self.av]
        if RBN.FF[0]:
            self.vff_f = self.vff_f + (self.qd_ff.clamp(-RBN.VFF_MAX, RBN.VFF_MAX) - self.vff_f) * (self.dt / RBN.TAU_FF)
            vff = self.vff_f.float()
        else:
            vff = 0.0
        u = self.kp * (self.q_tgt.float() - q) + self.kdv * (vff - qd)
        self.ctrl[:, self.aa] = torch.maximum(torch.minimum(u, self.lim), -self.lim)
        self.ctrl[:, self.ag] = self.grip.float()
        wp.capture_launch(self.graph)
        self.k_glob += 1
        self._pin()
        self._contacts()
        t = self.touches()
        if DIAG:
            wf_all = (self.sens[:, self.s_wf].double() - self.guard).norm(dim=-1)
            for i in range(self.N):
                tl = self.info[i].get("trips")
                if tl and len(tl[-1].setdefault("after", [])) < 4:
                    tl[-1]["after"].append(([round(float(x), 2) for x in t[i]], round(float(wf_all[i]), 2)))
        live = self.phase != PH_DONE
        self.peak = torch.where((live & (self.phase >= PH_DESC))[:, None], torch.maximum(self.peak, t), self.peak)
        avg = (self.phase == PH_HOLD) & (self.k_phase >= int(1.0 / self.dt) - HOLD_AVG)
        self.t_sum = torch.where(avg[:, None], self.t_sum + t, self.t_sum)
        self.t_n = torch.where(avg, self.t_n + 1, self.t_n)
        # contact guard (every step): touch pad > TOUCH_N, else the wrist force step > FT_N
        g = self.guard_on & (self.trip < 0) & live
        if bool(g.any()):
            tm, ti = t.max(-1)
            wf = (self.sens[:, self.s_wf].double() - self.guard).norm(dim=-1)
            tr = torch.where(tm > RBN.TOUCH_N, ti, torch.where(wf > RBN.FT_N, torch.full_like(ti, 4), torch.full_like(ti, -1)))
            new = g & (tr >= 0)
            for i in torch.nonzero(new).flatten().tolist():      # the sensors at the trip (diagnostics)
                tr_ = dict(ph=PH_NAME[int(self.phase[i])], touch=[round(float(x), 2) for x in t[i]],
                           wf=round(float(wf[i]), 2), k=int(self.k_phase[i]))
                if DIAG:
                    wp = self.wp
                    nc = int(wp.to_torch(self.wd.nacon)[0])
                    gg = wp.to_torch(self.wd.contact.geom)[:nc].cpu().numpy()
                    ww = wp.to_torch(self.wd.contact.worldid)[:nc].cpu().numpy()
                    pp = wp.to_torch(self.wd.contact.pos)[:nc].cpu().numpy()
                    dd = wp.to_torch(self.wd.contact.dist)[:nc].cpu().numpy()
                    nm = lambda g: mujoco.mj_id2name(self.km, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or f"#{g}"  # noqa: E731
                    tr_["contacts"] = [(nm(gg[j, 0]), nm(gg[j, 1]), [round(float(x), 4) for x in pp[j]], round(float(dd[j]) * 1e3, 3))
                                       for j in range(nc) if ww[j] == i]
                    tr_["state"] = (self.qpos[i].double().cpu().numpy().tolist(), self.qvel[i].double().cpu().numpy().tolist(),
                                    self.ctrl[i].double().cpu().numpy().tolist())
                self.info[i].setdefault("trips", []).append(tr_)
            self.trip = torch.where(new, tr, self.trip)
            self.k_tick = torch.where(new, torch.zeros_like(self.k_tick), self.k_tick)   # the tick ends at the trip
        self.k_phase += 1

    def _pin(self):
        """The bus held at its station (the bench: fixed; rl/gpu_reach_moving.py: carried with the client)."""
        self.qpos[:, self.cq:self.cq + 7] = self.pin.float()
        self.qvel[:, self.cv:self.cv + 6] = 0.0

    def _update_estimate(self):
        """Bench: the estimate is fixed (Bench.track). Moving clients: carried with the estimated motion."""

    def point_vel(self, p, idx):
        """The estimated velocity of world points fixed to the client (Bench.point_vel: 0, the client at rest)."""
        return torch.zeros_like(p)

    def com_vel(self):
        """The client's COM velocity (world, m/s): the free joint's origin velocity + w x (R r_com)."""
        tq = self.qpos[:, self.tq:self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        w = (Rc @ self.qvel[:, self.tv + 3:self.tv + 6].double()[:, :, None]).squeeze(-1)
        return self.qvel[:, self.tv:self.tv + 3].double() + torch.cross(w, Rc @ self.r_com, dim=-1)

    def run(self, max_steps=40000):
        import time as _t
        tb = _t.time()
        for k in range(max_steps):
            tb = throttle(k, tb)
            self.control()
            if bool((self.phase == PH_DONE).all()):
                break
            self.physics_step()
            self.k_tick = (self.k_tick - 1).clamp(min=0)
        for i in range(self.N):
            if self.result[i] is None:
                self.result[i] = "timeout"
        return self.result

    # ------------------------------------------------------------------------------------------------------ control
    def _set_phase(self, m, ph):
        self.phase = torch.where(m, torch.full_like(self.phase, ph), self.phase)
        self.k_phase = torch.where(m, torch.zeros_like(self.k_phase), self.k_phase)

    def control(self):
        """Called before every physics step; acts on the worlds whose tick has ended (or every step in the cart)."""
        self._update_estimate()
        dt, ph = self.dt, self.phase
        N, dev = self.N, self.dev
        # ---- cart: per physics step interpolation between waypoints (Bench.cart)
        m = ph == PH_CART
        if bool(m.any()):
            trip = m & (self.trip >= 0)
            for i in torch.nonzero(trip).flatten().tolist():
                nm = (list(TOUCH) + ["wrist_ft"])[int(self.trip[i])]
                self._finish(i, f"abort: contact in the approach ({nm})")
            m = m & ~trip
            ar = torch.arange(N, device=dev)
            seg = self.c_seg.clamp(max=W_MAX - 1)
            n_s = self.c_n[ar, seg]
            fin = m & ((seg >= self.c_Nw) | (n_s == 0))
            if bool(fin.any()):
                last = self.c_qs[ar, (self.c_Nw - 1).clamp(min=0)]
                self.q_tgt = torch.where(fin[:, None], last, self.q_tgt)
                self.qd_ff = torch.where(fin[:, None], torch.zeros_like(self.qd_ff), self.qd_ff)
                self._set_phase(fin, PH_SETTLE)
            mm = m & ~fin
            if bool(mm.any()):
                k = self.c_k + 1
                qa, qb = self.c_prev[ar, seg], self.c_qs[ar, seg]
                qt = qa + (qb - qa) * (k.double() / n_s.clamp(min=1).double())[:, None]
                self.q_tgt = torch.where(mm[:, None], qt, self.q_tgt)
                self.qd_ff = torch.where(mm[:, None], (qb - qa) / (n_s.clamp(min=1).double() * self.dt)[:, None], self.qd_ff)
                adv = mm & (k >= n_s)
                self.c_k = torch.where(mm, torch.where(adv, torch.zeros_like(k), k), self.c_k)
                self.c_seg = torch.where(adv, self.c_seg + 1, self.c_seg)
        # ---- settle 0.5 s (a guard trip cuts it short, as Bench.step's early return)
        m = (self.phase == PH_SETTLE) & ((self.k_phase >= int(0.5 / dt)) | (self.trip >= 0))
        if bool(m.any()):
            self._start_descent(m)
        # ---- descent ticks
        m = (self.phase == PH_DESC) & (self.k_tick == 0)
        if bool(m.any()):
            self._descent_tick(m)
        # ---- close ticks
        m = (self.phase == PH_CLOSE) & (self.k_tick == 0)
        if bool(m.any()):
            self._close_tick(m)
        # ---- hold 1 s, then score and the load test
        m = (self.phase == PH_HOLD) & (self.k_phase >= int(1.0 / dt))
        if bool(m.any()):
            self._end_hold(m)
        m = self.phase == PH_LOAD
        if bool(m.any()):
            self._load(m)

    def _start_descent(self, m):
        idx = torch.nonzero(m).flatten()
        p, _ = self._site()
        self.p_cmd[idx] = p[idx]
        self.h[idx] = RBN.H_CLOSE
        self.at_top[idx] = True
        self.n_wait[idx] = 0
        self.n_lost[idx] = 0
        self.v_ff[idx] = 0.0
        self.guard[idx] = self.sens[idx][:, self.s_wf].double()
        self.trip[idx] = -1
        self.guard_on[idx] = True
        self.peak[idx] = 0.0
        self.q_tgt[idx] = self.qpos[idx][:, self.aq].double()
        self.qd_ff[idx] = 0.0
        s = self.seat()
        for i in idx.tolist():
            self.info[i]["seat_pre"] = {k: round(float(v[i]), 2) for k, v in s.items()}
        self._set_phase(m, PH_DESC)
        self.k_tick[idx] = 0
        self.first_tick = self.first_tick | m

    def _descent_tick(self, m):
        dt = self.dt
        # the end of the previous tick (not on the first): arm tracking monitor, then the guard trip
        prev = m & ~self.first_tick
        self.first_tick = self.first_tick & ~m
        if bool(prev.any()):
            p, _ = self._site()
            trk = (p - self.p_tick).norm(dim=-1)
            self.trk_max = torch.where(prev, torch.maximum(self.trk_max, trk), self.trk_max)
            bad_trk = prev & (trk > RBN.TRACK_MM / 1e3)
            for i in torch.nonzero(bad_trk).flatten().tolist():
                self.info[i]["trk_abort"] = dict(h_mm=round(float(self.h[i]) * 1e3, 1), n_wait=int(self.n_wait[i]),
                                                 at_top=bool(self.at_top[i]),
                                                 gap_mm=round(float((self.sect(self.h[i:i + 1], torch.tensor([i], device=self.dev))[0][0]
                                                                     - self.p_cmd[i]).norm()) * 1e3, 2))
            for i in torch.nonzero(bad_trk).flatten().tolist():
                self._finish(i, f"abort: arm tracking ({float(trk[i]) * 1e3:.1f} mm)")
            tr = prev & ~bad_trk & (self.trip >= 0)
            for i in torch.nonzero(tr).flatten().tolist():
                nm = (list(TOUCH) + ["wrist_ft"])[int(self.trip[i])]
                self.info[i]["descent"] = dict(stop=nm, h_mm=round(float(self.h[i]) * 1e3, 1))
                if nm == "palm":
                    self._start_close(i)
                else:
                    self._finish(i, f"abort: descent ({nm})")
            m = m & (self.phase == PH_DESC)
        if not bool(m.any()):
            return
        # loop condition
        end = m & (self.h <= -RBN.OVER)
        for i in torch.nonzero(end).flatten().tolist():
            self.info[i]["descent"] = dict(stop="none", h_mm=round(float(self.h[i]) * 1e3, 1))
            self._finish(i, "abort: descent (no contact)")
        m = m & ~end
        self.n_wait = torch.where(m, self.n_wait + 1, self.n_wait)
        late = m & self.at_top & (self.n_wait * 5 * dt > RBN.CATCH_S)
        for i in torch.nonzero(late).flatten().tolist():
            self._finish(i, "abort: descent (grasp point not caught up)")
        m = m & ~late
        lost = m & (self.n_lost * 5 * dt > RBN.CATCH_S)              # s25.4 (reach_bench.reach_close)
        for i in torch.nonzero(lost).flatten().tolist():
            self._finish(i, "abort: descent (lost the grasp point)")
        m = m & ~lost
        slow = m & (self.n_wait * 5 * dt > RBN.T_DESC_MAX)
        for i in torch.nonzero(slow).flatten().tolist():
            self._finish(i, "abort: descent (time)")
        m = m & ~slow
        if not bool(m.any()):
            return
        idx = torch.nonzero(m).flatten()
        h = self.h[idx]
        p_cmd = self.p_cmd[idx]
        n_, r_ = self.n[idx], self.rad[idx]
        o = self.offset[idx]                                           # policy hook (a = 0: none)
        o_w = o[:, :1] * r_ + o[:, 1:2] * torch.cross(n_, r_, dim=-1) + o[:, 2:] * n_
        p_t, R = self.sect(h, idx)
        if RBN.CATCH_FF[0]:          # the tracked point's velocity from the estimate's state (v + w x r), clipped
            vf = self.point_vel(p_t, idx)
            nv = vf.norm(dim=-1, keepdim=True)
            vf = vf * (RBN.V_FF_MAX / nv.clamp(min=RBN.V_FF_MAX))
            self.v_ff[idx] = vf
            p_cmd = p_cmd + vf * 5 * dt
        p_t = p_t + o_w
        gap = p_t - p_cmd
        gn = gap.norm(dim=-1)
        step_max = RBN.V_CATCH * dt * 5
        caught = gn <= RBN.CATCH_MM / 1e3
        v = torch.where(h > RBN.H_SLOW, torch.full_like(h, RBN.V_DESC), torch.full_like(h, RBN.V_SLOW))
        h = torch.where(caught, h - v * dt * 5, h)
        p_t2, R2 = self.sect(h, idx)
        p_t2 = p_t2 + o_w
        gap = torch.where(caught[:, None], p_t2 - p_cmd, gap)
        R = torch.where(caught[:, None, None], R2, R)
        gn = gap.norm(dim=-1)
        p = p_cmd + torch.where((gn <= step_max)[:, None], gap, gap * (step_max / gn.clamp(min=1e-12))[:, None])
        q, ep, er = self.A.ik(self.bp[idx], self.bR[idx], p, R, self.q_tgt[idx], 20)
        bad = (ep > 1e-3) | (er > 0.1)
        for j in torch.nonzero(bad).flatten().tolist():
            self._finish(int(idx[j]), "abort: IK in the descent")
        okk = ~bad
        ii = idx[okk]
        self.h[ii] = h[okk]
        self.at_top[ii] = self.at_top[ii] & ~caught[okk]
        self.n_lost[ii] = torch.where(caught[okk], torch.zeros_like(self.n_lost[ii]),
                                      self.n_lost[ii] + (~self.at_top[ii]).long())
        self.p_cmd[ii] = p[okk]
        self.p_tick[ii] = p[okk]
        self.qd_ff[ii] = (q[okk] - self.q_tgt[ii]) / (5 * dt)
        self.q_tgt[ii] = q[okk]
        self.k_tick[ii] = 5

    def _start_close(self, i):
        self.guard_on[i] = False
        self.trip[i] = -1
        p, _ = self._site()
        p0, _ = self.sect(torch.zeros(1, dtype=torch.float64, device=self.dev), torch.tensor([i], device=self.dev))
        self.h[i] = float(((p[i] - p0[0]) * self.n[i]).sum())
        d = torch.tensor([-self.sq, self.sq], dtype=torch.float64, device=self.dev) - self.q_open.double()
        T_ramp = float(d.abs().max()) / RBN.V_JAW[0]
        self.n_ramp = max(1, int(T_ramp / self.dt))
        self.n_tot = self.n_ramp + int(2.0 / self.dt)
        self.d_jaw = d
        self.k_close[i] = 0
        self.phase[i] = PH_CLOSE
        self.k_phase[i] = 0
        self.k_tick[i] = 0
        self.info[i]["close_s"] = round(T_ramp + 2.0, 2)

    def _close_tick(self, m):
        dt = self.dt
        done = m & (self.k_close >= self.n_tot)
        if bool(done.any()):
            idx = torch.nonzero(done).flatten()
            jv = self.qpos[idx][:, self.gq]
            for j, e in enumerate(self.ev.tolist()):
                self.eqm[idx, e, 0] = jv[:, j]
            self.eqa[idx[:, None], self.ev[None]] = True
            self.grip[idx] = torch.tensor([-self.sq, self.sq], dtype=torch.float64, device=self.dev)
            self.qd_ff[idx] = 0.0
            for i, row in zip(idx.tolist(), jv.tolist()):
                self.info[i]["jaw_mm"] = [round(x * 1e3, 2) for x in row]
            self._set_phase(done, PH_HOLD)
        m = m & ~done
        if not bool(m.any()):
            return
        idx = torch.nonzero(m).flatten()
        fp = self.touches()[idx, 0]
        h = self.h[idx] - RBN.V_FOLLOW * 5 * dt * ((RBN.F_PALM - fp) / RBN.F_PALM).clamp(-1.0, 1.0)
        p, R = self.sect(h, idx)
        q, _, _ = self.A.ik(self.bp[idx], self.bR[idx], p, R, self.q_tgt[idx], 20)
        k = self.k_close[idx]
        frac = ((k + 5).double() / self.n_ramp).clamp(max=1.0)
        self.h[idx] = h
        self.qd_ff[idx] = 0.0                     # no feed-forward in contact (as reach_bench)
        self.q_tgt[idx] = q
        self.grip[idx] = self.q_open.double()[None] + self.d_jaw[None] * frac[:, None]
        self.k_close[idx] = k + 5
        self.k_tick[idx] = 5

    def _end_hold(self, m):
        idx = torch.nonzero(m).flatten()
        t = self.t_sum / self.t_n.clamp(min=1)[:, None]     # the pads averaged over the hold's last HOLD_AVG steps
        s = self.seat()
        vcli = self.com_vel().norm(dim=-1) * 1e3
        for i in idx.tolist():
            self.info[i]["closed"] = dict(seat={k: round(float(v[i]), 2) for k, v in s.items()},
                                          touch={k: round(float(t[i, j]), 1) for j, k in enumerate(TOUCH)},
                                          client_v_mm_s=round(float(vcli[i]), 2))
        if not self.hold_check:
            for i in idx.tolist():
                self._classify(i, None)
            return
        p, R = self._rel()
        if self.rel0 is None:
            self.rel0 = (p.clone(), R.clone())
        self.rel0[0][idx], self.rel0[1][idx] = p[idx], R[idx]
        self.load_ax[idx] = 0
        self._set_phase(m, PH_LOAD)

    def _load(self, m):
        """3 N m about client axis k for 2 s, 0.5 s off, slip after each (reach_close's hold check)."""
        dt = self.dt
        n_on, n_off = int(2.0 / dt), int(0.5 / dt)
        kp = self.k_phase
        ax = self.load_ax
        on = m & (kp < n_on)
        w = torch.zeros(self.N, 6, dtype=self.xfrc.dtype, device=self.dev)
        w[torch.arange(self.N, device=self.dev), 3 + ax.clamp(max=2)] = RBN.LOAD_NM
        self.xfrc[:, self.bid_t] = torch.where(on[:, None], w, torch.where(m[:, None], torch.zeros_like(w),
                                                                                    self.xfrc[:, self.bid_t]))
        blk = m & (kp >= n_on + n_off)
        if bool(blk.any()):
            p, R = self._rel()
            p0, R0 = self.rel0
            dp = (p - p0).norm(dim=-1) * 1e3
            dang = torch.rad2deg(rotvec(R0.transpose(1, 2) @ R).norm(dim=-1))
            self.slip[:, 0] = torch.where(blk, torch.maximum(self.slip[:, 0], dp), self.slip[:, 0])
            self.slip[:, 1] = torch.where(blk, torch.maximum(self.slip[:, 1], dang), self.slip[:, 1])
            self.load_ax = torch.where(blk, ax + 1, ax)
            self.k_phase = torch.where(blk, torch.zeros_like(kp), self.k_phase)
            fin = blk & (self.load_ax >= 3)
            for i in torch.nonzero(fin).flatten().tolist():
                self.info[i]["load"] = dict(slip_mm=round(float(self.slip[i, 0]), 2), slip_deg=round(float(self.slip[i, 1]), 2))
                self._classify(i, self.info[i]["load"])
            if bool(fin.any()):
                self.eqa[torch.nonzero(fin).flatten()[:, None], self.ev[None]] = False

    def _classify(self, i, load):
        c = self.info[i]["closed"]
        t, s = c["touch"], c["seat"]
        self.info[i]["peak"] = {f"pad_{k}": round(float(self.peak[i, j]), 1) for j, k in enumerate(TOUCH)}
        grip = t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and (t["palm"] > 5 or t["lip"] > 5)
        seated = (abs(s["rad_mm"] - RBN.SEAT0["rad_mm"]) < RBN.SEAT_MM and abs(s["h_mm"] - RBN.SEAT0["h_mm"]) < RBN.SEAT_MM
                  and s["tilt_deg"] < RBN.SEAT_DEG)
        held = load is None or (load["slip_mm"] < RBN.SLIP_MM and load["slip_deg"] < RBN.SLIP_DEG)
        res = ("grip" if grip and seated and held else "grip, slips" if grip and seated else
               "grip, off seat" if grip else "no grip")
        self._finish(i, res)

    def outcome(self, i):
        r = self.result[i]
        if bool(self.bad[i]) or bool(self.bad0[i]):
            r = "collision" if r is not None and not r.startswith("abort") else r
        return r


def gpu_temp():
    import subprocess
    try:
        return int(subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                                  capture_output=True, text=True, timeout=10).stdout.split()[0])
    except Exception:  # noqa: BLE001
        return -1


def throttle(k, tb):
    """Half GPU load (s25, 23 Nov: the laptop shut down under sustained load): every 200 physics steps sleep
    (1 / DUTY - 1) x the time since tb; every 2000 check the GPU temperature and pause at T_PAUSE until 8 C cooler.
    Returns the new tb."""
    import time as _t
    if k % 200 or not k:
        return tb
    torch.cuda.synchronize()
    if DUTY[0] < 1.0:
        _t.sleep(max(0.0, _t.time() - tb) * (1.0 / DUTY[0] - 1.0))
    if k % 2000 == 0:
        t = gpu_temp()
        if t >= T_PAUSE[0]:
            print(f"GPU {t} C: pausing", flush=True)
            while t > T_PAUSE[0] - 8:
                _t.sleep(10)
                t = gpu_temp()
    return _t.time()


def _copy(m):
    import copy
    return copy.deepcopy(m)
