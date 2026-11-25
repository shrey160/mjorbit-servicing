"""P7.4 bench: the reach on the estimate (SPEC_7 section 6 and the P7.4 gate: reaches the flange on the estimate, no
truth in the control path).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/reach_bench.py [--n 25] [--probe]

Scene: make_scene_rev2.OUT (servicer rev 2 + ring tool v3 + client_p7), plain MuJoCo, friction 0.4 on tool and ring.
Per case (bench seeds 7400000 + i, not a mission split):
  station   the client free (300 kg, at rest, identity attitude: everything is relative); the servicer at the grasp
            station of tool_gate (its +X face to the ring, bus centre on the ring axis 1.0 m beyond the ring top) rolled
            about the ring axis at random, plus a fly-in error (<= 40 mm, <= 2 deg; P6 hand-over 22-45 mm), held there
            (pinned: station keeping); arm at servicer_ready. Bench mount error and lidar noise as ring_bench (+-10 mm
            / +-1 deg), random sun.
  sense     bus_lidar points (own bodies masked) -> lidar_nav box fit (prior: truth + 50 mm / 2 deg, the P6
            close-range error, as ring_bench) -> ring_fit -> E0 (ring centre and axis, in the frame the servicer
            believes: nominal mount).
  choose    grasp section on E0: the ring section nearest the shoulder, turned >= KEEP_WING away from the solar wing
            (client +Y from the box fit), first that the IK reaches (steps of 15 deg).
  check     arm to the pre-reach pose (grip site H_PRE above the estimated ring top, square on), joint speed <= 10
            deg/s; wrist camera (lamp at the camera: the headlight) shift search on the visible arc of the ring's top
            edges (ring_cam.search on the in-view part) -> E1 (agree / disagree: corrected centre; no verdict / no
            light: E0 kept).
  reach     re-plan the section on E1, IK to H_CLOSE above the top, then a guarded descent along the axis at 10 mm/s
            that stops on any jaw / palm touch > TOUCH_N (or 15 mm past the estimated top).
  close     jaws to the squeeze targets, then the lead-screw lock (as tool_gate), 1 s hold.
Scored with the truth (never used to act): the grip site's radial / axial / tilt error against the true ring at the
start of the descent and after the close, which touch sensors carry load, contacts outside tool-on-ring (any is a
collision), the client's push speed. Success: palm, inboard jaw and hook all loaded, seat within SEAT_MM / SEAT_DEG of
the fixed-client seat (tool_gate), no other contact. Writes runs/reach_bench.json (and runs/reach_*.png with --probe).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_cam as RC  # noqa: E402
import ring_fit  # noqa: E402
import lidar_nav  # noqa: E402
import tool_gate as TG  # noqa: E402

MU = 0.4
RING_R = TG.RING_R                       # wall mid radius: the grip site's radius in the fixed-client seat
STATION = 1.0                            # bus centre beyond the ring top (tool_gate)
FLY_M, FLY_DEG = 0.040, 2.0              # fly-in error at the station
H_APP, H_PRE, H_CLOSE = 0.30, 0.15, 0.075             # grip site above the ring top: camera check, start of the descent
V_DESC, OVER = 0.010, 0.025              # descent speed (m/s), descent past the estimated top before giving up
                                         # (R0, 22 Nov: 15 mm stopped 1.9 mm short of the ring on the mission's
                                         # 12-17 mm axial error, seed 6; any contact on the way still stops it)
TOUCH_N = 1.0                            # (3 N let a free client be pushed along unseen)
WRIST_RES = 128                          # wrist_cam pixels (22 Nov, user: the bar-mission spec, 75 deg / 128 px, conservative; was 384)
LAMP = 0.8                               # wrist lamp brightness (0 = sun only)
QD_MAX = np.radians(10.0)                # joint speed limit (D7.4)
KEEP_WING = np.radians(40.0)
SEAT_MM, SEAT_DEG = 3.0, 1.5
LOAD_NM, SLIP_MM, SLIP_DEG = 3.0, 3.0, 3.0  # hold check (tool_gate: 3 N m turned the locked tool <= 2.6 deg)
RC.SPAN[0.15] = (0.04, 0.002)            # wrist check: +-40 mm, 2 mm steps
RC.AGREE[0.15] = 0.006
FT_N = 2.0                               # wrist force step that stops the descent (contact off the touch pads)
H_SLOW, V_SLOW = 0.015, 0.003            # slow zone above the estimated top (covers the axial error), m/s
F_PALM, V_FOLLOW = 5.0, 0.005           # palm force kept during the close, max follow speed (m/s)
TRACK_MM = 5.0                           # arm tracking monitor in the descent
RC.NS = 720                              # edge samples per circle (0.5 deg: the inboard camera sees a ~40 deg far arc)
CAM_MAX_MM = 40.0                        # a larger correction than this: lidar and camera disagree, no reach
TOOL_B = ("wrist3", "jaw_u", "jaw_l")


class Bench(RB.World):
    def __init__(self):
        super().__init__()
        km = self.km
        for g in range(km.ngeom):
            nm = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if nm.startswith(("ring_wall", "ring_lip", "pad_u", "pad_w", "pad_l", "hook_finger")) or nm in ("palm", "tool_housing", "tool_neck"):
                km.geom_friction[g, 0] = MU
        self.I = TG.ids(km)
        self.wcam = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
        self.lamp = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "wrist_lamp")
        self.WS = WRIST_RES
        self.wr = mujoco.Renderer(km, self.WS, self.WS)
        self.ws = mujoco.Renderer(km, self.WS, self.WS)
        self.ws.enable_segmentation_rendering()
        J = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
        self.gq = [km.jnt_qposadr[J(j)] for j in ("grip_u", "grip_l")]
        self.tv, self.cv = km.jnt_dofadr[J("target_joint")], km.jnt_dofadr[J("chaser_joint")]
        num = lambda n, k: km.numeric_data[km.numeric_adr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_NUMERIC, n)]:][:k].copy()  # noqa: E731
        self.q_open, self.sq = num("grip_open", 2), float(num("grip_squeeze", 1)[0])
        self.shoulder = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, "arm_base")
        B = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
        self.tool_b = {B(n) for n in TOOL_B}
        self.cam_g = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_GEOM, "cam_unit")
        self.ring_g = {g for g in range(km.ngeom) if (mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("ring_")}
        root = km.body_rootid[self.bid_c]
        self.srv_b = {b for b in range(km.nbody) if km.body_rootid[b] == root}
        self.cli_b = {b for b in range(km.nbody) if km.body_rootid[b] == km.body_rootid[self.bid_t]}
        self.kp = np.array([400, 400, 300, 60, 60, 60.0])
        self.kdv = 2 * 0.9 * np.sqrt(self.kp * np.array([3, 3, 1.5, 0.2, 0.2, 0.2]))
        jr = km.jnt_range[[km.joint(j).id for j in RB.ARM_J]]
        self.jlo, self.jhi = jr[:, 0] + 0.02, jr[:, 1] - 0.02
        self.guard, self.trip, self.stop_on_trip = None, None, False
        self.peak, self._f6 = None, np.zeros(6)
        self.q_app = self._approach_pose()

    def _approach_pose(self):
        """The arm pose the servicer flies into the station with (the ready pose reaches ~0.3 m into the client at
        the station): grip site H_APP above the design ring top at the section nearest the shoulder, square on.
        Computed once at the error-free station (the ring is round, so it is the same for every roll)."""
        kd = self.kd
        mujoco.mj_resetData(self.km, kd)
        kd.qpos[self.tq:self.tq + 7] = [0, 0, 0, 1, 0, 0, 0]
        kd.qpos[self.cq:self.cq + 7] = [TG.RING_X + STATION, 0, 0, 0, 0, 0, 1]
        kd.qpos[self.arm] = self.poses["ready"]
        mujoco.mj_forward(self.km, kd)
        est = (ring_fit.C_B.copy(), ring_fit.N_B.copy())
        c, n = est
        s = kd.xpos[self.shoulder] - c
        rad = s - (s @ n) * n
        rad /= np.linalg.norm(rad)
        p, R = self.section(est, rad, H_APP)
        q, ep, er = self.ik(p, R, np.array([0.0, 0.3, -0.6, 0.3, 0.0, 0.0]))   # tool_gate's elbow configuration
        assert ep < 1e-3 and er < 0.1, (ep, er)
        return q

    # ---- placement -------------------------------------------------------------------------------------------
    def place_station(self, seed):
        km, kd, rig = self.km, self.kd, self.rig
        rng = np.random.default_rng([seed, 74])
        km.cam_pos[:] = self.base["pos"]
        km.cam_quat[:] = self.base["quat"]
        mujoco.mj_resetData(km, kd)
        self._vff = np.zeros(6)                             # feed-forward filter state: per episode (s24.4)
        # nominal station (tool_gate): turned 180 deg about z; then a roll about the ring axis (client x)
        R0 = Rotation.from_quat([0, 0, 1, 0])               # x, y, z, w: 180 deg about z
        roll = Rotation.from_rotvec([rng.uniform(-np.pi, np.pi), 0, 0])
        p0 = np.array([TG.RING_X + STATION, 0, 0])
        dv = rng.normal(size=3)
        da = rng.normal(size=3)
        dp = FLY_M * rng.uniform(0, 1) * dv / np.linalg.norm(dv)
        dR = Rotation.from_rotvec(np.radians(FLY_DEG) * rng.uniform(0, 1) * da / np.linalg.norm(da))
        Rs = (dR * roll * R0).as_matrix()
        ps = roll.apply(p0) + dp
        kd.qpos[self.tq:self.tq + 7] = [0, 0, 0, 1, 0, 0, 0]
        kd.qpos[self.cq:self.cq + 7] = [*ps, *RB._q_wxyz(Rs)]
        kd.qpos[self.arm] = self.q_app
        kd.qpos[self.gq] = self.q_open
        ids = [self.bcam, self.lid]
        o = 0.5 * (self.base["pos"][ids[0]] + self.base["pos"][ids[1]])
        Rb = Rotation.from_rotvec(np.radians(rng.uniform(-RB.MOUNT_DEG, RB.MOUNT_DEG, 3)))
        tb = rng.uniform(-RB.MOUNT_M, RB.MOUNT_M, 3)
        for c in ids:
            Ro = Rotation.from_rotvec(np.radians(rng.uniform(-RB.OWN_DEG, RB.OWN_DEG, 3)))
            km.cam_pos[c] = o + Rb.apply(self.base["pos"][c] - o) + tb + rng.uniform(-RB.OWN_M, RB.OWN_M, 3)
            qq = self.base["quat"][c]
            x, y, z, w = (Rb * Ro * Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]])).as_quat()
            km.cam_quat[c] = [w, x, y, z]
        rig.l_sig0, rig.l_drop = rng.uniform(0.005, 0.02), rng.uniform(0.01, 0.05)
        rig._nrng = np.random.default_rng([seed, 74, 7])
        rig.gain, rig.noise = rng.uniform(0.75, 1.25), rng.uniform(0.0, 3.0)
        s = RC.sun(self, rng)
        km.vis.headlight.diffuse[:] = 0.0
        mujoco.mj_forward(km, kd)
        self.rng = rng
        self.pin_bus = kd.qpos[self.cq:self.cq + 7].copy()
        return dict(sun=s, fly_mm=float(np.linalg.norm(dp)) * 1e3, roll_deg=float(np.degrees(roll.magnitude())))

    # ---- sensing ---------------------------------------------------------------------------------------------
    def sense_lidar(self):
        """E0 in the world frame the servicer believes (nominal lidar mount); None if the ring fit fails."""
        P, own = self.lidar_points()
        (pn, Rn), _ = self.frames(self.lid)
        rng = self.rng
        # prior: truth (client at identity) in the nominal lidar frame + 50 mm / 2 deg (the P6 close-range error)
        Rp = Rn.T
        tp = Rn.T @ (-pn)
        dv, da = rng.normal(size=3), rng.normal(size=3)
        t0 = tp + 0.05 * dv / np.linalg.norm(dv) * rng.uniform(0.3, 1.0)
        R0 = Rotation.from_rotvec(np.radians(2.0) * da / np.linalg.norm(da) * rng.uniform(0.3, 1.0)).as_matrix() @ Rp
        b = lidar_nav.fit(P, R0, t0)
        Rb_, tb_ = (b["R"], b["t"]) if b["ok"] else (R0, t0)
        r = ring_fit.fit(P, Rb_, tb_)
        out = dict(n_pts=int(len(P)), own_px=own, box_ok=bool(b["ok"]), ring_ok=bool(r["ok"]), why=r["why"], source="ring")
        if not r["ok"] and b["ok"]:
            # at the station the lidar sees the face close up and partly; the box fit can lock 0.1-0.2 m off (seen
            # 22 Nov), so the ring fit is retried from the prior (the navigation filter's prediction)
            r = ring_fit.fit(P, R0, t0)
            out.update(ring_ok=bool(r["ok"]), why2=r["why"], source="ring (prior)")
        wing = Rn @ Rb_[:, 1]                      # client +Y (the wing side), from the box fit
        if not r["ok"]:
            return out, None, wing
        return out, (pn + Rn @ r["c"], Rn @ r["n"]), wing

    def wrist_check(self, est, D=0.15):
        km, kd = self.km, self.kd
        km.light_active[self.lamp] = 1           # the wrist lamp (make_scene_rev2: LED in the camera unit)
        km.light_diffuse[self.lamp] = LAMP
        self.wr.update_scene(kd, camera=self.wcam, scene_option=self.rig.opt)
        img = self.wr.render().astype(np.float32) * self.rig.gain
        km.light_active[self.lamp] = 0
        img = np.clip(img + self.rig._nrng.normal(0, self.rig.noise, img.shape), 0, 255).astype(np.uint8)
        G, grey = RC.edge_map(img)
        self.ws.update_scene(kd, camera=self.wcam, scene_option=self.rig.opt)
        sg = self.ws.render()
        own = (sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(sg[..., 0], self.self_geoms)
        own = ndimage.binary_dilation(own, iterations=3)
        cam = (kd.cam_xpos[self.wcam].copy(), kd.cam_xmat[self.wcam].reshape(3, 3).copy())   # arm kinematics
        f = (self.WS / 2) / np.tan(np.radians(km.cam_fovy[self.wcam]) / 2)
        return search_arc(G, grey, own, est, cam, f, self.WS, D), img, cam, f

    # ---- grasp section and IK ----------------------------------------------------------------------------------
    @staticmethod
    def section(est, rad, h):
        c, n = est
        x = -n
        R = np.column_stack([x, np.cross(rad, x), rad])
        return c + (ring_fit.H_TOP + h) * n + RING_R * rad, R

    def choose(self, est, wing):
        c, n = est
        u, v = ring_fit._frame(n)
        s = self.kd.xpos[self.shoulder] - c
        a0 = np.arctan2(s @ v, s @ u)
        w = wing - (wing @ n) * n
        aw = np.arctan2(w @ v, w @ u)
        cands = []
        for k in range(0, 13):
            for sg in ((1,) if k == 0 else (1, -1)):
                a = a0 + sg * np.radians(15.0) * k
                dw = abs((a - aw + np.pi) % (2 * np.pi) - np.pi)
                if dw < KEEP_WING:
                    continue
                cands.append(a)
        q_ref = self.kd.qpos[self.arm].copy()
        for a in cands:
            rad = np.cos(a) * u + np.sin(a) * v
            ok, qs = True, []
            for h in (H_PRE, H_CLOSE, -OVER):
                p, R = self.section(est, rad, h)
                q, ep, er = self.ik(p, R, qs[-1] if qs else q_ref, seeds=not qs)
                if ep > 1e-3 or er > 0.1:
                    ok = False
                    break
                qs.append(q)
            if ok:
                return a, rad, qs
        return None, None, None

    def ik(self, p, R, q0, seeds=True):
        """IK on a scratch copy (does not disturb the simulation state)."""
        return ik_lim(self, p, R, q0, seeds=seeds)

    # ---- dynamics --------------------------------------------------------------------------------------------
    def step(self, q_tgt, grip, n=1, qd_tgt=None):
        """Joint PD to q_tgt (+ the target's velocity qd_tgt as feed-forward on the damping term when FF[0]: SPEC_7
        s24.4, without it the PD lags a moving target by kd / kp ~ 0.16 s)."""
        km, kd, I = self.km, self.kd, self.I
        lim = km.actuator_ctrlrange[I["act_arm"], 1]
        v_t = np.clip(qd_tgt, -VFF_MAX, VFF_MAX) if (FF[0] and qd_tgt is not None) else np.zeros(6)
        a_ff = km.opt.timestep / TAU_FF
        for _ in range(n):
            # low-passed (a velocity step through kd is a torque step: the wrist F/T guard tripped on it, s24.4)
            self._vff = getattr(self, "_vff", np.zeros(6)) + (v_t - getattr(self, "_vff", np.zeros(6))) * a_ff
            vff = self._vff if FF[0] else 0.0
            kd.ctrl[I["act_arm"]] = np.clip(self.kp * (q_tgt - kd.qpos[I["arm_q"]]) + self.kdv * (vff - kd.qvel[I["arm_v"]]), -lim, lim)
            kd.ctrl[[I["act_u"], I["act_l"]]] = grip
            self._before_step()
            mujoco.mj_step(km, kd)
            self._after_step()
            self._contacts()
            if self.guard is not None and self.trip is None:      # contact guard: every step (spikes are short)
                t = self.touches()
                if max(t.values()) > TOUCH_N:
                    self.trip = max(t, key=t.get)
                elif np.linalg.norm(TG.sens(km, kd, "wrist_force") - self.guard) > FT_N:
                    self.trip = "wrist_ft"
            if self.trip is not None and self.stop_on_trip:
                return

    def _before_step(self):
        """Hook before each physics step (mission/ring_mission.py: the bus's own control and actuators)."""

    def _after_step(self):
        """The bench's bus is held at its station (ideal station keeping)."""
        kd = self.kd
        kd.qpos[self.cq:self.cq + 7] = self.pin_bus
        kd.qvel[self.cv:self.cv + 6] = 0.0
        mujoco.mj_forward(self.km, kd)

    def _contacts(self):
        km, kd = self.km, self.kd
        fsum = 0.0
        for i in range(kd.ncon):
            c = kd.contact[i]
            b1, b2 = km.geom_bodyid[c.geom1], km.geom_bodyid[c.geom2]
            sv = (b1 in self.srv_b and b2 in self.cli_b) or (b2 in self.srv_b and b1 in self.cli_b)
            if not sv:
                continue
            tool = ((b1 in self.tool_b and c.geom2 in self.ring_g and c.geom1 != self.cam_g) or
                    (b2 in self.tool_b and c.geom1 in self.ring_g and c.geom2 != self.cam_g))
            if tool and self.peak is not None:
                mujoco.mj_contactForce(km, kd, i, self._f6)
                fsum = max(fsum, self._f6[0])          # the largest single contact (not a sum: the clamp's contacts
                                                       # push in opposite directions)
            if not tool:
                gn = lambda g: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or f"#{g}"  # noqa: E731
                self.bad.add(f"{gn(c.geom1)} - {gn(c.geom2)} ({self.phase})")
        if self.peak is not None:
            self.peak["F"] = max(self.peak["F"], float(fsum))
            lim = km.actuator_ctrlrange[self.I["act_arm"], 1]
            self.peak["tau"] = max(self.peak["tau"], float(np.max(np.abs(kd.ctrl[self.I["act_arm"]]) / lim)))
            for k_, v_ in self.touches().items():
                self.peak["pad_" + k_] = max(self.peak.get("pad_" + k_, 0.0), v_)

    def move(self, q1, grip):
        q0 = self.kd.qpos[self.I["arm_q"]].copy()
        T = max(0.5, float(np.max(np.abs(q1 - q0))) / QD_MAX)
        dt = self.km.opt.timestep
        n = int(T / dt)
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)
            self.step(q0 + (q1 - q0) * s, grip)
        self.step(q1, grip, int(0.5 / dt))
        return T

    def track(self, E, rad):
        """The estimate for the next reach update (fixed here: the bus is pinned and the client starts at rest)."""
        return E, rad

    def point_vel(self, p):
        """The estimated velocity of a world point fixed to the client (here 0: the client is held at rest)."""
        return np.zeros(3)

    def arm_guard(self, on):
        """Contact guard on the arm's own sensing (touch pads, wrist F/T step from the baseline now)."""
        self.guard = TG.sens(self.km, self.kd, "wrist_force").copy() if on else None
        self.trip, self.stop_on_trip = None, on

    def cart(self, p1, R1, grip, v=0.05, w=np.radians(10.0)):
        """Straight-line move of the grip site (position lerp, rotation slerp; IK along the line), timed so no
        joint exceeds QD_MAX. Returns the duration, or None if the IK loses the line."""
        kd, I = self.kd, self.I
        q = kd.qpos[I["arm_q"]].copy()
        p0, R0 = kd.site_xpos[I["grip"]].copy(), kd.site_xmat[I["grip"]].reshape(3, 3).copy()
        dR = Rotation.from_matrix(R1 @ R0.T).as_rotvec()
        N = max(10, int(np.linalg.norm(p1 - p0) / 0.004), int(np.degrees(np.linalg.norm(dR)) / 1.0))
        qs = []
        for t in np.linspace(0, 1, N + 1)[1:]:
            Rt = Rotation.from_rotvec(t * dR).as_matrix() @ R0
            q, ep, er = ik_lim(self, p0 + t * (p1 - p0), Rt, q, iters=60)
            if ep > 2e-3 or er > 0.2:
                return None
            qs.append(q)
        dt = self.km.opt.timestep
        T_lin = max(np.linalg.norm(p1 - p0) / v, np.linalg.norm(dR) / w)
        n_min = max(1, int(T_lin / dt / N))
        qa, T = kd.qpos[I["arm_q"]].copy(), 0.0
        for qb in qs:
            n = max(n_min, int(np.ceil(np.max(np.abs(qb - qa)) / (QD_MAX * dt))))
            for k in range(1, n + 1):
                self.step(qa + (qb - qa) * k / n, grip, qd_tgt=(qb - qa) / (n * dt))
                if self.trip is not None and self.stop_on_trip:
                    return None
            T += n * dt
            qa = qb
        self.step(qa, grip, int(0.5 / dt))
        return T

    def touches(self):
        return {k: float(TG.sens(self.km, self.kd, f"touch_{k}")[0]) for k in ("palm", "in", "hook", "lip")}

    def seat(self, rad_t=None):
        """Grip site against the TRUE ring (scoring only): radial error, height above the top, tilt, tangential."""
        kd = self.kd
        Rc = kd.xmat[self.bid_t].reshape(3, 3)
        c = kd.xpos[self.bid_t] + Rc @ ring_fit.C_B
        n = Rc @ ring_fit.N_B
        p = kd.site_xpos[self.I["grip"]]
        Rg = kd.site_xmat[self.I["grip"]].reshape(3, 3)
        v = p - c
        h = float(v @ n)
        rv = v - h * n
        r = float(np.linalg.norm(rv))
        tilt = RB._ang(Rg[:, 0], -n)
        rz = RB._ang(Rg[:, 2], rv)                                  # tool z (radial out) vs the true radial
        return dict(rad_mm=round((r - RING_R) * 1e3, 2), h_mm=round((h - ring_fit.H_TOP) * 1e3, 2),
                    tilt_deg=round(tilt, 2), yaw_deg=round(rz, 2))


SEEDS = (np.array([0.0, 0.3, -0.6, 0.3, 0.0, 0.0]),          # tool_gate's elbow configuration
         np.array([0.0, 0.962, -2.093, 1.131, 0.0, 3.142]))   # the fixed-client grasp (tool_gate)


def _ik1(B, p_des, R_des, q0, iters):
    m, I = B.km, B.I
    d = _scratch(B)
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    q = q0.copy()
    e = np.ones(6)
    for _ in range(iters):
        d.qpos[I["arm_q"]] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        pp, RR = d.site_xpos[I["grip"]], d.site_xmat[I["grip"]].reshape(3, 3)
        e = np.r_[p_des - pp, Rotation.from_matrix(R_des @ RR.T).as_rotvec()]
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
            break
        mujoco.mj_jacSite(m, d, jp, jr, I["grip"])
        J = np.vstack([jp, jr])[:, I["arm_v"]]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        q = q + dq * min(1.0, 0.3 / max(np.max(np.abs(dq)), 1e-12))
    # wrap into the joint ranges (2 pi), then check them
    for k in range(6):
        while q[k] > B.jhi[k] and q[k] - 2 * np.pi >= B.jlo[k]:
            q[k] -= 2 * np.pi
        while q[k] < B.jlo[k] and q[k] + 2 * np.pi <= B.jhi[k]:
            q[k] += 2 * np.pi
    inside = bool(np.all(q >= B.jlo) and np.all(q <= B.jhi))
    return q, float(np.linalg.norm(e[:3])), float(np.degrees(np.linalg.norm(e[3:]))) if inside else 999.0


def ik_lim(B, p_des, R_des, q0, iters=300, seeds=False):
    """tool_gate.ik (step-capped) from q0, the result wrapped into the joint ranges (0.02 rad inside them); a
    solution outside them counts as a failure (rotation error 999). seeds=True also tries SEEDS, keeping the solution
    nearest q0."""
    best = None
    for s0 in [q0] + (list(SEEDS) if seeds else []):
        q, ep, er = _ik1(B, p_des, R_des, s0, iters)
        if ep < 1e-3 and er < 0.1:
            dist = float(np.max(np.abs(q - q0)))
            if best is None or dist < best[3]:
                best = (q, ep, er, dist)
        elif best is None and s0 is q0:
            fail = (q, ep, er)
    return best[:3] if best is not None else fail


def search_arc(G, grey, own, est, cam, f, S, D=0.15):
    """ring_cam.search on the part of the ring's top edges that is in view (the wrist camera sees an arc)."""
    cp, cR = cam
    c, n = est
    u, v = ring_fit._frame(n)
    span, step = RC.SPAN[D]
    base = np.concatenate([RC.circle_pts(c, n, r, h) for r, h in RC.EDGES.values()])
    p0, ok0 = RC.project(base, cp, cR, f, S)
    q = (base - cp) @ cR
    ok0 &= -q[:, 2] < 1.2                       # within 1.2 m (the inboard camera sees the far arc across the ring)
    if ok0.sum() < 30:
        return dict(verdict="out of view", n_in=int(ok0.sum()))
    base = base[ok0]
    lo = np.clip(np.floor(p0[ok0].min(0)).astype(int) - 10, 0, S - 1)
    hi = np.clip(np.ceil(p0[ok0].max(0)).astype(int) + 10, 0, S - 1)
    box, gbox = G[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1], grey[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1]
    ref = float(np.percentile(box, 99)) if box.size else 0.0
    if gbox.mean() < RC.DARK_T or ref < 1.0:
        return dict(verdict="no light", grey=round(float(gbox.mean()), 1))
    Gn = ndimage.maximum_filter(G, size=3) / ref
    g = np.arange(-span, span + 1e-9, step)
    sc = np.full((len(g), len(g)), np.nan)
    for i, a in enumerate(g):
        for j, b in enumerate(g):
            p, ok = RC.project(base + a * u + b * v, cp, cR, f, S)
            ij = np.round(p[ok]).astype(int)
            keep = ~own[ij[:, 1], ij[:, 0]]
            if keep.sum() < 0.3 * len(base):
                continue
            sc[i, j] = float(Gn[ij[keep, 1], ij[keep, 0]].mean())
    if not np.isfinite(sc).any():
        return dict(verdict="occluded")
    i, j = np.unravel_index(np.nanargmax(sc), sc.shape)
    peak = float(sc[i, j] / max(np.nanmedian(sc), 1e-9))
    dd = g[i] * u + g[j] * v
    out = dict(shift_mm=round(float(np.hypot(g[i], g[j])) * 1e3, 1), peak=round(peak, 2), c=c + dd,
               grey=round(float(gbox.mean()), 1), n_in=int(len(base)),
               edge=bool(abs(g[i]) >= span - 1e-9 or abs(g[j]) >= span - 1e-9))
    if peak < RC.PEAK_T or out["edge"]:
        out["verdict"] = "no verdict"
    else:
        out["verdict"] = "agree" if np.hypot(g[i], g[j]) <= RC.AGREE[D] + 1e-9 else "disagree"
    return out


def est_err(est, truth):
    """In-plane and axial error of a ring estimate against the true ring (scoring)."""
    c, n = est
    ct, nt = truth
    e = c - ct
    ax = float(e @ nt)
    return dict(inpl_mm=round(float(np.linalg.norm(e - ax * nt)) * 1e3, 1), ax_mm=round(ax * 1e3, 1),
                deg=round(RB._ang(n, nt), 2))


def case(B, seed, probe=False):
    B.bad, B.phase = set(), "start"
    B.arm_guard(False)
    B.peak = None
    row = dict(seed=seed, **{k: round(v, 1) for k, v in B.place_station(seed).items() if k != "sun"})
    truth = (ring_fit.C_B.copy(), ring_fit.N_B.copy())          # client at identity at the start
    sl, E0, wing = B.sense_lidar()
    row.update(sl)
    if E0 is None:
        row["result"] = "abort: no ring fit"
        return row
    row["E0"] = est_err(E0, truth)
    a, rad, qs = B.choose(E0, wing)
    if a is None:
        row["result"] = "abort: no reachable section"
        return row
    q_open = B.q_open
    B.phase = "to pre-reach"
    T = B.cart(*B.section(E0, rad, H_PRE), q_open)
    if T is None:
        row["result"] = "abort: path"
        return row
    row["t_move_s"] = round(T, 1)
    if NO_CAM[0]:                              # --no-cam: the reach on the lidar estimate alone (same path)
        o, img = dict(verdict="agree", shift_mm=0.0, c=E0[0].copy(), skipped=True), None
    else:
        o, img, cam, f = B.wrist_check(E0)
    row["cam"] = {k: v for k, v in o.items() if k != "c"}
    # the inboard camera sees the far arc across the ring: it fixes the radial direction through the grasp section
    # and hardly the tangential one (its tangential shifts reached 30 mm, which turns the local radial ~6 deg and
    # skewed the jaws: low clamp load); so only the radial part of its correction is applied
    if o["verdict"] in ("agree", "disagree"):
        dc = o["c"] - E0[0]
        E1 = (E0[0] + float(dc @ rad) * rad, E0[1])
        o["shift_mm"] = round(abs(float(dc @ rad)) * 1e3, 1)
    else:
        E1 = E0
    row["E1"] = est_err(E1, truth)
    if o["verdict"] not in ("agree", "disagree"):
        row["result"] = f"abort: wrist check ({o['verdict']})"      # safe: the arm goes back to the approach pose
        return row
    if o["shift_mm"] > CAM_MAX_MM:
        row["result"] = "abort: camera and lidar disagree"
        return row
    if probe and img is not None:
        _overlay(img, dict(truth=truth, E0=E0, E1=E1), cam, f, B.WS, ensure_runs_dir() / f"reach_{seed}.png")
    return reach_close(B, row, E1, a)


def reach_close(B, row, E1, a1, on_descent=None, hold_check=True, policy=None):
    """Pre-close, guarded descent, close + lock, hold check on the estimate E1 at azimuth a1 (also grasp_map.py and
    mission/ring_mission.py). on_descent(B): called once at the start of the descent (grasp_map: the client's
    residual rate). B.track(E1, rad1) -> (E1, rad1) is called before every descent / close update (Bench: unchanged;
    the ring mission: the grasp point carried with the client's estimated motion). hold_check False: no 3 N m test
    (the mission's client is not held by anything but the grip)."""
    q_open = B.q_open
    # the grasp azimuth stays the one chosen on E0; the section moves with the camera's correction
    u1, v1 = ring_fit._frame(E1[1])
    rad1 = np.cos(a1) * u1 + np.sin(a1) * v1
    E1, rad1 = B.track(E1, rad1)
    row["az_deg"] = round(float(np.degrees(a1)), 1)
    B.phase = "to pre-close"
    B.arm_guard(True)
    if B.cart(*B.section(E1, rad1, H_CLOSE), q_open) is None:
        row["result"] = ("abort: pre-close (nav anomaly)" if B.trip == "nav anomaly" else
                         f"abort: {'contact in the approach (' + B.trip + ')' if B.trip else 'path'}")
        row["client_v_mm_s"] = round(float(np.linalg.norm(B.kd.qvel[B.tv:B.tv + 3])) * 1e3, 2)
        return row
    row["seat_pre"] = B.seat()
    # guarded descent along the estimated axis
    B.phase = "descent"
    dt = B.km.opt.timestep
    h, q = H_CLOSE, B.kd.qpos[B.I["arm_q"]].copy()
    stop = None
    B.arm_guard(True)                          # wrist F/T baseline here (slow descent: inertial loads small)
    if on_descent is not None:
        on_descent(B)
    B.peak = dict(F=0.0, tau=0.0)              # loads from here on: tool-on-ring normal force sum, arm torque / limit
    # the commanded grip point closes on the tracked one at a bounded speed (R0, 22 Nov: the pre-close move ends on
    # the point tracked when it started; the tracked point moved 4-23 mm meanwhile, and a one-step jump to it jerked
    # the arm: wrist-force trip or tracking abort with no contact, seeds 1, 2, 5); the descent waits until caught up
    p_cmd = B.kd.site_xpos[B.I["grip"]].copy()
    n_wait = n_lost = 0
    if policy is not None:
        policy.reset()
    # s25.4 (23 Nov): the tracked point's velocity from the estimate's state (v + w x r: the client's estimated COM
    # velocity and angular rate, B.point_vel) is fed forward to the commanded point, so V_CATCH bounds the closing
    # speed only (at 4 deg/s the point moves ~40 mm/s, faster than V_CATCH: the descent lost it and chased it round the
    # client for 40-54 s until the jaw met the bus); a descent that loses the point for CATCH_S, or runs T_DESC_MAX,
    # aborts (safe)
    while h > -OVER:
        n_wait += 1
        if h == H_CLOSE and n_wait * 5 * dt > CATCH_S:
            row["result"] = "abort: descent (grasp point not caught up)"
            return row
        if n_lost * 5 * dt > CATCH_S:
            row["result"] = "abort: descent (lost the grasp point)"
            return row
        if n_wait * 5 * dt > T_DESC_MAX:
            row["result"] = "abort: descent (time)"
            return row
        E1, rad1 = B.track(E1, rad1)
        p_t, R = B.section(E1, rad1, h)
        if CATCH_FF[0]:
            v_ff = B.point_vel(p_t)
            nv = float(np.linalg.norm(v_ff))
            if nv > V_FF_MAX:
                v_ff = v_ff * (V_FF_MAX / nv)
            p_cmd = p_cmd + v_ff * 5 * dt
        o_w = np.zeros(3)
        if policy is not None:
            a_w = B.nav.predict(float(B.kd.time))[2] if hasattr(B, "nav") else np.zeros(3)
            t_d = B.touches()
            touches = np.array([t_d["palm"], t_d["in"], t_d["hook"], t_d["lip"]], dtype=float)
            wf_delta = TG.sens(B.km, B.kd, "wrist_force") - B.guard
            o_w = policy.step(
                rad=rad1,
                n=E1[1],
                p_target=p_t,
                p_site=B.kd.site_xpos[B.I["grip"]],
                p_cmd=p_cmd,
                a_w=a_w,
                h=h,
                at_top=(h == H_CLOSE),
                n_wait=n_wait,
                touches=touches,
                wf_delta=wf_delta,
            )
            p_t = p_t + o_w
        gap = p_t - p_cmd
        gn = float(np.linalg.norm(gap))
        step_max = V_CATCH * dt * 5
        if gn <= CATCH_MM / 1e3:               # caught up: descend (slow through the axial uncertainty: a free client
            h -= (V_DESC if h > H_SLOW else V_SLOW) * dt * 5    # is kicked away by the touch, 6.7 mm/s at 10 mm/s)
            p_t, R = B.section(E1, rad1, h)
            p_t = p_t + o_w
            gap = p_t - p_cmd
            gn = float(np.linalg.norm(gap))
            n_lost = 0
        elif h < H_CLOSE:
            n_lost += 1
        p = p_cmd + (gap if gn <= step_max else gap * (step_max / gn))
        p_cmd = p
        q_prev = q
        q, ep, er = ik_lim(B, p, R, q, iters=20)
        if ep > 1e-3 or er > 0.1:
            row["result"] = "abort: IK in the descent"
            return row
        B.step(q, q_open, 5, qd_tgt=(q - q_prev) / (5 * dt))
        trk = float(np.linalg.norm(B.kd.site_xpos[B.I["grip"]] - p))
        if trk > TRACK_MM / 1e3:             # arm-side monitor (encoders + kinematics vs the command)
            row["result"] = f"abort: arm tracking ({trk * 1e3:.1f} mm)"
            return row
        if B.trip is not None:                 # touch pad, or a wrist F/T step (contact off the pads)
            stop = B.trip
            break
    row["descent"] = dict(stop=stop or "none", h_mm=round(h * 1e3, 1), touch={k: round(v, 1) for k, v in B.touches().items()})
    if stop != "palm":                         # only the palm on the ring top starts the close; else back off
        row["result"] = f"abort: descent ({stop or 'no contact'})"
        row["contacts_bad"] = sorted(B.bad)
        row["client_v_mm_s"] = round(float(np.linalg.norm(B.kd.qvel[B.tv:B.tv + 3])) * 1e3, 2)
        return row
    B.arm_guard(False)
    B.phase = "close"
    # palm force accommodation during the close (own sensing: the palm touch pad): the arm steps the grip point along
    # the estimated axis to keep ~F_PALM on the palm, so it follows a client the touch has set drifting (no fixed
    # press: pressing 2 mm kicked the free client away and the jaws then closed on a ring 7.6 mm below the palm)
    h = float((B.kd.site_xpos[B.I["grip"]] - B.section(E1, rad1, 0.0)[0]) @ E1[1])     # where the palm is now
    # close at the lead screws' speed (22 Nov, P7.5: a step to the squeeze target drove the hook tip 1.6 mm into the
    # wall in one step, tens of kN; a lead-screw jaw has a bounded speed). V_JAW[0] = 0: the old step command.
    sq_cmd = np.array([-B.sq, B.sq])
    d = sq_cmd - q_open
    T_ramp = float(np.max(np.abs(d))) / V_JAW[0] if V_JAW[0] > 0 else 0.0
    n_ramp = max(1, int(T_ramp / dt))
    n_tot = n_ramp + int(2.0 / dt)
    q_hold = q
    for k in range(0, n_tot, 5):
        fp = B.touches()["palm"]
        h -= V_FOLLOW * 5 * dt * float(np.clip((F_PALM - fp) / F_PALM, -1.0, 1.0))
        E1, rad1 = B.track(E1, rad1)
        p, R = B.section(E1, rad1, h)
        q_prev = q_hold
        q_hold, _, _ = ik_lim(B, p, R, q_hold, iters=20)
        # both jaws ramp over the same time, so they meet the ring together (at equal speeds the inboard jaw, 6 mm
        # less travel, came first and pushed the free client 11 mm off the seat)
        B.step(q_hold, q_open + d * min(1.0, (k + 5) / n_ramp), 5)   # no feed-forward in contact (s24.4: it doubled
                                                                    # the push, 6 -> 18 mm/s at the hold's end)
    row["close_s"] = round(T_ramp + 2.0, 2)
    km, kd = B.km, B.kd
    q_lock = kd.qpos[B.gq].copy()
    for nm, qv in zip(("lock_u", "lock_l"), q_lock):
        e = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm)
        km.eq_data[e, :5] = [qv, 0, 0, 0, 0]
        kd.eq_active[e] = 1
    B.phase = "hold"
    B.step(q_hold, np.array([-B.sq, B.sq]), int(1.0 / dt))
    t = B.touches()
    s = B.seat()
    row["closed"] = dict(seat=s, touch={k: round(v, 1) for k, v in t.items()}, jaw_mm=np.round(q_lock * 1e3, 2).tolist(),
                         client_v_mm_s=round(float(np.linalg.norm(kd.qvel[B.tv:B.tv + 3])) * 1e3, 2))
    if not hold_check:
        row["contacts_bad"] = sorted(B.bad)
        t = B.touches()
        row["result"] = ("collision" if B.bad else "grip" if t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and
                         (t["palm"] > 5 or t["lip"] > 5) and abs(s["rad_mm"] - SEAT0["rad_mm"]) < SEAT_MM and
                         abs(s["h_mm"] - SEAT0["h_mm"]) < SEAT_MM and s["tilt_deg"] < SEAT_DEG else "no grip")
        row["peak"] = dict(contact_F_N=round(B.peak["F"], 1), arm_tau_frac=round(B.peak["tau"], 2),
                           **{k: round(v, 1) for k, v in B.peak.items() if k.startswith("pad_")})
        return row
    # hold check: 3 N m about each client axis (2 s each, applied to the client; the bus is held), grip slip on the ring
    B.phase = "load"
    def rel():
        Rc = kd.xmat[B.bid_t].reshape(3, 3)
        return Rc.T @ (kd.site_xpos[B.I["grip"]] - kd.xpos[B.bid_t]), Rc.T @ kd.site_xmat[B.I["grip"]].reshape(3, 3)
    p_a, R_a = rel()
    slip = []
    for k in range(3):
        w = np.zeros(6)
        w[3 + k] = LOAD_NM
        kd.xfrc_applied[B.bid_t] = w
        B.step(q_hold, np.array([-B.sq, B.sq]), int(2.0 / dt))
        kd.xfrc_applied[B.bid_t] = 0.0
        B.step(q_hold, np.array([-B.sq, B.sq]), int(0.5 / dt))
        p_b, R_b = rel()
        slip.append((float(np.linalg.norm(p_b - p_a)) * 1e3, float(np.degrees(Rotation.from_matrix(R_a.T @ R_b).magnitude()))))
    row["load"] = dict(slip_mm=round(max(x[0] for x in slip), 2), slip_deg=round(max(x[1] for x in slip), 2))
    row["peak"] = dict(contact_F_N=round(B.peak["F"], 1), arm_tau_frac=round(B.peak["tau"], 2),
                       **{k: round(v, 1) for k, v in B.peak.items() if k.startswith("pad_")})
    for nm in ("lock_u", "lock_l"):
        kd.eq_active[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm)] = 0
    row["contacts_bad"] = sorted(B.bad)
    # the ring held radially between the inboard jaw and the hook (its face or its wedge on the lip), axially on the
    # palm or the wedge (tool v3: the wedge draws the lip onto the palm; either datum can carry the closing load)
    grip = t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and (t["palm"] > 5 or t["lip"] > 5)
    seated = abs(s["rad_mm"] - SEAT0["rad_mm"]) < SEAT_MM and abs(s["h_mm"] - SEAT0["h_mm"]) < SEAT_MM and s["tilt_deg"] < SEAT_DEG
    held = row["load"]["slip_mm"] < SLIP_MM and row["load"]["slip_deg"] < SLIP_DEG
    row["result"] = ("collision" if B.bad else "grip" if grip and seated and held else
                     "grip, slips" if grip and seated else "grip, off seat" if grip else "no grip")
    return row


V_CATCH = 0.030                           # commanded grip-point speed toward the tracked point in the descent (m/s; the
                                          # grasp point moves up to ~20 mm/s at 2 deg/s)
CATCH_MM = 1.0                            # within this of the tracked point, the descent goes on
CATCH_S = 3.0                             # not caught up in this time (before the descent starts, or lost in it): abort
CATCH_FF = [True]                         # s25.4: the tracked point's velocity (estimate v + w x r) fed forward in
                                          # the descent catch-up
V_FF_MAX = 0.060                          # m/s, its clip
T_DESC_MAX = 30.0                         # s, descent time limit (a normal descent ~13 s)
NO_CAM = [False]
TAU_FF = 0.15                            # s, feed-forward low-pass
VFF_MAX = 0.5                            # rad/s, feed-forward clip (an IK 2 pi wrap between ticks must not become a kick)
FF = [True]                              # joint velocity feed-forward in Bench.step, free-space moves (SPEC_7 s24.4)
V_JAW = [0.010]                          # jaw closing speed (the faster jaw), m/s (lead screws); 0 = step command
SEAT0 = dict(rad_mm=1.2, h_mm=0.0)        # the fixed-client seat after the close (runs/tool_gate.json, mu 0.4)


def _scratch(B):
    if not hasattr(B, "_d2"):
        B._d2 = mujoco.MjData(B.km)
    B._d2.qpos[:] = B.kd.qpos
    return B._d2


def _overlay(img, ests, cam, f, S, path):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    col = dict(truth=(0, 255, 0), E0=(255, 255, 0), E1=(0, 255, 255))
    for k, c in col.items():
        for r, h in RC.EDGES.values():
            p, ok = RC.project(RC.circle_pts(*ests[k], r, h), *cam, f, S)
            for (x, y), o in zip(p, ok):
                if o:
                    dr.point((x, y), fill=c)
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--probe", action="store_true", help="overlay PNGs of the wrist check")
    ap.add_argument("--seeds", type=int, nargs="*", help="case indices instead of --start / --n")
    ap.add_argument("--no-cam", action="store_true", help="skip the wrist-camera check (lidar estimate alone)")
    ap.add_argument("--out", default=str(RUNS_DIR / "reach_bench.json"))
    ap.add_argument("--noslip", type=int, default=None, help="override the scene's noslip iterations (SPEC_7 s24 G0c)")
    ap.add_argument("--no-ff", action="store_true", help="no joint velocity feed-forward (before SPEC_7 s24.4)")
    a = ap.parse_args()
    FF[0] = not a.no_ff
    B = Bench()
    if a.noslip is not None:
        B.km.opt.noslip_iterations = a.noslip
    NO_CAM[0] = a.no_cam
    rows = []
    for i in (a.seeds if a.seeds else range(a.start, a.start + a.n)):
        r = case(B, 7400000 + i, probe=a.probe)
        rows.append(r)
        print(json.dumps(r), flush=True)
    res = [r["result"] for r in rows]
    summ = dict(n=len(rows), results={k: res.count(k) for k in sorted(set(res))})
    for k in ("E0", "E1"):
        v = [r[k]["inpl_mm"] for r in rows if k in r]
        if v:
            summ[f"{k}_inpl_mm"] = dict(p50=round(float(np.median(v)), 1), p95=round(float(np.percentile(v, 95)), 1), max=max(v))
    v = [abs(r["seat_pre"]["rad_mm"]) for r in rows if "seat_pre" in r]
    if v:
        summ["pre_close_rad_mm"] = dict(p50=round(float(np.median(v)), 1), p95=round(float(np.percentile(v, 95)), 1), max=max(v))
    cams = [r["cam"]["verdict"] for r in rows if "cam" in r]
    summ["cam"] = {k: cams.count(k) for k in sorted(set(cams))}
    print(json.dumps(summ, indent=1))
    out_p = Path(a.out)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float))
    sys.stdout.flush()
    import os
    os._exit(0)                  # skip the EGL teardown noise


if __name__ == "__main__":
    main()
