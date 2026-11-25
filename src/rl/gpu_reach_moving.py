"""The GPU reach on a moving client with a modelled estimate (SPEC_7 section 24, G2): the harder-client environment.

Truth: the client free (MuJoCo), started with an angular rate w0 = w_ax about its ring axis + w_tr across it (random
direction in the plane), its COM at rest; torque-free from there (it nutates on its own inertia).

Bus: held at its station relative to the client's FREE motion (a torque-free ghost integrated from the start state, so
a push moves the client away from the bus, as on the bench) plus a smooth station-keeping error (3 sinusoids per axis,
0.02-0.2 Hz, rms SIG_ST mm / SIG_STA deg; white noise per step shook the arm); kinematic, as the bench's pin.

Estimate (what the reach acts on; no lidar or camera is simulated, the error model stands in for them):
  ring in the estimated client body frame: the design ring + the residual after the camera correction at pre-reach
    (radial / tangential N(0, SIG_CAM), axial N(AX_MU, SIG_AX)), its axis tilted by the cell's tilt_r about the
    section radial (the tool leans along the ring, P7.5's failing case) + N(0, SIG_TILT) about radial and tangent;
  client pose and rate: re-anchored every T_REFIT (the 2 Hz refit of section 23) to the truth + AR(1) errors in
    position (SIG_P), attitude (SIG_A) and rate (SIG_W), correlation time TAU_E; propagated at constant rate between.
All sigmas scale with the cell's `scale` (1 = the mission's measured spread, roughly; see SPEC_7 s24.3).
The truth is used only to build the estimate and the bus motion (sensor and station simulation) and to score.
"""
from __future__ import annotations

import math

import numpy as np
import torch

from arm_torch import axis_angle_mat, frame, mat_to_quat, quat_to_mat, rot_from_rotvec, section  # noqa: F401
from gpu_reach import GpuReach, PH_CART

T_REFIT = 0.5
# scale 1 fitted to the mission's own reach (checks/calib_track.py, 5 tune seeds, multi-rate, 22 Nov; SPEC_7 s24.4):
# grasp-point steps at the refits median 0.57 / p90 0.98 / max 1.86 mm; slow drift std 1.6-3.0 mm radial over 14-22 s;
# offsets radial -0.7..-2.3, tangential 0..+7, axial -5..+13 mm, axis 0.35-1.14 deg
SIG_CAM, SIG_TAN, AX_MU, SIG_AX, SIG_TILT = 2.5e-3, 3.0e-3, 5e-3, 7e-3, 0.6      # m, m, m, m, deg
# s25 (23 Nov): the wrist stereo module, measured from the camera's actual section frame, with the tilt applied
# (--perc stereo): residual radial / tangential from the 23 mission seeds (|err| p50 0.16 / 0.22, p95 0.53 / 0.89 mm);
# axis tilt from the held-out bench by tilt bin (p50 0.16-0.23, p95 0.86-1.42 deg at 0-3 deg tilt, no trend): per
# axis 90 % N(0, 0.18 deg) + 10 % N(0, 0.7 deg); the cell's tilt_r is measured and removed. The axial error is the
# lidar estimate's (the stereo's axial is not applied in the mission), unchanged.
PERC = ["net"]
ST_CAM, ST_TAN, ST_TILT, ST_TILT_OUT, ST_OUT_P = 0.35e-3, 0.55e-3, 0.18, 0.7, 0.10
SIG_P, SIG_A, SIG_W, TAU_E = 2.0e-3, 0.10, 0.02, 20.0           # m, deg, deg/s, s (first pass 1.5 mm, 0.15, 0.04, 3 s:
                                                                # refit steps ~3x the mission's)
SIG_ST, SIG_STA, TAU_ST = 1.5e-3, 0.15, 3.0                     # m, deg, s
# s25.5 (23 Nov; user: "add a bit lag for realism"): measurement latency. A refit at t anchors on the truth at t - L
# (L = LAT +- LAT_J per world and refit: lidar scan + fit on flight hardware), time-stamped t - L, and the estimate
# propagates from there to now at its constant rate; PROP_NOW False: the anchor is taken as current (no time stamps)
LAT, LAT_J, PROP_NOW = [0.15], [0.02], [True]
H_DT = 0.01                                                     # truth history step (s) for the latency


def _rv(rng, n, sig):
    return rng.normal(0, sig, (n, 3))


class GpuReachMoving(GpuReach):
    def __init__(self, B, starts, cells, seed=0, **kw):
        """starts: the bench pre-reach states (g1_starts), cells: one dict per world (w_ax, w_tr deg/s, tilt_r deg,
        scale) with 'base' = the index of its start."""
        self.cells = cells
        base = [starts[c["base"]] for c in cells]
        self._base_starts = base
        super().__init__(B, base, **kw)
        self.rng = np.random.default_rng(seed)
        self._setup_motion()

    # ------------------------------------------------------------------------------------------------------ setup
    def _setup_motion(self):
        N, dev, f64, rng = self.N, self.dev, torch.float64, self.rng
        T = lambda x: torch.tensor(np.asarray(x), dtype=f64, device=dev)  # noqa: E731
        sc = T([c["scale"] for c in self.cells])
        self.scale = sc
        # the true client at t0 (identity on the bench) and its rate
        tq = self.qpos[:, self.tq:self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        n_w = Rc @ self.N_B
        a = torch.randn(N, 3, dtype=f64, device=dev)
        perp = a - (a * n_w).sum(-1, keepdim=True) * n_w
        perp = perp / perp.norm(dim=-1, keepdim=True)
        w0 = (torch.deg2rad(T([c["w_ax"] for c in self.cells]))[:, None] * n_w
              * T(rng.choice([-1.0, 1.0], N))[:, None]
              + torch.deg2rad(T([c["w_tr"] for c in self.cells]))[:, None] * perp)
        self.qvel[:, self.tv + 3:self.tv + 6] = (Rc.transpose(1, 2) @ w0[:, :, None]).squeeze(-1).float()
        self.qvel[:, self.tv:self.tv + 3] = (-torch.cross(w0, Rc @ self.r_com, dim=-1)).float()
        # station: the bus relative to the true client
        bR0 = quat_to_mat(self.pin[:, 3:7])
        self.st_R = Rc.transpose(1, 2) @ bR0
        self.st_p = (Rc.transpose(1, 2) @ (self.pin[:, :3] - tq[:, :3])[:, :, None]).squeeze(-1)
        # the bus follows the client's FREE motion (a torque-free "ghost" from the start state: a push moves the client
        # away from the bus, as on the bench; a bus following the pushed truth dragged the client to 60 mm/s)
        km = self.km
        Rq = quat_to_mat(T(km.body_iquat[self.bid_t])[None])[0]
        self.I_b = Rq @ torch.diag(T(km.body_inertia[self.bid_t])) @ Rq.T
        self.g_R, self.g_c, self.g_w = Rc.clone(), (tq[:, :3] + Rc @ self.r_com).clone(), w0.clone()
        # smooth station-keeping error: 3 sinusoids per component, 0.02-0.2 Hz, random phases
        f = T(rng.uniform(0.02, 0.2, (N, 6, 3)))
        self.st_f, self.st_ph = 2 * math.pi * f, T(rng.uniform(0, 2 * math.pi, (N, 6, 3)))
        sig = torch.cat([torch.full((3,), SIG_ST), torch.full((3,), math.radians(SIG_STA))]).to(dev).double()
        self.st_A = (sig[None, :, None] * sc[:, None, None]) * math.sqrt(2.0 / 3.0)
        self.e_st = torch.zeros(N, 6, dtype=f64, device=dev)
        # the estimated ring in the estimated body frame
        u, v = frame(self.N_B[None].repeat(N, 1))
        a1 = T([s["a1"] for s in self._base_starts])
        rad_b = torch.cos(a1)[:, None] * u + torch.sin(a1)[:, None] * v
        tan_b = torch.cross(self.N_B[None].repeat(N, 1), rad_b, dim=-1)
        z = lambda s_: T(rng.normal(0, 1, N)) * s_  # noqa: E731
        stereo = PERC[0] == "stereo"
        s_cam, s_tan = (ST_CAM, ST_TAN) if stereo else (SIG_CAM, SIG_TAN)
        d = (z(s_cam) * sc)[:, None] * rad_b + (z(s_tan) * sc)[:, None] * tan_b \
            + (AX_MU * torch.clamp(sc, max=1.0) + z(SIG_AX) * sc)[:, None] * self.N_B[None]
        if stereo:          # the cell's tilt_r is measured and removed; the stereo net's tilt residual remains
            st = lambda: T(np.where(rng.uniform(size=N) < ST_OUT_P, ST_TILT_OUT, ST_TILT))  # noqa: E731
            tilt_r = torch.deg2rad(z(1.0) * st() * sc)
            tilt_t = torch.deg2rad(z(1.0) * st() * sc)
        else:
            tilt_r = torch.deg2rad(T([c["tilt_r"] for c in self.cells]) * T(rng.choice([-1.0, 1.0], N)) + z(SIG_TILT) * sc)
            tilt_t = torch.deg2rad(z(SIG_TILT) * sc)
        Rt = rot_from_rotvec(tilt_r[:, None] * rad_b + tilt_t[:, None] * tan_b + 1e-12)
        self.cb = self.C_B[None] + d
        self.nb = (Rt @ self.N_B)
        self.radb = rad_b - (rad_b * self.nb).sum(-1, keepdim=True) * self.nb
        self.radb = self.radb / self.radb.norm(dim=-1, keepdim=True)
        # estimate errors (AR(1)) and the first anchor
        self.e_p = T(_rv(rng, N, 1.0)) * (SIG_P * sc)[:, None]
        self.e_a = T(_rv(rng, N, 1.0)) * (math.radians(SIG_A) * sc)[:, None]
        self.e_w = T(_rv(rng, N, 1.0)) * (math.radians(SIG_W) * sc)[:, None]
        self.t_fit = -1e9
        self.t_anchor = torch.zeros(N, dtype=f64, device=dev)
        self.hist, self.hist_t = [], -1e9
        self._refit(force=True)
        self._update_estimate(force=True)
        # the arm to the pre-reach pose on this estimate (IK from the recorded pose), then the cart is re-planned
        q0 = self.qpos[:, self.aq].double()
        p, R = self.sect(torch.full((N,), 0.15, dtype=f64, device=dev))
        q, ep, er = self.A.ik(self.bp, self.bR, p, R, q0, 100)
        self.start_ok = (ep < 1e-3) & (er < 0.1)
        self.qpos[:, self.aq] = torch.where(self.start_ok[:, None], q, q0).float()
        self.qvel[:, self.av] = 0.0
        self._pin()
        self.mjw.forward(self.wm, self.wd)
        self._init_state()
        self.v_peak = torch.zeros(N, dtype=f64, device=dev)
        for i in torch.nonzero(~self.start_ok).flatten().tolist():
            self._finish(i, "abort: no start pose")

    # ------------------------------------------------------------------------------------------------------ truth
    def _truth(self):
        tq = self.qpos[:, self.tq:self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        com = tq[:, :3] + Rc @ self.r_com
        w = (Rc @ self.qvel[:, self.tv + 3:self.tv + 6].double()[:, :, None]).squeeze(-1)
        return tq[:, :3], Rc, com, w, self.com_vel()

    @property
    def t_now(self):
        return self.k_glob * self.dt

    def _refit(self, force=False):
        t = self.t_now
        if not force and t - self.t_fit < T_REFIT - 1e-9:
            return
        sc = self.scale[:, None]
        if not force:
            rho = math.exp(-T_REFIT / TAU_E)
            k = math.sqrt(1 - rho * rho)
            rn = lambda: torch.randn(self.N, 3, dtype=torch.float64, device=self.dev)  # noqa: E731
            self.e_p = rho * self.e_p + k * rn() * SIG_P * sc
            self.e_a = rho * self.e_a + k * rn() * math.radians(SIG_A) * sc
            self.e_w = rho * self.e_w + k * rn() * math.radians(SIG_W) * sc
        _, Rc, com, w, vc = self._truth()
        t_a = torch.full((self.N,), t, dtype=torch.float64, device=self.dev)
        if LAT[0] > 0 and self.hist:              # the truth L ago (the newest history entry at or before t - L)
            L = LAT[0] + (torch.rand(self.N, dtype=torch.float64, device=self.dev) * 2 - 1) * LAT_J[0]
            k = torch.clamp(torch.round(L / H_DT).long(), 0, len(self.hist) - 1)
            ar = torch.arange(self.N, device=self.dev)
            Hs = [torch.stack([h[j] for h in self.hist]) for j in range(4)]     # (K, N, ...) newest first
            Rc, com, w, vc = (Hs[0][k, ar], Hs[1][k, ar], Hs[2][k, ar], Hs[3][k, ar])
            t_a = t - k.double() * H_DT
        self.a_R = rot_from_rotvec(self.e_a + 1e-12) @ Rc
        self.a_c = com + self.e_p
        self.a_w = w + self.e_w
        self.a_v = vc
        self.t_fit = t
        self.t_anchor = t_a if PROP_NOW[0] else torch.full_like(t_a, t)

    def _update_estimate(self, force=False):
        if LAT[0] > 0 and self.t_now - self.hist_t >= H_DT - 1e-9:      # the truth history (sensor simulation)
            _, Rc, com, w, vc = self._truth()
            self.hist.insert(0, (Rc, com, w, vc))
            del self.hist[int(round((LAT[0] + LAT_J[0]) / H_DT)) + 2:]
            self.hist_t = self.t_now
        self._refit()
        dt_ = (self.t_now - self.t_anchor)[:, None]
        Rw = rot_from_rotvec(self.a_w * dt_ + 1e-12) @ self.a_R
        cw = self.a_c + self.a_v * dt_
        self.e_com = cw
        self.e_R = Rw
        po = cw - (Rw @ self.r_com)
        self.c = po + (Rw @ self.cb[:, :, None]).squeeze(-1)
        self.n = (Rw @ self.nb[:, :, None]).squeeze(-1)
        self.rad = (Rw @ self.radb[:, :, None]).squeeze(-1)

    def point_vel(self, p, idx):
        """The estimated velocity of world points fixed to the client: v + w x (p - com), from the estimate's state
        (constant rate between refits; s25.4, the descent catch-up's feed-forward)."""
        return self.a_v[idx] + torch.cross(self.a_w[idx], p - self.e_com[idx], dim=-1)

    # ------------------------------------------------------------------------------------------------------ the bus
    def _ghost_step(self):
        """Torque-free rigid body, one physics step (midpoint): I_w w' = -w x (I_w w); R' = [w] R."""
        dt = self.dt

        def wdot(R, w):
            Iw = R @ self.I_b @ R.transpose(1, 2)
            return torch.linalg.solve(Iw, -torch.cross(w, (Iw @ w[:, :, None]).squeeze(-1), dim=-1)[:, :, None]).squeeze(-1)
        w_m = self.g_w + 0.5 * dt * wdot(self.g_R, self.g_w)
        R_m = rot_from_rotvec(0.5 * dt * self.g_w + 1e-15) @ self.g_R
        self.g_w = self.g_w + dt * wdot(R_m, w_m)
        self.g_R = rot_from_rotvec(dt * w_m + 1e-15) @ self.g_R

    def _pin(self):
        if hasattr(self, "e_st"):
            if self.k_glob > 0:
                self._ghost_step()
            t = self.t_now
            ph = self.st_f * t + self.st_ph
            self.e_st = (self.st_A * torch.sin(ph)).sum(-1)
            de = (self.st_A * self.st_f * torch.cos(ph)).sum(-1)
            Rg, cg, w = self.g_R, self.g_c, self.g_w
            Rb = Rg @ self.st_R @ rot_from_rotvec(self.e_st[:, 3:] + 1e-12)
            po = cg - Rg @ self.r_com
            pb = po + (Rg @ self.st_p[:, :, None]).squeeze(-1) + (Rg @ self.e_st[:, :3, None]).squeeze(-1)
            self.bp, self.bR = pb, Rb
            self.qpos[:, self.cq:self.cq + 3] = pb.float()
            self.qpos[:, self.cq + 3:self.cq + 7] = mat_to_quat(Rb).float()
            vb = torch.cross(w, pb - cg, dim=-1) + (Rg @ de[:, :3, None]).squeeze(-1)
            self.qvel[:, self.cv:self.cv + 3] = vb.float()
            self.qvel[:, self.cv + 3:self.cv + 6] = ((Rb.transpose(1, 2) @ w[:, :, None]).squeeze(-1) + de[:, 3:]).float()
            if hasattr(self, "v_peak"):
                vc = self.com_vel()
                live = (self.phase >= 2) & (self.phase <= 4)          # descent, close, hold (not the load test)
                self.v_peak = torch.where(live, torch.maximum(self.v_peak, vc.norm(dim=-1) * 1e3), self.v_peak)
        else:
            super()._pin()

