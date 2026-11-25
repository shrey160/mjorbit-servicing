"""P7.6 (lean, user 22 Nov: "ring mission harness"): the ring capture as one episode on own sensing, from the 3 m hold
to a closed grip, with a free, spinning client and a free servicer flying on its rev-2 actuators.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/mission/ring_mission.py [--n 25] [--workers 5]

Not the P6 chain (its P3-P6 classes are bar-specific); the far leg, the 30 / 8 m holds and the synchronisation from
the 3 m stop are taken from P6's results (the hold errors below), not re-run. Per episode (seeds 7600000 + i):
  setup    client_p7 free (300 kg) at a random attitude, spinning 0-2 deg/s about a random body axis (D3; torque-free,
           so it nutates); servicer (rev 2, 196 kg, arm in the approach pose) at the 3 m hold on the ring axis,
           synchronised with the P6 hold errors (<= 50 mm, <= 2 deg, <= 3 mm/s, <= 0.1 deg/s); its roll about the
           ring axis puts the shoulder away from the solar wing. Bench mount error, lidar noise and the sun as
           reach_bench. Truth is used here (the hand-over from the P6 chain) and nowhere in the loop below.
  sense    bus_lidar at 1 Hz -> box fit (> 1.5 m) and ring fit, both from the filter's prediction (the first fix:
           from the hand-over prior, truth + 50 mm / 2 deg as ring_bench) -> client pose in the believed frame;
           RelNav: Gauss-Newton fit over a 20 s window of a constant-rate rotation and a constant-velocity centre of
           mass (design COM with a 10 mm error) -> predicted client pose, rate and velocity at any time.
  fly      body-fixed station on the estimated ring axis: hold 3 m (20 s) -> 20 mm/s to 1.8 m -> hold (15 s) ->
           to the grasp station (bus centre 1.0 m beyond the ring top) -> settle; translation PD with the rigid-motion
           feed-forward, through the 16 nozzles (allocation, 50 ms minimum on-time, scale / misalignment errors);
           attitude PD through the 4-wheel pyramid (0.2 N m, 12 N m s; thruster unloading at 80%). Bus own state
           (attitude, rate, inertial motion) is taken as known (star trackers, gyros, accelerometers).
  grasp    the P7.4 / P7.5 reach on the estimate (reach_bench: section nearest the shoulder clear of the wing,
           wrist-camera radial correction, pre-close, guarded descent, palm-following close, lead-screw lock); the
           grasp point is carried with the client's estimated motion (track()). Then the bus thrusters stop and the
           stack drifts 5 s (the post-capture phases are P6's, not re-run).
  aborts   station error > 0.15 m / 5 deg, no ring fit for 15 s before the reach, the reach's own aborts: the episode
           ends there (a back-off is the P6 chain's).
Scored with the truth only: grip and seat, contacts outside tool-on-ring (any is a collision), client push, plume
pulses (nozzles firing within 30 deg of the client inside 3 m; recorded, D7.7), propellant, wheel fill.
Writes runs/<out>.json.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR  # noqa: E402
import reach_bench as RB4  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_cam as RC  # noqa: E402
import ring_fit  # noqa: E402
import lidar_nav  # noqa: E402
import gen_ringcam  # noqa: E402
from actuation import Pulses, Rcs, Wheels, load_bus  # noqa: E402

CTRL_DT, LIDAR_DT = 0.1, 1.0
HOLD = dict(pos=0.05, deg=2.0, vel=0.003, rate_deg=0.1)       # P6 hold errors at the hand-over
S_HOLD3, S_HOLD18, S_STATION = 3.2, 2.0, RB4.STATION          # bus centre beyond the ring top, m
V_APP = 0.020
T_HOLD3, T_HOLD18, T_SETTLE = 20.0, 15.0, 30.0
F_MAX = 1.5                                                    # N per axis wanted (16 x 1 N, canted)
KP_T, KD_T = 0.3 ** 2, 2 * 0.9 * 0.3                           # translation (1/s^2, 1/s)
KP_A, KD_A = 0.5 ** 2, 2 * 0.9 * 0.5                           # attitude
UNLOAD_ON, UNLOAD_OFF, UNLOAD_K, UNLOAD_MAX = 0.8, 0.5, 0.05, 0.2
WIN = 20.0
BOX_MIN_M = 2.2           # box fits beyond this range only (closer they came 3 deg off and kicked the rate estimate)
COM_ERR = 0.010
I_ERR = 0.05                                                   # design inertia error (principal moments), as P6
ABORT_POS, ABORT_DEG, NOFIX_S, ABORT_S = 0.15, 5.0, 15.0, 5.0
NET, NET_SIG_MAX = [None], 0.003    # the ring net, and the radial sigma above which the reach aborts
STEREO = dict(act="in+out", tilt=False, nets={}, mot_m=0.0, frame="camera")   # s25: --cam stereo (nets/stereo_net.py): every
# net is run and scored at the pre-reach pose, the `act` one corrects the estimate (radial + tangential, + axis tilt
# with tilt=True); the motion view is taken after the arm moved mot_m radially out
USE_CAM = [False]          # the wrist camera's radial correction (--cam); off: in the mission (the section is always on
                           # the side away from the wing) it shifted the estimate 10-20 mm inboard, past the window
RB4.TRACK_MM = 12.0        # arm tracking monitor: the grasp point moves with the client here (lag 6-10 mm at 5 mm)
SETTLE_POS, SETTLE_DEG = 0.015, 1.0
# section 23, multi-rate sensing (defaults: the section 22 behaviour)
LAT = [0.15]               # s25.5: lidar fix latency (scan + point-cloud fit on flight hardware): a fix measured at t
LAT_J = [0.02]             # reaches the filter at t + LAT +- LAT_J, with its own time stamp t (the fit and the prediction
                           # propagate to now); LAT 0: delivered at once (before 23 Nov)
FIT_DT = [None]            # navigation refit interval, s (None: at every fix)
FIT_MAX = [0]              # at most this many fixes in a refit (0: all in the window)
INNOV = ["off"]            # innovation monitor: off / record / on
G_MM, G_DEG = [25.0], [3.0]   # gate on the ring-centre and ring-axis innovation: ~2x the largest normal innovation
                              # (section 23, record mode, 2 seeds at 10 Hz: max 13.3 mm, 1.83 deg)
RING_ROLL_LOCK = [True]    # s26 F1: ring-only refits do not change the roll about the ring axis (a ring cannot measure it)
SWEEP_MIN = [4.0]          # ... when the ring axis sweeps less than this over the fit window (deg): roll unobservable
                           # (tune seeds: 21 0.2, 3 1.4 deg oscillated; 0 6.2, 22 11.5 (10 deg: chattered, no grip), 11, 14 need it free)
SPIN_RATED, SPIN_MAX = 2.0, [2.5]   # s26 (user, 23 Nov): rated client spin 2 deg/s, 2.5 max. Thrust-limited: 1.5 N /
                                    # 196 kg = 0.0077 m/s^2 per axis; the 3.2 m hold is 3.98 m from the COM: w^2 r =
                                    # 63 % of it at 2, 99 % at 2.5 deg/s (worst spin axis). Physics and the numbers:
                                    # SPEC_7 s26.4, checks/spin_limit.py (rerun if the thrusters change)
ETA_SPIN = [0.85]          # s26.4 (user: gate on the acceleration): the share of the thrust authority the spin may use
SPIN = [0.0, 2.0]          # s26 F4: client spin range (deg/s), uniform; the tune / test distribution is 0-2
FLOG = [False]             # s26 F0: the flight log (estimate vs truth, target vs bus, torque, wheels, fixes), 2 Hz
FLIGHT_ONLY = [False]      # s26 F0: end the episode at the station (no reach)
ANOM_N = [5]               # rejected fixes in a row that make an anomaly


def _exp(w):
    """Rodrigues (as Rotation.from_rotvec(w).as_matrix(), to rounding; section 23: scipy's per-call cost dominated
    the refit's propagation)."""
    th = float(np.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]))
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * (K @ K)
    return np.eye(3) + (np.sin(th) / th) * K + ((1.0 - np.cos(th)) / (th * th)) * (K @ K)


def _cross(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def _log(R):
    return Rotation.from_matrix(R).as_rotvec()


class RelNav:
    """Client pose / rate / velocity from lidar fixes (believed frame). Torque-free attitude motion (Euler's
    equations with the design inertia, an estimate with error: the client nutates, so a fitted rate is carried
    through it; P6 step 3 did the same in its filter), constant COM velocity."""

    def __init__(self, com_b, I_b):
        self.com_b, self.I, self.Ii = com_b, I_b, np.linalg.inv(I_b)
        self.h = []                       # (t, R, p, kind)
        self.sol = None                   # (t_ref, R_ref, w_body_ref, c_ref, v)
        self.full = None                  # (t, w_body) of the last fit from box fixes only
        self.t_fit = -1e9

    def _wdot(self, wb):
        return self.Ii @ (-_cross(wb, self.I @ wb))

    def prop(self, R, wb, dt, h=0.25):
        """(R, w_body) after dt (either sign), RK4 on the rate, exponential map on the attitude."""
        n = max(1, int(np.ceil(abs(dt) / h)))
        s = dt / n
        for _ in range(n):
            k1 = self._wdot(wb)
            k2 = self._wdot(wb + 0.5 * s * k1)
            k3 = self._wdot(wb + 0.5 * s * k2)
            k4 = self._wdot(wb + s * k3)
            wn = wb + s / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            R = R @ _exp(0.5 * s * (wb + wn))
            wb = wn
        return R, wb

    def _prop_many(self, R, wb, dts):
        """Attitudes at the (non-positive, any order) offsets dts from (R, wb) at the reference time."""
        order = np.argsort(-np.asarray(dts))           # 0, then backwards
        out, t_cur, Rc, wc = {}, 0.0, R, wb
        for i in order:
            Rc, wc = self.prop(Rc, wc, dts[i] - t_cur)
            t_cur = dts[i]
            out[i] = Rc
        return [out[i] for i in range(len(dts))]

    def wb_at(self, t):
        """Body rate at t from the last box-fix fit (its own evolution needs no attitude)."""
        t0, wb = self.full
        return self.prop(np.eye(3), wb, t - t0)[1]

    def add(self, t, R, p, kind):
        self.h.append((t, R, p, kind))
        self.h = [x for x in self.h if x[0] > t - WIN]
        # section 23: the refit at its own rate (FIT_DT None: at every fix, as before); the prediction in between
        # propagates the last solution
        if self.sol is None or FIT_DT[0] is None or t - self.t_fit >= FIT_DT[0] - 1e-9:
            self._fit(t)
            self.t_fit = t

    def _sel(self):
        """The fixes the refit uses: all, or at most FIT_MAX spread evenly over the window (the newest always in)."""
        h, n = self.h, len(self.h)
        if not FIT_MAX[0] or n <= FIT_MAX[0]:
            return h
        return [h[i] for i in np.unique(np.round(np.linspace(0, n - 1, FIT_MAX[0])).astype(int))]

    def _fit(self, t):
        h = self._sel()
        t_ref, R_ref = h[-1][0], h[-1][1]
        if len(h) < 3:
            self.sol = (t_ref, R_ref, np.zeros(3), h[-1][2] + R_ref @ self.com_b, np.zeros(3))
            return
        dts = [ti - t_ref for ti, _, _, _ in h]
        full = all(k == "full" for _, _, _, k in h)     # the rate is fitted only from box-fix windows: in a
        # mixed window the small systematic box / ring difference was fitted as a rate (0.2 deg/s at the switch)

        def rot_res(wb, e0):
            Rs = self._prop_many(_exp(e0) @ R_ref, wb, dts)
            r = []
            for Rp, (_, Ri, _, kind) in zip(Rs, h):
                if kind == "full":
                    r.append(_log(Rp.T @ Ri))
                else:                     # ring fix: its axis only (its roll is the prediction's, not a measurement)
                    r.append(_cross(Rp[:, 0], Ri[:, 0]))
            return np.concatenate(r)
        def _fallback_sol():
            if self.sol is not None:
                return self.sol
            return (t_ref, R_ref, np.zeros(3), h[-1][2] + R_ref @ self.com_b, np.zeros(3))

        if full or self.full is None:
            w0 = np.clip(self.sol[2] if self.sol is not None else np.zeros(3), -0.45, 0.45)
            try:
                x = least_squares(
                    lambda x: rot_res(x[:3], x[3:]),
                    np.r_[w0, np.zeros(3)],
                    method="trf",
                    bounds=([-0.5, -0.5, -0.5, -1.0, -1.0, -1.0], [0.5, 0.5, 0.5, 1.0, 1.0, 1.0]),
                    loss="linear",
                ).x
            except Exception:
                self.sol = _fallback_sol()
                return
            if not np.all(np.isfinite(x)):
                self.sol = _fallback_sol()
                return
            self.full = (t_ref, x[:3].copy())
        else:
            # ring fixes in the window (close range): the rate is the last box-fix fit carried by Euler's
            # equations (a ring cannot show the roll about its axis: fitting the roll freely drifted 7 deg at 1.3
            # m; carrying the predicted roll as a measurement locked a 0.3 deg/s rate error in; a frozen rate
            # missed the nutation: 9 / 25 episodes did not settle), the attitude follows the measured axis, the
            # roll held weakly to the prediction
            wf = self.wb_at(t_ref)
            # the roll about the ring axis is observable through the dynamics only when the axis moves: a client
            # spinning across the axis sweeps it, and the axis history then carries the full attitude (the free fit
            # below uses that: 22 / 25 with the lock always on, seeds 11, 14, 22 at 1-1.75 deg/s lost it). Locked
            # when the axis sweeps less than SWEEP_MIN over the window (the estimated rate across the axis)
            n_r = R_ref[:, 0]
            w_w = R_ref @ wf
            sweep = float(np.degrees(np.linalg.norm(w_w - (w_w @ n_r) * n_r))) * max(1e-9, -min(dts))
            try:
                if RING_ROLL_LOCK[0] and sweep < SWEEP_MIN[0]:
                    # s26 F1 (23 Nov): the correction only across the ring axis. A free 3-axis correction held by the
                    # 0.05 weight let the window's axis noise pull the roll (unmeasured) 1-2 deg a refit once no box fix
                    # was left in the window; each refit started from the last, and the bus rolling after the target fed
                    # the lidar's view back: +-17 deg on seed 21, the station abort
                    n_r = R_ref[:, 0]
                    a_ = np.cross(n_r, [1.0, 0.0, 0.0] if abs(n_r[0]) < 0.9 else [0.0, 1.0, 0.0])
                    a_ /= np.linalg.norm(a_)
                    b_ = np.cross(n_r, a_)
                    ab = least_squares(lambda y: np.r_[rot_res(wf, y[0] * a_ + y[1] * b_), 0.05 * y], np.zeros(2),
                                       method="lm").x
                    e = ab[0] * a_ + ab[1] * b_
                else:
                    e = least_squares(lambda e: np.r_[rot_res(wf, e), 0.05 * e], np.zeros(3), method="lm").x
            except Exception:
                self.sol = _fallback_sol()
                return
            x = np.r_[wf, e]
        if not np.all(np.isfinite(x)):
            self.sol = _fallback_sol()
            return
        wb, Rt = x[:3], _exp(x[3:]) @ R_ref
        Rs = self._prop_many(Rt, wb, dts)
        T = np.array(dts)
        C = np.array([pi + Rp @ self.com_b for Rp, (_, _, pi, _) in zip(Rs, h)])
        A = np.c_[np.ones_like(T), T]
        if not (np.all(np.isfinite(A)) and np.all(np.isfinite(C))):
            self.sol = _fallback_sol()
            return
        try:
            coef, *_ = np.linalg.lstsq(A, C, rcond=None)
        except np.linalg.LinAlgError:
            self.sol = _fallback_sol()
            return
        if not np.all(np.isfinite(coef)):
            self.sol = _fallback_sol()
            return
        self.sol = (t_ref, Rt, wb, coef[0], coef[1])

    def predict(self, t):
        t_ref, Rt, wb, c, v = self.sol
        R, wbt = self.prop(Rt, wb, t - t_ref)
        cc = c + v * (t - t_ref)
        return R, cc - R @ self.com_b, R @ wbt, v, cc

    def age(self, t):
        return t - self.h[-1][0] if self.h else 1e9


class Mission(RB4.Bench):
    def __init__(self):
        super().__init__()
        km = self.km
        nm = lambda a: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""  # noqa: E731
        self.busd = load_bus()
        self.rcs = Rcs(self.busd)
        self.noz = np.array([next(a for a in range(km.nu) if nm(a) == "test_rcs_" + n[4:]) for n in self.rcs.names])
        self.rwm = np.array([next(a for a in range(km.nu) if nm(a) == f"test_rwm_{ax}") for ax in "xyz"])
        self.c_body = self.cli_b
        self.com_true = km.body_ipos[self.bid_t].copy()          # the client's COM in its body frame (design)

    # ---- episode setup (truth: the hand-over from the P6 chain) ------------------------------------------------
    def place_episode(self, seed):
        km, kd, rig = self.km, self.kd, self.rig
        rng = np.random.default_rng([seed, 76])
        km.cam_pos[:] = self.base["pos"]
        km.cam_quat[:] = self.base["quat"]
        mujoco.mj_resetData(km, kd)
        self._vff = np.zeros(6)                             # feed-forward filter state: per episode (s24.4)
        Rc = Rotation.random(random_state=rng.integers(2**31))
        spin = rng.uniform(SPIN[0], SPIN[1])
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        w_c = np.radians(spin) * Rc.apply(ax)                      # world
        # servicer: hold station in the client frame (turned 180 deg about z, rolled so the shoulder (-z) is away
        # from the wing (+y): servicer -z along client -y)
        self.R_rel = (Rotation.from_rotvec([-np.pi / 2, 0, 0]) * Rotation.from_quat([0, 0, 1, 0])).as_matrix()
        p_st = ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + S_HOLD3)
        dv, da = rng.normal(size=3), rng.normal(size=3)
        p_s = Rc.apply(p_st + HOLD["pos"] * rng.uniform(0, 1) * dv / np.linalg.norm(dv))
        R_s = Rotation.from_rotvec(np.radians(HOLD["deg"]) * rng.uniform(0, 1) * da / np.linalg.norm(da)) * Rc * Rotation.from_matrix(self.R_rel)
        com_w = Rc.apply(self.com_true)
        kd.qpos[self.tq:self.tq + 7] = [0.0, 0.0, 0.0, *RB._q_wxyz(Rc.as_matrix())]
        # the free joint's linear velocity is the body origin's: the client's COM at rest -> v_origin = -w x com_w
        kd.qvel[self.tv:self.tv + 3] = -np.cross(w_c, com_w)
        kd.qvel[self.tv + 3:self.tv + 6] = Rc.inv().apply(w_c)       # body-frame rate
        dvv = rng.normal(size=3)
        dw = rng.normal(size=3)
        v_s = np.cross(w_c, p_s - com_w) + HOLD["vel"] * rng.uniform(0, 1) * dvv / np.linalg.norm(dvv)
        w_s = w_c + np.radians(HOLD["rate_deg"]) * rng.uniform(0, 1) * dw / np.linalg.norm(dw)
        kd.qpos[self.cq:self.cq + 7] = [*p_s, *RB._q_wxyz(R_s.as_matrix())]
        kd.qvel[self.cv:self.cv + 3] = v_s
        kd.qvel[self.cv + 3:self.cv + 6] = R_s.inv().apply(w_s)
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
        rig._nrng = np.random.default_rng([seed, 76, 7])
        rig.gain, rig.noise = rng.uniform(0.75, 1.25), rng.uniform(0.0, 3.0)
        RC.sun(self, rng)
        mujoco.mj_forward(km, kd)
        self.rng = rng
        # own design data with an error: the client's COM
        de = rng.normal(size=3)
        q = km.body_iquat[self.bid_t]
        Rq = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        I_est = Rq @ np.diag(km.body_inertia[self.bid_t] * (1 + rng.uniform(-I_ERR, I_ERR, 3))) @ Rq.T
        self.nav = RelNav(self.com_true + COM_ERR * de / np.linalg.norm(de), I_est)
        # the hand-over prior for the first fix (P6 close-range error), in the believed lidar frame
        (pn, Rn), _ = self.frames(self.lid)
        dv, da = rng.normal(size=3), rng.normal(size=3)
        self.prior0 = (Rotation.from_rotvec(np.radians(2.0) * da / np.linalg.norm(da) * rng.uniform(0.3, 1)).as_matrix() @ Rn.T @ Rc.as_matrix(),
                       Rn.T @ (kd.xpos[self.bid_t] - pn) + 0.05 * dv / np.linalg.norm(dv) * rng.uniform(0.3, 1))
        # actuators and the bus's own state
        self.pulses = Pulses(self.rcs, seed=seed)
        self.wheels = Wheels(self.busd)
        self.unloading, self.u, self.t_ctrl, self.t_lidar = False, np.zeros(self.rcs.n), -1.0, 0.0
        self.thr_on, self.s, self.sdot = True, S_HOLD3, 0.0
        self.F_cmd, self.T_cmd, self.tau_thr = np.zeros(3), np.zeros(3), np.zeros(3)
        self.plume, self.fixes, self.fix_fail = 0, 0, 0
        self.last_fix_ok = 0.0
        self.pend, self.lat_rng = [], np.random.default_rng([seed, 77])
        self.flog, self.fk, self.n_law, self.fdiag = [], {}, 0, []
        self.t_bad = None
        self.err = (0.0, 0.0)
        self.innov, self.n_rej_run, self.n_rej, self.anomaly = [], 0, 0, None
        return dict(spin_deg_s=round(spin, 2), spin_axis_b=np.round(ax, 3).tolist())

    # ---- own state ---------------------------------------------------------------------------------------------
    def bus_state(self):
        kd = self.kd
        R = kd.xmat[self.bid_c].reshape(3, 3).copy()
        com = kd.subtree_com[self.bid_c].copy()
        v = kd.qvel[self.cv:self.cv + 3].copy()                    # origin velocity (world)
        wb = kd.qvel[self.cv + 3:self.cv + 6].copy()               # body rate
        return kd.xpos[self.bid_c].copy(), R, v, wb, com

    def bus_mass_inertia(self):
        km, kd = self.km, self.kd
        M = np.zeros((km.nv, km.nv))
        mujoco.mj_fullM(km, M, kd.qM)
        return float(km.body_subtreemass[self.bid_c]), M[self.cv + 3:self.cv + 6, self.cv + 3:self.cv + 6].copy()

    # ---- sensing -----------------------------------------------------------------------------------------------
    def fix(self):
        """One lidar fix -> nav.add; returns the kind ('full', 'ring', or None)."""
        kd = self.kd
        t = kd.time
        P, _ = self.lidar_points()
        (pn, Rn), _ = self.frames(self.lid)
        if self.nav.sol is None:
            R0, t0 = self.prior0
        else:
            Rw, pw, _, _, _ = self.nav.predict(t)
            R0, t0 = Rn.T @ Rw, Rn.T @ (pw - pn)
        S = float(np.linalg.norm(t0 + R0 @ ring_fit.C_B))
        kind = None
        Rb_, tb_ = R0, t0
        if S > BOX_MIN_M:
            b = lidar_nav.fit(P, R0, t0)
            if b["ok"]:
                Rb_, tb_, kind = b["R"], b["t"], "full"
        r = ring_fit.fit(P, Rb_, tb_)
        if not r["ok"] and kind == "full":
            r = ring_fit.fit(P, R0, t0)
            if r["ok"]:
                Rb_, tb_, kind = R0, t0, None
        self.fixes += 1
        if r["ok"]:
            # align the pose's axis with the ring's, put its ring centre on the fitted one
            n_b = Rb_ @ ring_fit.N_B
            v = np.cross(n_b, r["n"])
            s_, c_ = np.linalg.norm(v), float(n_b @ r["n"])
            Ra = _exp(v / s_ * np.arctan2(s_, c_)) if s_ > 1e-12 else np.eye(3)
            Rm = Ra @ Rb_
            tm = r["c"] - Rm @ ring_fit.C_B
            if FLOG[0]:                       # s26 F1: the roll at each step of a ring fix (vs the truth at t)
                self.fdiag.append(dict(t=round(t, 2), kind=kind or "ring", prior=self._roll(Rn @ R0), box=self._roll(Rn @ Rb_),
                                       fix=self._roll(Rn @ Rm), ra_deg=round(float(np.degrees(np.linalg.norm(_log(Ra)))), 3),
                                       bus_roll=self._roll(self.kd.xmat[self.bid_c].reshape(3, 3) @ self.R_rel.T)))
            self._queue(t, (pn + Rn @ r["c"], Rn @ r["n"]), (Rn @ Rm, pn + Rn @ tm, kind or "ring"))
            return kind or "ring"
        if kind == "full":
            self._queue(t, None, (Rn @ Rb_, pn + Rn @ tb_, "full"))
            return "full"
        self.fix_fail += 1
        return None

    def _queue(self, t, innov, fix):
        """s25.5: a fix measured at t reaches the filter after the latency (with its time stamp t)."""
        lat = LAT[0] + (self.lat_rng.uniform(-LAT_J[0], LAT_J[0]) if LAT[0] > 0 else 0.0)
        self.pend.append((t + lat, t, innov, fix))
        self._deliver(t)

    def _deliver(self, now):
        while self.pend and self.pend[0][0] <= now + 1e-9:
            _, t, innov, (R, p, kind) = self.pend.pop(0)
            if innov is not None and self.innovation(t, *innov):
                self.fk["rejected"] = self.fk.get("rejected", 0) + 1
                continue
            pre = self._roll(self.nav.predict(t)[0]) if FLOG[0] and self.nav.sol is not None else None
            self.nav.add(t, R, p, kind)
            self.last_fix_ok = t
            if FLOG[0] and self.fdiag and abs(self.fdiag[-1]["t"] - t) < 0.006:
                self.fdiag[-1].update(pre=pre, post=self._roll(self.nav.predict(t)[0]),
                                      now=self._roll(self.nav.predict(now)[0]))
            self.fk[kind] = self.fk.get(kind, 0) + 1

    def innovation(self, t, c_meas, n_meas):
        """Section 23: the fix's ring centre / axis against the prediction (own data only). Records it; in 'on' mode
        returns True (reject the fix) above the gate, and after ANOM_N rejections in a row raises the anomaly: the arm
        stops through the contact-guard trip while the guard is armed (pre-close move, descent), the flight aborts
        (fly), the pre-reach move aborts after it; after the palm touch and the lock it is recorded only."""
        if INNOV[0] == "off" or self.nav.sol is None or len(self.nav.h) < 3 or t < 5.0:
            return False
        Rw, pw, _, _, _ = self.nav.predict(t)
        e_mm = float(np.linalg.norm(c_meas - (pw + Rw @ ring_fit.C_B))) * 1e3
        e_deg = float(np.degrees(np.arccos(np.clip(abs(float(n_meas @ (Rw @ ring_fit.N_B))), -1.0, 1.0))))
        rej = INNOV[0] == "on" and (e_mm > G_MM[0] or e_deg > G_DEG[0])
        self.innov.append((round(t, 2), self.phase, round(e_mm, 2), round(e_deg, 3), rej))
        if not rej:
            self.n_rej_run = 0
            return False
        self.n_rej_run += 1
        self.n_rej += 1
        if self.n_rej_run >= ANOM_N[0] and self.anomaly is None:
            self.anomaly = dict(t=round(t, 2), phase=self.phase, mm=round(e_mm, 1), deg=round(e_deg, 2))
            if self.guard is not None and self.trip is None:
                self.trip = "nav anomaly"
        return True

    # ---- bus control (every physics step: actuators; every CTRL_DT: the law) ---------------------------------------
    def _before_step(self):
        km, kd = self.km, self.kd
        t = kd.time
        if self.pend:
            self._deliver(t)
        if t + 1e-9 >= self.t_lidar:
            self.t_lidar = t + LIDAR_DT
            self.fix()
        if t + 1e-9 >= self.t_ctrl:
            self.t_ctrl = t + CTRL_DT
            self._law(t)
        f = self.pulses.step(self.u if self.thr_on else np.zeros(self.rcs.n), km.opt.timestep)
        kd.ctrl[self.noz] = f
        p, R, _, wb, _ = self.bus_state()
        dF = f[:, None] * (self.pulses.D_true - self.rcs.D)
        dT = np.cross(self.rcs.P, dF).sum(0)
        kd.xfrc_applied[self.bid_c, :3] = R @ dF.sum(0)
        tau, _ = self.wheels.command(self.T_cmd - self.tau_thr if self.thr_on else self.T_cmd, wb, km.opt.timestep)
        kd.ctrl[self.rwm] = tau + dT
        if self.thr_on and f.any() and self.nav.sol is not None:     # plume record (D7.7): within 30 deg, inside 3 m
            _, _, _, _, cc = self.nav.predict(t)
            for i in np.flatnonzero(f):
                pz = p + R @ self.rcs.P[i]
                d = cc - pz
                if np.linalg.norm(d) < 3.0 and float(-(R @ self.rcs.D[i]) @ d) / np.linalg.norm(d) > np.cos(np.radians(30)):
                    self.plume += 1

    def _after_step(self):
        pass

    def station(self, t):
        """Desired bus origin pose, velocity, rate and the feed-forward acceleration from the estimate."""
        Rw, pw, w, v, cc = self.nav.predict(t)
        r_b = ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + self.s)
        p_d = pw + Rw @ r_b
        rr = p_d - cc
        v_d = v + np.cross(w, rr) + Rw @ ring_fit.N_B * self.sdot
        a_ff = np.cross(w, np.cross(w, rr))
        return p_d, Rw @ self.R_rel, v_d, w, a_ff

    def spin_gate(self, row):
        """s26.4: can the bus hold the station line on this client? From the estimate (w, COM, ring axis): the worst
        over the approach (s from S_HOLD3 to S_STATION) of the centripetal |w x (w x rr)| + the Coriolis of the
        approach 2 V_APP |w x n|, against ETA_SPIN x the force per axis / the bus mass; and the rate cap SPIN_MAX."""
        t = self.kd.time
        Rw, pw, w, _, cc = self.nav.predict(t)
        n = Rw @ ring_fit.N_B
        m, _ = self.bus_mass_inertia()
        a_lim = ETA_SPIN[0] * F_MAX / m
        a_need = 0.0
        for s_ in np.linspace(S_STATION, S_HOLD3, 12):
            rr = pw + Rw @ (ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + s_)) - cc
            a_need = max(a_need, float(np.linalg.norm(np.cross(w, np.cross(w, rr)))
                                       + 2 * V_APP * np.linalg.norm(np.cross(w, n))))
        w_d = float(np.degrees(np.linalg.norm(w)))
        row["spin_gate"] = dict(w_deg_s=round(w_d, 3), a_need=round(a_need, 5), a_lim=round(a_lim, 5))
        if w_d > SPIN_MAX[0]:
            return f"spin beyond rating ({w_d:.2f} deg/s > {SPIN_MAX[0]:g})"
        if a_need > a_lim:
            return f"spin beyond thrust ({a_need * 1e3:.2f} > {a_lim * 1e3:.2f} mm/s^2 at {w_d:.2f} deg/s)"
        return None

    def _law(self, t):
        if self.nav.sol is None:
            self.u, self.T_cmd, self.tau_thr = np.zeros(self.rcs.n), np.zeros(3), np.zeros(3)
            return
        p, R, v, wb, com = self.bus_state()
        p_d, R_d, v_d, w_d, a_ff = self.station(t)
        m, I = self.bus_mass_inertia()
        e_p, e_v = p_d - p, v_d - v
        self.err = (float(np.linalg.norm(e_p)), float(np.degrees(np.linalg.norm(_log(R.T @ R_d)))))
        F = np.clip(m * (a_ff + KP_T * e_p + KD_T * e_v), -F_MAX, F_MAX)
        e_R = _log(R.T @ R_d)
        w_db = R.T @ w_d
        self.T_cmd = I @ (KP_A * e_R + KD_A * (w_db - wb)) + np.cross(wb, I @ wb)
        fill = self.wheels.fill()
        self.unloading = fill > UNLOAD_ON or (self.unloading and fill > UNLOAD_OFF)
        self.tau_thr = np.clip(-UNLOAD_K * self.wheels.H(), -UNLOAD_MAX, UNLOAD_MAX) if self.unloading else np.zeros(3)
        w = np.r_[R.T @ F, self.tau_thr]
        ref = R.T @ (com - p)
        self.u = self.rcs.allocate(w, ref) if np.any(np.abs(w) > 1e-9) else np.zeros(self.rcs.n)
        self.n_law += 1
        if FLOG[0] and self.n_law % 5 == 0:
            self._flog(t, p, R, v, wb, p_d, R_d, F)

    def _roll(self, R):
        """s26 F1 (diagnostics): roll of a world attitude about the true ring axis vs the true client attitude now
        (the client turns ~0.01 deg/s on the seeds this is for: the 0.15 s latency is negligible)."""
        Rt = self.kd.xmat[self.bid_t].reshape(3, 3)
        return round(float(np.degrees(_log(R @ Rt.T) @ (Rt @ ring_fit.N_B))), 3)

    def _flog(self, t, p, R, v, wb, p_d, R_d, F):
        """s26 F0 (diagnostics; reads the truth): the estimate vs the truth, split along the true ring axis (roll) and
        across it (tilt); the bus vs its target, split along the estimated ring axis; torque, wheels, fixes."""
        kd = self.kd
        Rt = kd.xmat[self.bid_t].reshape(3, 3).copy()
        pt = kd.xpos[self.bid_t].copy()
        wt = Rt @ kd.qvel[self.tv + 3:self.tv + 6]
        Rw, pw, w, vv, cc = self.nav.predict(t)
        n_t, n_e = Rt @ ring_fit.N_B, Rw @ ring_fit.N_B
        dE = _log(Rw @ Rt.T)                                   # estimate attitude error (world)
        roll_e = float(dE @ n_t)
        dT = R @ _log(R.T @ R_d)                               # tracking error (world)
        roll_t = float(dT @ n_e)
        dw = w - wt
        last = self.nav.h[-1] if self.nav.h else None
        H = self.wheels.H()
        r = lambda x, k=3: round(float(x), k)  # noqa: E731
        self.flog.append(dict(
            t=r(t, 1), ph=self.phase, s=r(self.s),
            est=dict(roll_deg=r(np.degrees(roll_e)), tilt_deg=r(np.degrees(np.linalg.norm(dE - roll_e * n_t))),
                     ring_mm=r(np.linalg.norm((pw + Rw @ ring_fit.C_B) - (pt + Rt @ ring_fit.C_B)) * 1e3, 2),
                     w_roll_dps=r(np.degrees(dw @ n_t), 4), w_tilt_dps=r(np.degrees(np.linalg.norm(dw - (dw @ n_t) * n_t)), 4),
                     w_true_dps=r(np.degrees(np.linalg.norm(wt)), 4)),
            trk=dict(roll_deg=r(np.degrees(roll_t)), tilt_deg=r(np.degrees(np.linalg.norm(dT - roll_t * n_e))),
                     pos_mm=r(np.linalg.norm(p_d - p) * 1e3, 1)),
            T_cmd=r(np.linalg.norm(self.T_cmd), 4), F=r(np.linalg.norm(F), 3), F_sat=bool(np.any(np.abs(F) >= F_MAX - 1e-9)),
            wheel=dict(fill=r(self.wheels.fill()), H=r(np.linalg.norm(H), 4), unload=bool(self.unloading)),
            fix=dict(kinds=dict(self.fk), last=last[3] if last else None, age=r(t - last[0], 2) if last else None)))
        self.fk = {}

    # ---- the reach: the grasp point carried with the client's estimated motion ------------------------------------
    def to_body(self, E, rad):
        Rw, pw, _, _, _ = self.nav.predict(self.kd.time)
        return (Rw.T @ (E[0] - pw), Rw.T @ E[1], Rw.T @ rad)

    def track(self, E, rad):
        if getattr(self, "Eb", None) is None:
            return E, rad
        Rw, pw, _, _, _ = self.nav.predict(self.kd.time)
        c, n, r = self.Eb
        return (pw + Rw @ c, Rw @ n), Rw @ r

    def point_vel(self, p):
        """The estimated velocity of a world point fixed to the client: v + w x (p - com) from the nav state
        (s25.4: the descent catch-up's feed-forward)."""
        if getattr(self, "Eb", None) is None or self.nav.sol is None:
            return np.zeros(3)
        _, _, w, v, cc = self.nav.predict(self.kd.time)
        return v + np.cross(w, p - cc)

    # ---- the flight -------------------------------------------------------------------------------------------
    def fly(self, T, s_to=None):
        """Fly for T s (s ramps toward s_to at V_APP); returns an abort reason or None."""
        dt = self.km.opt.timestep
        n = int(T / dt)
        q = self.kd.qpos[self.arm].copy()
        for k in range(n):
            if s_to is not None:
                ds = s_to - self.s
                self.sdot = float(np.clip(ds / 0.5, -V_APP, V_APP)) if abs(ds) > 1e-4 else 0.0
                self.s += self.sdot * dt
            self.step(q, self.q_open)
            if k % 50 == 0 and self.nav.sol is not None and self.kd.time > 5.0:
                # a station error that lasts ABORT_S (an estimate update can step the target for a moment)
                self.t_bad = (self.t_bad or self.kd.time) if (self.err[0] > ABORT_POS or self.err[1] > ABORT_DEG) else None
                if self.t_bad is not None and self.kd.time - self.t_bad > ABORT_S:
                    return f"station error ({self.err[0] * 1e3:.0f} mm, {self.err[1]:.1f} deg)"
                if self.nav.age(self.kd.time) > NOFIX_S:
                    return "no fix"
            if self.anomaly is not None:
                return "nav anomaly"
            if self.bad:
                return "collision"
        self.sdot = 0.0
        return None


def truth_ring(M):
    kd = M.kd
    Rc = kd.xmat[M.bid_t].reshape(3, 3)
    return kd.xpos[M.bid_t] + Rc @ ring_fit.C_B, Rc @ ring_fit.N_B


def episode(M, seed):
    t0 = time.time()
    M.bad, M.phase, M.Eb = set(), "hold3", None
    M.arm_guard(False)
    M.peak = None
    row = dict(seed=seed, **M.place_episode(seed))
    row["phases"] = {}

    def done(res):
        row["result"] = res
        row["contacts_bad"] = sorted(M.bad)
        row["plume_pulses"] = M.plume
        row["prop_g"] = round(M.pulses.prop_kg * 1e3, 1)
        row["wheel_fill_max"] = round(M.wheels.fill(), 2)
        row["fixes"] = dict(n=M.fixes, failed=M.fix_fail, rejected=M.n_rej)
        if M.innov:                      # section 23: innovation statistics (flight phases / the reach)
            st = {}
            for g, phs in (("flight", ("hold3", "to1.8", "hold1.8", "to_station", "settle")), ("reach", None)):
                x = np.array([(e[2], e[3]) for e in M.innov if (e[1] in phs if phs else e[1] not in
                                                                   ("hold3", "to1.8", "hold1.8", "to_station", "settle"))])
                if len(x):
                    st[g] = dict(n=len(x), mm_p50=round(float(np.median(x[:, 0])), 2),
                                 mm_p99=round(float(np.percentile(x[:, 0], 99)), 2), mm_max=round(float(x[:, 0].max()), 2),
                                 deg_p99=round(float(np.percentile(x[:, 1], 99)), 3), deg_max=round(float(x[:, 1].max()), 3))
            row["innov"] = st
        row["anomaly"] = M.anomaly
        row["client_v_mm_s"] = round(float(np.linalg.norm(M.kd.qvel[M.tv:M.tv + 3] + np.cross(
            M.kd.xmat[M.bid_t].reshape(3, 3) @ M.kd.qvel[M.tv + 3:M.tv + 6], M.kd.xmat[M.bid_t].reshape(3, 3) @ M.com_true))) * 1e3, 2)
        row["sim_s"] = round(M.kd.time, 1)
        if FLOG[0]:
            row["flog"] = M.flog
            row["fdiag"] = M.fdiag
        row["wall_s"] = round(time.time() - t0, 1)
        return row

    for ph, T, s_to in (("hold3", T_HOLD3, None), ("to1.8", abs(S_HOLD3 - S_HOLD18) / V_APP + 5, S_HOLD18),
                        ("hold1.8", T_HOLD18, None), ("to_station", abs(S_HOLD18 - S_STATION) / V_APP + 5, S_STATION)):
        M.phase = ph
        why = M.fly(T, s_to)
        row["phases"][ph] = dict(t=round(M.kd.time, 1), err_mm=round(M.err[0] * 1e3, 1), err_deg=round(M.err[1], 2))
        if why:
            return done(f"abort: {ph}: {why}")
        if ph == "hold3" and M.nav.sol is not None:    # the box fixes' state: beyond the rating, no approach (safe)
            why = M.spin_gate(row)
            if why:
                return done(f"abort: hold3: {why}")
    # settle at the station
    M.phase = "settle"
    dt = M.km.opt.timestep
    t_end = M.kd.time + T_SETTLE
    while M.kd.time < t_end:
        why = M.fly(1.0)
        if why:
            return done(f"abort: settle: {why}")
        if M.err[0] < SETTLE_POS and M.err[1] < SETTLE_DEG:
            break
    row["phases"]["settle"] = dict(t=round(M.kd.time, 1), err_mm=round(M.err[0] * 1e3, 1), err_deg=round(M.err[1], 2))
    if M.err[0] > 2 * SETTLE_POS or M.err[1] > 2 * SETTLE_DEG:
        return done("abort: settle: not settled")
    if FLIGHT_ONLY[0]:
        return done("flight done")
    # the estimate at the reach start; scoring of it against the truth
    Rw, pw, w, v, _ = M.nav.predict(M.kd.time)
    E0 = (pw + Rw @ ring_fit.C_B, Rw @ ring_fit.N_B)
    row["E0"] = RB4.est_err(E0, truth_ring(M))
    row["w_est_err_deg_s"] = round(float(np.degrees(np.linalg.norm(
        w - M.kd.xmat[M.bid_t].reshape(3, 3) @ M.kd.qvel[M.tv + 3:M.tv + 6]))), 3)
    wing = Rw[:, 1]
    a, rad, _ = M.choose(E0, wing)
    if a is None:
        return done("abort: no reachable section")
    M.Eb = M.to_body(E0, rad)
    M.phase = "to pre-reach"
    E0, rad = M.track(E0, rad)
    if M.cart(*M.section(E0, rad, RB4.H_PRE), M.q_open) is None:
        return done("abort: path" if not M.bad else "collision")
    if M.anomaly is not None:
        return done("abort: to pre-reach (nav anomaly)")
    E0, rad = M.track(E0, rad)
    if USE_CAM[0] == "net":                   # the wrist-camera ring net (nets/ringcam_net.py)
        img = gen_ringcam.render(M)
        d, sg = NET[0].predict(img)
        row["cam"] = dict(net_mm=np.round(d * 1e3, 1).tolist(), sigma_mm=np.round(sg * 1e3, 2).tolist())
        if sg[0] > NET_SIG_MAX:
            return done("abort: ring net uncertain")
        dc = float(d[0])
        # R0 (22 Nov): the net's tangential output matched the true tangential error to ~2 mm on all 24 tune seeds
        # that reached the reach (the shift search's tangential, section 17, did not); an uncorrected 21 mm turns
        # the section's radial 3.9 deg and skews the jaws (seed 6: inboard pad 3.4 N after the drift, lost grip).
        # Its axial output carries a -7 +- 3 mm offset in the mission (unused; the descent depth covers it)
        dt_ = float(d[1])
    elif USE_CAM[0] == "stereo":
        why = stereo_check(M, E0, rad, row)
        if why:
            return done(why)
        E0, rad = M.track(E0, rad)            # the corrected estimate (M.Eb), now
        dc, dt_ = 0.0, 0.0
    elif USE_CAM[0]:
        o, img, cam, f = M.wrist_check(E0)
        row["cam"] = {k: v_ for k, v_ in o.items() if k != "c"}
        if o["verdict"] not in ("agree", "disagree"):
            return done(f"abort: wrist check ({o['verdict']})")
        dc, dt_ = float((o["c"] - E0[0]) @ rad), 0.0
    else:
        dc, dt_ = 0.0, 0.0
    if np.hypot(dc, dt_) * 1e3 > RB4.CAM_MAX_MM:
        return done("abort: camera and lidar disagree")
    E1 = (E0[0] + dc * rad + dt_ * np.cross(E0[1], rad), E0[1])
    M.Eb = M.to_body(E1, rad)
    row["E1"] = RB4.est_err(E1, truth_ring(M))
    ct = truth_ring(M)[0]
    row["rad_err_mm"] = dict(E0=round(float((E0[0] - ct) @ rad) * 1e3, 1), E1=round(float((E1[0] - ct) @ rad) * 1e3, 1))
    u1, v1 = ring_fit._frame(E1[1])
    a1 = float(np.arctan2(rad @ v1, rad @ u1))          # the same section in reach_close's frame convention
    r = RB4.reach_close(M, dict(), E1, a1, hold_check=False)
    for k in ("seat_pre", "descent", "closed", "peak", "close_s"):
        if k in r:
            row[k] = r[k]
    row["close_scored"] = r["result"]                              # truth-scored (seat), not used to act
    if r["result"].startswith(("abort", "collision")):
        return done(r["result"])
    t = M.touches()                                                # the grip decision on the touch pads only
    if not (t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and (t["palm"] > 5 or t["lip"] > 5)):
        return done("no grip (sensed)")
    # captured: thrusters off, the stack drifts 5 s on the locked jaws and the held arm
    M.phase = "drift"
    M.thr_on = False
    q = M.kd.qpos[M.arm].copy()
    s0 = M.seat()
    M.step(q, np.array([-M.sq, M.sq]), int(5.0 / dt))
    s1 = M.seat()
    t = M.touches()
    row["drift"] = dict(seat_change_mm=round(abs(s1["rad_mm"] - s0["rad_mm"]) + abs(s1["h_mm"] - s0["h_mm"]), 2),
                        touch={k: round(v_, 1) for k, v_ in t.items()})
    ok = t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and not M.bad
    if M.bad:
        return done("collision")
    if not ok:
        return done("lost grip")
    return done("captured" if row["close_scored"] == "grip" else "captured, off seat")


def _shot(M, cam, lamp):
    km = M.km
    km.light_active[M.lamp] = 1
    km.light_diffuse[M.lamp] = lamp
    M.wr.update_scene(M.kd, camera=cam, scene_option=M.rig.opt)
    img = M.wr.render().astype(np.float32) * M.rig.gain
    km.light_active[M.lamp] = 0
    img = np.clip(img + M.rig._nrng.normal(0, M.rig.noise, img.shape), 0, 255)
    return img.mean(-1).astype(np.uint8)


def stereo_check(M, E0, rad, row):
    """s25: the stereo module's two shots at the pre-reach pose, then the inboard camera again MOT_M radially out (motion
    stereo); every stereo net predicts (centre offset in the section frame, axis tilt), each is scored against the
    truth at the first shot (logging only), and the STEREO["act"] net's correction is written into M.Eb (the body
    frame at the first shot: the client moves on while the arm makes the motion shot). Returns an abort reason."""
    if not hasattr(M, "wcam2"):
        M.wcam2 = mujoco.mj_name2id(M.km, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam2")
    rad0 = rad
    lamp = float(M.rng.uniform(0.5, 1.0))
    ims = dict(**{"in": _shot(M, M.wcam, lamp)}, out=_shot(M, M.wcam2, lamp))
    Rw1, pw1 = M.nav.predict(M.kd.time)[:2]      # the client pose the servicer believes at the first shot
    # the nets were trained with the camera exactly at the pre-reach pose on the estimate; in the mission the arm lags
    # the moving estimate (6-10 mm, the tracking monitor), so the section the camera ACTUALLY looks from is rebuilt
    # from the grip site's pose (arm kinematics: known) and the nets' outputs are taken relative to it (23 Nov: with
    # the estimate's frame instead, mono and stereo both read -3.6 mm radial / -7 mm axial on seed 0, the ring net's
    # old axial offset)
    if not hasattr(M, "gsite"):
        M.gsite = mujoco.mj_name2id(M.km, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
    Rs = M.kd.site_xmat[M.gsite].reshape(3, 3)
    n_a, rad_a = -Rs[:, 0], Rs[:, 2]
    c_a = M.kd.site_xpos[M.gsite] - (ring_fit.H_TOP + RB4.H_PRE) * n_a - RB4.RING_R * rad_a
    d_a = c_a - E0[0]
    lag = [d_a @ rad, d_a @ np.cross(E0[1], rad), d_a @ E0[1], float(np.degrees(np.arccos(np.clip(n_a @ E0[1], -1, 1))))]
    if STEREO["frame"] == "camera":
        c, n, rad = c_a, n_a, rad_a
    else:
        c, n = E0
    tan = np.cross(n, rad)
    ct, nt = truth_ring(M)
    dd = ct - c
    lab = np.array([dd @ rad, dd @ tan, dd @ n, nt @ rad, nt @ tan])
    if STEREO["mot_m"] > 0:
        p, Rm = M.section(E0, rad0, RB4.H_PRE)
        if M.cart(p + STEREO["mot_m"] * rad0, Rm, M.q_open) is None:
            return "abort: path" if not M.bad else "collision"
        ims["mot"] = _shot(M, M.wcam, lamp)
    unit = np.array([1e3, 1e3, 1e3, np.degrees(1), np.degrees(1)])
    row["stereo"] = dict(label=np.round(lab * unit, 2).tolist(), frame=STEREO["frame"],
                         arm_lag_mm_deg=np.round(np.array(lag) * [1e3, 1e3, 1e3, 1], 2).tolist())
    pred = {}
    for v, net in STEREO["nets"].items():
        if not all(k in ims for k in v.split("+")):
            continue
        y, sg = net.predict(np.stack([ims[k] for k in v.split("+")]))
        pred[v] = (y, sg)
        row["stereo"][v] = dict(err=np.round((y - lab) * unit, 2).tolist(), sig=np.round(sg * unit, 2).tolist())
    y, sg = pred[STEREO["act"]]
    if sg[0] > NET_SIG_MAX:
        return "abort: stereo net uncertain"
    if np.hypot(y[0], y[1]) * 1e3 > RB4.CAM_MAX_MM:
        return "abort: camera and lidar disagree"
    c1 = c + y[0] * rad + y[1] * tan
    c1 = c1 + ((E0[0] - c1) @ E0[1]) * E0[1]        # radial + tangential only: the axial stays the estimate's (the
                                                    # guarded descent finds the top)
    n1 = n + y[3] * rad + y[4] * tan if STEREO["tilt"] else E0[1]
    n1 = n1 / np.linalg.norm(n1)
    M.Eb = (Rw1.T @ (c1 - pw1), Rw1.T @ n1, Rw1.T @ rad0)
    return None


def _job(seed):
    global _M
    if "_M" not in globals():
        _M = Mission()
        if USE_CAM[0] == "stereo" and not STEREO["nets"]:
            import torch
            torch.set_num_threads(1)
            from stereo_net import StereoNet
            for v in ("in", "in+out", "in+mot"):
                STEREO["nets"][v] = StereoNet.load(v)
        if USE_CAM[0] == "net" and NET[0] is None:   # load in the worker: torch threads hang across a fork
            import torch
            torch.set_num_threads(1)
            from ringcam_net import RingCamNet
            NET[0] = RingCamNet.load()
    try:
        r = episode(_M, seed)
    except Exception as e:  # noqa: BLE001
        r = dict(seed=seed, result=f"error: {e!r}")
    print(json.dumps(r, default=float), flush=True)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--seeds", type=int, nargs="*")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--stereo-act", default="in+out", choices=["in", "in+out", "in+mot"],
                    help="--cam stereo: the net whose correction is used (all are scored)")
    ap.add_argument("--stereo-tilt", action="store_true", help="--cam stereo: also correct the axis tilt")
    ap.add_argument("--stereo-mot", type=float, default=0.0, help="--cam stereo: motion-stereo move (m; 0: none; s25: 0.04 m tripped the wrist F/T at the descent start on 13 / 25 seeds)")
    ap.add_argument("--stereo-frame", choices=["camera", "estimate"], default="camera",
                    help="--cam stereo: the nets' outputs relative to the section the camera looks from, or the estimate")
    ap.add_argument("--cam", nargs="?", const="shift", default="", choices=["", "shift", "net", "stereo"],
                    help="the wrist camera's radial correction: shift search, or the ring net")
    ap.add_argument("--lidar-hz", type=float, default=1.0,
                    help="bus lidar fix rate (22 Nov, user: sensors up to 10 Hz, for experiment)")
    ap.add_argument("--fit-hz", type=float, default=0.0, help="navigation refit rate (0: at every fix; section 23)")
    ap.add_argument("--fit-max", type=int, default=0, help="at most this many fixes in a refit (0: all)")
    ap.add_argument("--innov", choices=["off", "record", "on"], default="off", help="innovation monitor (section 23)")
    ap.add_argument("--g-mm", type=float, default=0.0, help="innovation gate, ring centre (mm)")
    ap.add_argument("--g-deg", type=float, default=0.0, help="innovation gate, ring axis (deg)")
    ap.add_argument("--no-ff", action="store_true", help="no joint velocity feed-forward in the reach (before s24.4)")
    ap.add_argument("--spin", type=float, nargs=2, default=None, help="s26 F4: client spin range, deg/s (default 0 2)")
    ap.add_argument("--lat", type=float, default=None, help="s26 F4: lidar fix latency, s (default 0.15)")
    ap.add_argument("--i-err", type=float, default=None, help="s26 F4: design inertia error of the nav (default 0.05)")
    ap.add_argument("--no-roll-lock", action="store_true", help="s26 F1: ring-only refits may change the roll (before 23 Nov)")
    ap.add_argument("--flight-log", action="store_true", help="s26 F0: log the flight at 2 Hz")
    ap.add_argument("--flight-only", action="store_true", help="s26 F0: end at the station (no reach)")
    ap.add_argument("--out", default=str(RUNS_DIR / "ring_mission.json"))
    a = ap.parse_args()
    FLOG[0], FLIGHT_ONLY[0] = a.flight_log, a.flight_only
    RING_ROLL_LOCK[0] = not a.no_roll_lock
    global I_ERR
    if a.spin:
        SPIN[:] = a.spin
    if a.lat is not None:
        LAT[0] = a.lat
    if a.i_err is not None:
        I_ERR = a.i_err
    RB4.FF[0] = not a.no_ff
    global LIDAR_DT
    LIDAR_DT = 1.0 / a.lidar_hz
    FIT_DT[0] = 1.0 / a.fit_hz if a.fit_hz else None
    FIT_MAX[0], INNOV[0] = a.fit_max, a.innov
    if a.g_mm:
        G_MM[0] = a.g_mm
    if a.g_deg:
        G_DEG[0] = a.g_deg
    USE_CAM[0] = a.cam
    STEREO.update(act=a.stereo_act, tilt=a.stereo_tilt, frame=a.stereo_frame, mot_m=a.stereo_mot)
    seeds = [7600000 + i for i in (a.seeds if a.seeds else range(a.start, a.start + a.n))]
    if a.workers > 1:
        with Pool(min(a.workers, 6)) as pool:
            rows = pool.map(_job, seeds, chunksize=1)
    else:
        rows = [_job(s) for s in seeds]
    res = [r["result"] for r in rows]
    summ = dict(n=len(rows), results={k: res.count(k) for k in sorted(set(res))},
                collisions=sum(1 for r in rows if r.get("contacts_bad")))
    print(json.dumps(summ, indent=1))
    out_p = Path(a.out)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
