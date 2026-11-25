"""P7 end-to-end calibration: ring tool v3 release, retract, rev-2 retreat & staged re-stow bench.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/release_bench.py [--n 10] [--workers 3]

Per seed (7520000 + i):
  1. Setup a realistic locked ring-tool v3 capture on free client_p7 + free servicer_rev2 (random servicer roll,
     grasp section clear of the wing, jaws closed and locked via lock_u/lock_l, 1 s settle so clamp forces are real,
     plus post-detumble residual velocity/rate and latched estimate error +-5 mm / +-0.5 deg).
  2. unlock_open: thrusters off, wheels hold rate; set jaw targets to q_lock, release lock_u/lock_l, settle 0.2 s,
     ramp jaws smoothly to q_open over --t-open s (default 4.0 s).
  3. retract: pull grip site straight back along latched estimated +n by --retract m (default 0.20 m) at --v-ret m/s
     (default 0.025 m/s).
  4. retreat_restow: move arm to servicer_ready, then reverse staged fold (ready -> qb -> qa -> stow) while rev-2
     thrusters back the bus from 1.0 m -> 3.2 m along +n_est (min-jerk, v_back = 0.05 m/s).

500 Hz contact audit (every physics step):
  - Tracks ANY servicer-client contact after jaws finish opening (fixing P6's 100 Hz blind spot).
  - Tracks non-tool-on-ring collisions across all phases.
  - Measures exact client Delta-v push (mm/s), Delta-w tip-off (deg/s), plume pulses (D7.7), propellant, wheel fill.
Writes runs/release_bench.json.
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
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
import reach_bench as RB4  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_fit  # noqa: E402
import ring_mission as RM  # noqa: E402
import far_nav  # noqa: E402


def _pct(xs, q):
    return round(float(np.percentile(xs, q)), 3) if xs else None


class ReleaseBench:
    def __init__(self):
        self.M = RM.Mission()
        km = self.M.km
        self.ev_u = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_u")
        self.ev_l = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_l")
        self.gsite = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "grip_site")

    def _any_cli_contact(self):
        """500 Hz check: returns list of (geom1_name, geom2_name, max_normal_F) for ANY servicer-client contact."""
        M = self.M
        km, kd = M.km, M.kd
        hits = []
        f6 = np.zeros(6)
        for i in range(kd.ncon):
            c = kd.contact[i]
            b1, b2 = km.geom_bodyid[c.geom1], km.geom_bodyid[c.geom2]
            if (b1 in M.srv_b and b2 in M.cli_b) or (b2 in M.srv_b and b1 in M.cli_b):
                mujoco.mj_contactForce(km, kd, i, f6)
                n1 = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or f"#{c.geom1}"
                n2 = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or f"#{c.geom2}"
                hits.append((n1, n2, float(abs(f6[0]))))
        return hits

    def _client_com_vel(self):
        M = self.M
        kd = M.kd
        Rc = kd.xmat[M.bid_t].reshape(3, 3)
        w = Rc @ kd.qvel[M.tv + 3:M.tv + 6]
        v = kd.qvel[M.tv:M.tv + 3] + np.cross(w, Rc @ M.com_true)
        return v, w

    def _setup_locked_capture(self, seed: int, resid_v=0.002, resid_w_deg=0.05):
        """Place servicer at grasp station, close & lock jaws on free client_p7 (as reach_bench), settle 1 s, then
        unpin bus and apply small post-detumble residual velocity/rate to the whole stack."""
        M = self.M
        km, kd = M.km, M.kd
        rng = np.random.default_rng([seed, 752])
        mujoco.mj_resetData(km, kd)
        M._vff = np.zeros(6)
        M.bad = set()
        M.phase = "unlock_open"
        M.arm_guard(False)
        kd.eq_active[self.ev_u] = 0
        kd.eq_active[self.ev_l] = 0

        # Client at origin, random attitude
        Rc = Rotation.random(random_state=rng.integers(2**31)).as_matrix()
        kd.qpos[M.tq:M.tq + 7] = [0.0, 0.0, 0.0, *RB._q_wxyz(Rc)]
        kd.qvel[M.tv:M.tv + 6] = 0.0

        # Servicer at station (bus centre 1.0 m beyond ring top, shoulder away from +Y wing, slight station offset)
        R_rel = (Rotation.from_rotvec([-np.pi / 2, 0, 0]) * Rotation.from_quat([0, 0, 1, 0])).as_matrix()
        roll_jit = Rotation.from_rotvec([np.radians(rng.uniform(-20.0, 20.0)), 0, 0]).as_matrix()
        p_st = ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + RB4.STATION)
        dp_st = rng.uniform(-0.015, 0.015, 3)
        p_s = Rc @ (p_st + dp_st)
        R_s = Rc @ roll_jit @ R_rel
        kd.qpos[M.cq:M.cq + 7] = [*p_s, *RB._q_wxyz(R_s)]
        kd.qvel[M.cv:M.cv + 6] = 0.0
        kd.qpos[M.arm] = M.q_app
        kd.qpos[M.gq] = M.q_open
        mujoco.mj_forward(km, kd)

        # Pin bus during initial bench-style seated close
        pin = kd.qpos[M.cq:M.cq + 7].copy()
        E_true = (Rc @ ring_fit.C_B, Rc @ ring_fit.N_B)
        wing = Rc[:, 1]
        a, rad, _ = M.choose(E_true, wing)
        if a is None:
            return None, None

        # Place arm directly at seated palm contact (h = 0.001 m) via IK, then ramp jaws closed & lock
        p_seat, R_seat = M.section(E_true, rad, 0.001)
        q_arm, ep, er = RB4.ik_lim(M, p_seat, R_seat, M.q_app, iters=120)
        if ep > 2e-3 or er > 0.2:
            return None, None
        kd.qpos[M.arm] = q_arm
        kd.qvel[:] = 0.0
        mujoco.mj_forward(km, kd)

        dt = float(km.opt.timestep)
        sq_cmd = np.array([-M.sq, M.sq])
        d_jaw = sq_cmd - M.q_open
        T_ramp = float(np.max(np.abs(d_jaw))) / RB4.V_JAW[0]
        n_ramp = max(1, int(T_ramp / dt))
        n_tot = n_ramp + int(1.5 / dt)
        lim = km.actuator_ctrlrange[M.I["act_arm"], 1]

        def step_pinned(q_a, g_cmd, n_st):
            for _ in range(n_st):
                qa, qda = kd.qpos[M.arm], kd.qvel[M.I["arm_v"]]
                kd.ctrl[M.I["act_arm"]] = np.clip(M.kp * (q_a - qa) - M.kdv * qda, -lim, lim)
                kd.ctrl[M.I["act_u"]] = g_cmd[0]
                kd.ctrl[M.I["act_l"]] = g_cmd[1]
                kd.ctrl[M.noz] = 0.0
                kd.ctrl[M.rwm] = 0.0
                mujoco.mj_step(km, kd)
                kd.qpos[M.cq:M.cq + 7] = pin
                kd.qvel[M.cv:M.cv + 6] = 0.0

        h = 0.001
        q_hold = q_arm.copy()
        for k in range(0, n_tot, 5):
            fp = M.touches()["palm"]
            h -= RB4.V_FOLLOW * 5 * dt * float(np.clip((RB4.F_PALM - fp) / RB4.F_PALM, -1.0, 1.0))
            p_t, R_t = M.section(E_true, rad, h)
            q_hold, _, _ = RB4.ik_lim(M, p_t, R_t, q_hold, iters=20)
            step_pinned(q_hold, M.q_open + d_jaw * min(1.0, (k + 5) / n_ramp), 5)

        # Lock jaws where they sit and settle 1.0 s
        q_lock = kd.qpos[M.gq].copy()
        for ev, qv in ((self.ev_u, q_lock[0]), (self.ev_l, q_lock[1])):
            km.eq_data[ev, :5] = [qv, 0, 0, 0, 0]
            kd.eq_active[ev] = 1
        step_pinned(q_hold, sq_cmd, int(1.0 / dt))

        t_hold = M.touches()
        grip_ok = t_hold["in"] > 5 and (t_hold["hook"] > 5 or t_hold["lip"] > 5) and (t_hold["palm"] > 5 or t_hold["lip"] > 5)
        if not grip_ok:
            return None, dict(why="setup_no_grip", touch={k: round(v, 1) for k, v in t_hold.items()})

        # Unpin bus, add small post-detumble rigid-body residual velocity & spin to BOTH bodies (stack moving together)
        v_res = rng.uniform(-resid_v, resid_v, 3)
        w_res = np.radians(rng.uniform(-resid_w_deg, resid_w_deg, 3))
        p_ref = kd.xpos[M.bid_t].copy()
        Rc_now = kd.xmat[M.bid_t].reshape(3, 3).copy()
        Rs_now = kd.xmat[M.bid_c].reshape(3, 3).copy()
        kd.qvel[M.tv:M.tv + 3] = v_res
        kd.qvel[M.tv + 3:M.tv + 6] = Rc_now.T @ w_res
        kd.qvel[M.cv:M.cv + 3] = v_res + np.cross(w_res, kd.xpos[M.bid_c] - p_ref)
        kd.qvel[M.cv + 3:M.cv + 6] = Rs_now.T @ w_res
        kd.qvel[M.I["arm_v"]] = 0.0
        mujoco.mj_forward(km, kd)

        # Latched estimate of ring axis +n in world frame (with realistic +-0.5 deg / +-5 mm capture error)
        n_true = Rc_now @ ring_fit.N_B
        n_est = Rotation.from_rotvec(np.radians(rng.normal(0.0, 0.5 / np.sqrt(3), 3))).as_matrix() @ n_true
        n_est /= np.linalg.norm(n_est)
        c_est = kd.xpos[M.bid_t] + Rc_now @ ring_fit.C_B + rng.normal(0.0, 0.005 / np.sqrt(3), 3)

        setup = dict(
            rng=rng, q_hold=kd.qpos[M.arm].copy(), q_lock=q_lock,
            n_est=n_est, c_est=c_est,
            hold_touch={k: round(v, 1) for k, v in t_hold.items()},
        )
        return setup, None

    def run_episode(self, seed: int, t_open=4.0, retract_m=0.20, v_ret=0.025, v_back=0.05, back_m=2.2,
                    resid_v=0.002, resid_w_deg=0.05):
        t_wall = time.time()
        setup, err = self._setup_locked_capture(seed, resid_v=resid_v, resid_w_deg=resid_w_deg)
        if setup is None:
            return dict(seed=seed, result="abort: setup failed", err=err, wall_s=round(time.time() - t_wall, 1))

        M = self.M
        km, kd = M.km, M.kd
        dt = float(km.opt.timestep)
        lim = km.actuator_ctrlrange[M.I["act_arm"], 1]
        M.bad = set()
        M.phase = "unlock_open"

        # Record true client COM velocity & angular rate right before unlock
        v0_cli, w0_cli = self._client_com_vel()

        # Rev-2 bus actuators (wheels hold initial attitude; thrusters OFF until retreat)
        pulses = RM.Pulses(M.rcs, seed=seed)
        wheels = RM.Wheels(M.busd)
        p_bus0, R_bus0, v_bus0, _, _ = M.bus_state()
        R_hold_att = R_bus0.copy()
        n_est = setup["n_est"]
        c_est = setup["c_est"]

        # State for bus control
        u_thr = np.zeros(M.rcs.n)
        T_cmd = np.zeros(3)
        tau_thr = np.zeros(3)
        unloading = False
        plume = 0
        t_ctrl = kd.time
        post_open_hits = 0
        post_open_peak_N = 0.0
        post_open_pairs = set()
        t_sep = None
        t_release_start = kd.time

        def step_free(q_arm_cmd, g_cmd, n_steps, thr_on=False, p_des=None, v_des=None, a_des=None, audit_after_open=False):
            nonlocal u_thr, T_cmd, tau_thr, unloading, plume, t_ctrl
            nonlocal post_open_hits, post_open_peak_N, t_sep
            for _ in range(n_steps):
                t = kd.time
                if t + 1e-9 >= t_ctrl:
                    t_ctrl = t + RM.CTRL_DT
                    p, R, v, wb, com = M.bus_state()
                    m, I = M.bus_mass_inertia()
                    e_R = RM._log(R.T @ R_hold_att)
                    T_cmd = I @ (RM.KP_A * e_R - RM.KD_A * wb) + np.cross(wb, I @ wb)
                    if thr_on and p_des is not None:
                        e_p = p_des - p
                        e_v = (v_des if v_des is not None else np.zeros(3)) - v
                        a_ff = a_des if a_des is not None else np.zeros(3)
                        F = np.clip(m * (a_ff + RM.KP_T * e_p + RM.KD_T * e_v), -RM.F_MAX, RM.F_MAX)
                        fill = wheels.fill()
                        unloading = fill > RM.UNLOAD_ON or (unloading and fill > RM.UNLOAD_OFF)
                        tau_thr = np.clip(-RM.UNLOAD_K * wheels.H(), -RM.UNLOAD_MAX, RM.UNLOAD_MAX) if unloading else np.zeros(3)
                        w_cmd = np.r_[R.T @ F, tau_thr]
                        ref = R.T @ (com - p)
                        u_thr = M.rcs.allocate(w_cmd, ref) if np.any(np.abs(w_cmd) > 1e-9) else np.zeros(M.rcs.n)
                    else:
                        u_thr = np.zeros(M.rcs.n)
                        tau_thr = np.zeros(3)

                f = pulses.step(u_thr if thr_on else np.zeros(M.rcs.n), dt)
                kd.ctrl[M.noz] = f
                p, R, _, wb, _ = M.bus_state()
                dF = f[:, None] * (pulses.D_true - M.rcs.D)
                dT = np.cross(M.rcs.P, dF).sum(0)
                kd.xfrc_applied[M.bid_c, :3] = R @ dF.sum(0)
                tau, _ = wheels.command(T_cmd - tau_thr if thr_on else T_cmd, wb, dt)
                kd.ctrl[M.rwm] = tau + dT

                # Plume check (D7.7: nozzles firing within 30 deg of client inside 3 m)
                if thr_on and f.any():
                    cc_t = kd.xpos[M.bid_t] + kd.xmat[M.bid_t].reshape(3, 3) @ M.com_true
                    for idx_n in np.flatnonzero(f):
                        pz = p + R @ M.rcs.P[idx_n]
                        d_c = cc_t - pz
                        dn = float(np.linalg.norm(d_c))
                        if dn < 3.0 and float(-(R @ M.rcs.D[idx_n]) @ d_c) / max(dn, 1e-9) > np.cos(np.radians(30)):
                            plume += 1

                # Arm & jaw PD (no feed-forward in contact/release)
                qa, qda = kd.qpos[M.arm], kd.qvel[M.I["arm_v"]]
                kd.ctrl[M.I["act_arm"]] = np.clip(M.kp * (q_arm_cmd - qa) - M.kdv * qda, -lim, lim)
                kd.ctrl[M.I["act_u"]] = g_cmd[0]
                kd.ctrl[M.I["act_l"]] = g_cmd[1]

                mujoco.mj_step(km, kd)
                M._contacts()

                hits = self._any_cli_contact()
                if not hits and t_sep is None and (kd.time - t_release_start) > 0.2:
                    t_sep = round(kd.time - t_release_start, 3)
                if audit_after_open and hits:
                    post_open_hits += 1
                    for n1, n2, fn in hits:
                        post_open_peak_N = max(post_open_peak_N, fn)
                        post_open_pairs.add(f"{n1}-{n2}")

        # ---- Phase 1: unlock_open (thrusters OFF)
        q_hold = setup["q_hold"].copy()
        q_lock = setup["q_lock"].copy()
        # Match jaw command to q_lock, release locks, settle 0.2 s
        step_free(q_hold, q_lock, int(0.05 / dt), thr_on=False)
        kd.eq_active[self.ev_u] = 0
        kd.eq_active[self.ev_l] = 0
        step_free(q_hold, q_lock, int(0.20 / dt), thr_on=False)
        # Ramp jaws smoothly from q_lock -> q_open over t_open
        n_open = max(1, int(round(t_open / dt)))
        for k in range(0, n_open, 5):
            frac = min(1.0, (k + 5) / n_open)
            g_cmd = q_lock + (M.q_open - q_lock) * frac
            step_free(q_hold, g_cmd, 5, thr_on=False)
        # 0.3 s quiet settle after open
        step_free(q_hold, M.q_open, int(0.3 / dt), thr_on=False, audit_after_open=True)

        v1_cli, w1_cli = self._client_com_vel()
        dv_open_mm_s = float(np.linalg.norm(v1_cli - v0_cli)) * 1e3
        dw_open_deg_s = float(np.degrees(np.linalg.norm(w1_cli - w0_cli)))

        # ---- Phase 2: retract straight back along latched +n_est (thrusters OFF)
        M.phase = "retract"
        p_grip0 = kd.site_xpos[self.gsite].copy()
        R_grip0 = kd.site_xmat[self.gsite].reshape(3, 3).copy()
        T_ret = retract_m / v_ret
        n_ret = max(1, int(round(T_ret / dt)))
        q_cmd = q_hold.copy()
        for k in range(0, n_ret, 5):
            frac = min(1.0, (k + 5) / n_ret)
            # Keep grip target relative to current bus pose so free-floating recoil doesn't fight IK
            p_t = p_grip0 + (frac * retract_m) * n_est
            # Actually compute in bus frame? No, bus hardly moves (~2 mm) during 2 kg arm retract; world-frame IK tracks it cleanly
            qn, ep, er = RB4.ik_lim(M, p_t, R_grip0, q_cmd, iters=20)
            if ep < 5e-3 and er < 0.5:
                q_cmd = qn
            step_free(q_cmd, M.q_open, 5, thr_on=False, audit_after_open=True)

        v2_cli, w2_cli = self._client_com_vel()
        dv_ret_mm_s = float(np.linalg.norm(v2_cli - v0_cli)) * 1e3
        dw_ret_deg_s = float(np.degrees(np.linalg.norm(w2_cli - w0_cli)))

        # ---- Phase 3: retreat_restow (bus backs off back_m along +n_est; hold retracted arm pose until bus has
        # backed off arm_clear_m >= 0.8 m so joint-space swing to ready/stow cannot reach the client)
        M.phase = "retreat_restow"
        p_b_start, _, v_b_start, _, _ = M.bus_state()
        path = far_nav.FarPath(np.zeros(3), back_m * n_est, v_max=v_back)
        path.k = 1.0
        q_ready = M.poses["ready"].copy()
        q_stow = M.poses["stow"].copy()
        qa = q_stow.copy()
        qa[1], qa[2] = q_stow[1] + 0.6, q_stow[2] - 0.9
        qb = qa.copy()
        qb[3] = 0.0
        # Build piecewise-linear joint trajectory: q_cmd -> q_ready -> qb -> qa -> q_stow
        qd_lim = np.radians(10.0)
        arm_segs = []
        for a_, b_ in ((q_cmd.copy(), q_ready), (q_ready, qb), (qb, qa), (qa, q_stow)):
            T_s = max(float(np.max(np.abs(b_ - a_))) / qd_lim, 0.5)
            arm_segs.append((a_, b_, T_s))
        t_arm0 = None
        arm_clear_m = 0.8

        def arm_ref(t_now):
            if t_arm0 is None:
                return q_cmd
            el = t_now - t_arm0
            for a_, b_, T_s in arm_segs:
                if el <= T_s:
                    return a_ + (b_ - a_) * (el / T_s)
                el -= T_s
            return q_stow

        n_ctrl = int(round(RM.CTRL_DT / dt))
        t_back_start = kd.time
        while not path.done():
            path.tau = min(path.T, path.tau + RM.CTRL_DT)
            rp, vp, ap = path.rel()
            if t_arm0 is None and float(np.linalg.norm(rp)) >= arm_clear_m:
                t_arm0 = kd.time
            el_b = kd.time - t_back_start
            # Propagate bus reference from p_b_start + v_b_start * el_b (free-drifting latched client frame)
            p_des = p_b_start + v_b_start * el_b + rp
            v_des = v_b_start + vp
            q_a = arm_ref(kd.time)
            step_free(q_a, M.q_open, n_ctrl, thr_on=True, p_des=p_des, v_des=v_des, a_des=ap, audit_after_open=True)

        # Ensure arm finishes stowing if retreat ended first
        if t_arm0 is None:
            t_arm0 = kd.time
        total_arm_T = sum(x[2] for x in arm_segs)
        while kd.time - t_arm0 < total_arm_T + 1.0:
            el_b = kd.time - t_back_start
            p_des = p_b_start + v_b_start * el_b + back_m * n_est
            step_free(arm_ref(kd.time), M.q_open, n_ctrl, thr_on=True, p_des=p_des, v_des=v_b_start,
                      a_des=np.zeros(3), audit_after_open=True)

        v3_cli, w3_cli = self._client_com_vel()
        dv_tot_mm_s = float(np.linalg.norm(v3_cli - v0_cli)) * 1e3
        dw_tot_deg_s = float(np.degrees(np.linalg.norm(w3_cli - w0_cli)))
        sep_dist = float(np.linalg.norm(kd.xpos[M.bid_c] - kd.xpos[M.bid_t]))
        stow_err_deg = float(np.degrees(np.max(np.abs(kd.qpos[M.arm] - q_stow))))

        return dict(
            seed=seed,
            result="collision" if M.bad else "ok",
            t_open=t_open, retract_m=retract_m, v_ret=v_ret,
            hold_touch=setup["hold_touch"],
            t_sep_s=t_sep,
            post_open_contact_steps=post_open_hits,
            post_open_peak_N=round(post_open_peak_N, 2),
            post_open_pairs=sorted(post_open_pairs),
            dv_open_mm_s=round(dv_open_mm_s, 3),
            dw_open_deg_s=round(dw_open_deg_s, 4),
            dv_ret_mm_s=round(dv_ret_mm_s, 3),
            dw_ret_deg_s=round(dw_ret_deg_s, 4),
            dv_tot_mm_s=round(dv_tot_mm_s, 3),
            dw_tot_deg_s=round(dw_tot_deg_s, 4),
            plume_pulses=plume,
            prop_g=round(pulses.prop_kg * 1e3, 1),
            wheel_fill_max=round(wheels.fill(), 3),
            end_dist_m=round(sep_dist, 2),
            stow_err_deg=round(stow_err_deg, 2),
            contacts_bad=sorted(M.bad),
            wall_s=round(time.time() - t_wall, 1),
        )


def _job(args):
    seed, kw = args
    global _RB
    if "_RB" not in globals():
        _RB = ReleaseBench()
    r = _RB.run_episode(seed, **kw)
    print(json.dumps(r), flush=True)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--t-open", type=float, default=4.0)
    ap.add_argument("--retract", type=float, default=0.20)
    ap.add_argument("--v-ret", type=float, default=0.025)
    ap.add_argument("--v-back", type=float, default=0.05)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    kw = dict(t_open=a.t_open, retract_m=a.retract, v_ret=a.v_ret, v_back=a.v_back)
    t0 = time.time()
    jobs = [(7520000 + i, kw) for i in range(a.n)]
    if a.workers > 1:
        with Pool(min(a.workers, 3)) as pool:
            rows = pool.map(_job, jobs, chunksize=1)
    else:
        rows = [_job(j) for j in jobs]
    ok = [r for r in rows if r["result"] == "ok"]
    summ = dict(
        n=len(rows), ok=len(ok),
        collisions=sum(1 for r in rows if r.get("contacts_bad")),
        any_post_open_contact=sum(1 for r in ok if r["post_open_contact_steps"] > 0),
        post_open_steps_max=max((r["post_open_contact_steps"] for r in ok), default=0),
        dv_open_p50_95=[_pct([r["dv_open_mm_s"] for r in ok], 50), _pct([r["dv_open_mm_s"] for r in ok], 95)],
        dv_tot_p50_95=[_pct([r["dv_tot_mm_s"] for r in ok], 50), _pct([r["dv_tot_mm_s"] for r in ok], 95)],
        dw_tot_p50_95=[_pct([r["dw_tot_deg_s"] for r in ok], 50), _pct([r["dw_tot_deg_s"] for r in ok], 95)],
        plume_max=max((r["plume_pulses"] for r in ok), default=0),
        prop_g_p50_95=[_pct([r["prop_g"] for r in ok], 50), _pct([r["prop_g"] for r in ok], 95)],
        wheel_fill_max=max((r["wheel_fill_max"] for r in ok), default=None),
        wall_s=round(time.time() - t0, 1),
    )
    print("SUMMARY:", json.dumps(summ, indent=1))
    out = Path(a.out) if a.out else (ensure_runs_dir() / "release_bench.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
