"""Phase 2a: 8 m lidar-only 6-DOF pose acquisition + 15 s RelNav observation + early spin gate."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _paths  # noqa: E402,F401
import lidar_acq  # noqa: E402
from gnc.nav import point_R  # noqa: E402

from .context import MissionContext, MissionState


def run_acq8(ctx: MissionContext, state: MissionState) -> Optional[str]:
    M = ctx.M
    kd = M.kd
    bc = ctx.servicer.bus_ctrl
    dt = ctx.backend.dt
    tracker = state.tracker
    r8_des = state.r8_des
    assert tracker is not None and r8_des is not None

    n_sub = int(round(1.0 / dt))
    n_ctrl = int(round(bc.ctrl_dt / dt))
    ctx.bus.phase = M.phase = "acq8"

    best_acq = None
    for _ in range(8):
        P, _ = ctx.sensors.lidar_points()
        Pl_nom, Rl_nom = ctx.sensors.cam_nominal(M.lid)
        _, _, _, _, com_s = ctx.bus.bus_state()
        c_bus_w = com_s + tracker.kf.x[:3]
        t_bus_l = Rl_nom.T @ (c_bus_w - Pl_nom)
        acq = lidar_acq.acquire(P, t_bus_l, client_spec=ctx.client)
        if acq["ok"]:
            if best_acq is None or acq["score"] < best_acq[3]:
                best_acq = (float(kd.time), Rl_nom @ acq["R"], Pl_nom + Rl_nom @ acq["t"], float(acq["score"]))
            if acq["score"] < 0.06:
                break
        # Keep FarKF pointing gently while sweeping
        F_w, T_b = np.zeros(3), np.zeros(3)
        t_c = float(kd.time)
        t_s = float(kd.time)

        def _tick_f(t: float) -> None:
            nonlocal t_c, t_s
            if tracker.pending:
                tracker.deliver(t)
            if t + 1e-9 >= t_s:
                t_s = t + 1.0
                P_f, _ = ctx.sensors.lidar_points()
                Pl_f, Rl_f = ctx.sensors.cam_nominal(M.lid)
                Pc_f, Rc_f = ctx.sensors.cam_nominal(M.bcam)
                img_f = ctx.sensors.bus_cam_rgb()
                _, _, _, _, com_f = ctx.bus.bus_state()
                tracker.queue_measurement(t, P_f, Pl_f, Rl_f, Pc_f, Rc_f, img_f, com_f, float(M.km.cam_fovy[M.bcam]))
            if t + 1e-9 >= t_c:
                tracker.predict(bc.ctrl_dt, F_w / ctx.bus.bus_mass_inertia()[0], t)
                t_c = t + bc.ctrl_dt
                _, R, _, wb, _ = ctx.bus.bus_state()
                m, I = ctx.bus.bus_mass_inertia()
                F_w[:] = np.clip(m * (bc.kp_t * (r8_des + tracker.kf.x[:3]) + bc.kd_t * tracker.kf.x[3:]), -bc.f_max, bc.f_max)
                R_d = point_R(ctx.bore_b, tracker.kf.x[:3], state.roll_ref)
                T_b[:] = I @ (bc.kp_a * Rotation.from_matrix(R.T @ R_d).as_rotvec() - bc.kd_a * wb) + np.cross(wb, I @ wb)

        ctx.bus.step(F_w, T_b, ctx.q_app, M.q_open, n_sub, thr_on=True, on_step=_tick_f)
        if ctx.bus.bad_contacts:
            return "abort: acq8: collision"

    if best_acq is None:
        return "abort: acq8: no_fit"

    t_a0, R_a0, p_a0, sc_a0 = best_acq
    M.nav.add(t_a0, R_a0, p_a0, "full")
    M.last_fix_ok = t_a0
    M.t_lidar = float(kd.time) + 1.0

    R_tr_now = kd.xmat[M.bid_t].reshape(3, 3)
    acq_err_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_a0.T @ R_tr_now).as_rotvec())))
    ctx.row["phases"]["acq8_init"] = dict(t=round(float(kd.time), 1), score=round(sc_a0, 4), err_deg=round(acq_err_deg, 2))

    # 15 s RelNav observation at 8 m to lock spin vector
    t_rel_sens = float(kd.time)
    for _ in range(15):
        F_w, T_b = np.zeros(3), np.zeros(3)
        t_c = float(kd.time)

        def _tick_r(t: float) -> None:
            nonlocal t_rel_sens, t_c
            if M.pend:
                M._deliver(t)
            if t + 1e-9 >= t_rel_sens:
                t_rel_sens = t + 1.0
                M.fix()
            if t + 1e-9 >= t_c:
                t_c = t + bc.ctrl_dt
                Rw, _, _, vv, cc = M.nav.predict(t)
                p_cur = cc + r8_des
                R_d = point_R(ctx.bore_b, cc - p_cur, state.roll_ref)
                p, R, v, wb, _ = ctx.bus.bus_state()
                m, I = ctx.bus.bus_mass_inertia()
                F_w[:] = np.clip(m * (bc.kp_t * (p_cur - p) + bc.kd_t * (vv - v)), -bc.f_max, bc.f_max)
                e_R = Rotation.from_matrix(R.T @ R_d).as_rotvec()
                T_b[:] = I @ (bc.kp_a * e_R - bc.kd_a * wb) + np.cross(wb, I @ wb)

        ctx.bus.step(F_w, T_b, ctx.q_app, M.q_open, n_sub, thr_on=True, on_step=_tick_r)
        if ctx.bus.bad_contacts:
            return "abort: acq8: collision"

    if M.nav.full is None:
        return "abort: acq8: no_rate"
    why_sg = M.spin_gate(ctx.row)
    if why_sg:
        return f"abort: acq8: {why_sg}"
    return None
