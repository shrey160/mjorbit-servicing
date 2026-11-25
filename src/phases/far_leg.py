"""Phase 1: 30 m FarKF hold, 30 -> 8 m min-jerk approach, and staged arm unfold at 8 m."""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from gnc.nav import point_R
import far_nav as _far_nav

from .context import MissionContext, MissionState


def run_far_leg(ctx: MissionContext, state: MissionState) -> Optional[str]:
    """Run far_hold (30 m), far_app (30 -> 8 m), and staged unfold at 8 m. Returns abort reason or None."""
    M = ctx.M
    kd = M.kd
    bc = ctx.servicer.bus_ctrl
    dt = ctx.backend.dt
    tracker = state.tracker
    assert tracker is not None

    n_sub = int(round(1.0 / dt))
    n_ctrl = int(round(bc.ctrl_dt / dt))
    t_sens, t_ctrl_f = 0.0, 0.0
    a_own = np.zeros(3)
    q_tgt = ctx.q_stow.copy()

    def far_step(r_des_rel: np.ndarray, v_des_rel: np.ndarray, a_des_rel: np.ndarray, q_arm_cmd: np.ndarray, n_steps: int) -> None:
        nonlocal t_sens, t_ctrl_f, a_own
        F_w, T_b = np.zeros(3), np.zeros(3)

        def _tick(t: float) -> None:
            nonlocal t_sens, t_ctrl_f, a_own
            if tracker.pending:
                tracker.deliver(t)
            if t + 1e-9 >= t_sens:
                t_sens = t + 1.0
                P, _ = ctx.sensors.lidar_points()
                Pl_nom, Rl_nom = ctx.sensors.cam_nominal(M.lid)
                Pc_nom, Rc_nom = ctx.sensors.cam_nominal(M.bcam)
                img = ctx.sensors.bus_cam_rgb()
                _, _, _, _, com_s = ctx.bus.bus_state()
                tracker.queue_measurement(
                    t, P, Pl_nom, Rl_nom, Pc_nom, Rc_nom, img, com_s, float(M.km.cam_fovy[M.bcam])
                )
            if t + 1e-9 >= t_ctrl_f:
                tracker.predict(bc.ctrl_dt if t_ctrl_f > 0 else 0.0, a_own, t)
                t_ctrl_f = t + bc.ctrl_dt
                _, R, _, wb, _ = ctx.bus.bus_state()
                m, I = ctx.bus.bus_mass_inertia()
                F_w[:] = np.clip(
                    m * (a_des_rel + bc.kp_t * (r_des_rel + tracker.kf.x[:3]) + bc.kd_t * (v_des_rel + tracker.kf.x[3:])),
                    -bc.f_max,
                    bc.f_max,
                )
                a_own = F_w / m
                R_d = point_R(ctx.bore_b, tracker.kf.x[:3], state.roll_ref)
                e_R = Rotation.from_matrix(R.T @ R_d).as_rotvec()
                T_b[:] = I @ (bc.kp_a * e_R - bc.kd_a * wb) + np.cross(wb, I @ wb)

        ctx.bus.step(F_w, T_b, q_arm_cmd, M.q_open, n_steps, thr_on=True, on_step=_tick)

    # 1a) far_hold at 30 m
    ctx.bus.phase = M.phase = "far_hold"
    r_hold0 = -tracker.kf.x[:3].copy()
    t_conv = None
    for sec in range(120):
        far_step(r_hold0, np.zeros(3), np.zeros(3), q_tgt, n_sub)
        if ctx.bus.bad_contacts:
            return "abort: far_hold: collision"
        sp, sv = tracker.kf.sig()
        if sec >= 15 and tracker.n_lid >= 10 and sp < 0.60 and sv < 0.025:
            t_conv = round(float(kd.time), 1)
            break
    if t_conv is None:
        return "abort: far_hold: no_converge"
    ctx.row["phases"]["far_hold"] = dict(t=t_conv, n_lid=tracker.n_lid, n_cam=tracker.n_cam)

    # 1b) far_app 30 m -> 8 m
    ctx.bus.phase = M.phase = "far_app"
    r0 = -tracker.kf.x[:3].copy()
    u_hat = r0 / max(float(np.linalg.norm(r0)), 1e-9)
    path = _far_nav.FarPath(r0, ctx.config.d_obs * u_hat, v_max=ctx.config.v_far)
    path.k = 1.0
    t_app_max = float(kd.time) + path.T + 120.0
    while not path.done() and float(kd.time) < t_app_max:
        stale = (float(kd.time) - tracker.kf.t_fix) > 10.0
        path.k = 0.0 if stale else 1.0
        if not stale:
            path.tau = min(path.T, path.tau + bc.ctrl_dt)
        rp, vp, ap = path.rel()
        far_step(rp, vp, ap, q_tgt, n_ctrl)
        if ctx.bus.bad_contacts:
            return "abort: far_app: collision"
    if not path.done():
        return "abort: far_app: timeout"

    r8_des = ctx.config.d_obs * u_hat
    for _ in range(10):
        far_step(r8_des, np.zeros(3), np.zeros(3), q_tgt, n_sub)
        if ctx.bus.bad_contacts:
            return "abort: far_app: collision"
    state.r8_des = r8_des
    ctx.row["phases"]["far_app"] = dict(t=round(float(kd.time), 1))

    # 1c) Staged arm unfold at 8 m: stow -> qa -> qb -> ready -> q_app
    ctx.bus.phase = M.phase = "unfold"
    for q_a, q_b in ctx.servicer.unfold_waypoints(ctx.q_stow, ctx.q_ready, ctx.q_app):
        T_seg = max(float(np.max(np.abs(q_b - q_a))) / ctx.servicer.arm.qd_max_rad_s, 1.0)
        n_seg = int(np.ceil(T_seg / bc.ctrl_dt))
        for k in range(1, n_seg + 1):
            q_tgt = q_a + (q_b - q_a) * (k / n_seg)
            far_step(r8_des, np.zeros(3), np.zeros(3), q_tgt, n_ctrl)
            if ctx.bus.bad_contacts:
                return "abort: unfold: collision"
    ctx.row["phases"]["unfold"] = dict(t=round(float(kd.time), 1))
    return None
