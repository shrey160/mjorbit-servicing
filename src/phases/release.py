"""Phase 5: Calibrated unlock -> retract -> 0.8 m bus clear -> retreat & reverse staged stow (500 Hz audit)."""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import far_nav
import reach_bench as RB4
from gnc.arm_ik import any_client_contact, build_staged_segments, client_com_vel, eval_staged_segments

from .context import MissionContext, MissionState


def run_release(ctx: MissionContext, state: MissionState) -> str:
    M = ctx.M
    kd = M.kd
    bc = ctx.servicer.bus_ctrl
    cfg = ctx.config
    dt = ctx.backend.dt
    n_ctrl = int(round(bc.ctrl_dt / dt))

    q_lock_arm = state.q_lock_arm
    q_lock_jaw = state.q_lock_jaw
    assert q_lock_arm is not None and q_lock_jaw is not None

    # Truth-free ring normal latched from own grip_site (-X axis of grip_site = +n)
    Rs_g = kd.site_xmat[ctx.gsite].reshape(3, 3).copy()
    n_est = -Rs_g[:, 0]
    n_est /= max(float(np.linalg.norm(n_est)), 1e-12)
    state.n_est = n_est

    v0_cli, w0_cli = client_com_vel(kd, M.bid_t, M.tv, M.com_true)
    _, R_hold_att, _, _, _ = ctx.bus.bus_state()
    post_open_hits = 0
    post_open_peak_N = 0.0

    def rel_step(q_a, g_c, n_st, *, thr_on=False, p_des=None, v_des=None, a_des=None, audit=False) -> None:
        nonlocal post_open_hits, post_open_peak_N
        F_w, T_b = np.zeros(3), np.zeros(3)
        t_c = float(kd.time)

        def _tick(t: float) -> None:
            nonlocal t_c, post_open_hits, post_open_peak_N
            if t + 1e-9 >= t_c:
                t_c = t + bc.ctrl_dt
                p, R, v, wb, _ = ctx.bus.bus_state()
                m, I = ctx.bus.bus_mass_inertia()
                e_R = Rotation.from_matrix(R.T @ R_hold_att).as_rotvec()
                T_b[:] = I @ (bc.kp_a * e_R - bc.kd_a * wb) + np.cross(wb, I @ wb)
                if thr_on and p_des is not None:
                    F_w[:] = np.clip(
                        m
                        * (
                            (a_des if a_des is not None else np.zeros(3))
                            + bc.kp_t * (p_des - p)
                            + bc.kd_t * ((v_des if v_des is not None else np.zeros(3)) - v)
                        ),
                        -bc.f_max,
                        bc.f_max,
                    )
                else:
                    F_w[:] = 0.0
            if audit:
                hits = any_client_contact(M.km, kd, ctx.bus.ids["srv_b"], ctx.bus.ids["cli_b"])
                if hits:
                    post_open_hits += 1
                    post_open_peak_N = max(post_open_peak_N, max(hits))

        ctx.bus.step(F_w, T_b, q_a, g_c, n_st, thr_on=thr_on, on_step=_tick)

    # 5a) unlock_open
    ctx.bus.phase = M.phase = "unlock_open"
    rel_step(q_lock_arm, q_lock_jaw, int(round(0.05 / dt)), thr_on=False)
    kd.eq_active[ctx.ev_u] = 0
    kd.eq_active[ctx.ev_l] = 0
    rel_step(q_lock_arm, q_lock_jaw, int(round(0.20 / dt)), thr_on=False)
    n_open = max(1, int(round(cfg.open_t_s / dt)))
    for k in range(0, n_open, 5):
        g_cmd = q_lock_jaw + (M.q_open - q_lock_jaw) * min(1.0, (k + 5) / n_open)
        rel_step(q_lock_arm, g_cmd, 5, thr_on=False)
    rel_step(q_lock_arm, M.q_open, int(round(0.3 / dt)), thr_on=False, audit=True)

    # 5b) retract along latched +n_est
    ctx.bus.phase = M.phase = "retract"
    p_g0 = kd.site_xpos[ctx.gsite].copy()
    R_g0 = kd.site_xmat[ctx.gsite].reshape(3, 3).copy()
    n_ret = max(1, int(round((cfg.retract_m / cfg.retract_v) / dt)))
    q_cmd = q_lock_arm.copy()
    for k in range(0, n_ret, 5):
        p_t = p_g0 + (cfg.retract_m * min(1.0, (k + 5) / n_ret)) * n_est
        qn, ep, er = RB4.ik_lim(M, p_t, R_g0, q_cmd, iters=20)
        if ep < 5e-3 and er < 0.5:
            q_cmd = qn
        rel_step(q_cmd, M.q_open, 5, thr_on=False, audit=True)

    # 5c) retreat & reverse staged stow after arm_clear_m clearance
    ctx.bus.phase = M.phase = "retreat_restow"
    p_b0, _, v_b0, _, _ = ctx.bus.bus_state()
    bpath = far_nav.FarPath(np.zeros(3), cfg.retreat_m * n_est, v_max=cfg.retreat_v)
    bpath.k = 1.0
    arm_segs = build_staged_segments(
        ctx.servicer.restow_waypoints(q_cmd, ctx.q_ready, ctx.q_stow), ctx.servicer.arm.qd_max_rad_s, min_seg_s=0.5
    )
    t_arm0 = None
    t_b0 = float(kd.time)
    while not bpath.done():
        bpath.tau = min(bpath.T, bpath.tau + bc.ctrl_dt)
        rp, vp, ap = bpath.rel()
        if t_arm0 is None and float(np.linalg.norm(rp)) >= cfg.arm_clear_m:
            t_arm0 = float(kd.time)
        el_b = float(kd.time) - t_b0
        q_arm_ref = q_cmd if t_arm0 is None else eval_staged_segments(arm_segs, float(kd.time) - t_arm0, ctx.q_stow)
        rel_step(
            q_arm_ref,
            M.q_open,
            n_ctrl,
            thr_on=True,
            p_des=p_b0 + v_b0 * el_b + rp,
            v_des=v_b0 + vp,
            a_des=ap,
            audit=True,
        )

    if t_arm0 is None:
        t_arm0 = float(kd.time)
    tot_arm_T = sum(x[2] for x in arm_segs)
    while float(kd.time) - t_arm0 < tot_arm_T + 1.0:
        el_b = float(kd.time) - t_b0
        rel_step(
            eval_staged_segments(arm_segs, float(kd.time) - t_arm0, ctx.q_stow),
            M.q_open,
            n_ctrl,
            thr_on=True,
            p_des=p_b0 + v_b0 * el_b + cfg.retreat_m * n_est,
            v_des=v_b0,
            a_des=np.zeros(3),
            audit=True,
        )

    v1_cli, w1_cli = client_com_vel(kd, M.bid_t, M.tv, M.com_true)
    ctx.row["release"] = dict(
        post_open_contact_steps=post_open_hits,
        post_open_peak_N=round(post_open_peak_N, 2),
        dv_mm_s=round(float(np.linalg.norm(v1_cli - v0_cli)) * 1e3, 3),
        dw_deg_s=round(float(np.degrees(np.linalg.norm(w1_cli - w0_cli))), 4),
        end_dist_m=round(float(np.linalg.norm(kd.xpos[M.bid_c] - kd.xpos[M.bid_t])), 2),
        stow_err_deg=round(float(np.degrees(np.max(np.abs(kd.qpos[M.arm] - ctx.q_stow)))), 2),
    )
    return "collision" if (ctx.bus.bad_contacts or M.bad) else "complete"
