"""Phase 2b: 8 m -> 3.2 m inertial transfer & pull_in3 with all 6 audited fixes."""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

import far_nav
import lidar_nav
import ring_fit
from gnc.nav import ang_deg, point_R

from .context import MissionContext, MissionState


def _quintic_intercept(u: np.ndarray, T: float, r0: np.ndarray, r1: np.ndarray, v1: np.ndarray, a1: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """C^2 quintic connecting (r0, 0, 0) at u=0 to (r1, v1, a1) at u=1 (zero terminal jerk/accel overshoot)."""
    u2, u3, u4, u5 = u * u, u**3, u**4, u**5
    d = r1 - r0
    tv = T * v1
    ta = (T * T) * a1

    p0 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    p0_d = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
    p0_dd = 60.0 * u - 180.0 * u2 + 120.0 * u3

    pv = -4.0 * u3 + 7.0 * u4 - 3.0 * u5
    pv_d = -12.0 * u2 + 28.0 * u3 - 15.0 * u4
    pv_dd = -24.0 * u + 84.0 * u2 - 60.0 * u3

    pa = 0.5 * u3 - u4 + 0.5 * u5
    pa_d = 1.5 * u2 - 4.0 * u3 + 2.5 * u4
    pa_dd = 3.0 * u - 12.0 * u2 + 10.0 * u3

    rr = r0 + p0 * d + pv * tv + pa * ta
    vr = (p0_d * d + pv_d * tv + pa_d * ta) / T
    ar = (p0_dd * d + pv_dd * tv + pa_dd * ta) / (T * T)
    return rr, vr, ar


def _size_quintic_T(
    M,
    t0: float,
    r0_w: np.ndarray,
    r_b_3axis: np.ndarray,
    a_budget: float,
    T_min: float = 30.0,
    T_max: float = 95.0,
) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Fix 1: Choose smallest feasible T in [T_min, T_max] so peak per-axis inertial accel <= a_budget."""
    best = None
    best_peak = 1e9
    for T_cand in np.linspace(T_min, T_max, 14):
        Rw_a, _, w_a, _, _ = M.nav.predict(t0 + float(T_cand))
        r1 = Rw_a @ r_b_3axis
        v1 = np.cross(w_a, r1)
        a1 = np.cross(w_a, v1)
        peak = 0.0
        for u in np.linspace(0.0, 1.0, 21):
            _, _, ar = _quintic_intercept(u, float(T_cand), r0_w, r1, v1, a1)
            peak = max(peak, float(np.max(np.abs(ar))))
        if peak < best_peak:
            best_peak = peak
            best = (float(T_cand), r1, v1, a1)
        if peak <= a_budget:
            return float(T_cand), r1, v1, a1
    assert best is not None
    return best


def _safe_sphere_chords(r_start_w: np.ndarray, u_end: np.ndarray, radius_m: float, max_step_deg: float = 55.0) -> list[np.ndarray]:
    """Subdivide spherical hop with antipodal guard (>150 deg -> orthogonal midpoint) and <= max_step_deg chords."""
    u_s = r_start_w / max(float(np.linalg.norm(r_start_w)), 1e-9)
    u_e = u_end / max(float(np.linalg.norm(u_end)), 1e-9)
    ang = ang_deg(u_s, u_e)
    if ang <= max_step_deg:
        return [r_start_w, radius_m * u_e]
    if float(u_s @ u_e) < -0.90:
        ref = np.array([0.0, 0.0, 1.0]) if abs(u_s[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        perp = np.cross(u_s, ref)
        perp /= float(np.linalg.norm(perp))
        pts_1 = _safe_sphere_chords(r_start_w, perp, radius_m, max_step_deg=max_step_deg)
        pts_2 = _safe_sphere_chords(radius_m * perp, u_e, radius_m, max_step_deg=max_step_deg)
        return pts_1 + pts_2[1:]
    n_ch = max(2, int(np.ceil(ang / max_step_deg)))
    pts = [r_start_w]
    for k in range(1, n_ch + 1):
        uk = (1.0 - k / n_ch) * u_s + (k / n_ch) * u_e
        uk /= max(float(np.linalg.norm(uk)), 1e-9)
        pts.append(radius_m * uk)
    return pts


def run_xfer(ctx: MissionContext, state: MissionState) -> Optional[str]:
    M = ctx.M
    kd = M.kd
    bc = ctx.servicer.bus_ctrl
    dt = ctx.backend.dt
    ctrl_dt = bc.ctrl_dt
    n_ctrl = int(round(ctrl_dt / dt))
    n_sub = int(round(1.0 / dt))

    ring = ctx.client.ring
    n_b = ring.n_arr
    c_b = ring.c_arr
    r_b_3axis = (c_b + n_b * (ring.h_top + ctx.client.r_hold3_m)) - M.nav.com_b

    m_bus, _ = ctx.bus.bus_mass_inertia()
    a_max = ctx.servicer.max_linear_accel(m_bus)
    a_budget = min(ctx.config.max_xfer_accel_frac * a_max, 0.72 * a_max)
    # Fix 4: track 8 m ring verification across both 2b and 2c so fast spinners never skip it
    ring_seen = False
    ring_checked = 0

    def maybe_check_ring(r_cur_w: np.ndarray, Rw: np.ndarray) -> bool:
        nonlocal ring_seen, ring_checked
        if ring_seen:
            return False
        if ang_deg(r_cur_w, Rw @ n_b) < 35.0:
            ring_checked += 1
            P_chk, _ = ctx.sensors.lidar_points()
            Pl_nom, Rl_nom = ctx.sensors.cam_nominal(M.lid)
            Rw_c, pw_c, _, _, _ = M.nav.predict(float(kd.time))
            R0_p, t0_p = Rl_nom.T @ Rw_c, Rl_nom.T @ (pw_c - Pl_nom)
            rf_c = ring_fit.fit(P_chk, R0_p, t0_p, min_top=6, spec=ring)
            if not (rf_c and rf_c.get("ok")):
                bf_c = lidar_nav.fit(P_chk, R0_p, t0_p)
                if bf_c and bf_c.get("ok"):
                    rf_c = ring_fit.fit(P_chk, bf_c["R"], bf_c["t"], min_top=6, spec=ring)
            if rf_c and rf_c.get("ok"):
                ring_seen = True
                return False
            if ring_checked >= 8:
                return True
        return False

    t_rel_sens = float(kd.time)

    def rel_hold_step(p_d: np.ndarray, v_d: np.ndarray, a_d: np.ndarray, R_d: np.ndarray, w_d: np.ndarray, n_steps: int) -> None:
        nonlocal t_rel_sens
        F_w, T_b = np.zeros(3), np.zeros(3)
        t_c = float(kd.time)

        def _tick(t: float) -> None:
            nonlocal t_rel_sens, t_c
            if M.pend:
                M._deliver(t)
            if t + 1e-9 >= t_rel_sens:
                t_rel_sens = t + 1.0
                M.fix()
            if t + 1e-9 >= t_c:
                t_c = t + ctrl_dt
                p, R, v, wb, _ = ctx.bus.bus_state()
                m, I = ctx.bus.bus_mass_inertia()
                F_w[:] = np.clip(m * (a_d + bc.kp_t * (p_d - p) + bc.kd_t * (v_d - v)), -bc.f_max, bc.f_max)
                e_R = Rotation.from_matrix(R.T @ R_d).as_rotvec()
                T_b[:] = I @ (bc.kp_a * e_R + bc.kd_a * (R.T @ w_d - wb)) + np.cross(wb, I @ wb)

        ctx.bus.step(F_w, T_b, ctx.q_app, M.q_open, n_steps, thr_on=True, on_step=_tick)

    def fly_inertial_leg(
        r0_w: np.ndarray,
        r1_w: np.ndarray,
        T_leg: float,
        *,
        align_rel: bool = False,
        quintic_target_b: Optional[np.ndarray] = None,
        init_arr: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None,
        min_r_floor: Optional[float] = None,
        smooth_keepout: bool = False,
        check_ring_at_1hz: bool = False,
    ) -> Optional[str]:
        t_arr = float(kd.time) + T_leg
        v1_w, a1_w = (init_arr[1], init_arr[2]) if init_arr is not None else (np.zeros(3), np.zeros(3))
        if init_arr is not None:
            r1_w = init_arr[0]

        R_d_prev = None
        tau = 0.0
        sec_acc = 0.0
        while tau < T_leg:
            tau = min(T_leg, tau + ctrl_dt)
            u = tau / T_leg
            if quintic_target_b is not None and (t_arr - float(kd.time)) > 4.0:
                Rw_a, _, w_a, _, _ = M.nav.predict(t_arr)
                r1_w = Rw_a @ quintic_target_b
                v1_w = np.cross(w_a, r1_w)
                a1_w = np.cross(w_a, v1_w)

            if quintic_target_b is not None:
                rr, vr, ar = _quintic_intercept(u, T_leg, r0_w, r1_w, v1_w, a1_w)
            else:
                s_, sv, sa = far_nav._mj(u)
                d_w = r1_w - r0_w
                rr = r0_w + s_ * d_w
                vr = (sv / T_leg) * d_w
                ar = (sa / (T_leg * T_leg)) * d_w

            Rw, _, _, vv, cc = M.nav.predict(float(kd.time))
            rn = float(np.linalg.norm(rr))
            # Fix 3: smooth C^1 keep-out from ClientSpec
            r_lim = 0.0
            if smooth_keepout:
                r_lim = max(r_lim, ctx.client.keepout_radius(Rw.T @ rr))
            if min_r_floor is not None:
                r_lim = max(r_lim, float(min_r_floor))
            if r_lim > 0.0 and rn < r_lim:
                ru = rr / max(rn, 1e-9)
                rr = r_lim * ru
                if float(vr @ ru) < 0.0:
                    vr = vr - float(vr @ ru) * ru
                if float(ar @ ru) < 0.0:
                    ar = ar - float(ar @ ru) * ru

            p_d, v_d, a_d = cc + rr, vv + vr, ar
            roll_w = (Rw @ ctx.client.r_rel)[:, 2] if align_rel else state.roll_ref
            R_d = point_R(ctx.bore_b, -rr, roll_w)
            # Fix 5: zero w_d for inertial chords, exact geometric R_d angular rate for roll-aligned legs
            if align_rel and R_d_prev is not None:
                w_d_body_step = Rotation.from_matrix(R_d_prev.T @ R_d).as_rotvec() / ctrl_dt
                wn = float(np.linalg.norm(w_d_body_step))
                if wn > np.radians(5.0):
                    w_d_body_step *= np.radians(5.0) / wn
                w_d = R_d @ w_d_body_step
            else:
                w_d = np.zeros(3)
            R_d_prev = R_d

            rel_hold_step(p_d, v_d, a_d, R_d, w_d, n_ctrl)
            if ctx.bus.bad_contacts or M.bad:
                return "collision"

            if check_ring_at_1hz:
                sec_acc += ctrl_dt
                if sec_acc >= 1.0 - 1e-9:
                    sec_acc = 0.0
                    p_now, _, _, _, _ = ctx.bus.bus_state()
                    Rw_chk, _, _, _, cc_chk = M.nav.predict(float(kd.time))
                    if maybe_check_ring(p_now - cc_chk, Rw_chk):
                        return "no ring at 8m"
        return None

    ctx.bus.phase = M.phase = "xfer_to_hold3"
    d_obs = ctx.config.d_obs
    d_stage = ctx.config.d_stage
    v_xfer = ctx.config.v_xfer

    # 2a) Safe <= 55 deg inertial chords on the 8 m sphere into believed +X ring hemisphere
    p_now, _, _, _, _ = ctx.bus.bus_state()
    _, _, _, _, cc0 = M.nav.predict(float(kd.time))
    r_cur_w = p_now - cc0
    T_in_nom = 42.0
    Rw_p, _, _, _, _ = M.nav.predict(float(kd.time) + T_in_nom)
    r_tgt_w = Rw_p @ r_b_3axis
    ang_off = ang_deg(r_cur_w, r_tgt_w)
    if ang_off > 35.0:
        n_ch_est = max(1, int(np.ceil(ang_off / 50.0)))
        Rw_p2, _, _, _, _ = M.nav.predict(float(kd.time) + n_ch_est * 75.0 + 30.0 + 35.0 + T_in_nom)
        u_end = Rw_p2 @ r_b_3axis
        pts_w = _safe_sphere_chords(r_cur_w, u_end, d_obs, max_step_deg=55.0)
        for a_w, b_w in zip(pts_w[:-1], pts_w[1:]):
            T_leg = max(1.875 * float(np.linalg.norm(b_w - a_w)) / v_xfer, 25.0)
            why_x = fly_inertial_leg(a_w, b_w, T_leg, align_rel=False, min_r_floor=7.0)
            if why_x:
                return f"abort: xfer_to_hold3: {why_x}"

    # 2b) Pre-align roll to Rw @ R_rel at 8 m (up to 30 s) + opportunistic ring_fit check
    p_now, _, _, _, _ = ctx.bus.bus_state()
    _, _, _, _, cc0 = M.nav.predict(float(kd.time))
    r_hold8_w = p_now - cc0
    for sec in range(30):
        Rw, _, w, vv, cc = M.nav.predict(float(kd.time))
        R_d = point_R(ctx.bore_b, -r_hold8_w, (Rw @ ctx.client.r_rel)[:, 2])
        # Pure roll rate around boresight only
        u_los = -r_hold8_w / max(float(np.linalg.norm(r_hold8_w)), 1e-9)
        w_los = float(w @ u_los) * u_los
        rel_hold_step(cc + r_hold8_w, vv, np.zeros(3), R_d, w_los, n_sub)
        if ctx.bus.bad_contacts or M.bad:
            return "abort: xfer_to_hold3: collision"
        if maybe_check_ring(r_hold8_w, Rw):
            return "abort: xfer_to_hold3: no ring at 8m"
        _, R_now, _, _, _ = ctx.bus.bus_state()
        att_e = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_now.T @ R_d).as_rotvec())))
        if sec >= 5 and att_e < 4.0 and (ring_seen or ring_checked == 0 or ring_checked >= 4):
            break

    # 2c) Fix 2: Bracketed T_st solver on 6 m staging sphere + antipodal guard + Fix 4 ring check during hop
    p_now, _, _, _, _ = ctx.bus.bus_state()
    _, _, _, _, cc0 = M.nav.predict(float(kd.time))
    r_cur_w = p_now - cc0
    r_stage_w = None
    T_st = 90.0
    for T_cand in np.linspace(25.0, 120.0, 20):
        Rw_st, _, _, _, _ = M.nav.predict(float(kd.time) + float(T_cand) + T_in_nom)
        u_st = Rw_st @ r_b_3axis
        u_st /= max(float(np.linalg.norm(u_st)), 1e-9)
        pts_cand = _safe_sphere_chords(r_cur_w, u_st, d_stage, max_step_deg=55.0)
        T_req = sum(max(1.875 * float(np.linalg.norm(b - a)) / v_xfer, 20.0) for a, b in zip(pts_cand[:-1], pts_cand[1:]))
        if T_req <= float(T_cand):
            T_st = float(T_cand)
            r_stage_w = pts_cand[-1]
            break
    if r_stage_w is None:
        Rw_st, _, _, _, _ = M.nav.predict(float(kd.time) + T_st + T_in_nom)
        u_st = Rw_st @ r_b_3axis
        u_st /= max(float(np.linalg.norm(u_st)), 1e-9)
    else:
        u_st = r_stage_w / max(float(np.linalg.norm(r_stage_w)), 1e-9)

    st_pts = _safe_sphere_chords(r_cur_w, u_st, d_stage, max_step_deg=55.0)
    for a_w, b_w in zip(st_pts[:-1], st_pts[1:]):
        T_leg = max(1.875 * float(np.linalg.norm(b_w - a_w)) / v_xfer, 20.0)
        why_x = fly_inertial_leg(
            a_w, b_w, T_leg, align_rel=True, min_r_floor=d_stage, smooth_keepout=True, check_ring_at_1hz=True
        )
        if why_x:
            return f"abort: xfer_to_hold3: {why_x}"

    # Consistent quintic intercept from 6 m stage onto moving 3.2 m station
    p_now, _, _, _, _ = ctx.bus.bus_state()
    _, _, _, _, cc0 = M.nav.predict(float(kd.time))
    r_cur_w = p_now - cc0
    T_int, r_arr_w, v_arr_w, a_arr_w = _size_quintic_T(
        M, float(kd.time), r_cur_w, r_b_3axis, a_budget=a_budget, T_min=30.0, T_max=95.0
    )
    why_x = fly_inertial_leg(
        r_cur_w,
        r_arr_w,
        T_int,
        align_rel=True,
        quintic_target_b=r_b_3axis,
        init_arr=(r_arr_w, v_arr_w, a_arr_w),
        smooth_keepout=True,
        check_ring_at_1hz=True,
    )
    if why_x:
        return f"abort: xfer_to_hold3: {why_x}"

    # 2d) Synchronize & pull-in settle at 3.2 m (Fix 5: bore_b -> R_d_st handover once pe < 0.30 m; Fix 6a: wing KO guard)
    ctx.bus.phase = M.phase = "pull_in3"
    M.s, M.sdot, M.thr_on = ctx.client.r_hold3_m, 0.0, True
    M.t_ctrl = float(kd.time)
    M.t_lidar = float(kd.time) + 0.5
    t_pull_end = float(kd.time) + ctx.config.pull_in_max_s
    settled_p3 = 0.0
    handover_pe = ctx.config.pull_in_att_handover_m
    pe_final = 999.0
    ae_final = 999.0

    while float(kd.time) < t_pull_end:
        p_b, _, _, _, _ = ctx.bus.bus_state()
        Rw_p, _, _, _, cc_p = M.nav.predict(float(kd.time))
        # Fix 6a: smooth wing keep-out guard in pull_in3
        r_now_w = p_b - cc_p
        r_ko = ctx.client.keepout_radius(Rw_p.T @ r_now_w, extra_margin=-0.15)
        if float(np.linalg.norm(r_now_w)) < r_ko:
            return "abort: pull_in3: wing_keepout"

        p_d, R_d_st, v_d, w_d, a_ff = M.station(float(kd.time))
        pe_cur = float(np.linalg.norm(p_d - p_b))
        # Fix 5: once close (< 0.30 m), align attitude directly to station attitude R_d_st instead of canted lidar boresight
        if pe_cur < handover_pe:
            R_cmd = R_d_st
        else:
            R_cmd = point_R(ctx.bore_b, cc_p - p_b, R_d_st[:, 2])

        rel_hold_step(p_d, v_d, a_ff, R_cmd, w_d, n_ctrl)
        if ctx.bus.bad_contacts or M.bad:
            return "abort: pull_in3: collision"

        p_b, R_b, _, _, _ = ctx.bus.bus_state()
        pe_final = float(np.linalg.norm(p_d - p_b))
        ae_final = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_b.T @ R_d_st).as_rotvec())))
        if pe_final < 0.08 and ae_final < 2.5:
            settled_p3 += ctrl_dt
            if settled_p3 >= 2.0:
                break
        else:
            settled_p3 = 0.0

    state.ring_seen8 = ring_seen
    ctx.row["phases"]["xfer_to_hold3"] = dict(
        t=round(float(kd.time), 1),
        ang_off_deg=round(ang_off, 1),
        ring_seen8=ring_seen,
        ring_checked=ring_checked,
        T_int_s=round(T_int, 1),
        pull_in_mm=round(pe_final * 1e3, 1),
        pull_in_deg=round(ae_final, 2),
    )
    return None
