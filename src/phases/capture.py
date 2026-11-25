"""Phase 3: close-range approach and ring-tool capture (3.2 m -> locked grip)."""
from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np

import reach_bench as RB4
import ring_fit
import ring_mission as RM

from .context import MissionContext, MissionState


def _stereo_check_with_earth_support(ctx: MissionContext, E0, rad) -> Optional[str]:
    """Run stereo net correction (rendering via earth-bg model if active, else verified RM.stereo_check)."""
    if ctx._bg_wr is None:
        return RM.stereo_check(ctx.M, E0, rad, ctx.row)

    M = ctx.M
    ctx.backend.sync_render()
    rkm, rkd = ctx.backend.render_km, ctx.backend.render_kd

    def _shot_bg(cam_id: int, lamp: float) -> np.ndarray:
        rkm.light_active[ctx._bg_lamp] = 1
        rkm.light_diffuse[ctx._bg_lamp] = lamp
        ctx._bg_wr.update_scene(rkd, camera=cam_id, scene_option=M.rig.opt)
        img = ctx._bg_wr.render().astype(np.float32) * M.rig.gain
        rkm.light_active[ctx._bg_lamp] = 0
        img = np.clip(img + M.rig._nrng.normal(0, M.rig.noise, img.shape), 0, 255)
        return img.mean(-1).astype(np.uint8)

    rad0 = rad
    lamp = float(M.rng.uniform(0.5, 1.0))
    ims = {"in": _shot_bg(ctx._bg_wcam1, lamp), "out": _shot_bg(ctx._bg_wcam2, lamp)}
    Rw1, pw1 = M.nav.predict(M.kd.time)[:2]
    Rs = M.kd.site_xmat[ctx.gsite].reshape(3, 3)
    n_a, rad_a = -Rs[:, 0], Rs[:, 2]
    c_a = M.kd.site_xpos[ctx.gsite] - (ctx.client.ring.h_top + RB4.H_PRE) * n_a - ctx.client.ring.r_grasp * rad_a
    c, n, rad = c_a, n_a, rad_a
    tan = np.cross(n, rad)
    ct, nt = RM.truth_ring(M)
    dd = ct - c
    lab = np.array([dd @ rad, dd @ tan, dd @ n, nt @ rad, nt @ tan])
    unit = np.array([1e3, 1e3, 1e3, np.degrees(1), np.degrees(1)])
    ctx.row["stereo"] = dict(label=np.round(lab * unit, 2).tolist(), frame="camera", earth_bg=True)
    pred = {}
    for v, net in RM.STEREO["nets"].items():
        if not all(k in ims for k in v.split("+")):
            continue
        y, sg = net.predict(np.stack([ims[k] for k in v.split("+")]))
        pred[v] = (y, sg)
        ctx.row["stereo"][v] = dict(err=np.round((y - lab) * unit, 2).tolist(), sig=np.round(sg * unit, 2).tolist())
    y, sg = pred[RM.STEREO["act"]]
    if sg[0] > RM.NET_SIG_MAX:
        return "abort: stereo net uncertain"
    if np.hypot(y[0], y[1]) * 1e3 > RB4.CAM_MAX_MM:
        return "abort: camera and lidar disagree"
    c1 = c + y[0] * rad + y[1] * tan
    c1 = c1 + ((E0[0] - c1) @ E0[1]) * E0[1]
    n1 = n + y[3] * rad + y[4] * tan if RM.STEREO["tilt"] else E0[1]
    n1 = n1 / np.linalg.norm(n1)
    M.Eb = (Rw1.T @ (c1 - pw1), Rw1.T @ n1, Rw1.T @ rad0)
    return None


def run_capture(ctx: MissionContext, state: MissionState) -> Optional[str]:
    """Execute verified ring_mission approach + stereo reach + close & lock."""
    M = ctx.M
    kd = M.kd
    c_b = ctx.client.ring.c_arr
    n_b = ctx.client.ring.n_arr

    # M.plume tracks capture-phase plume pulses only (added to ctx.bus.plume_pulses in finish())
    M.plume = 0
    M.t_ctrl = float(kd.time)
    M.t_lidar = float(kd.time) + 0.5
    M.s, M.sdot, M.thr_on = ctx.client.r_hold3_m, 0.0, True
    M.t_bad = None

    for ph, T, s_to in (
        ("hold3", RM.T_HOLD3, None),
        ("to1.8", abs(ctx.client.r_hold3_m - ctx.client.r_hold18_m) / RM.V_APP + 5, ctx.client.r_hold18_m),
        ("hold1.8", RM.T_HOLD18, None),
        ("to_station", abs(ctx.client.r_hold18_m - ctx.client.r_station_m) / RM.V_APP + 5, ctx.client.r_station_m),
    ):
        ctx.bus.phase = M.phase = ph
        why = M.fly(T, s_to)
        ctx.row["phases"][ph] = dict(t=round(float(kd.time), 1), err_mm=round(M.err[0] * 1e3, 1), err_deg=round(M.err[1], 2))
        if why:
            return f"abort: {ph}: {why}"
        if ph == "hold3" and M.nav.sol is not None:
            why = M.spin_gate(ctx.row)
            if why:
                return f"abort: hold3: {why}"

    ctx.bus.phase = M.phase = "settle"
    t_end = float(kd.time) + RM.T_SETTLE
    while float(kd.time) < t_end:
        why = M.fly(1.0)
        if why:
            return f"abort: settle: {why}"
        if M.err[0] < RM.SETTLE_POS and M.err[1] < RM.SETTLE_DEG:
            break
    ctx.row["phases"]["settle"] = dict(t=round(float(kd.time), 1), err_mm=round(M.err[0] * 1e3, 1), err_deg=round(M.err[1], 2))
    if M.err[0] > 2 * RM.SETTLE_POS or M.err[1] > 2 * RM.SETTLE_DEG:
        return "abort: settle: not settled"

    Rw, pw, _, _, _ = M.nav.predict(float(kd.time))
    E0 = (pw + Rw @ c_b, Rw @ n_b)
    ctx.row["E0"] = RB4.est_err(E0, RM.truth_ring(M))
    a_sec, rad, _ = M.choose(E0, Rw[:, 1])
    if a_sec is None:
        return "abort: no reachable section"
    M.Eb = M.to_body(E0, rad)
    ctx.bus.phase = M.phase = "to pre-reach"
    E0, rad = M.track(E0, rad)
    if M.cart(*M.section(E0, rad, RB4.H_PRE), M.q_open) is None:
        return "collision" if M.bad else "abort: path"
    E0, rad = M.track(E0, rad)

    why_st = _stereo_check_with_earth_support(ctx, E0, rad)
    if why_st:
        return why_st
    E1, rad = M.track(E0, rad)
    ctx.row["E1"] = RB4.est_err(E1, RM.truth_ring(M))
    u1, v1 = ring_fit._frame(E1[1])
    a1 = float(np.arctan2(rad @ v1, rad @ u1))
    g3 = None
    if ctx.config.g3_policy_path:
        if getattr(ctx, "_g3_key", None) != ctx.config.g3_policy_path:
            from rl.policy import G3Controller

            ctx._g3 = G3Controller.load(ctx.config.g3_policy_path, dt=float(M.km.opt.timestep), catch_s=RB4.CATCH_S)
            ctx._g3_key = ctx.config.g3_policy_path
        g3 = ctx._g3
    r_grasp = RB4.reach_close(M, dict(), E1, a1, hold_check=False, policy=g3)
    for k in ("seat_pre", "descent", "closed", "peak"):
        if k in r_grasp:
            ctx.row[k] = r_grasp[k]
    if r_grasp["result"].startswith(("abort", "collision")):
        return r_grasp["result"]
    t_pad = M.touches()
    if not (t_pad["in"] > 5 and (t_pad["hook"] > 5 or t_pad["lip"] > 5) and (t_pad["palm"] > 5 or t_pad["lip"] > 5)):
        return "no grip (sensed)"

    ctx.row["phases"]["capture"] = dict(t=round(float(kd.time), 1), scored=r_grasp["result"])
    return None
