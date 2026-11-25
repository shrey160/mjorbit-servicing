"""Phase 4: Post-capture stack rate damping on rev-2 wheels (Fix 6b: 180 s cap for 2 deg/s stacks)."""
from __future__ import annotations

from typing import Optional

import numpy as np

from .context import MissionContext, MissionState


def run_detumble(ctx: MissionContext, state: MissionState) -> Optional[str]:
    M = ctx.M
    kd = M.kd
    bc = ctx.servicer.bus_ctrl
    dt = ctx.backend.dt
    n_ctrl = int(round(bc.ctrl_dt / dt))

    ctx.bus.phase = M.phase = "detumble"
    state.q_lock_arm = kd.qpos[M.arm].copy()
    state.q_lock_jaw = kd.qpos[M.gq].copy()
    sq_cmd = np.asarray(ctx.servicer.tool.jaw_close_signs, dtype=float) * M.sq

    t_det0 = float(kd.time)
    settled_s = 0.0
    grip_lost = False
    max_dur = float(bc.detumble_max_s)

    while float(kd.time) - t_det0 < max_dur:
        _, _, _, wb, _ = ctx.bus.bus_state()
        T_damp = -float(bc.detumble_gain) * wb
        ctx.bus.step(np.zeros(3), T_damp, state.q_lock_arm, sq_cmd, n_ctrl, thr_on=True)
        t_pad = ctx.bus.touches()
        if not (t_pad["in"] > 5 and (t_pad["hook"] > 5 or t_pad["lip"] > 5)):
            grip_lost = True
            break
        if float(np.degrees(np.linalg.norm(wb))) < bc.detumble_tol_deg_s:
            settled_s += bc.ctrl_dt
            if settled_s >= bc.detumble_hold_s:
                break
        else:
            settled_s = 0.0

    _, _, _, wb_det, _ = ctx.bus.bus_state()
    ctx.row["phases"]["detumble"] = dict(
        t=round(float(kd.time), 1),
        dur_s=round(float(kd.time) - t_det0, 1),
        w_end_deg_s=round(float(np.degrees(np.linalg.norm(wb_det))), 3),
        grip_lost=grip_lost,
    )
    if grip_lost:
        return "lost grip in detumble"
    return None
