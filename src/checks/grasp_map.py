"""P7.5 first pass: grasp-boundary map of ring tool v3 on a free, 300 kg client (SPEC_7 section 7: the jaws closing on
the flange of a free-floating client over position offset, angle and relative rate; record a good grip, peak force,
client push; gate: a region wide enough for the measured close-range errors, loads under the limits).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/grasp_map.py [--only k ...]

Each case is the P7.4 reach (reach_bench.reach_close: pre-close 75 mm above the estimated top, guarded descent at
10 mm/s, palm-only close, lead-screw lock, 3 N m hold check) on an estimate that is the truth plus ONE imposed error
(the truth is used here to set the error, which is the map's input; nothing in the reach reads it):
  dr      radial offset of the estimated ring centre along the grasp section's radial (+ = tool outboard, hook side)
  tilt_t  estimated axis tilted about the section's tangent (the tool leans radially), deg
  tilt_r  estimated axis tilted about the section's radial (the tool leans along the ring), deg
  w_ax    client rate about the ring axis from the start of the descent (deg/s; the residual after synchronisation)
  w_tr    client rate about the grasp section's tangent (tilts the ring face under the tool), deg/s
  v_jaw   jaw closing speed, mm/s (default 10; lead screws; 0 = a step command, as tool_gate)
Station: bench case 7400001 (fly-in and mount errors play no part: the estimate is built from the truth). Loads from
the start of the descent: the largest single tool-on-ring contact force, each touch pad's peak load, and the peak arm
joint torque as a fraction
of its limit (60 / 60 / 40 / 12 / 12 / 12 N m). Ring load limit: none published for a 610 mm ring's lip; F_LIM is a
conservative stand-in (recorded, not a requirement). Writes runs/grasp_map.json.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
import reach_bench as R  # noqa: E402
import ring_fit  # noqa: E402

SEED = 7400001
F_LIM = 1000.0          # N, peak tool-on-ring force stand-in (lip of a 610 mm ring, local; conservative)

CASES = ([dict(dr=d) for d in (-20, -16, -12, -8, -4, 0, 4, 8, 12, 16, 20)]
         + [dict(tilt_t=t) for t in (-4, -2, 2, 4)] + [dict(tilt_r=t) for t in (2, 4)]
         + [dict(w_ax=w) for w in (0.5, 1.0, 2.0)] + [dict(w_tr=w) for w in (0.25, 0.5, 1.0)]
         + [dict(dr=8, tilt_t=2, w_tr=0.5), dict(dr=-8, tilt_t=-2, w_ax=1.0)]
         + [dict(v_jaw=v) for v in (5, 20, 40, 0)])          # jaw closing speed, mm/s (0: the old step command)


def run_case(B, k, c):
    B.bad, B.phase = set(), "start"
    B.arm_guard(False)
    B.peak = None
    B.place_station(SEED)
    row = dict(case=k, **c)
    truth = (ring_fit.C_B.copy(), ring_fit.N_B.copy())           # client at identity at the start
    wing = np.array([0.0, 1.0, 0.0])                              # client +Y (identity)
    a, rad, _ = B.choose(truth, wing)
    if a is None:
        row["result"] = "abort: no reachable section"
        return row
    tan = np.cross(truth[1], rad)
    Rt = (Rotation.from_rotvec(np.radians(c.get("tilt_t", 0.0)) * tan) *
          Rotation.from_rotvec(np.radians(c.get("tilt_r", 0.0)) * rad))
    est = (truth[0] + c.get("dr", 0.0) * 1e-3 * rad, Rt.apply(truth[1]))
    row["az_deg"] = round(float(np.degrees(a)), 1)
    B.phase = "to pre-reach"
    if B.cart(*B.section(est, rad, R.H_PRE), B.q_open) is None:
        row["result"] = "abort: path"
        return row

    def spin(B):
        w = np.radians(c.get("w_ax", 0.0)) * truth[1] + np.radians(c.get("w_tr", 0.0)) * tan
        B.kd.qvel[B.tv + 3:B.tv + 6] = B.kd.xmat[B.bid_t].reshape(3, 3).T @ w     # free joint: body-frame rate

    R.V_JAW[0] = c.get("v_jaw", 10) * 1e-3
    row = R.reach_close(B, row, est, a, on_descent=spin)
    row["client_w_deg_s"] = round(float(np.degrees(np.linalg.norm(B.kd.qvel[B.tv + 3:B.tv + 6]))), 2)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, nargs="*", help="case indices")
    ap.add_argument("--out", default=str(RUNS_DIR / "grasp_map.json"))
    a = ap.parse_args()
    R.NO_CAM[0] = True
    B = R.Bench()
    rows = []
    for k, c in enumerate(CASES):
        if a.only and k not in a.only:
            continue
        r = run_case(B, k, c)
        rows.append(r)
        pk = r.get("peak", {})
        print(json.dumps(dict(case=k, err=c, result=r["result"], seat_pre=r.get("seat_pre"), stop=r.get("descent", {}).get("stop"),
                              seat=r.get("closed", {}).get("seat"), load=r.get("load"), peak=pk,
                              push=r.get("closed", {}).get("client_v_mm_s", r.get("client_v_mm_s")),
                              F_over=pk.get("contact_F_N", 0) > F_LIM, bad=r.get("contacts_bad"))), flush=True)
    out_p = Path(a.out)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(dict(F_LIM=F_LIM, rows=rows), indent=1, default=float))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
