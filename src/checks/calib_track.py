"""SPEC_7 section 24, G2 calibration: how the mission's grasp-point estimate moves during the reach (to fit the GPU
env's error model, rl/gpu_reach_moving.py).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/calib_track.py --seeds 0 1 2 3 4

The ring mission (the multi-rate setup of section 23: 10 Hz fixes, 2 Hz refit over <= 40, monitor on, the ring net)
with Mission.track wrapped: at every call with the estimate in the client body set (from the pre-reach on), the
estimated ring top at the grasp section against the TRUE ring top at the same section (the true radial: the
estimate's radial projected on the true ring plane), in the section frame (radial, tangential, axial, mm), the axis
error (deg), the time and the phase. Truth is read for the record only. -> runs/calib_track_<seed>.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
import ring_mission as RM  # noqa: E402
import ring_fit  # noqa: E402
import reach_bench as RB4  # noqa: E402

LOG = []
_orig_track = RM.Mission.track


def track(self, E, rad):
    out = _orig_track(self, E, rad)
    if getattr(self, "Eb", None) is not None:
        (c, n), r = out
        ct, nt = RM.truth_ring(self)
        rt = r - (r @ nt) * nt
        rt /= np.linalg.norm(rt)
        pe = c + ring_fit.H_TOP * n + RB4.RING_R * r
        pt = ct + ring_fit.H_TOP * nt + RB4.RING_R * rt
        e = pe - pt
        tan = np.cross(nt, rt)
        LOG.append((float(self.kd.time), self.phase, float(e @ rt) * 1e3, float(e @ tan) * 1e3, float(e @ nt) * 1e3,
                    float(np.degrees(np.arccos(np.clip(n @ nt, -1, 1))))))
    return out


RM.Mission.track = track


def job(seed):
    LOG.clear()
    r = RM._job(seed)
    out = dict(seed=seed, result=r.get("result"), log=list(LOG))
    (ensure_runs_dir() / f"calib_track_{seed}.json").write_text(json.dumps(out))
    return r.get("result")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[0, 1, 2, 3, 4])
    ap.add_argument("--workers", type=int, default=5)
    a = ap.parse_args()
    RM.LIDAR_DT, RM.FIT_DT[0], RM.FIT_MAX[0], RM.INNOV[0], RM.USE_CAM[0] = 0.1, 0.5, 40, "on", "net"
    seeds = [7600000 + s for s in a.seeds]
    from multiprocessing import Pool
    with Pool(min(a.workers, 5)) as pool:
        res = pool.map(job, seeds, chunksize=1)
    print(res, flush=True)
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
