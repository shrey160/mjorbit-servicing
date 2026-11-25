"""Section 23 fault checks of the innovation monitor (ring mission, multi-rate sensing).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/mr_faults.py --seed 4 --workers 4

Cases (fault x monitor off / on, same seed):
  lidar: from 1 s after the descent starts, for 2 s, every ring fit's centre is biased 40 mm (lidar frame, a fixed
         direction across the ring axis) - a sensor fault the filter would follow;
  kick:  1 s after the descent starts, a 0.1 s force on the client's COM across its ring axis giving ~10 mm/s - the
         client no longer moves as predicted.
Wanted with the monitor on: a safe abort before contact, no collision. The faults are injected by truth-side code
(sensor / world simulation); the monitor and the reach read only own data.
Writes runs/mr_faults.json.
"""
from __future__ import annotations

import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
import ring_mission as RM  # noqa: E402

BIAS_M, KICK_V = 0.040, 0.010


def run(job):
    fault, innov, seed, cfg = job
    import torch
    torch.set_num_threads(1)
    from ringcam_net import RingCamNet
    RM.NET[0] = RingCamNet.load()
    RM.USE_CAM[0] = "net"
    RM.LIDAR_DT = 1.0 / cfg["lidar_hz"]
    RM.FIT_DT[0], RM.FIT_MAX[0], RM.INNOV[0] = 1.0 / cfg["fit_hz"], cfg["fit_max"], innov
    RM.G_MM[0], RM.G_DEG[0] = cfg["g_mm"], cfg["g_deg"]
    M = RM.Mission()
    st = dict(t0=None, kicked=False, biased=0)
    fit0 = RM.ring_fit.fit

    def fit(P, R0, t0):
        r = fit0(P, R0, t0)
        if fault == "lidar" and st["t0"] is not None and r["ok"]:
            t = M.kd.time - st["t0"]
            if 1.0 <= t <= 3.0:
                n = r["n"] / np.linalg.norm(r["n"])
                d = np.cross(n, [1.0, 0.0, 0.0])
                r = dict(r, c=r["c"] + BIAS_M * d / np.linalg.norm(d))
                st["biased"] += 1
        return r
    RM.ring_fit.fit = fit

    bs0 = RM.Mission._before_step
    def before(self):
        if self.phase == "descent" and st["t0"] is None:
            st["t0"] = self.kd.time
        if fault == "kick" and st["t0"] is not None:
            t = self.kd.time - st["t0"]
            kd, b = self.kd, self.bid_t
            if 1.0 <= t < 1.1:
                n = kd.xmat[b].reshape(3, 3) @ RM.ring_fit.N_B
                d = np.cross(n, [0.0, 0.0, 1.0])
                m = float(self.km.body_subtreemass[b])
                kd.xfrc_applied[b, :3] = m * KICK_V / 0.1 * d / np.linalg.norm(d)
                st["kicked"] = True
            elif st["kicked"]:
                kd.xfrc_applied[b, :3] = 0.0
        bs0(self)
    M._before_step = before.__get__(M)

    r = RM.episode(M, 7600000 + seed)
    out = dict(fault=fault, innov=innov, seed=seed, result=r["result"], anomaly=r.get("anomaly"),
               t_descent=st["t0"], biased_fixes=st["biased"], fixes=r.get("fixes"), descent=r.get("descent"),
               bad=r.get("contacts_bad"), seat=(r.get("closed") or {}).get("seat"), peak=r.get("peak"),
               client_v_mm_s=r.get("client_v_mm_s"), wall_s=r.get("wall_s"))
    print(json.dumps(out, default=float), flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--lidar-hz", type=float, default=10.0)
    ap.add_argument("--fit-hz", type=float, default=2.0)
    ap.add_argument("--fit-max", type=int, default=40)
    ap.add_argument("--g-mm", type=float, default=RM.G_MM[0])
    ap.add_argument("--g-deg", type=float, default=RM.G_DEG[0])
    a = ap.parse_args()
    cfg = dict(lidar_hz=a.lidar_hz, fit_hz=a.fit_hz, fit_max=a.fit_max, g_mm=a.g_mm, g_deg=a.g_deg)
    jobs = [(f, m, a.seed, cfg) for f in ("lidar", "kick") for m in ("off", "on")]
    with Pool(min(a.workers, 6)) as pool:
        rows = pool.map(run, jobs, chunksize=1)
    (ensure_runs_dir() / "mr_faults.json").write_text(json.dumps(dict(cfg=cfg, rows=rows), indent=1, default=float))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
