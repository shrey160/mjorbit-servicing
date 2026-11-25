"""Thin modular end-to-end P7 mission orchestrator (plain MuJoCo or full mjorbit + Earth/stars).

  MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    bash tools/rlrun.sh python ../projects/p7_unprepared_client/mission/run_e2e.py \
      [--backend plain|mjorbit] [--earth-bg] [--client p7] [--n 10] [--workers 3]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path
from typing import Any, Dict

import numpy as np

HERE = Path(__file__).resolve().parent
SRC_DIR = HERE.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
from _paths import RUNS_DIR  # noqa: E402

from phases import (  # noqa: E402
    MissionContext,
    run_acq8,
    run_capture,
    run_detumble,
    run_far_leg,
    run_release,
    run_xfer,
)
from specs import MissionConfig, client_p7, servicer_rev2  # noqa: E402


def _pct(xs, q: float):
    return round(float(np.percentile(xs, q)), 3) if xs else None


def run_episode(ctx: MissionContext, seed: int) -> Dict[str, Any]:
    state = ctx.reset_episode(seed)
    for step_fn in (run_far_leg, run_acq8, run_xfer, run_capture, run_detumble):
        why = step_fn(ctx, state)
        if why is not None:
            return ctx.finish(why)
    return ctx.finish(run_release(ctx, state))


def _worker_job(args):
    seed, cfg, client_name = args
    global _CTX
    if "_CTX" not in globals():
        c_spec = {"p7": client_p7}[client_name]()
        s_spec = servicer_rev2()
        _CTX = MissionContext(c_spec, s_spec, cfg)
    try:
        r = run_episode(_CTX, seed)
    except Exception as e:  # noqa: BLE001
        import traceback

        r = dict(seed=seed, result=f"error: {e!r}", tb=traceback.format_exc())
    print(json.dumps(r, default=float), flush=True)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["plain", "mjorbit"], default="plain")
    ap.add_argument("--earth-bg", action="store_true", help="Render Earth limb + starfield + orbital sun in cameras (with --backend mjorbit)")
    ap.add_argument("--orbit-alt-km", type=float, default=550.0)
    ap.add_argument("--orbit-inc-deg", type=float, default=97.6)
    ap.add_argument("--client", choices=["p7"], default="p7")
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--seed-base", type=int, default=7710000)
    ap.add_argument("--seeds", type=int, nargs="*", default=None)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--no-stereo-tilt", action="store_true")
    ap.add_argument("--g3-policy", default=None, help="Optional G3 RL residual checkpoint for descent control")
    ap.add_argument("--out", default=str(RUNS_DIR / "e2e_modular.json"))
    a = ap.parse_args()

    cfg = MissionConfig(
        backend=a.backend,
        earth_bg=bool(a.earth_bg),
        orbit_alt_km=float(a.orbit_alt_km),
        orbit_inc_deg=float(a.orbit_inc_deg),
        stereo_tilt=not a.no_stereo_tilt,
        g3_policy_path=a.g3_policy,
    )
    t0 = time.time()
    seeds = a.seeds if a.seeds else [a.seed_base + i for i in range(a.start, a.start + a.n)]
    jobs = [(s, cfg, a.client) for s in seeds]
    if a.workers > 1:
        with Pool(min(a.workers, 3)) as pool:
            rows = pool.map(_worker_job, jobs, chunksize=1)
    else:
        rows = [_worker_job(j) for j in jobs]

    res = [r["result"] for r in rows]
    comp = [r for r in rows if r["result"] == "complete"]
    summ = dict(
        backend=cfg.backend,
        earth_bg=cfg.earth_bg,
        client=a.client,
        n=len(rows),
        results={k: res.count(k) for k in sorted(set(res))},
        collisions=sum(1 for r in rows if r.get("contacts_bad")),
        prop_g_p50_95=[_pct([r["prop_g"] for r in comp], 50), _pct([r["prop_g"] for r in comp], 95)],
        rel_dv_p50_95=[
            _pct([r["release"]["dv_mm_s"] for r in comp], 50),
            _pct([r["release"]["dv_mm_s"] for r in comp], 95),
        ],
        post_open_contacts=sum(1 for r in comp if r["release"]["post_open_contact_steps"] > 0),
        wall_s=round(time.time() - t0, 1),
    )
    print("SUMMARY:", json.dumps(summ, indent=1))
    out_p = Path(a.out)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float), encoding="utf-8")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
