"""SPEC_7 G2 envelope evaluation over orbital or plain MJWarp backends with calibrated perception noise."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

RL_DIR = Path(__file__).resolve().parent
if str(RL_DIR.parent) not in sys.path:
    sys.path.insert(0, str(RL_DIR.parent))
from _paths import ensure_runs_dir, find_weight  # noqa: E402

from common import (  # noqa: E402
    DEFAULT_DUTY,
    RUNS,
    StereoNoiseModel,
    classify_outcome,
    load_starts,
    make_eval_grid,
)
from gpu_env import GpuReachEnv  # noqa: E402  (imports mjorbit before torch)
from policy import G3Policy  # noqa: E402


def resolve_noise(perc: str, noise_json: str, t_refit_s: float = 1.0) -> StereoNoiseModel:
    if perc == "orb":
        p = Path(noise_json)
        if p.exists():
            return StereoNoiseModel.from_json(p, t_refit_s=t_refit_s)
        return StereoNoiseModel.preset("stereo_plain", t_refit_s=t_refit_s)
    if perc == "stereo_plain":
        return StereoNoiseModel.preset("stereo_plain", t_refit_s=t_refit_s)
    if perc == "net":
        return StereoNoiseModel.preset("net", t_refit_s=t_refit_s)
    raise ValueError(f"Unsupported perc mode: {perc!r}")


def summarize(env: GpuReachEnv, cells: Sequence[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Per-world rows and per-cell outcome counts for GpuReachEnv (shared with g3_ppo.py)."""
    import numpy as np

    rows: List[Dict[str, Any]] = []
    for i, c in enumerate(cells):
        res = env.result[i] or "timeout"
        bad = bool(env.bad[i]) or bool(env.bad0[i])
        inf = env.info[i]
        rows.append(
            dict(
                cell=c["cell"],
                base=c["base"],
                res=res,
                cls=classify_outcome(res, bad),
                bad_phase=int(env.bad_phase[i]),
                v_peak_mm_s=round(float(env.v_peak[i]), 2),
                peak=inf.get("peak"),
                load=inf.get("load"),
                closed=inf.get("closed"),
                descent=inf.get("descent"),
                trk_max_mm=round(float(env.trk_max[i]) * 1e3, 2),
                trk_abort=inf.get("trk_abort"),
            )
        )
    by: Dict[str, List[Dict[str, Any]]] = {}
    for r in rows:
        by.setdefault(r["cell"], []).append(r)
    summ: List[Dict[str, Any]] = []
    for k, rs in by.items():
        n = len(rs)
        cnt = {x: sum(1 for r in rs if r["cls"] == x) for x in ("captured", "safe abort", "failed close", "collision")}
        ab: Dict[str, int] = {}
        for r in rs:
            if r["cls"] != "captured":
                ab[r["res"]] = ab.get(r["res"], 0) + 1
        vp = [r["v_peak_mm_s"] for r in rs]
        ve = [r["closed"]["client_v_mm_s"] for r in rs if r["closed"]]
        summ.append(
            dict(
                cell=json.loads(k),
                n=n,
                **cnt,
                why=ab,
                v_peak_med=round(float(np.median(vp)), 2),
                v_hold_med=round(float(np.median(ve)), 2) if ve else None,
                v_hold_p90=round(float(np.percentile(ve, 90)), 2) if ve else None,
            )
        )
    return rows, summ


def run_eval(
    cells: List[Dict[str, Any]],
    *,
    seed: int,
    out_path: Path,
    backend: str,
    noise: StereoNoiseModel,
    hold_check: bool,
    policy_path: Optional[str],
    duty: float,
    max_steps: int,
) -> Dict[str, Any]:
    starts = load_starts()
    for i, c in enumerate(cells):
        c["base"] = i % len(starts)

    pol: Optional[G3Policy] = None
    o_max, a_vel, dec = 0.008, 0.008, 50
    if policy_path:
        pol, meta = G3Policy.load(policy_path, device="cuda")
        o_max = float(meta.get("o_max", o_max))
        a_vel = float(meta.get("a_vel", a_vel))
        dec = int(meta.get("dec", dec))

    t0 = time.time()
    env = GpuReachEnv(
        starts,
        cells,
        backend=backend,
        moving=True,
        hold_check=hold_check,
        noise=noise,
        seed=seed,
        duty=duty,
        o_max=o_max,
        a_vel=a_vel,
    )
    t1 = time.time()
    _, cls = env.run(pol, max_steps=max_steps, dec=dec, deterministic=True, collect=False)
    t2 = time.time()

    rows, summ = summarize(env, cells)
    totals = {k: cls.count(k) for k in ("captured", "safe abort", "failed close", "collision")}
    out = dict(
        tag=out_path.stem,
        backend=backend,
        noise=noise.name,
        hold_check=hold_check,
        policy=str(policy_path) if policy_path else None,
        n=len(rows),
        totals=totals,
        init_s=round(t1 - t0, 1),
        run_s=round(t2 - t1, 1),
        steps=env.k_glob,
        cells=summ,
    )
    out_path.write_text(json.dumps(dict(summary=out, rows=rows), indent=1, default=float), encoding="utf-8")
    print(json.dumps({k: v for k, v in out.items() if k != "cells"}), flush=True)
    for s in summ:
        print(json.dumps(s), flush=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("smoke", "orbital", "sweep", "spin"))
    ap.add_argument("--backend", choices=("mjorbit_warp", "plain_warp"), default="mjorbit_warp")
    ap.add_argument("--perc", choices=("orb", "stereo_plain", "net"), default="orb")
    ap.add_argument("--noise-json", default=str(find_weight("stereo_net_in+out_orb.json")))
    ap.add_argument("--hold-check", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--per", type=int, default=None)
    ap.add_argument("--scales", type=float, nargs="*", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duty", type=float, default=DEFAULT_DUTY)
    ap.add_argument("--max-steps", type=int, default=None)
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    noise = resolve_noise(a.perc, a.noise_json)
    hold_check = (a.mode != "orbital") if a.hold_check is None else bool(a.hold_check)
    max_steps = a.max_steps or (18000 if a.mode == "orbital" else 40000)

    if a.mode == "smoke":
        per0 = a.per or 22
        cells = (
            make_eval_grid([0.0], [0.0], [0.0], [0.0], per0)
            + make_eval_grid([1.0], [0.5], [0.0], [1.0], per0 * 2)
            + make_eval_grid([2.0], [1.0], [0.0], [2.0], per0 * 2)
        )
    elif a.mode == "orbital":
        per = a.per or 20
        scales = a.scales or [1.0, 2.0]
        cells = make_eval_grid([0.0, 2.0, 4.0], [0.0, 0.5, 1.0], [0.0], scales, per)
    elif a.mode == "spin":
        per = a.per or 37
        scales = a.scales or [1.0]
        cells = make_eval_grid([2.0, 4.0], [0.0, 0.5, 1.0], [0.0, 1.0, 2.0, 3.0], scales, per)
    else:
        per = a.per or 37
        scales = a.scales or [1.0, 2.0, 3.0]
        cells = make_eval_grid([0.0, 2.0, 4.0], [0.0, 0.5, 1.0], [0.0, 1.0, 2.0, 3.0], scales, per)

    out_path = Path(a.out) if a.out else (ensure_runs_dir() / f"g2_{a.mode}{a.tag}.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    run_eval(
        cells,
        seed=a.seed,
        out_path=out_path,
        backend=a.backend,
        noise=noise,
        hold_check=hold_check,
        policy_path=a.policy,
        duty=a.duty,
        max_steps=max_steps,
    )
    sys.stdout.flush()
    import os

    os._exit(0)


if __name__ == "__main__":
    main()
