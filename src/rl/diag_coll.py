"""SPEC_7 s25.3: what touches in the stereo base's descent collisions (G2 sweep: 2 / 37 in w_ax 4 / w_tr 1 / tilt 2).

  bash tools/rlrun.sh python ../projects/p7_unprepared_client/rl/diag_coll.py [--per 200]

Runs the w_ax 4 cells (w_tr 0.5 / 1, tilt 0-3) with the stereo error model and the load test off; for every world
with a bad contact prints the first contact's geom pairs, phase, height and the true seat at that moment, then a count
by geom pair. -> runs/diag_coll.json
"""
from __future__ import annotations

import argparse
import collections
import json
import pickle
import sys
from pathlib import Path

import torch

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
from _paths import RUNS_DIR as RUNS, ensure_runs_dir  # noqa: E402
import reach_bench as RBN  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per", type=int, default=50)
    ap.add_argument("--seed", type=int, default=5)
    a = ap.parse_args()
    RBN.FF[0] = True
    import gpu_reach_moving
    from gpu_reach_moving import GpuReachMoving
    from g2_envelope import klass
    gpu_reach_moving.PERC[0] = "stereo"
    starts = [r for r in pickle.loads((RUNS / "g1_starts.pkl").read_bytes()) if "state" in r]
    cells = []
    for wt in (0.5, 1.0):
        for tr in (0.0, 1.0, 2.0, 3.0):
            key = json.dumps(dict(w_ax=4.0, w_tr=wt, tilt_r=tr, scale=1.0))
            cells += [dict(cell=key, w_ax=4.0, w_tr=wt, tilt_r=tr, scale=1.0) for _ in range(a.per)]
    for i, c in enumerate(cells):
        c["base"] = i % len(starts)
    torch.manual_seed(a.seed)
    G = GpuReachMoving(RBN.Bench(), starts, cells, seed=a.seed, hold_check=False)
    G.bad0[:] = torch.tensor([starts[c["base"]]["bad0"] for c in cells], device="cuda")
    G.run(max_steps=30000)
    rows, pairs = [], collections.Counter()
    cnt = collections.Counter()
    for i, c in enumerate(cells):
        res = G.result[i] or "timeout"
        k = klass(res, bool(G.bad[i]) or bool(G.bad0[i]))
        cnt[k] += 1
        if k != "collision":
            continue
        inf = G.info[i]
        r = dict(cell=c["cell"], base=c["base"], res=res, bad0=bool(G.bad0[i]), bad=inf.get("bad_contact"),
                 seat_bad=inf.get("seat_bad"), seat_pre=inf.get("seat_pre"), descent=inf.get("descent"),
                 trips=inf.get("trips"), v_peak=round(float(G.v_peak[i]), 1))
        rows.append(r)
        for p in (r["bad"] or {}).get("pairs", []):
            pairs[tuple(sorted(p[:2]))] += 1
        print(json.dumps(r), flush=True)
    print(json.dumps(dict(n=len(cells), classes=cnt)), flush=True)
    for p, n in pairs.most_common():
        print(n, p, flush=True)
    (RUNS / "diag_coll.json").write_text(json.dumps(dict(n=len(cells), classes=cnt, pairs=[[list(p), n] for p, n in
                                                                                        pairs.most_common()], rows=rows), indent=1))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
