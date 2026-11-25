"""SPEC_7 section 24, G1 check (R2): the batched GPU base (rl/gpu_reach.py, a = 0) against the CPU bench, same cases.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/rl/g1_check.py record --n 25
  bash tools/rlrun.sh python ../projects/p7_unprepared_client/rl/g1_check.py gpu

record  reach_bench.case on the CPU (the scene as is, noslip 20) for bench seeds 7400000 + i; at the entry of
        reach_close the full state, the bus pin, E1, a1 and whether a bad contact happened before are saved, then the
        CPU reach runs on: its row is the reference -> runs/g1_starts.pkl
gpu     all cases that reached reach_close as one batch of worlds on the GPU; per case the result class, the seat
        after the close, the slip under load, the descent stop and height, against the CPU -> runs/g1_check.json
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
from _paths import RUNS_DIR as RUNS, ensure_runs_dir  # noqa: E402
import reach_bench as RBN  # noqa: E402

SEEDS = []


def record(seeds):
    B = RBN.Bench()
    orig = RBN.reach_close
    out = []
    for s in seeds:
        snap = {}

        def wrap(B_, row, E1, a1, **kw):
            spec = mujoco.mjtState.mjSTATE_INTEGRATION
            st = np.zeros(mujoco.mj_stateSize(B_.km, spec))
            mujoco.mj_getState(B_.km, B_.kd, st, spec)
            snap.update(state=st, pin=B_.pin_bus.copy(), c=np.array(E1[0]), n=np.array(E1[1]), a1=float(a1),
                        bad0=bool(B_.bad))
            return orig(B_, row, E1, a1, **kw)

        RBN.reach_close = wrap
        t0 = time.time()
        row = RBN.case(B, 7400000 + s)
        RBN.reach_close = orig
        out.append(dict(seed=s, row=row, wall_s=round(time.time() - t0, 1), **snap))
        print(s, row["result"], flush=True)
    (RUNS / "g1_starts.pkl").write_bytes(pickle.dumps(out))


def cls(r):
    if r is None:
        return None
    return r.split(" (")[0] if r.startswith("abort: descent") else r


def gpu():
    from gpu_reach import GpuReach
    import torch
    recs = [r for r in pickle.loads((RUNS / "g1_starts.pkl").read_bytes()) if "state" in r]
    if SEEDS:
        recs = [r for r in recs if r["seed"] in SEEDS]
    B = RBN.Bench()
    t0 = time.time()
    G = GpuReach(B, recs)
    G.bad0[:] = torch.tensor([r["bad0"] for r in recs], device="cuda")
    t1 = time.time()
    G.run()
    t2 = time.time()
    rows, agree = [], 0
    for i, r in enumerate(recs):
        cr, gr = r["row"], G.outcome(i)
        gi = G.info[i]
        row = dict(seed=r["seed"], cpu=cr["result"], gpu=gr,
                   cpu_desc=cr.get("descent", {}).get("stop"), gpu_desc=gi.get("descent", {}).get("stop"),
                   cpu_h=cr.get("descent", {}).get("h_mm"), gpu_h=gi.get("descent", {}).get("h_mm"),
                   cpu_seat=cr.get("closed", {}).get("seat"), gpu_seat=gi.get("closed", {}).get("seat"),
                   cpu_load=cr.get("load"), gpu_load=gi.get("load"),
                   cpu_pre=cr.get("seat_pre"), gpu_pre=gi.get("seat_pre"),
                   gpu_bad_phase=int(G.bad_phase[i]), gpu_trips=gi.get("trips"))
        row["same"] = cls(cr["result"]) == cls(gr)
        agree += row["same"]
        rows.append(row)
        print(json.dumps(row, default=float), flush=True)
    summ = dict(n=len(rows), same_class=agree, init_s=round(t1 - t0, 1), run_s=round(t2 - t1, 1),
                cpu={k: sum(1 for x in rows if x["cpu"] == k) for k in sorted({x["cpu"] for x in rows})},
                gpu={k: sum(1 for x in rows if x["gpu"] == k) for k in sorted({str(x["gpu"]) for x in rows})})
    print(json.dumps(summ, indent=1))
    (RUNS / "g1_check.json").write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float))


def speed(nw):
    """The same starts repeated to nw worlds: wall time per physics step and per world, and whether the copies agree."""
    from gpu_reach import GpuReach
    import torch
    recs = [r for r in pickle.loads((RUNS / "g1_starts.pkl").read_bytes()) if "state" in r]
    reps = [recs[i % len(recs)] for i in range(nw)]
    B = RBN.Bench()
    G = GpuReach(B, reps)
    G.bad0[:] = torch.tensor([r["bad0"] for r in reps], device="cuda")
    n = [0]
    orig = G.physics_step

    def counted():
        n[0] += 1
        orig()
    G.physics_step = counted
    torch.cuda.synchronize()
    t0 = time.time()
    G.run()
    torch.cuda.synchronize()
    w = time.time() - t0
    res = [G.outcome(i) for i in range(nw)]
    same = sum(res[i] == res[i % len(recs)] for i in range(nw))
    out = dict(nworld=nw, steps=n[0], wall_s=round(w, 1), ms_per_step=round(w / n[0] * 1e3, 2),
               env_steps_per_s=round(n[0] * nw / w), copies_agree=f"{same}/{nw}",
               results={k: res.count(k) for k in sorted(set(map(str, res)))})
    print(json.dumps(out), flush=True)
    odd = [dict(world=i, seed=reps[i]["seed"], res=res[i], ref=res[i % len(recs)], info=G.info[i])
           for i in range(nw) if res[i] != res[i % len(recs)]]
    (RUNS / f"g1_speed_{nw}.json").write_text(json.dumps(dict(summary=out, odd=odd), indent=1, default=float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("record", "gpu", "speed"))
    ap.add_argument("--worlds", type=int, nargs="*", default=[1024, 4096])
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--seeds", type=int, nargs="*", default=[])
    a = ap.parse_args()
    SEEDS[:] = a.seeds
    if a.mode == "record":
        record(range(a.n))
    elif a.mode == "speed":
        for nw in a.worlds:
            speed(nw)
    else:
        gpu()
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
