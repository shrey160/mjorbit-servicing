"""Orbital G3 PPO residual descent trainer and evaluator on mjorbit_warp + StereoNet_orb noise."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

RL_DIR = Path(__file__).resolve().parent
if str(RL_DIR.parent) not in sys.path:
    sys.path.insert(0, str(RL_DIR.parent))
from _paths import ensure_runs_dir, find_weight  # noqa: E402

from common import DEFAULT_DUTY, GPUThermalStop, RUNS, gpu_temp, load_starts, make_eval_grid  # noqa: E402
from g2_envelope import resolve_noise, summarize  # noqa: E402
from gpu_env import GpuReachEnv  # noqa: E402  (imports mjorbit before torch)
from policy import G3Policy  # noqa: E402
import torch  # noqa: E402
from torch import nn  # noqa: E402

DEC = 50
R_CAP, R_FAIL, R_COLL = 1.0, -1.0, -3.0


def cells_for(n: int, rng: np.random.Generator, level: float, scale_lo: float = 0.8, scale_hi: float = 2.0) -> List[Dict[str, Any]]:
    wa = rng.uniform(0.0, 2.0 + 2.5 * level, n)
    wt = rng.uniform(0.0, 1.0, n)
    tr = rng.uniform(0.0, 1.0 + 2.0 * level, n)
    sc = rng.uniform(scale_lo, scale_hi, n)
    return [dict(cell="train", w_ax=float(a), w_tr=float(b), tilt_r=float(c), scale=float(d)) for a, b, c, d in zip(wa, wt, tr, sc)]


def compute_returns(
    buf: Dict[str, torch.Tensor],
    cls: List[str],
    *,
    c_act: float,
    gamma: float = 0.995,
    lam: float = 0.95,
):
    T, N = buf["valid"].shape
    dev = buf["val"].device
    rt = {"captured": R_CAP, "failed close": R_FAIL, "collision": R_COLL, "safe abort": 0.0}
    r_end = torch.tensor([rt[c] for c in cls], dtype=torch.float32, device=dev)
    valid = buf["valid"]
    nxt = torch.cat([valid[1:], torch.zeros(1, N, dtype=torch.bool, device=dev)])
    last = valid & ~nxt
    rew = -c_act * (buf["act"] ** 2).sum(-1) * valid + r_end[None] * last
    adv = torch.zeros(T, N, dtype=torch.float32, device=dev)
    a_next = torch.zeros(N, dtype=torch.float32, device=dev)
    v_next = torch.zeros(N, dtype=torch.float32, device=dev)
    for t in range(T - 1, -1, -1):
        cont = (~last[t]).float()
        delta = rew[t] + gamma * v_next * cont - buf["val"][t]
        a_next = delta + gamma * lam * cont * a_next
        adv[t] = torch.where(valid[t], a_next, torch.zeros_like(a_next))
        v_next = torch.where(valid[t], buf["val"][t], v_next)
        a_next = torch.where(valid[t], a_next, torch.zeros_like(a_next))
    return adv, adv + buf["val"]


def ppo_update(
    ac: G3Policy,
    opt: torch.optim.Optimizer,
    buf: Dict[str, torch.Tensor],
    adv: torch.Tensor,
    ret: torch.Tensor,
    *,
    epochs: int = 5,
    mb: int = 4096,
    clip: float = 0.2,
    c_ent: float = 0.005,
) -> Dict[str, float]:
    v = buf["valid"].flatten()
    o = buf["obs"].flatten(0, 1)[v]
    a = buf["act"].flatten(0, 1)[v]
    lp = buf["logp"].flatten(0, 1)[v]
    A = adv.flatten()[v]
    R = ret.flatten()[v]
    A = (A - A.mean()) / (A.std() + 1e-8)
    n = len(o)
    st = dict(pi=0.0, v=0.0, ent=0.0, kl=0.0, k=0)
    for _ in range(epochs):
        perm = torch.randperm(n, device=o.device)
        for i in range(0, n, mb):
            b = perm[i : i + mb]
            d = ac.dist(o[b])
            lpn = d.log_prob(a[b]).sum(-1)
            ent = d.entropy().sum(-1).mean()
            ratio = (lpn - lp[b]).exp()
            l_pi = -torch.min(ratio * A[b], ratio.clamp(1.0 - clip, 1.0 + clip) * A[b]).mean()
            l_v = ((ac.value(o[b]) - R[b]) ** 2).mean()
            loss = l_pi + 0.5 * l_v - c_ent * ent
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(ac.parameters(), 1.0)
            opt.step()
            st["pi"] += float(l_pi)
            st["v"] += float(l_v)
            st["ent"] += float(ent)
            st["kl"] += float((lp[b] - lpn).mean())
            st["k"] += 1
    k = max(st["k"], 1)
    return {m: round(val / k, 4) for m, val in st.items() if m != "k"} | dict(n_dec=n)


def train(args: argparse.Namespace) -> None:
    ensure_runs_dir()
    starts = load_starts()
    noise = resolve_noise(args.perc, args.noise_json)
    ac = G3Policy().cuda()
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    ckpt_p = RUNS / f"g3_ppo{args.tag}.ckpt"
    pt_p = RUNS / f"g3_ppo{args.tag}.pt"
    log_p = RUNS / f"g3_ppo{args.tag}.log.jsonl"
    meta = dict(
        o_max=args.o_max,
        a_vel=args.a_vel,
        dec=DEC,
        backend=args.backend,
        noise=noise.name,
        c_act=args.c_act,
        c_ent=args.c_ent,
    )

    it0 = 0
    best_score = -1e9
    if ckpt_p.exists() and not args.fresh:
        ck = torch.load(ckpt_p, map_location="cuda", weights_only=False)
        ac.load_state_dict(ck["ac"])
        opt.load_state_dict(ck["opt"])
        it0 = int(ck["it"]) + 1
        best_score = float(ck.get("best_score", -1e9))
        print(f"Resumed after iteration {ck['it']}", flush=True)

    # Instantiate persistent orbital GPU env once; reset() reuses CUDA graph & Warp allocations.
    rng0 = np.random.default_rng(1_000_000)
    cells0 = cells_for(args.worlds, rng0, 0.0)
    for i, c in enumerate(cells0):
        c["base"] = i % len(starts)
    env = GpuReachEnv(
        starts,
        cells0,
        backend=args.backend,
        moving=True,
        hold_check=False,
        noise=noise,
        seed=1_000_000,
        duty=args.duty,
        o_max=args.o_max,
        a_vel=args.a_vel,
    )

    if not ckpt_p.exists() or args.fresh:
        log_p.unlink(missing_ok=True)
    try:
        for it in range(it0, args.iters):
            t0 = time.time()
            level = min(1.0, it / max(1, args.iters // 4))
            rng = np.random.default_rng(1_000_000 + it)
            cells = cells_for(args.worlds, rng, level)
            for i, c in enumerate(cells):
                c["base"] = i % len(starts)
            if it != 0:
                env.reset(cells, seed=1_000_000 + it)

            buf, cls = env.run(
                ac,
                max_steps=args.max_steps,
                dec=DEC,
                deterministic=False,
                collect=True,
                early_stop_frac=args.early_stop_frac,
            )
            steps = env.k_glob
            cnt = {k: cls.count(k) for k in ("captured", "safe abort", "failed close", "collision")}
            if buf is None:
                print(json.dumps(dict(it=it, level=round(level, 2), skipped="no_desc", **cnt)), flush=True)
                continue

            adv, ret = compute_returns(buf, cls, c_act=args.c_act)
            st = ppo_update(ac, opt, buf, adv, ret, c_ent=args.c_ent)
            score = cnt["captured"] - 3.0 * cnt["collision"] - cnt["failed close"]
            row = dict(
                it=it,
                level=round(level, 2),
                **cnt,
                steps=steps,
                s=round(time.time() - t0),
                gpu_c=gpu_temp(),
                std=[round(x, 3) for x in ac.log_std.exp().tolist()],
                **st,
            )
            print(json.dumps(row), flush=True)
            with open(log_p, "a", encoding="utf-8") as f:
                f.write(json.dumps(row) + "\n")

            tmp = ckpt_p.with_suffix(".tmp")
            torch.save(dict(ac=ac.state_dict(), opt=opt.state_dict(), it=it, best_score=best_score, meta=meta), tmp)
            tmp.replace(ckpt_p)
            if level >= 0.99 and score >= best_score:
                best_score = score
                ac.save(pt_p, meta={**meta, "it": it, "cnt": cnt})
        # Ensure final .pt exists even for very short runs (level < 1.0)
        if not pt_p.exists():
            ac.save(pt_p, meta=meta)
    except GPUThermalStop as exc:
        print(f"THERMAL_STOP: {exc}", flush=True)
        if not pt_p.exists():
            ac.save(pt_p, meta=meta)
        raise


def evaluate(args: argparse.Namespace) -> None:
    starts = load_starts()
    noise = resolve_noise(args.perc, args.noise_json)
    cells = make_eval_grid([0.0, 2.0, 4.0], [0.0, 0.5, 1.0], [0.0], [1.0, 2.0], args.per)
    for i, c in enumerate(cells):
        c["base"] = i % len(starts)

    pol: G3Policy | None = None
    o_max, a_vel = args.o_max, args.a_vel
    if not args.zero:
        src = Path(args.policy) if args.policy else find_weight(f"g3_ppo{args.tag}.pt")
        pol, meta = G3Policy.load(src, device="cuda")
        o_max = float(meta.get("o_max", o_max))
        a_vel = float(meta.get("a_vel", a_vel))

    t0 = time.time()
    env = GpuReachEnv(
        starts,
        cells,
        backend=args.backend,
        moving=True,
        hold_check=args.hold_check,
        noise=noise,
        seed=args.seed,
        duty=args.duty,
        o_max=o_max,
        a_vel=a_vel,
    )
    t1 = time.time()
    _, cls = env.run(pol, max_steps=args.max_steps, dec=DEC, deterministic=True, collect=False)
    t2 = time.time()
    rows, summ = summarize(env, cells)
    totals = {k: cls.count(k) for k in ("captured", "safe abort", "failed close", "collision")}
    out_p = Path(args.out) if args.out else (ensure_runs_dir() / f"g3_eval{'_zero' if args.zero else args.tag}.json")
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out = dict(tag=out_p.stem, backend=args.backend, noise=noise.name, totals=totals, n=len(rows), init_s=round(t1 - t0, 1), run_s=round(t2 - t1, 1), steps=env.k_glob, cells=summ)
    out_p.write_text(json.dumps(dict(summary=out, rows=rows), indent=1, default=float), encoding="utf-8")
    print(json.dumps({k: v for k, v in out.items() if k != "cells"}), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("train", "eval"))
    ap.add_argument("--backend", choices=("mjorbit_warp", "plain_warp"), default="mjorbit_warp")
    ap.add_argument("--perc", choices=("orb", "stereo_plain", "net"), default="orb")
    ap.add_argument("--noise-json", default=str(find_weight("stereo_net_in+out_orb.json")))
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--worlds", type=int, default=512)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--o-max", type=float, default=0.008)
    ap.add_argument("--a-vel", type=float, default=0.008)
    ap.add_argument("--c-act", type=float, default=0.001)
    ap.add_argument("--c-ent", type=float, default=0.005)
    ap.add_argument("--early-stop-frac", type=float, default=0.98)
    ap.add_argument("--duty", type=float, default=DEFAULT_DUTY)
    ap.add_argument("--tag", default="_orb")
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--zero", action="store_true")
    ap.add_argument("--hold-check", action="store_true")
    ap.add_argument("--policy", default=None)
    ap.add_argument("--per", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-steps", type=int, default=None)
    a = ap.parse_args()
    if a.max_steps is None:
        a.max_steps = 16000 if a.mode == "train" else 18000
    torch.manual_seed(0)
    train(a) if a.mode == "train" else evaluate(a)
    sys.stdout.flush()
    import os

    os._exit(0)


if __name__ == "__main__":
    main()
