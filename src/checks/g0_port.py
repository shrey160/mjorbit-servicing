"""SPEC_7 section 24, G0: can the reach-and-close run on the GPU (MuJoCo Warp) with the mission's physics?

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/g0_port.py record --seeds 0 1 2 3
  bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/g0_port.py replay [--gpu-worlds 1 1024 4096]

record   the scripted reach bench (reach_bench.case, noslip 20 as the scene) on the CPU; from the start of the
         descent every physics-step command is logged (arm joint target, jaw targets, lock activation and values, the
         hold check's client torque) with the full physics state there -> runs/g0_rec_<seed>.pkl
replay   the same commands, open loop on the plan level (the arm PD and the bus pin run per step on each sim's own
         state, as Bench.step), from the saved state, on:
           cpu20        CPU, noslip 20 (must reproduce the record: a determinism check)
           cpu0         CPU, noslip 0 (what Warp can do: G0c)
           cpu20_trim   CPU, noslip 20, collisions trimmed to the grasp sector (G0a: must equal cpu20)
           gpu          MuJoCo Warp, noslip 0, trimmed, world 0 scored (G0d); timed at each --gpu-worlds (G0b)
         Every variant is scored by one evaluator on its sampled (qpos, qvel) (CPU mj_forward): the seat after the
         close and after the hold, the touch pads, the slip under the 3 N m loads, the grip-site path against the
         record, the client speed. -> runs/g0_port.json
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
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR as RUNS, ensure_runs_dir  # noqa: E402
import reach_bench as RBN  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_fit  # noqa: E402

SAMPLE = 25                     # physics steps between scored samples
SECTOR_DEG = 40.0               # trim: ring segments within +-SECTOR_DEG of the grasp section keep colliding
LOCKS = ("lock_u", "lock_l")
NCON, NJ = 160, 800             # Warp buffers per world (the clamp makes up to ~85 box-box contacts)


# ---------------------------------------------------------------------------------------------------------- record
class RecBench(RBN.Bench):
    log = None

    def step(self, q_tgt, grip, n=1, qd_tgt=None):
        # G0 is a physics port check: recorded and replayed without the joint velocity feed-forward (s24.4; the
        # replay's PD has none), see record()
        if self.log is None:
            return super().step(q_tgt, grip, n, qd_tgt=qd_tgt)
        km, kd = self.km, self.kd
        ev = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm) for nm in LOCKS]
        t0 = kd.time
        super().step(q_tgt, grip, n, qd_tgt=qd_tgt)
        done = int(round((kd.time - t0) / km.opt.timestep))
        self.log.append(dict(q=np.array(q_tgt, float).copy(), g=np.array(grip, float).copy(), n=done,
                             eq=kd.eq_active[ev].copy(), eqd=km.eq_data[ev, 0].copy(),
                             f=kd.xfrc_applied[self.bid_t].copy(), ph=self.phase))


def record(seeds):
    RBN.FF[0] = False                  # (s25, 23 Nov) the replay applies no feed-forward, so the record has none either
    B = RecBench()
    orig = RBN.reach_close
    out = []
    for s in seeds:
        snap = {}

        def start(Bx):
            spec = mujoco.mjtState.mjSTATE_INTEGRATION
            st = np.zeros(mujoco.mj_stateSize(Bx.km, spec))
            mujoco.mj_getState(Bx.km, Bx.kd, st, spec)
            snap.update(state=st, pin=Bx.pin_bus.copy(), eqd0=Bx.km.eq_data[[mujoco.mj_name2id(
                Bx.km, mujoco.mjtObj.mjOBJ_EQUALITY, nm) for nm in LOCKS], 0].copy())
            Bx.log = []

        RBN.reach_close = lambda B_, row, E1, a1, **kw: orig(B_, row, E1, a1, on_descent=start, **kw)
        B.log = None
        row = RBN.case(B, 7400000 + s)
        RBN.reach_close = orig
        log, B.log = B.log, None
        rec = dict(seed=s, row=row, log=log or [], **snap)
        (RUNS / f"g0_rec_{s}.pkl").write_bytes(pickle.dumps(rec))
        print(s, row["result"], "calls", len(rec["log"]), "steps", sum(c["n"] for c in rec["log"]), flush=True)
        out.append(row["result"])
    return out


# ---------------------------------------------------------------------------------------------------------- scoring
def grasp_dir_client(B, kd):
    """The grasp section's radial direction in the client frame (from the grip site at the descent start)."""
    Rc = kd.xmat[B.bid_t].reshape(3, 3)
    v = Rc.T @ (kd.site_xpos[B.I["grip"]] - kd.xpos[B.bid_t]) - ring_fit.C_B
    v -= (v @ ring_fit.N_B) * ring_fit.N_B
    return v / np.linalg.norm(v)


def trim(km, B, rad_c):
    """Collisions off for the ring segments outside the grasp sector (client frame); returns how many were cut."""
    n = 0
    for g in B.ring_g:
        v = km.geom_pos[g] - ring_fit.C_B
        v -= (v @ ring_fit.N_B) * ring_fit.N_B
        if np.linalg.norm(v) < 1e-6:
            continue
        ang = np.degrees(np.arccos(np.clip(v @ rad_c / np.linalg.norm(v), -1, 1)))
        if ang > SECTOR_DEG:
            km.geom_contype[g] = 0
            km.geom_conaffinity[g] = 0
            n += 1
    return n


def score(B, rec, samples):
    """samples: list of (k_step, phase, qpos, qvel, ctrl, command). Scored on a CPU MjData with mj_forward (one
    evaluator; the sample's own actuator commands, lock state and client torque applied)."""
    km = B.km
    ev = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm) for nm in LOCKS]
    eqd0 = km.eq_data[ev, 0].copy()
    d = mujoco.MjData(km)
    kd0 = B.kd
    out = dict(path_dev_mm=0.0)
    ref = rec.get("ref")
    rows = []
    for k, ph, qp, qv, u, c in samples:
        d.qpos[:], d.qvel[:], d.ctrl[:] = qp, qv, u
        km.eq_data[ev, 0] = c["eqd"]
        d.eq_active[ev] = c["eq"]
        d.xfrc_applied[B.bid_t] = c["f"]
        mujoco.mj_forward(km, d)
        B.kd = d
        s, t = B.seat(), B.touches()
        Rc = d.xmat[B.bid_t].reshape(3, 3)
        prel = Rc.T @ (d.site_xpos[B.I["grip"]] - d.xpos[B.bid_t])
        Rrel = Rc.T @ d.site_xmat[B.I["grip"]].reshape(3, 3)
        rows.append(dict(k=k, ph=ph, seat=s, touch=t, prel=prel, Rrel=Rrel,
                         v_cli=float(np.linalg.norm(qv[B.tv:B.tv + 3])) * 1e3))
    B.kd = kd0
    km.eq_data[ev, 0] = eqd0
    by = {}
    for r in rows:
        by.setdefault(r["ph"], []).append(r)
    last = lambda ph: by[ph][-1] if ph in by else None  # noqa: E731
    for ph in ("descent", "close", "hold"):
        r = last(ph)
        if r is not None:
            out[ph] = dict(seat=r["seat"], touch={a: round(b, 1) for a, b in r["touch"].items()},
                           v_cli_mm_s=round(r["v_cli"], 2))
    if "load" in by and "hold" in by:
        p_a, R_a = by["hold"][-1]["prel"], by["hold"][-1]["Rrel"]
        sl = [(float(np.linalg.norm(r["prel"] - p_a)) * 1e3,
               float(np.degrees(Rotation.from_matrix(R_a.T @ r["Rrel"]).magnitude()))) for r in by["load"]]
        out["load_slip"] = dict(mm=round(max(x[0] for x in sl), 2), deg=round(max(x[1] for x in sl), 2))
    out["peak_touch"] = {a: round(max(r["touch"][a] for r in rows), 1) for a in rows[0]["touch"]}
    if ref is not None:
        n = min(len(ref), len(rows))
        out["path_dev_mm"] = round(max(float(np.linalg.norm(rows[i]["prel"] - ref[i])) for i in range(n)) * 1e3, 2)
    return out, [r["prel"] for r in rows]


# ---------------------------------------------------------------------------------------------------------- replays
def expand(log):
    """Per physics step: (q_tgt, grip, eq_active, xfrc, phase)."""
    for c in log:
        for _ in range(c["n"]):
            yield c


def replay_cpu(B, rec, noslip, do_trim):
    km = B.km
    ns0, ct0, ca0 = km.opt.noslip_iterations, km.geom_contype.copy(), km.geom_conaffinity.copy()
    km.opt.noslip_iterations = noslip
    kd = mujoco.MjData(km)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    mujoco.mj_setState(km, kd, rec["state"], spec)
    mujoco.mj_forward(km, kd)
    ncut = trim(km, B, grasp_dir_client(B, kd)) if do_trim else 0
    I = B.I
    ev = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm) for nm in LOCKS]
    lim = km.actuator_ctrlrange[I["act_arm"], 1]
    samples, k = [], 0
    lock_vals, was_on = None, False
    t0 = time.time()
    for c in expand(rec["log"]):
        on = bool(np.any(c["eq"]))
        if on and not was_on:                  # the lock holds the jaws where this sim has them
            lock_vals = kd.qpos[B.gq].copy()
        was_on = on
        if lock_vals is not None:
            c = dict(c, eqd=lock_vals)
        km.eq_data[ev, 0] = c["eqd"]
        kd.eq_active[ev] = c["eq"]
        kd.xfrc_applied[B.bid_t] = c["f"]
        kd.ctrl[I["act_arm"]] = np.clip(B.kp * (c["q"] - kd.qpos[I["arm_q"]]) - B.kdv * kd.qvel[I["arm_v"]], -lim, lim)
        kd.ctrl[[I["act_u"], I["act_l"]]] = c["g"]
        mujoco.mj_step(km, kd)
        kd.qpos[B.cq:B.cq + 7] = rec["pin"]
        kd.qvel[B.cv:B.cv + 6] = 0.0
        mujoco.mj_forward(km, kd)
        k += 1
        if k % SAMPLE == 0:
            samples.append((k, c["ph"], kd.qpos.copy(), kd.qvel.copy(), kd.ctrl.copy(), c))
    wall = time.time() - t0
    km.opt.noslip_iterations = ns0
    km.geom_contype[:], km.geom_conaffinity[:] = ct0, ca0
    km.eq_data[ev, 0] = rec["eqd0"]
    return samples, dict(steps=k, wall_s=round(wall, 1), steps_per_s=round(k / wall), trimmed=ncut)


def replay_gpu(B, rec, nworld, keep_samples=True):
    import torch
    import warp as wp
    import mujoco_warp as mjw
    km0 = B.km
    km = _copy_model(km0)
    km.opt.noslip_iterations = 0
    kd = mujoco.MjData(km)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    mujoco.mj_setState(km, kd, rec["state"], spec)
    mujoco.mj_forward(km, kd)
    ncut = trim(km, B, grasp_dir_client(B, kd))
    ev = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, nm) for nm in LOCKS]
    wm = mjw.put_model(km)
    wd = mjw.put_data(km, kd, nworld=nworld, nconmax=NCON, njmax=NJ)
    T = lambda a: wp.to_torch(a)  # noqa: E731
    qpos, qvel, ctrl = T(wd.qpos), T(wd.qvel), T(wd.ctrl)
    xfrc, eqa = T(wd.xfrc_applied), T(wd.eq_active)
    I = B.I
    dev = qpos.device
    aq = torch.tensor(I["arm_q"], device=dev)
    av = torch.tensor(I["arm_v"], device=dev)
    aa = torch.tensor(I["act_arm"], device=dev)
    ag = torch.tensor([I["act_u"], I["act_l"]], device=dev)
    kp = torch.tensor(B.kp, device=dev, dtype=qpos.dtype)
    kdv = torch.tensor(B.kdv, device=dev, dtype=qpos.dtype)
    lim = torch.tensor(km.actuator_ctrlrange[I["act_arm"], 1], device=dev, dtype=qpos.dtype)
    pin = torch.tensor(rec["pin"], device=dev, dtype=qpos.dtype)
    evt = torch.tensor(ev, device=dev)
    mjw.step(wm, wd)                    # compile (then reset to the saved state)
    wd2 = mjw.put_data(km, kd, nworld=nworld, nconmax=NCON, njmax=NJ)
    wp.copy(wd.qpos, wd2.qpos); wp.copy(wd.qvel, wd2.qvel); wp.copy(wd.time, wd2.time)  # noqa: E702
    if hasattr(wd, "act") and wd.act.shape[-1] > 0:
        wp.copy(wd.act, wd2.act)
    wp.synchronize()
    with wp.ScopedCapture() as cap:
        mjw.step(wm, wd)
    samples, k = [], 0
    cur = None
    eqm = T(wm.eq_data)
    gqt = torch.tensor(B.gq, device=dev)
    lock_vals, was_on = None, False
    torch.cuda.synchronize()
    t0 = time.time()
    for c in expand(rec["log"]):
        if c is not cur:
            cur = c
            q_t = torch.tensor(c["q"], device=dev, dtype=qpos.dtype)
            g_t = torch.tensor(c["g"], device=dev, dtype=qpos.dtype)
            on = bool(np.any(c["eq"]))
            if on and not was_on:          # the lock holds the jaws where THIS sim has them (world 0; shared model)
                lock_vals = qpos[0, gqt].double().cpu().numpy()
                eqm[..., evt, 0] = torch.tensor(lock_vals, device=dev, dtype=eqm.dtype)
            was_on = on
            eqa[:, evt] = torch.tensor(c["eq"], device=dev, dtype=eqa.dtype)
            xfrc[:, B.bid_t] = torch.tensor(c["f"], device=dev, dtype=xfrc.dtype)
        u = kp * (q_t - qpos[:, aq]) - kdv * qvel[:, av]
        ctrl[:, aa] = torch.maximum(torch.minimum(u, lim), -lim).to(ctrl.dtype)
        ctrl[:, ag] = g_t.to(ctrl.dtype)
        wp.capture_launch(cap.graph)
        qpos[:, B.cq:B.cq + 7] = pin
        qvel[:, B.cv:B.cv + 6] = 0.0
        k += 1
        if keep_samples and k % SAMPLE == 0:
            samples.append((k, c["ph"], qpos[0].double().cpu().numpy(), qvel[0].double().cpu().numpy(),
                            ctrl[0].double().cpu().numpy(), dict(c, eqd=lock_vals) if lock_vals is not None else c))
    torch.cuda.synchronize()
    wall = time.time() - t0
    nan = int(torch.isnan(qpos).any(dim=1).sum())
    return samples, dict(nworld=nworld, steps=k, wall_s=round(wall, 1),
                         env_steps_per_s=round(k * nworld / wall), nan_worlds=nan, trimmed=ncut)


def _copy_model(m):
    import copy
    return copy.deepcopy(m)


def replay(seeds, gpu_worlds):
    B = RBN.Bench()
    res = {}
    for s in seeds:
        p = RUNS / f"g0_rec_{s}.pkl"
        if not p.exists():
            continue
        rec = pickle.loads(p.read_bytes())
        r = dict(bench_result=rec["row"]["result"], seat_bench=rec["row"].get("closed", {}).get("seat"),
                 load_bench=rec["row"].get("load"))
        if not rec["log"]:
            res[s] = r
            continue
        smp, info = replay_cpu(B, rec, 20, False)
        sc, ref = score(B, rec, smp)
        rec["ref"] = ref
        r["cpu20"] = dict(score=sc, **info)
        for name, ns, tr in (("cpu0", 0, False), ("cpu20_trim", 20, True)):
            smp, info = replay_cpu(B, rec, ns, tr)
            r[name] = dict(score=score(B, rec, smp)[0], **info)
        for nw in gpu_worlds:
            try:
                smp, info = replay_gpu(B, rec, nw, keep_samples=(nw == gpu_worlds[0]))
                if nw == gpu_worlds[0]:
                    r["gpu"] = dict(score=score(B, rec, smp)[0], **info)
                else:
                    r[f"gpu_{nw}"] = info
            except Exception as e:  # noqa: BLE001
                r[f"gpu_{nw}"] = dict(error=f"{type(e).__name__}: {str(e)[:300]}")
        res[s] = r
        print(json.dumps({s: r}, default=float), flush=True)
    (ensure_runs_dir() / "g0_port.json").write_text(json.dumps(res, indent=1, default=float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("record", "replay"))
    ap.add_argument("--seeds", type=int, nargs="*", default=[0, 1, 2, 3])
    ap.add_argument("--gpu-worlds", type=int, nargs="*", default=[1, 1024, 4096])
    a = ap.parse_args()
    if a.mode == "record":
        print(record(a.seeds))
    else:
        replay(a.seeds, a.gpu_worlds)
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
