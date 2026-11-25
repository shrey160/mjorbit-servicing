"""Orbital training/validation data for far-range bus_cam bearing (BusNet).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/gen_buscam.py --n 12000 --workers 3

Renders 512x512 RGB bus_cam through MjorbitBackend(earth_bg=True) at ranges 8-35 m with randomized
sun phase, optional space optics, and FarKF-like ROI center offsets (+-1.5 m). Crops the 3.5 m
half-width ROI around c_pred, resizes to 64x64 RGB, and labels the true bus center offset
(du_norm, dv_norm) in normalized crop coordinates [-1, 1].
"""
from __future__ import annotations

import argparse
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR  # noqa: E402
import far_bench as FB  # noqa: E402
import ring_bench as RB  # noqa: E402
from gnc.nav import _roi_bounds  # noqa: E402
from sim.backend import MjorbitBackend  # noqa: E402
from sim.optics import apply_space_optics  # noqa: E402

CROP_SZ = 64
WIN_M = 3.5


def make_world(sun_phase_deg=None, optical_fx=False):
    W = RB.World()
    W.backend = MjorbitBackend(RB.OUT, earth_bg=True, sun_phase_deg=0.0 if sun_phase_deg is None else float(sun_phase_deg))
    W.sun_phase_deg = sun_phase_deg
    W.optical_fx = bool(optical_fx)
    W.wr = mujoco.Renderer(W.backend.render_km, 512, 512)
    return W


def sample(W, seed):
    rng = np.random.default_rng([seed, 79])
    D = float(rng.uniform(8.0, 35.0))
    FB._place_far(W, seed, D)
    W.backend.sun_phase_deg = float(rng.uniform(-85.0, 85.0)) if W.sun_phase_deg is None else float(W.sun_phase_deg)
    W.backend.reset(rng=rng)
    W.backend.kd.qpos[:] = W.kd.qpos
    W.backend.km.cam_pos[:] = W.km.cam_pos
    W.backend.km.cam_quat[:] = W.km.cam_quat
    W.backend.sync_render()

    W.wr.update_scene(W.backend.render_kd, camera=W.bcam, scene_option=W.rig.opt)
    img = W.wr.render().astype(np.float32) * W.rig.gain
    img = np.clip(img + W.rig._nrng.normal(0, W.rig.noise, img.shape), 0, 255).astype(np.uint8)
    if W.optical_fx:
        img = apply_space_optics(img, rng)

    Pc = W.backend.render_kd.cam_xpos[W.bcam].copy()
    Rcam = W.backend.render_kd.cam_xmat[W.bcam].reshape(3, 3).copy()
    p_true = W.kd.xpos[W.bid_t].copy()

    # Simulated FarKF prediction error around true client bus center
    err_lat = min(1.5, 0.06 * D)
    c_pred = p_true + rng.uniform(-err_lat, err_lat, size=3)
    fovy = float(W.km.cam_fovy[W.bcam])
    S = img.shape[0]
    roi = _roi_bounds(S, Pc, Rcam, c_pred, fovy, WIN_M)
    if roi is None:
        return None
    f, x0, x1, y0, y1 = roi
    if x1 - x0 < 12 or y1 - y0 < 12:
        return None

    t = Rcam.T @ (p_true - Pc)
    if -t[2] < 1.0:
        return None
    u_true = S / 2.0 + f * t[0] / -t[2]
    v_true = S / 2.0 - f * t[1] / -t[2]
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    hw, hh = 0.5 * (x1 - x0), 0.5 * (y1 - y0)
    du = float((u_true - cx) / hw)
    dv = float((v_true - cy) / hh)
    if abs(du) > 1.2 or abs(dv) > 1.2:
        return None

    crop = Image.fromarray(img[y0:y1, x0:x1]).resize((CROP_SZ, CROP_SZ), Image.BILINEAR)
    arr = np.transpose(np.asarray(crop, dtype=np.uint8), (2, 0, 1))
    return arr, np.array([du, dv], dtype=np.float32)


def work(job):
    k, seeds, out_dir, sun_phase_deg, optical_fx = job
    W = make_world(sun_phase_deg=sun_phase_deg, optical_fx=optical_fx)
    imgs, ys = [], []
    for sd in seeds:
        r = sample(W, sd)
        if r is not None:
            imgs.append(r[0])
            ys.append(r[1])
    np.savez_compressed(out_dir / f"part_{k}.npz", img=np.array(imgs), y=np.array(ys))
    print(f"part {k}: {len(imgs)} / {len(seeds)}", flush=True)
    return len(imgs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=12000)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--chunk", type=int, default=500)
    ap.add_argument("--seed-base", type=int, default=7910000)
    ap.add_argument("--sun-phase-deg", type=float, default=None)
    ap.add_argument("--optical-fx", action="store_true")
    ap.add_argument("--out", default=str(RUNS_DIR / "buscam_data_orb.npz"))
    a = ap.parse_args()
    out_npz = Path(a.out)
    out_dir = out_npz.with_suffix("")
    out_dir.mkdir(parents=True, exist_ok=True)
    seeds = [a.seed_base + i for i in range(a.n)]
    jobs = [(k, seeds[i:i + a.chunk], out_dir, a.sun_phase_deg, a.optical_fx) for k, i in enumerate(range(0, a.n, a.chunk))]
    with Pool(min(a.workers, 6)) as pool:
        n = sum(pool.map(work, jobs, chunksize=1))
    parts = sorted(out_dir.glob("part_*.npz"), key=lambda p: int(p.stem.split("_")[1]))
    img = np.concatenate([np.load(p)["img"] for p in parts])
    y = np.concatenate([np.load(p)["y"] for p in parts])
    np.savez(out_npz, img=img, y=y)
    print(f"total {n}: img {img.shape}, y {y.shape} -> {out_npz}", flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
