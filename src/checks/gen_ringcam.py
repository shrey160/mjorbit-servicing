"""Training data for the wrist-camera ring net (P7.6 lead, user 22 Nov: "learned wrist-cam ring net"; camera: the
bar-mission spec, 75 deg / 128 px, the inboard camera + lamp unit).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/gen_ringcam.py --n 18000 --workers 5

Per sample (seeds 7800000 + i; not a mission or test seed range): reach_bench's station (random roll about the ring
axis, fly-in error, bench mount error, sun, camera gain / noise), lamp brightness 0.5-1.0; an ESTIMATE of the ring =
the truth + an error (centre: radial and tangential +-30 mm, axial +-15 mm, uniform; axis tilted up to 1.5 deg); the
grasp azimuth on the estimate nearest the shoulder +-40 deg; the arm by IK at the pre-reach pose on the estimate
(grip site 0.15 m above the estimated section, square on, as the reach); the wrist camera image (grey, 128 px).
Label (the truth, as a training label only): the true ring centre minus the estimated one, in the estimated section
frame (radial along the section's outward radial, tangential, axial along the estimated axis), metres.
Writes runs/ringcam_data/part_<k>.npz (img uint8 (n,128,128), y float32 (n,3)), then one ringcam_data.npz.
"""
from __future__ import annotations

import argparse
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
import reach_bench as R  # noqa: E402
import ring_fit  # noqa: E402

OUT = RUNS_DIR / "ringcam_data"
E_RT, E_AX, E_DEG, AZ_DEG = 0.030, 0.015, 1.5, 40.0


def render(B):
    km, kd = B.km, B.kd
    km.light_active[B.lamp] = 1
    km.light_diffuse[B.lamp] = B.rng.uniform(0.5, 1.0)
    B.wr.update_scene(kd, camera=B.wcam, scene_option=B.rig.opt)
    img = B.wr.render().astype(np.float32) * B.rig.gain
    km.light_active[B.lamp] = 0
    img = np.clip(img + B.rig._nrng.normal(0, B.rig.noise, img.shape), 0, 255)
    return img.mean(-1).astype(np.uint8)


def sample(B, seed):
    B.place_station(seed)
    rng = B.rng
    c_t, n_t = ring_fit.C_B.copy(), ring_fit.N_B.copy()          # client at identity
    ax = rng.normal(size=3)
    ax -= (ax @ n_t) * n_t
    n_e = Rotation.from_rotvec(np.radians(rng.uniform(0, E_DEG)) * ax / np.linalg.norm(ax)).apply(n_t)
    u, v = ring_fit._frame(n_e)
    s = B.kd.xpos[B.shoulder] - c_t
    a = np.arctan2(s @ v, s @ u) + np.radians(rng.uniform(-AZ_DEG, AZ_DEG))
    rad = np.cos(a) * u + np.sin(a) * v
    tan = np.cross(n_e, rad)
    d = rng.uniform(-E_RT, E_RT) * rad + rng.uniform(-E_RT, E_RT) * tan + rng.uniform(-E_AX, E_AX) * n_e
    c_e = c_t - d                                                   # so that truth - estimate = d
    est = (c_e, n_e)
    p, Rm = B.section(est, rad, R.H_PRE)
    q, ep, er = B.ik(p, Rm, B.q_app)
    if ep > 1e-3 or er > 0.1:
        return None
    B.kd.qpos[B.arm] = q
    mujoco.mj_forward(B.km, B.kd)
    dd = c_t - c_e
    return render(B), np.array([dd @ rad, dd @ tan, dd @ n_e], np.float32)


def work(job):
    k, seeds = job
    B = R.Bench()
    imgs, ys = [], []
    for sd in seeds:
        r = sample(B, sd)
        if r is not None:
            imgs.append(r[0])
            ys.append(r[1])
    np.savez_compressed(OUT / f"part_{k}.npz", img=np.array(imgs), y=np.array(ys))
    print(f"part {k}: {len(imgs)} / {len(seeds)}", flush=True)
    return len(imgs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=18000)
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--chunk", type=int, default=1000)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    seeds = [7800000 + i for i in range(a.n)]
    jobs = [(k, seeds[i:i + a.chunk]) for k, i in enumerate(range(0, a.n, a.chunk))]
    with Pool(min(a.workers, 6)) as pool:
        n = sum(pool.map(work, jobs, chunksize=1))
    parts = sorted(OUT.glob("part_*.npz"), key=lambda p: int(p.stem.split("_")[1]))
    img = np.concatenate([np.load(p)["img"] for p in parts])
    y = np.concatenate([np.load(p)["y"] for p in parts])
    np.savez(ensure_runs_dir() / "ringcam_data.npz", img=img, y=y)
    print(f"total {n}: img {img.shape}, y {y.shape}; |radial| mean {np.abs(y[:, 0]).mean() * 1e3:.1f} mm", flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
