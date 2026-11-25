"""Training data for the wrist stereo experiment (SPEC_7 s25, 23 Nov; user: "go ahead with stereo vision").

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/gen_stereo.py --n 15000 --workers 5

As gen_ringcam.py (bench station, sun, camera gain / noise, lamp 0.5-1.0; an ESTIMATE = truth + error; the arm by IK
at the pre-reach pose on the estimate), with three images per sample (grey, 128 px):
  in     wrist_cam   (the stereo module's inboard camera, on the housing's -y face, pitched 10 deg; the mono view)
  out    wrist_cam2  (the module's second camera, 50 mm further out along the jaws' travel: the stereo partner)
  mot    wrist_cam again after the arm moved MOT_M radially out (along the section's radial) by IK: motion stereo
         with the one camera (the client is static here; in the mission it moves between the two shots)
Wider error ranges than gen_ringcam (the mission's axial error, -8..-20 mm, was outside the ring net's +-15 mm, and
the hard set's failures are axis tilt): centre radial / tangential +-30 mm, axial +-25 mm, axis tilt up to 3 deg.
Label (truth, training only), in the estimated section frame: centre offset (radial, tangential, axial; m) and the
true axis' components along the estimated radial and tangential (rad, small angle).
Writes runs/stereo_data/part_<k>.npz, then runs/stereo_data.npz (img uint8 (n, 3, 128, 128), y float32 (n, 5)).
Seeds 7810000 + i (not a mission, test or ringcam range).
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
from sim.backend import MjorbitBackend  # noqa: E402
from sim.optics import apply_space_optics  # noqa: E402

OUT = RUNS_DIR / "stereo_data"
E_RT, E_AX, E_DEG, AZ_DEG = 0.030, 0.025, 3.0, 40.0
MOT_M = 0.040
VIEWS = ("in", "out", "mot")


def shot(B, cam):
    km, kd = B.km, B.kd
    if B.backend is not None:
        B.backend.kd.qpos[:] = kd.qpos
        B.backend.km.light_active[B.lamp] = km.light_active[B.lamp]
        B.backend.km.light_diffuse[B.lamp] = km.light_diffuse[B.lamp]
        B.backend.sync_render()
        B.wr.update_scene(B.backend.render_kd, camera=cam, scene_option=B.rig.opt)
    else:
        B.wr.update_scene(kd, camera=cam, scene_option=B.rig.opt)
    img = B.wr.render().astype(np.float32) * B.rig.gain
    img = np.clip(img + B.rig._nrng.normal(0, B.rig.noise, img.shape), 0, 255)
    if B.optical_fx:
        img = apply_space_optics(img.astype(np.uint8), B.rng).astype(np.float32)
    return img.mean(-1).astype(np.uint8)


def sample(B, seed):
    B.place_station(seed)
    rng = B.rng
    if B.backend is not None:
        if B.sun_phase_deg is None:
            B.backend.sun_phase_deg = float(rng.uniform(-85.0, 85.0))
        else:
            B.backend.sun_phase_deg = float(B.sun_phase_deg)
        B.backend.reset(rng=rng)
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
    c_e = c_t - d
    est = (c_e, n_e)
    p, Rm = B.section(est, rad, R.H_PRE)
    q, ep, er = B.ik(p, Rm, B.q_app)
    if ep > 1e-3 or er > 0.1:
        return None
    q2, ep2, er2 = B.ik(p + MOT_M * rad, Rm, q, seeds=False)
    if ep2 > 1e-3 or er2 > 0.1:
        return None
    km = B.km
    km.light_active[B.lamp] = 1
    km.light_diffuse[B.lamp] = rng.uniform(0.5, 1.0)
    B.kd.qpos[B.arm] = q
    mujoco.mj_forward(km, B.kd)
    im_in, im_out = shot(B, B.wcam), shot(B, B.wcam2)
    B.kd.qpos[B.arm] = q2
    mujoco.mj_forward(km, B.kd)
    im_mot = shot(B, B.wcam)
    km.light_active[B.lamp] = 0
    B.kd.qpos[B.arm] = q
    mujoco.mj_forward(km, B.kd)
    dd = c_t - c_e
    y = np.array([dd @ rad, dd @ tan, dd @ n_e, n_t @ rad, n_t @ tan], np.float32)
    return np.stack([im_in, im_out, im_mot]), y


def bench(earth_bg=False, sun_phase_deg=None, optical_fx=False):
    B = R.Bench()
    B.wcam2 = mujoco.mj_name2id(B.km, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam2")
    B.sun_phase_deg = sun_phase_deg
    B.optical_fx = bool(optical_fx)
    if earth_bg:
        B.backend = MjorbitBackend(R.RB.OUT, earth_bg=True, sun_phase_deg=0.0 if sun_phase_deg is None else float(sun_phase_deg))
        B.wr = mujoco.Renderer(B.backend.render_km, B.WS, B.WS)
    else:
        B.backend = None
    return B


def work(job):
    k, seeds, out_dir, earth_bg, sun_phase_deg, optical_fx = job
    B = bench(earth_bg=earth_bg, sun_phase_deg=sun_phase_deg, optical_fx=optical_fx)
    imgs, ys = [], []
    for sd in seeds:
        r = sample(B, sd)
        if r is not None:
            imgs.append(r[0])
            ys.append(r[1])
    np.savez_compressed(out_dir / f"part_{k}.npz", img=np.array(imgs), y=np.array(ys))
    print(f"part {k}: {len(imgs)} / {len(seeds)}", flush=True)
    return len(imgs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=15000)
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--chunk", type=int, default=500)
    ap.add_argument("--seed-base", type=int, default=7810000)
    ap.add_argument("--earth-bg", action="store_true")
    ap.add_argument("--sun-phase-deg", type=float, default=None)
    ap.add_argument("--optical-fx", action="store_true")
    ap.add_argument("--out", default=str(RUNS_DIR / "stereo_data.npz"))
    ap.add_argument("--png", action="store_true", help="write a few samples to runs/stereo_samples.png and stop")
    a = ap.parse_args()
    out_npz = Path(a.out)
    out_dir = out_npz.with_suffix("")
    out_dir.mkdir(parents=True, exist_ok=True)
    if a.png:
        from PIL import Image
        B = bench(earth_bg=a.earth_bg, sun_phase_deg=a.sun_phase_deg, optical_fx=a.optical_fx)
        rows = []
        for sd in range(a.seed_base, a.seed_base + 6):
            r = sample(B, sd)
            if r is not None:
                rows.append(np.hstack([np.pad(x, ((2, 2), (2, 2)), constant_values=255) for x in r[0]]))
                print(sd, np.round(r[1][:3] * 1e3, 1), np.round(np.degrees(r[1][3:]), 2))
        png_name = "stereo_samples_orb.png" if a.earth_bg else "stereo_samples.png"
        Image.fromarray(np.vstack(rows)).resize((rows[0].shape[1] * 2, len(rows) * rows[0].shape[0] * 2)).save(
            ensure_runs_dir() / png_name)
        return
    seeds = [a.seed_base + i for i in range(a.n)]
    jobs = [(k, seeds[i:i + a.chunk], out_dir, a.earth_bg, a.sun_phase_deg, a.optical_fx) for k, i in enumerate(range(0, a.n, a.chunk))]
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
