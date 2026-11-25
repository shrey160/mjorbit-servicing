"""P7.3 camera checks: does the camera agree with the lidar ring fit? (SPEC_7 section 5: a ring ellipse check from the
bus camera; the flange edge in the wrist camera's view at close range.)

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/ring_cam.py [--n 25]

Same cases as ring_bench.py (World.place, bench seeds 7300000 + i) plus a random sun (any direction: some faces are in
shadow). The ring's two top edges (inner r 0.283 and the lip's outer r 0.317, 60 mm above the face) are projected
into the camera at its believed pose (bus_cam: the nominal mount, on the lidar's optical bench; wrist_cam: arm
kinematics). The check is a measurement, not a threshold on one estimate (21 Nov: the ring is the face's grey, its
top edges are faint, the ring's own shadow and the arm give strong edges, and an edge-support threshold passed wrong
estimates and failed right ones): from the estimate, search in-plane shifts of the ring (grid, span SPAN of the
range) for the best edge alignment (mean normalised gradient on the projected edges; the servicer's own bodies, which
it knows, masked and dilated). The best shift is the camera's correction:
  agree       |shift| <= AGREE (15 mm at 1.8 and 3 m, 40 mm at 8 m);
  disagree    larger: the lidar fit and the camera differ (a hold / refit, not a capture);
  no verdict  the peak is weak (best / median score < PEAK_T), or the ring's box is dark (no light), or out of view.
Scored from the lidar ring fit and from wrong starts (40 mm across the axis, 3 deg tilt), against the truth as seen
through the servicer's own (mis-mounted) bench (bus_cam) or the truth (wrist_cam). Writes runs/ring_cam.json and a few
overlay PNGs (runs/ring_cam_*.png: green truth, yellow lidar fit, cyan camera-corrected).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_fit  # noqa: E402

DARK_T, PEAK_T, NS = 8.0, 1.15, 180
EDGES = {"inner": (ring_fit.R_IN, ring_fit.H_TOP), "outer": (ring_fit.R_LIP, ring_fit.H_TOP)}
SPAN = {8.0: (0.15, 0.010), 3.0: (0.06, 0.003), 1.8: (0.06, 0.003)}     # m: search half span, step
AGREE = {8.0: 0.040, 3.0: 0.015, 1.8: 0.015}


def project(pw, cam_p, cam_R, f, S):
    """World points (n, 3) -> pixel (u col, v row), in-view mask. MuJoCo camera: looks along -z, x right, y up."""
    q = (pw - cam_p) @ cam_R
    z = -q[:, 2]
    u = S / 2 + f * q[:, 0] / np.maximum(z, 1e-6)
    v = S / 2 - f * q[:, 1] / np.maximum(z, 1e-6)
    return np.column_stack([u, v]), (z > 0.05) & (u >= 0) & (u < S - 1) & (v >= 0) & (v < S - 1)


def circle_pts(c, n, r, h):
    u, v = ring_fit._frame(n)
    t = np.linspace(0, 2 * np.pi, NS, endpoint=False)
    return c + h * n + r * (np.cos(t)[:, None] * u + np.sin(t)[:, None] * v)


def edge_map(img):
    g = ndimage.gaussian_filter(img.astype(np.float64).mean(-1), 1.0)
    return np.hypot(ndimage.sobel(g, 0), ndimage.sobel(g, 1)), g


def search(G, grey, own, est, cam, f, S, D):
    """Best in-plane shift of the ring for the image edges, from est = (c, n)."""
    cp, cR = cam
    c, n = est
    u, v = ring_fit._frame(n)
    span, step = SPAN[D]
    base = np.concatenate([circle_pts(c, n, r, h) for r, h in EDGES.values()])
    p0, ok0 = project(base, cp, cR, f, S)
    if ok0.mean() < 0.6:
        return dict(verdict="out of view")
    lo = np.clip(np.floor(p0[ok0].min(0)).astype(int) - 10, 0, S - 1)
    hi = np.clip(np.ceil(p0[ok0].max(0)).astype(int) + 10, 0, S - 1)
    box, gbox = G[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1], grey[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1]
    ref = float(np.percentile(box, 99)) if box.size else 0.0
    if gbox.mean() < DARK_T or ref < 1.0:
        return dict(verdict="no light", grey=round(float(gbox.mean()), 1))
    Gn = ndimage.maximum_filter(G, size=3) / ref
    g = np.arange(-span, span + 1e-9, step)
    sc = np.full((len(g), len(g)), np.nan)
    for i, a in enumerate(g):
        for j, b in enumerate(g):
            p, ok = project(base + a * u + b * v, cp, cR, f, S)
            ij = np.round(p[ok]).astype(int)
            keep = ~own[ij[:, 1], ij[:, 0]]
            if keep.sum() < 0.3 * len(base):
                continue
            sc[i, j] = float(Gn[ij[keep, 1], ij[keep, 0]].mean())
    if not np.isfinite(sc).any():
        return dict(verdict="occluded")
    i, j = np.unravel_index(np.nanargmax(sc), sc.shape)
    peak = float(sc[i, j] / max(np.nanmedian(sc), 1e-9))
    d = g[i] * u + g[j] * v
    out = dict(shift_mm=round(float(np.hypot(g[i], g[j])) * 1e3, 1), peak=round(peak, 2), c=c + d,
               grey=round(float(gbox.mean()), 1))
    if peak < PEAK_T:
        out["verdict"] = "no verdict"
    else:
        out["verdict"] = "agree" if np.hypot(g[i], g[j]) <= AGREE[D] + 1e-9 else "disagree"
    return out


def shifted(c, n, rng, kind):
    u, v = ring_fit._frame(n)
    a = rng.uniform(0, 2 * np.pi)
    d = np.cos(a) * u + np.sin(a) * v
    if kind == "shift40":
        return c + 0.040 * d, n
    return c, Rotation.from_rotvec(np.radians(3.0) * d).apply(n)


def sun(W, rng):
    km = W.km
    lid = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
    s = rng.normal(size=3)
    s /= np.linalg.norm(s)
    km.light_dir[lid] = s
    km.light_pos[lid] = -10 * s
    km.light_diffuse[lid] = rng.uniform(0.5, 1.3)
    km.vis.headlight.ambient[:] = rng.uniform(0.03, 0.15)
    km.vis.headlight.diffuse[:] = 0.0
    km.vis.headlight.specular[:] = 0.0
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--d", type=float, nargs="*", default=[8.0, 3.0, 1.8])
    ap.add_argument("--png", type=int, default=2, help="overlay images per range")
    ap.add_argument("--out", default=str(RUNS_DIR / "ring_cam.json"))
    a = ap.parse_args()
    W = RB.World()
    km = W.km
    wcam = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
    cams = {"bus_cam": (W.bcam, 512), "wrist_cam": (wcam, 256)}
    rend = {k: mujoco.Renderer(km, S, S) for k, (_, S) in cams.items()}
    segr = {k: mujoco.Renderer(km, S, S) for k, (_, S) in cams.items()}
    for r in segr.values():
        r.enable_segmentation_rendering()
    starts = ["ring", "shift40", "tilt3"]
    rows = []
    for D in a.d:
        for i in range(a.n):
            seed = 7300000 + i
            res = RB.case(W, seed, D)
            rng = np.random.default_rng([seed, int(D * 10), 11])
            s = sun(W, rng)
            mujoco.mj_forward(km, W.kd)
            ring = res["_est"].get("ring")
            row = dict(seed=seed, D=D, ring_ok=res["ring_ok"], sun_face_cos=round(float(-s @ res["_truth"][1]), 2))
            if ring is None:
                rows.append(row)
                continue
            ests = dict(ring=ring, shift40=shifted(*ring, rng, "shift40"), tilt3=shifted(*ring, rng, "tilt3"))
            for cname, (cid, S) in cams.items():
                if cname == "wrist_cam" and D > 2.5:
                    continue
                r = rend[cname]
                r.update_scene(W.kd, camera=cid, scene_option=W.rig.opt)
                img = r.render().astype(np.float32) * W.rig.gain
                img = np.clip(img + W.rig._nrng.normal(0, W.rig.noise, img.shape), 0, 255).astype(np.uint8)
                G, grey = edge_map(img)
                sr = segr[cname]
                sr.update_scene(W.kd, camera=cid, scene_option=W.rig.opt)
                sg = sr.render()
                own = (sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(sg[..., 0], W.self_geoms)
                own = ndimage.binary_dilation(own, iterations=3)
                if cname == "bus_cam":
                    cam = W.frames(cid)[0]                 # believed pose (nominal bench mount, as the lidar)
                    tb = res["_truth_b"][0]                # the true ring as seen through that bench
                else:
                    cam = (W.kd.cam_xpos[cid].copy(), W.kd.cam_xmat[cid].reshape(3, 3).copy())
                    tb = res["_truth"][0]
                f = (S / 2) / np.tan(np.radians(km.cam_fovy[cid]) / 2)
                nn = ring[1]
                for k, e in ests.items():
                    o = search(G, grey, own, e, cam, f, S, D)
                    if "c" in o:
                        def inpl(x):
                            dd = x - tb
                            return round(float(np.linalg.norm(dd - (dd @ nn) * nn)) * 1e3, 1)
                        o["start_err_mm"] = inpl(e[0])
                        o["cam_err_mm"] = inpl(o.pop("c"))
                    row[f"{cname}/{k}"] = o
                if i < a.png:
                    _overlay(img, dict(truth=(tb, nn), ring=ring), cam, f, S,
                             ensure_runs_dir() / f"ring_cam_{cname}_{D:.1f}_{i}.png")
            rows.append(row)
            print(D, seed, {k: (v["verdict"], v.get("shift_mm"), v.get("peak"), v.get("start_err_mm"), v.get("cam_err_mm"))
                            for k, v in row.items() if isinstance(v, dict)}, flush=True)
    summ = {}
    for D in a.d:
        rs = [r for r in rows if r["D"] == D]
        for cname in cams:
            for k in starts:
                vs = [r[f"{cname}/{k}"] for r in rs if f"{cname}/{k}" in r]
                if not vs:
                    continue
                vd = [v["verdict"] for v in vs]
                e = [v["cam_err_mm"] for v in vs if v["verdict"] in ("agree", "disagree")]
                s0 = [v["start_err_mm"] for v in vs if v["verdict"] in ("agree", "disagree")]
                summ[f"{D}/{cname}/{k}"] = dict(verdicts={x: vd.count(x) for x in sorted(set(vd))},
                                                start_err_p50=round(float(np.median(s0)), 1) if s0 else None,
                                                cam_err_p50=round(float(np.median(e)), 1) if e else None,
                                                cam_err_p95=round(float(np.percentile(e, 95)), 1) if e else None)
    print(json.dumps(summ, indent=1))
    Path(a.out).write_text(json.dumps(dict(summary=summ, rows=rows), indent=1))


def _overlay(img, ests, cam, f, S, path):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    col = dict(truth=(0, 255, 0), ring=(255, 255, 0))
    for k, c in col.items():
        for r, h in EDGES.values():
            p, ok = project(circle_pts(*ests[k], r, h), *cam, f, S)
            for (x, y), o in zip(p, ok):
                if o:
                    dr.point((x, y), fill=c)
    im.save(path)


if __name__ == "__main__":
    main()
