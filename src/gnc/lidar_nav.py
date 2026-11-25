"""P6.3: client pose from the servicer's flash lidar (bus_lidar, 128 x 128, fovy 45) at close range.

Below r_hand (3.2 m) the corner network is off (one face fills the view) and the filter coasted
for 2-4 minutes with no position input (p63k: client centre 100-310 mm off at settle; the Project 5
v3 network then reads the bar wrongly too, because the bus is off its station: seed 5000024 went
11 mm inside the keep-out margin). The lidar sees the client at any lighting.

Method (the field's usual close-range tracker, a point-to-model fit from a good prior; SPEC_6 5l):
  points: range image -> 3D points in the lidar frame (MuJoCo camera: looks along -z, x right,
          y up, image row 0 at the top);
  model:  the client's box geoms (design data: main box, grapple bar, its two posts), signed
          distance to their union;
  self:   points on the servicer's own bodies (its arm) are masked by the caller (the servicer
          knows its geometry and joint angles; in the sim, a segmentation render of bus_lidar);
  fit:    Gauss-Newton on the client pose (lidar frame) minimising the robust (Huber) signed
          distance of the points to the box, from the filter's prediction; points farther than a
          gate (shrinking 0.30 -> 0.06 m) from the box are dropped (servicer arm, bar, outliers);
  sigma:  (J^T W J)^-1 x the residual variance, so a face seen square-on gives a small sigma along
          its normal and a larger one in its plane (only its edges fix that).

  python lidar_nav.py check <cpose dir> [--n ...]   offline accuracy on the P6.2 lidar frames
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _paths  # noqa: E402,F401
from fastrot import rotvec_to_matrix  # noqa: E402  (scipy Rotation, same bits, faster)

BOX_H = np.array([0.75, 0.5, 0.5])  # client main box half sizes [m] (capture_env.client_boxes[0])


def ray_dirs(res, fovy_deg):
    """(res, res, 3) unit ray directions in the lidar frame; range image pixel [row, col]."""
    f = (res / 2) / np.tan(np.radians(fovy_deg) / 2)
    u = (np.arange(res) + 0.5 - res / 2) / f
    x = np.broadcast_to(u[None, :], (res, res))
    y = np.broadcast_to(-u[:, None], (res, res))
    d = np.stack([x, y, -np.ones((res, res))], -1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def points(rng_mm, dirs):
    m = rng_mm > 0
    return (rng_mm[m].astype(np.float64) / 1000.0)[:, None] * dirs[m]


BOXES = [(np.zeros(3), BOX_H)]  # client model: [(centre, half sizes)] in the body frame; set_model() adds the bar


def set_model(boxes):
    """Client model as boxes in its body frame (design data: the main box, the bar and its posts)."""
    global BOXES
    BOXES = [(np.asarray(c, float), np.asarray(h, float)) for c, h in boxes]


def model_from(km, bid):
    """The client body's box geoms from the model (its design data)."""
    import mujoco
    out = []
    for g in range(km.ngeom):
        if km.geom_bodyid[g] == bid and km.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            q = km.geom_quat[g]
            assert np.allclose(np.abs(q), [1, 0, 0, 0], atol=1e-6), "rotated box geom"
            out.append((km.geom_pos[g].copy(), km.geom_size[g].copy()))
    return out


_STACK = {}  # id(boxes list) -> (list, centres (B, 3), half sizes (B, 3))


def _sdf_grad(x, boxes=None):
    """Signed distance of points x (n, 3; body frame) to the union of the model boxes, and its
    gradient (unit), from the nearest box (the first one on a tie). All boxes in one numpy pass
    (wall-time work, 19 Nov): the same element-wise operations as _sdf_grad_loop, so the same bits."""
    boxes = BOXES if boxes is None else boxes
    st = _STACK.get(id(boxes))
    if st is None or st[0] is not boxes:
        st = (boxes, np.array([c for c, _ in boxes], float), np.array([h for _, h in boxes], float))
        _STACK.clear()
        _STACK[id(boxes)] = st
    _, C, H = st
    y = x[:, None, :] - C[None]  # (n, B, 3) box frames
    q = np.abs(y) - H
    out = np.maximum(q, 0.0)
    no = np.linalg.norm(out, axis=2)
    inside = no <= 0
    d = np.where(inside, q.max(axis=2), no)
    n = len(x)
    r = np.arange(n)
    b = d.argmin(axis=1)  # first minimum, as the loop's strict "<"
    yb, qb, ob, nb, ib = y[r, b], q[r, b], out[r, b], no[r, b], inside[r, b]
    s = np.sign(yb)
    s[s == 0] = 1.0
    g = np.zeros_like(yb)
    oi = ~ib
    g[oi] = ob[oi] / nb[oi, None] * s[oi]
    k = qb[ib].argmax(axis=1)
    g[np.where(ib)[0], k] = s[ib, k]
    return d[r, b], g


def _sdf_grad_loop(x, boxes=None):
    """Reference (the loop over boxes, as before 19 Nov); _sdf_grad must equal it bit for bit."""
    boxes = BOXES if boxes is None else boxes
    best_d, best_g = None, None
    for c, h in boxes:
        d, g = _sdf_box(x - c, h)
        if best_d is None:
            best_d, best_g = d, g
        else:
            m = d < best_d
            best_d = np.where(m, d, best_d)
            best_g[m] = g[m]
    return best_d, best_g


def _sdf_box(x, h):
    """Signed distance of points x (n, 3; box frame) to the box and its gradient (unit)."""
    q = np.abs(x) - h
    s = np.sign(x)
    s[s == 0] = 1.0
    out = np.maximum(q, 0.0)
    no = np.linalg.norm(out, axis=1)
    inside = no <= 0
    d = np.where(inside, q.max(axis=1), no)
    g = np.zeros_like(x)
    oi = ~inside
    g[oi] = out[oi] / no[oi, None] * s[oi]
    k = q[inside].argmax(axis=1)
    g[np.where(inside)[0], k] = s[inside, k]
    return d, g


def fit(P, R0, t0, iters=12, gates=(0.30, 0.20, 0.12, 0.08, 0.06), huber=0.02, att=True, min_pts=60,
        sig_floor=0.005):
    """Client pose (R, t: body -> lidar frame) from points P (n, 3; lidar frame) and a prior.
    Returns dict(R, t, Cp (3x3), Ca (3x3, rotation left-perturbation, lidar frame), n, rms, ok)."""
    R, t = R0.copy(), np.asarray(t0, float).copy()
    nd = 6 if att else 3
    n = 0
    for it in range(iters):
        gate = gates[min(it, len(gates) - 1)]
        q = P - t
        x = q @ R  # R^T q, row-wise
        d, g = _sdf_grad(x)
        m = np.abs(d) < gate
        n = int(m.sum())
        if n < min_pts:
            return dict(ok=False, n=n)
        d, g, q = d[m], g[m], q[m]
        gw = g @ R.T  # world (lidar-frame) normals
        J = np.zeros((n, nd))
        J[:, 0:3] = -gw
        if att:
            # R_true = exp(phi) R: x_b = R^T (q + q x phi), dd/dphi = gw^T [q]x = (gw x q)^T
            J[:, 3:6] = np.cross(gw, q)
        w = np.where(np.abs(d) <= huber, 1.0, huber / np.abs(d))
        A = (J * w[:, None]).T @ J
        b = (J * w[:, None]).T @ d
        try:
            dx = -np.linalg.solve(A + 1e-9 * np.eye(nd), b)
        except np.linalg.LinAlgError:
            return dict(ok=False, n=n)
        t = t + dx[0:3]
        if att:
            R = rotvec_to_matrix(dx[3:6]) @ R
        if np.linalg.norm(dx[0:3]) < 1e-5 and (not att or np.linalg.norm(dx[3:6]) < 1e-6) and it >= len(gates) - 1:
            break
    s2 = max(float((w * d * d).sum() / max(w.sum() - nd, 1.0)), sig_floor**2)
    C = s2 * np.linalg.inv(A + 1e-9 * np.eye(nd))
    out = dict(ok=True, R=R, t=t, n=n, rms=float(np.sqrt((d * d).mean())), Cp=C[0:3, 0:3])
    out["Ca"] = C[3:6, 3:6] if att else None
    return out


# ---------------------------------------------------------------------------------------------
# offline check on the P6.2 frames (labels: client centre and attitude in the bus frame)


def cmd_check(a):
    import mujoco
    scene = _paths.ASSETS_DIR / "servicer_rev2" / "servicer_rev2_scene.xml"
    xml = "\n".join(line for line in scene.read_text().splitlines()
                    if "<mjorbit " not in line and "use_gravity_gradient" not in line)  # as GraspSim's plain copy
    import os
    cwd = os.getcwd()
    os.chdir(scene.parent)
    try:
        km = mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(cwd)
    if not a.box_only:
        set_model(model_from(km, mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, a.client_body)))
    print("model boxes:", [(np.round(c, 3).tolist(), np.round(h, 3).tolist()) for c, h in BOXES])
    lid = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, "bus_lidar")
    q = km.cam_quat[lid]
    R_bl = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()  # lidar -> bus
    p_bl = km.cam_pos[lid].copy()
    res = 128
    dirs = ray_dirs(res, km.cam_fovy[lid])
    rng = np.random.default_rng(0)
    files = sorted(Path(a.dir).glob("ep_*.npz"))[: a.eps]
    rows = []
    for fp in files:
        z = np.load(fp)
        for i in range(len(z["pos"])):
            if a.near and not z["kind"][i]:
                continue
            r6 = z["rot6"][i].astype(np.float64)
            a1, a2 = r6[0:3], r6[3:6]
            b1 = a1 / np.linalg.norm(a1)
            b2 = a2 - (b1 @ a2) * b1
            b2 /= np.linalg.norm(b2)
            Rb = np.stack([b1, b2, np.cross(b1, b2)], 1)  # client attitude in the bus frame
            pb = z["pos"][i].astype(np.float64)
            Rt, tt = R_bl.T @ Rb, R_bl.T @ (pb - p_bl)  # truth in the lidar frame
            P = points(z["lidar"][i], dirs)
            if len(P) < 60:
                continue
            for _ in range(a.k):
                e_p = rng.normal(0, a.p0, 3)
                e_a = rng.normal(0, np.radians(a.a0), 3)
                R0 = Rotation.from_rotvec(e_a).as_matrix() @ Rt
                r = fit(P, R0, tt + e_p, att=not a.pos_only)
                if not r["ok"]:
                    rows.append((float(np.linalg.norm(tt)), np.nan, np.nan, np.nan, np.nan, 0, len(P)))
                    continue
                ep = r["t"] - tt
                ea = Rotation.from_matrix(r["R"] @ Rt.T).as_rotvec() if not a.pos_only else np.zeros(3)
                nees = float(ep @ np.linalg.solve(r["Cp"], ep))
                rows.append((float(np.linalg.norm(tt)), 1e3 * float(np.linalg.norm(ep)), 1e3 * float(np.linalg.norm(e_p)),
                             float(np.degrees(np.linalg.norm(ea))), nees, r["n"], len(P)))
    A = np.array(rows)
    ok = ~np.isnan(A[:, 1])
    print(f"{len(A)} fits ({'near' if a.near else 'all'} views, prior {a.p0 * 1e3:.0f} mm / {a.a0:.0f} deg per axis, "
          f"{'position only' if a.pos_only else '6 dof'}): failed {100 * (~ok).mean():.1f}%")
    B = A[ok]
    for lo, hi in ((1.5, 2.5), (2.5, 3.5), (3.5, 5.0), (5.0, 10.0)):
        s = (B[:, 0] >= lo) & (B[:, 0] < hi)
        if s.sum() < 5:
            continue
        b = B[s]
        print(f"  range {lo}-{hi} m: n {s.sum():4d}, points med {np.median(b[:, 6]):5.0f} used {np.median(b[:, 5]):5.0f}; "
              f"pos err med {np.median(b[:, 1]):5.1f} p90 {np.percentile(b[:, 1], 90):5.1f} max {b[:, 1].max():6.1f} mm "
              f"(prior med {np.median(b[:, 2]):4.0f}); att err med {np.median(b[:, 3]):4.2f} p90 {np.percentile(b[:, 3], 90):4.2f} deg; "
              f"pos NEES med {np.median(b[:, 4]):5.1f} (3 ideal) > 7.8: {100 * (b[:, 4] > 7.8).mean():3.0f}%")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("dir")
    c.add_argument("--eps", type=int, default=20)
    c.add_argument("--k", type=int, default=2, help="fits per frame (prior draws)")
    c.add_argument("--p0", type=float, default=0.15, help="prior position error 1-sigma per axis [m]")
    c.add_argument("--a0", type=float, default=3.0, help="prior attitude error 1-sigma per axis [deg]")
    c.add_argument("--near", action="store_true", help="only the near (pre-grasp station) views")
    c.add_argument("--pos-only", action="store_true")
    c.add_argument("--box-only", action="store_true", help="main box only (no bar, no posts)")
    c.add_argument("--client-body", default="target")
    a = ap.parse_args()
    dict(check=cmd_check)[a.cmd](a)


if __name__ == "__main__":
    main()
