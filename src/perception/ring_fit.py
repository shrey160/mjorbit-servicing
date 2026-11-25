"""P7.3: adapter-ring centre and axis from the servicer's flash lidar (SPEC_7 section 5; user 19 Nov: lidar fit first).

The client's 610 mm launch adapter ring (client_p7, design data) stands 60 mm proud of the +X face:
  wall  r 0.283 .. 0.305 m, h 0 .. 0.060 m above the face;
  lip   r 0.305 .. 0.317 m, h 0.045 .. 0.060 m (the clamp lip, the hook's purchase);
  face  the bus's +X face (x = 0.75 in the body frame), 1.0 x 1.0 m; a small thruster inside the ring and four
        small thrusters near the face corners (clutter).

Method (classical, no training; the P6 box fit gives the prior):
  A  face plane: robust (IRLS, Huber) plane fit to the face points around the ring (prior frame, shrinking gates)
     -> the axis and the face height;
  B  ring top: points 30-90 mm above that plane, radius 0.22-0.40 m from the prior centre -> robust geometric circle
     fit in the plane -> the centre in the plane and a radius check (the top annulus, r 0.283-0.317);
  C  model fit: Gauss-Newton (Huber) of the centre (3) and axis (2) on the signed distance of every face-region point
     to the ring section model in (h, rho) (face half-space U wall U lip), so the inner and outer edges, the lip and
     the face are all in the model and cannot be swapped (the DLR interface-ring pitfall); the main thruster inside
     the ring (rho < 0.12) and points beyond the face edge are left out.
The ring is rotationally symmetric, so the fit gives no roll; the roll (and so the grasp azimuth in the client frame)
comes from the box fit, as SPEC_7 section 5 says.

  fit(P, R0, t0) -> dict(ok, c, n, rad, n_top, n_pts, rms, Cc, Cn, why)   (lidar frame; c = ring centre on the face)
"""
from __future__ import annotations

import numpy as np

# design data (client_p7, body frame)
FACE_X = 0.75
C_B = np.array([FACE_X, 0.0, 0.0])        # ring centre on the face
N_B = np.array([1.0, 0.0, 0.0])           # ring axis (out of the face)
R_IN, R_WALL, R_LIP = 0.283, 0.305, 0.317
H_TOP, H_LIP = 0.060, 0.045
FACE_H = 0.5                              # face half size (y, z)
RHO_MIN = 0.12                            # main thruster inside the ring left out
R_TOP = 0.5 * (R_IN + R_LIP)              # the top annulus' mean radius (circle-fit check)


def _frame(n):
    """Two unit vectors orthogonal to n."""
    a = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, a)
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def _huber_w(r, k):
    a = np.abs(r)
    return np.where(a <= k, 1.0, k / np.maximum(a, 1e-12))


def _plane(Q, n0, d0, gates=(0.10, 0.05, 0.03, 0.02), k=0.01):
    """Robust plane n . q = d through points Q from (n0, d0)."""
    n, d = n0.copy(), d0
    m = np.zeros(len(Q), bool)
    for g in gates:
        r = Q @ n - d
        m = np.abs(r) < g
        if m.sum() < 20:
            return None
        for _ in range(3):
            q = Q[m]
            w = _huber_w(q @ n - d, k)
            mu = (w[:, None] * q).sum(0) / w.sum()
            C = ((q - mu) * w[:, None]).T @ (q - mu)
            ev, evec = np.linalg.eigh(C)
            nn = evec[:, 0]
            n = nn if nn @ n > 0 else -nn
            d = float(n @ mu)
    return n, d, int(m.sum())


def _circle(X, c0, r0, k=0.01, iters=15):
    """Robust geometric circle fit to 2D points X from (c0, r0): residual |x - c| - r."""
    c, r = c0.copy(), r0
    for _ in range(iters):
        v = X - c
        rho = np.linalg.norm(v, axis=1)
        res = rho - r
        w = _huber_w(res, k)
        J = np.column_stack([-v / np.maximum(rho, 1e-9)[:, None], -np.ones(len(X))])
        A = (J * w[:, None]).T @ J
        b = (J * w[:, None]).T @ res
        dx = -np.linalg.solve(A + 1e-9 * np.eye(3), b)
        c, r = c + dx[:2], r + dx[2]
        if np.linalg.norm(dx) < 1e-6:
            break
    return c, r


def _rect_sdf(h, rho, h0, h1, r0, r1):
    """2D signed distance to the rectangle [h0, h1] x [r0, r1] in (h, rho)."""
    dh = np.maximum(h0 - h, h - h1)
    dr = np.maximum(r0 - rho, rho - r1)
    out = np.hypot(np.maximum(dh, 0.0), np.maximum(dr, 0.0))
    return np.where((dh <= 0) & (dr <= 0), np.maximum(dh, dr), out)


def section_sdf(h, rho, r_in=R_IN, r_wall=R_WALL, r_lip=R_LIP, h_top=H_TOP, h_lip=H_LIP):
    """Signed distance in the ring's (height above the face, radius) half plane to face U wall U lip."""
    face = h                                  # the face half-space h <= 0 (points kept within the face)
    wall = _rect_sdf(h, rho, -0.01, h_top, r_in, r_wall)
    lip = _rect_sdf(h, rho, h_lip, h_top, r_wall - 0.002, r_lip)
    return np.minimum(face, np.minimum(wall, lip))


def _hr(Q, c, n):
    v = Q - c
    h = v @ n
    rho = np.linalg.norm(v - h[:, None] * n, axis=1)
    return h, rho


def _model_fit(
    Q,
    c,
    n,
    gates=(0.04, 0.025, 0.015, 0.01),
    k=0.008,
    iters=12,
    r_in=R_IN,
    r_wall=R_WALL,
    r_lip=R_LIP,
    h_top=H_TOP,
    h_lip=H_LIP,
    rho_min=RHO_MIN,
):
    """Gauss-Newton on (centre 3, axis tilt 2) of section_sdf; numeric Jacobian (the sdf is piecewise smooth)."""
    eps = 1e-4
    nd = 5
    A = np.eye(nd)
    w = r = None
    for it in range(iters):
        g = gates[min(it, len(gates) - 1)]
        u, v = _frame(n)
        h, rho = _hr(Q, c, n)
        r = section_sdf(h, rho, r_in=r_in, r_wall=r_wall, r_lip=r_lip, h_top=h_top, h_lip=h_lip)
        m = (np.abs(r) < g) & (rho > rho_min)
        if m.sum() < 40:
            return None
        q, r = Q[m], r[m]
        J = np.zeros((len(q), nd))
        for j in range(nd):
            dc, dn = np.zeros(3), n.copy()
            if j < 3:
                dc[j] = eps
            else:
                dn = n + eps * (u if j == 3 else v)
                dn /= np.linalg.norm(dn)
            hh, rr = _hr(q, c + dc, dn)
            J[:, j] = (section_sdf(hh, rr, r_in=r_in, r_wall=r_wall, r_lip=r_lip, h_top=h_top, h_lip=h_lip) - r) / eps
        w = _huber_w(r, k)
        A = (J * w[:, None]).T @ J
        b = (J * w[:, None]).T @ r
        dx = -np.linalg.solve(A + 1e-8 * np.eye(nd), b)
        c = c + dx[:3]
        n = n + dx[3] * u + dx[4] * v
        n /= np.linalg.norm(n)
        if np.linalg.norm(dx) < 2e-6 and it >= len(gates) - 1:
            break
    s2 = max(float((w * r * r).sum() / max(w.sum() - nd, 1.0)), 0.003 ** 2)
    C = s2 * np.linalg.inv(A + 1e-8 * np.eye(nd))
    return c, n, C, int(len(r)), float(np.sqrt((r * r).mean()))


def _unpack_ring_spec(spec):
    if spec is None:
        return C_B, N_B, R_IN, R_WALL, R_LIP, H_TOP, H_LIP, FACE_H, RHO_MIN
    return (
        np.asarray(spec.c_b, float),
        np.asarray(spec.n_b, float) / max(float(np.linalg.norm(spec.n_b)), 1e-12),
        float(spec.r_in),
        float(spec.r_wall),
        float(spec.r_lip),
        float(spec.h_top),
        float(spec.h_lip),
        float(spec.face_half_size),
        float(spec.rho_min),
    )


def fit(P, R0, t0, min_top=12, rad_tol=0.02, max_shift=0.15, max_tilt_deg=10.0, stage_c=True, spec=None):
    """Ring centre c (on the face) and axis n in the lidar frame from points P (n, 3; lidar frame) and a client pose
    prior (R0, t0: body -> lidar). ok False with a reason (why) if a stage fails or the result disagrees with the
    prior by more than max_shift / max_tilt_deg (the box fit is the cross-check)."""
    c_b, n_b, r_in, r_wall, r_lip, h_top, h_lip, face_h, rho_min = _unpack_ring_spec(spec)
    P = np.asarray(P, float)
    c0 = t0 + R0 @ c_b
    n0 = R0 @ n_b
    out = dict(ok=False, c=c0, n=n0, rad=np.nan, n_top=0, n_pts=0, rms=np.nan, Cc=None, Cn=None, why="")
    # face region in the prior frame (generous: the prior can be 10 cm / 5 deg off)
    v = P - c0
    h = v @ n0
    rho = np.linalg.norm(v - h[:, None] * n0, axis=1)
    reg = (np.abs(h) < 0.20) & (rho < face_h - 0.03)
    Q = P[reg]
    if len(Q) < 40:
        out["why"] = f"few face points ({len(Q)})"
        return out
    # A: face plane, from the points around the ring (inside and outside), not on the ring
    hq, rq = _hr(Q, c0, n0)
    fm = ((rq > r_lip + 0.03) | ((rq < r_in - 0.03) & (rq > rho_min))) & (hq < 0.03)
    pl = _plane(Q[fm], n0, float(n0 @ c0)) if fm.sum() >= 20 else None
    if pl is None:
        out["why"] = "face plane"
        return out
    n, d, _ = pl
    c = c0 - (n @ c0 - d) * n                  # prior centre dropped on the fitted face
    hq, _ = _hr(Q[fm], c, n)
    sig = float(1.4826 * np.median(np.abs(hq - np.median(hq))))   # range noise on the face (MAD)
    out["sig"] = sig
    # B: matched filter in the face plane
    h, _ = _hr(Q, c, n)
    near = np.abs(h) < max(0.15, 4 * sig)
    u, w = _frame(n)
    X = np.column_stack([(Q[near] - c) @ u, (Q[near] - c) @ w])
    hn = h[near]
    best = (-np.inf, np.zeros(2), 0)
    for span, step, ctr in ((0.08, 0.004, None), (0.006, 0.001, "best")):
        c2 = best[1] if ctr else np.zeros(2)
        g = np.arange(-span, span + 1e-9, step)
        G = c2 + np.stack(np.meshgrid(g, g, indexing="ij"), -1).reshape(-1, 2)
        r2c = np.hypot(X[:, 0] - c2[0], X[:, 1] - c2[1])
        keep = (r2c > r_in - 0.05 - 1.5 * span) & (r2c < r_lip + 0.05 + 1.5 * span)
        Xk, hk = X[keep], hn[keep]
        b1, b2, b3, b4 = r_in ** 2, r_lip ** 2, (r_in - 0.05) ** 2, (r_in - 0.01) ** 2
        b5, b6 = (r_lip + 0.01) ** 2, (r_lip + 0.05) ** 2
        for i0 in range(0, len(G), 256):
            Gc = G[i0:i0 + 256]
            dx = Xk[None, :, 0] - Gc[:, None, 0]
            dy = Xk[None, :, 1] - Gc[:, None, 1]
            r2 = dx * dx + dy * dy
            band = (r2 > b1) & (r2 < b2)
            nb = ((r2 > b3) & (r2 < b4)) | ((r2 > b5) & (r2 < b6))
            n1, n2 = band.sum(1), nb.sum(1)
            with np.errstate(invalid="ignore", divide="ignore"):
                s = (band @ hk) / n1 - (nb @ hk) / n2
            s = np.where((n1 < min_top) | (n2 < min_top), -np.inf, s)
            k = int(np.argmax(s))
            if s[k] > best[0]:
                best = (float(s[k]), Gc[k].copy(), int(n1[k]))
    step_h, cc, nband = best
    out["n_top"], out["rad"] = nband, float(step_h)       # rad: the measured ring height step (design 60 mm)
    if not np.isfinite(step_h) or step_h < 0.5 * h_top or step_h > 1.6 * h_top:
        out["why"] = f"ring step {step_h * 1e3:.0f} mm" if np.isfinite(step_h) else "few ring points"
        return out
    c = c + cc[0] * u + cc[1] * w
    Cc, Cn = None, None
    npts, rms = nband, np.nan
    # C: full section model (gates and Huber width scaled to the range noise)
    if stage_c:
        s = max(sig, 0.004)
        r = _model_fit(
            Q,
            c,
            n,
            gates=(max(0.04, 3 * s), max(0.025, 2 * s), 0.015, 0.01),
            k=0.008,
            r_in=r_in,
            r_wall=r_wall,
            r_lip=r_lip,
            h_top=h_top,
            h_lip=h_lip,
            rho_min=rho_min,
        )
        if r is None:
            out["why"] = "model fit"
            return out
        c, n, C, npts, rms = r
        u, w = _frame(n)
        Cc = C[:3, :3]
        Cn = C[3:, 3:]
    out.update(c=c, n=n, n_pts=npts, rms=rms, Cc=Cc, Cn=Cn)
    shift = float(np.linalg.norm(c - c0))
    tilt = float(np.degrees(np.arccos(np.clip(n @ n0, -1, 1))))
    if shift > max_shift or tilt > max_tilt_deg:
        out["why"] = f"disagrees with the box fit ({shift * 1e3:.0f} mm, {tilt:.1f} deg)"
        return out
    out["ok"] = True
    return out
