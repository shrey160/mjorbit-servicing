"""Far leg (plan step 1, 19 Nov; decision D3): 30 m -> the 8 m hold point on the servicer's own sensors.

Position only (no attitude needed this far out, learning/RESEARCH_SERVICING.md "P6 far leg"):
  FarKF       relative position and velocity of the client centre (world axes, client minus the
              servicer's centre of mass). Prediction with the servicer's own commanded acceleration and
              the tidal (Hill) term, as VisEst; white acceleration noise.
  lidar_fix   bus_lidar returns (own bodies masked) within 1.5 m of their median, mean moved BLOB_BACK
              along the line of sight (the returns lie on the near faces): a 3D position, sigma
              LIDAR_SP + LIDAR_SP_R x range per axis (the 8 m blob errors are a 0.12-0.35 m bias per seed).
  cam_bearing bus_cam render, the pixels brighter than CAM_THR inside a window around the predicted
              client: their centroid is a bearing; used as a position update across the line of sight
              (sigma CAM_LAT_M, the lit part's centroid sits up to half a client off the centre).
  FarPath     bus reference: hold at the start until the filter has converged, then a min-jerk straight
              line in inertial axes (relative to the ESTIMATED client centre) to the aim point, peak
              speed v_max; the path clock stops while the lidar track is stale (lost track -> hold).
The aim point (step 1 only) is the true-axis hold point the episode used to start at; step 2 replaces it.
"""
from __future__ import annotations

import numpy as np

LIDAR_SP, LIDAR_SP_R = 0.25, 0.005  # m, per m of range
CAM_LAT_M = 0.5  # m across the line of sight
CAM_THR = 40  # 8-bit brightness: client pixels against black space
BLOB_BACK = 0.4
SIG_ACC = 5e-5  # m/s^2 white acceleration noise (thrust error, unmodelled forces)
GATE = 25.0  # chi-square gate for a lidar fix (3 dof)


def _unit(v):
    return v / max(float(np.linalg.norm(v)), 1e-12)


def _mj(s):
    s = min(max(s, 0.0), 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5, 30 * s**2 - 60 * s**3 + 30 * s**4, 60 * s - 180 * s**2 + 120 * s**3


class FarKF:
    """x = [r, v]: client centre minus servicer CoM and its rate, world axes."""

    def __init__(self, r0, v0, sp, sv, n):
        self.x = np.r_[np.asarray(r0, float), np.asarray(v0, float)]
        self.P = np.diag([sp**2] * 3 + [sv**2] * 3)
        self.n = n
        self.n_lidar = self.n_cam = self.rej = 0
        self.t_fix = -np.inf  # time of the last accepted lidar fix

    def predict(self, dt, a_own, t):
        r, v = self.x[:3], self.x[3:]
        th = self.n * t
        u = np.array([np.cos(th), np.sin(th), 0.0])
        a = -np.asarray(a_own, float) - self.n**2 * (r - 3.0 * (r @ u) * u)  # as VisEst's tide term
        self.x = np.r_[r + v * dt + 0.5 * a * dt**2, v + a * dt]
        F = np.eye(6)
        F[:3, 3:] = dt * np.eye(3)
        G = np.r_[0.5 * dt**2 * np.eye(3), dt * np.eye(3)]
        self.P = F @ self.P @ F.T + SIG_ACC**2 * G @ G.T

    def update(self, z, R, gate=None):
        H = np.c_[np.eye(3), np.zeros((3, 3))]
        y = np.asarray(z, float) - self.x[:3]
        S = H @ self.P @ H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        if gate is not None and d2 > gate:
            self.rej += 1
            return False, d2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        return True, d2

    def sig(self):
        return float(np.sqrt(np.trace(self.P[:3, :3]))), float(np.sqrt(np.trace(self.P[3:, 3:])))


def lidar_fix(P, Pl, Rl, max_off_deg=20.0):
    """Client centre (world) from one lidar frame (points P in the lidar frame, lidar pose Pl, Rl), or None."""
    if len(P) < 10:
        return None
    W = Pl + P @ Rl.T
    W = W[np.linalg.norm(W - np.median(W, axis=0), axis=1) < 1.5]
    if len(W) < 10:
        return None
    c = W.mean(axis=0)
    c = c + BLOB_BACK * _unit(c - Pl)
    cl = Rl.T @ (c - Pl)
    if np.degrees(np.arccos(np.clip(-cl[2] / np.linalg.norm(cl), -1, 1))) > max_off_deg:
        return None  # near the edge of the view: part of the client only
    return c, len(W)


def cam_bearing(img, Pc, Rcam, c_pred, fovy_deg, win_m=3.0):
    """Unit bearing (world) to the centroid of the bright pixels within win_m of the predicted centre, or
    None (nothing lit there, or the window leaves the image)."""
    S = img.shape[0]
    f = (S / 2) / np.tan(np.radians(fovy_deg) / 2)
    t = Rcam.T @ (np.asarray(c_pred, float) - Pc)
    if -t[2] < 1.0:
        return None
    u0, v0 = S / 2 + f * t[0] / -t[2], S / 2 - f * t[1] / -t[2]
    h = f * win_m / -t[2]
    x0, x1 = int(max(u0 - h, 0)), int(min(u0 + h, S))
    y0, y1 = int(max(v0 - h, 0)), int(min(v0 + h, S))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    g = img[y0:y1, x0:x1].astype(np.float32).mean(axis=2)
    m = g > CAM_THR
    if m.sum() < 6:
        return None
    ys, xs = np.nonzero(m)
    w = g[m]
    uc, vc = x0 + float((xs * w).sum() / w.sum()) + 0.5, y0 + float((ys * w).sum() / w.sum()) + 0.5
    d = np.array([(uc - S / 2) / f, -(vc - S / 2) / f, -1.0])
    return _unit(Rcam @ d)


class FarPath:
    """Bus reference for the far leg, relative to the estimated client centre (world axes). Interface as
    HoldRef / FlyIn: reference(t, pc, vc, Rc, w_c, R_rel0, k) -> (p, v, a, R, w); t is ignored, the
    path clock tau is advanced by FarLeg."""
    total = 1e9

    def __init__(self, r_start, r_aim, v_max):
        self.r0, self.r1 = np.asarray(r_start, float), np.asarray(r_aim, float)
        L = float(np.linalg.norm(self.r1 - self.r0))
        self.T = max(1.875 * L / v_max, 30.0)
        self.tau, self.k = 0.0, 0.0  # path clock, rate factor (0: hold)

    def rel(self):
        p, v, a = _mj(self.tau / self.T)
        d = self.r1 - self.r0
        return self.r0 + p * d, self.k * v / self.T * d, self.k**2 * a / self.T**2 * d

    def reference(self, t, pc, vc, Rc, w_c, R_rel0, k=1.0):
        r, v, a = self.rel()
        return pc + r, vc + v, a, Rc @ R_rel0, w_c

    def done(self):
        return self.tau >= self.T
