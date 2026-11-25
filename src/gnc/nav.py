"""Far-range Kalman filter tracker and attitude-pointing helpers."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from scipy.ndimage import label
from scipy.spatial.transform import Rotation

from specs.client_spec import ClientPerceptionSpec

from .sensors import lidar_fix_gated

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _paths  # noqa: E402,F401
import far_nav  # noqa: E402


def ang_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return 180.0
    return float(np.degrees(np.arccos(np.clip(float(a @ b) / (na * nb), -1.0, 1.0))))


def far_cov(axis_u: np.ndarray, sig_par: float, sig_perp: float) -> np.ndarray:
    """Anisotropic 3x3 measurement covariance: sig_par along axis_u, sig_perp perpendicular."""
    u = axis_u / max(float(np.linalg.norm(axis_u)), 1e-12)
    P_par = np.outer(u, u)
    P_perp = np.eye(3) - P_par
    return (sig_par**2) * P_par + (sig_perp**2) * P_perp


def point_R(bore_b: np.ndarray, want_w: np.ndarray, roll_ref_w: Optional[np.ndarray] = None) -> np.ndarray:
    """Servicer attitude putting body unit vector bore_b onto world unit vector want_w, minimizing roll drift."""
    w = want_w / max(float(np.linalg.norm(want_w)), 1e-12)
    v = np.cross(bore_b, w)
    s, c = float(np.linalg.norm(v)), float(bore_b @ w)
    if s < 1e-12:
        if c > 0:
            R0 = np.eye(3)
        else:
            ref = np.array([1.0, 0.0, 0.0]) if abs(bore_b[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            ax = np.cross(bore_b, ref)
            ax /= np.linalg.norm(ax)
            R0 = Rotation.from_rotvec(np.pi * ax).as_matrix()
    else:
        R0 = Rotation.from_rotvec((v / s) * np.arctan2(s, c)).as_matrix()
    if roll_ref_w is None:
        return R0
    z0 = R0[:, 2]
    zt = roll_ref_w - float(roll_ref_w @ w) * w
    if float(np.linalg.norm(zt)) < 1e-6:
        return R0
    zt /= float(np.linalg.norm(zt))
    cc = float(np.clip(z0 @ zt, -1.0, 1.0))
    ss = float(np.cross(z0, zt) @ w)
    return Rotation.from_rotvec(w * np.arctan2(ss, cc)).as_matrix() @ R0


def _roi_bounds(S: int, Pc: np.ndarray, Rcam: np.ndarray, c_pred: np.ndarray, fovy_deg: float, win_m: float) -> Optional[Tuple[float, int, int, int, int]]:
    """Project c_pred into image and return (f, x0, x1, y0, y1) for a square half-width win_m crop."""
    f = (S / 2.0) / float(np.tan(np.radians(fovy_deg) / 2.0))
    t = Rcam.T @ (np.asarray(c_pred, float) - Pc)
    if -t[2] < 1.0:
        return None
    u0, v0 = S / 2.0 + f * t[0] / -t[2], S / 2.0 - f * t[1] / -t[2]
    h = f * win_m / -t[2]
    x0, x1 = int(max(u0 - h, 0)), int(min(u0 + h, S))
    y0, y1 = int(max(v0 - h, 0)), int(min(v0 + h, S))
    return f, x0, x1, y0, y1


def _roi_has_earth_bg(img: np.ndarray, Pc: np.ndarray, Rcam: np.ndarray, c_pred: np.ndarray, fovy_deg: float, win_m: float) -> bool:
    """Truth-free check: reject cam_bearing if extended background (Earth limb) intersects the ROI crop."""
    roi = _roi_bounds(img.shape[0], Pc, Rcam, c_pred, fovy_deg, 1.25 * win_m)
    if roi is None:
        return True
    _, x0, x1, y0, y1 = roi
    if x1 - x0 < 6 or y1 - y0 < 6:
        return True
    g = img[y0:y1, x0:x1].astype(np.float32).mean(axis=2)
    border = np.concatenate([g[0, :], g[-1, :], g[:, 0], g[:, -1]])
    if float(np.max(border)) > far_nav.CAM_THR or float((border > 20.0).mean()) > 0.05:
        return True
    if float(np.median(border)) > 4.5 or float(np.median(g)) > 15.0 or float((g > far_nav.CAM_THR).mean()) > 0.25:
        return True
    return False


def _clean_cam_bearing(
    img: np.ndarray,
    Pc: np.ndarray,
    Rcam: np.ndarray,
    c_pred: np.ndarray,
    fovy_deg: float,
    win_m: float,
    min_cc_px: int = 8,
    bus_net=None,
) -> Optional[np.ndarray]:
    """Earth-limb & star-speck filtered bearing, with optional BusNet orbital detector."""
    S = img.shape[0]
    roi = _roi_bounds(S, Pc, Rcam, c_pred, fovy_deg, win_m)
    if roi is None:
        return None
    f, x0, x1, y0, y1 = roi
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None

    if bus_net is not None:
        m, sig = bus_net.predict(img[y0:y1, x0:x1])
        sig_m = float(np.hypot(sig[0], sig[1])) * win_m
        if sig_m <= 1.2 and abs(float(m[0])) <= 1.05 and abs(float(m[1])) <= 1.05:
            uc = 0.5 * (x0 + x1) + float(m[0]) * 0.5 * (x1 - x0)
            vc = 0.5 * (y0 + y1) + float(m[1]) * 0.5 * (y1 - y0)
            dw = Rcam @ np.array([(uc - S / 2.0) / f, -(vc - S / 2.0) / f, -1.0])
            return dw / max(float(np.linalg.norm(dw)), 1e-12)
        return None

    if _roi_has_earth_bg(img, Pc, Rcam, c_pred, fovy_deg, win_m):
        return None
    g = img[y0:y1, x0:x1].astype(np.float32).mean(axis=2)
    lbl, n_cc = label(g > far_nav.CAM_THR, structure=np.ones((3, 3)))
    if n_cc == 0:
        return None
    sizes = np.bincount(lbl.ravel())
    keep = np.zeros_like(sizes, dtype=bool)
    keep[1:] = sizes[1:] >= min_cc_px
    m = keep[lbl]
    if int(m.sum()) < 6:
        return None
    ys, xs = np.nonzero(m)
    w = g[m]
    uc = x0 + float((xs * w).sum() / w.sum()) + 0.5
    vc = y0 + float((ys * w).sum() / w.sum()) + 0.5
    dw = Rcam @ np.array([(uc - S / 2.0) / f, -(vc - S / 2.0) / f, -1.0])
    return dw / max(float(np.linalg.norm(dw)), 1e-12)


class FarNavTracker:
    """Manages FarKF + delayed camera-gated lidar / optical bearing measurement delivery."""

    def __init__(
        self,
        r0: np.ndarray,
        v0: np.ndarray,
        *,
        sp: float = 3.0,
        sv: float = 0.01,
        orbit_n: float = 0.0,
        perception: ClientPerceptionSpec,
        lat_s: float = 0.15,
        lat_j_s: float = 0.02,
        rng: Optional[np.random.Generator] = None,
        bus_net=None,
    ):
        self.kf = far_nav.FarKF(r0, v0, sp=sp, sv=sv, n=orbit_n)
        self.perc = perception
        self.lat_s = float(lat_s)
        self.lat_j_s = float(lat_j_s)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.bus_net = bus_net
        self.pending: list = []
        self.n_lid = 0
        self.n_gated = 0
        self.n_cam = 0

    def queue_measurement(
        self,
        t_now: float,
        P: np.ndarray,
        Pl_nom: np.ndarray,
        Rl_nom: np.ndarray,
        Pc_nom: np.ndarray,
        Rc_nom: np.ndarray,
        img_bus_cam: np.ndarray,
        com_s: np.ndarray,
        fovy_deg: float,
    ) -> None:
        c_pred = com_s + self.kf.x[:3]
        b_est = _clean_cam_bearing(img_bus_cam, Pc_nom, Rc_nom, c_pred, fovy_deg, self.perc.cam_win_m, bus_net=self.bus_net)
        lg = lidar_fix_gated(P, Pl_nom, Rl_nom, Pc_nom, b_est, perc=self.perc)
        lat = self.lat_s + (float(self.rng.uniform(-self.lat_j_s, self.lat_j_s)) if self.lat_s > 0 else 0.0)
        self.pending.append((t_now + lat, t_now, com_s.copy(), Pc_nom.copy(), b_est, lg))

    def deliver(self, now: float) -> None:
        while self.pending and self.pending[0][0] <= now + 1e-9:
            _, t_m, com_m, Pc_m, b_est, lg = self.pending.pop(0)
            dt_l = now - t_m
            D_est = max(float(np.linalg.norm(self.kf.x[:3])), 2.0)
            if lg is not None:
                c_est, _, was_gated, _ = lg
                z = (c_est - com_m) + self.kf.x[3:] * dt_l
                los_u = z / max(float(np.linalg.norm(z)), 1e-9)
                sig_los = 0.15 + 0.005 * D_est
                sig_lat = (0.30 + 0.015 * D_est) if was_gated else 1.5
                ok, _ = self.kf.update(z, far_cov(los_u, sig_los, sig_lat), gate=far_nav.GATE)
                if ok:
                    self.n_lid += 1
                    if was_gated:
                        self.n_gated += 1
                    self.kf.t_fix = t_m
            if b_est is not None:
                r_proj = float(np.dot(self.kf.x[:3] + (com_m - Pc_m), b_est))
                if r_proj > 1.0:
                    z_c = (Pc_m + r_proj * b_est - com_m) + self.kf.x[3:] * dt_l
                    ok, _ = self.kf.update(z_c, far_cov(b_est, 100.0, 0.35 + 0.015 * D_est), gate=25.0)
                    if ok:
                        self.n_cam += 1

    def predict(self, dt: float, a_own: np.ndarray, t: float) -> None:
        self.kf.predict(dt, a_own, t)

    def rel_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Estimated (r_client_minus_com, v_client_minus_com) and inverse convenience."""
        return self.kf.x[:3].copy(), self.kf.x[3:].copy()
