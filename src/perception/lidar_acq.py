"""Truth-free 8 m lidar-only 6-DOF attitude & position acquisition for client_p7.

Exploits client_p7's asymmetric geometry (1.5 x 1.0 x 1.0 m bus + one 3-panel +Y solar wing extending to y = 3.84 m,
thin along X and offset to x = +0.2 m):
  1. Given lidar points P (sensor frame) and an approximate bus centre t_bus (from camera-gated FarKF, ~0.2 m error):
  2. Wing points (1.0 < ||p - t_bus|| < 4.5 m) give +Y_body (direction from bus to wing) and +-X_body (wing plane
     normal, smallest eigenvector of wing covariance).
  3. Test both (+X, +Y) and (-X, +Y) (and if wing is edge-on, a 4-angle roll grid around +Y) with lidar_nav.fit(P, R0,
     t_bus), scoring each converged fit by median point-to-model residual (Huber-trimmed) so the x = +0.2 m wing offset
     and radiator / bus aspect ratio disambiguate +X from -X without any truth or neural net.

Self-test (25 random attitudes at 8 m):
  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/perception/lidar_acq.py --n 25
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
import _paths  # noqa: E402,F401
import lidar_nav  # noqa: E402
import ring_fit  # noqa: E402


def _score_fit(P, R, t, client_spec=None):
    """Point-to-box residual + ring-fit bonus when ring normal faces the sensor."""
    Pb = (P - t) @ R
    d = np.full(len(Pb), 1e9)
    for c, h in lidar_nav.BOXES:
        q = np.abs(Pb - c) - h
        dist = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(np.max(q, axis=1), 0.0)
        d = np.minimum(d, np.abs(dist))
    sc = float(np.mean(np.minimum(d, 0.15))) + 0.5 * float(np.percentile(np.minimum(d, 0.25), 90))
    ring_spec = client_spec.ring if client_spec is not None else None
    n_b = ring_spec.n_arr if ring_spec is not None else ring_fit.N_B
    n_x = R @ n_b
    los = -t / max(float(np.linalg.norm(t)), 1e-9)
    if float(n_x @ los) > 0.35:
        rf = ring_fit.fit(P, R, t, spec=ring_spec)
        if rf and rf.get("ok"):
            sc -= 0.025
    return sc


def _pca_cands(wing_pts: np.ndarray, t_bus: np.ndarray, signed: bool) -> list[np.ndarray]:
    w_mu = wing_pts.mean(axis=0)
    w_rel = wing_pts - w_mu
    _, evecs = np.linalg.eigh(w_rel.T @ w_rel)
    y_base = evecs[:, 2]
    y_signs = [y_base]
    if signed:
        y_dir = w_mu - t_bus
        y_dir /= max(float(np.linalg.norm(y_dir)), 1e-9)
        if float(y_base @ y_dir) < 0:
            y_signs = [-y_base]
    else:
        y_signs = [y_base, -y_base]

    cands = []
    for y_hat in y_signs:
        u0 = evecs[:, 0]
        u0 = u0 - float(u0 @ y_hat) * y_hat
        if float(np.linalg.norm(u0)) < 1e-6:
            ref = np.array([1.0, 0.0, 0.0]) if abs(y_hat[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
            u0 = np.cross(ref, y_hat)
        u0 /= max(float(np.linalg.norm(u0)), 1e-9)
        v0 = np.cross(y_hat, u0)
        for ang in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False):
            x_hat = np.cos(ang) * u0 + np.sin(ang) * v0
            z_hat = np.cross(x_hat, y_hat)
            cands.append(np.column_stack([x_hat, y_hat, z_hat]))
    return cands


def _bus_seed_from_pts(P: np.ndarray, r_cluster: float = 0.90, depth_m: float = 0.45) -> np.ndarray:
    """Estimate bus center from the densest point cluster in P (robust to FarKF wing bias)."""
    idx = np.linspace(0, len(P) - 1, min(200, len(P)), dtype=int)
    S = P[idx]
    d2 = ((S[:, None, :] - P[None, :, :]) ** 2).sum(axis=2)
    counts = (d2 < (r_cluster * r_cluster)).sum(axis=1)
    best = int(np.argmax(counts))
    c = P[d2[best] < (r_cluster * r_cluster)].mean(axis=0)
    los = c / max(float(np.linalg.norm(c)), 1e-9)
    return c + depth_m * los


def _cands_for_seed(P: np.ndarray, t_seed: np.ndarray, strategy: str, r_min: float, r_max: float) -> tuple[list[np.ndarray], int]:
    rel = P - t_seed
    dist = np.linalg.norm(rel, axis=1)
    wing = P[(dist > r_min) & (dist < r_max)]

    cands: list[np.ndarray] = []
    if strategy in ("single_wing_pca", "multi_wing_pca") and len(wing) >= 20:
        cands = _pca_cands(wing, t_seed, signed=(strategy == "single_wing_pca"))
    else:
        for ax in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            x_hat = np.array(ax, float)
            ref = np.array([0, 0, 1.0]) if abs(x_hat[2]) < 0.9 else np.array([1.0, 0, 0])
            y_hat = np.cross(ref, x_hat)
            y_hat /= np.linalg.norm(y_hat)
            z_hat = np.cross(x_hat, y_hat)
            cands.append(np.column_stack([x_hat, y_hat, z_hat]))
            if strategy == "box_grid":
                for roll in (0.5 * np.pi, np.pi, 1.5 * np.pi):
                    Rot = Rotation.from_rotvec(roll * x_hat).as_matrix()
                    cands.append(np.column_stack([x_hat, Rot @ y_hat, Rot @ z_hat]))
    return cands, len(wing)


def acquire(P: np.ndarray, t_bus: np.ndarray, client_spec=None):
    """Return dict(ok, R, t, score, n_cand) in the lidar frame from points P and approximate bus centre t_bus."""
    if len(P) < 40:
        return dict(ok=False, why="few_pts")
    t_bus = np.asarray(t_bus, float)
    perc = client_spec.perception if client_spec is not None else None
    strategy = perc.acq_strategy if perc is not None else "single_wing_pca"
    r_min = perc.acq_shell_min_m if perc is not None else 1.15
    r_max = perc.acq_shell_max_m if perc is not None else 4.5

    t_pc = _bus_seed_from_pts(P)
    seeds = [t_bus] if float(np.linalg.norm(t_pc - t_bus)) < 0.25 else [t_pc, t_bus]

    best = None
    total_cand = 0
    for t_seed in seeds:
        cands, n_wing = _cands_for_seed(P, t_seed, strategy, r_min, r_max)
        total_cand += len(cands)
        for R0 in cands:
            bf = lidar_nav.fit(P, R0, t_seed)
            if bf and bf.get("ok"):
                sc = _score_fit(P, bf["R"], bf["t"], client_spec=client_spec)
                if best is None or sc < best["score"]:
                    best = dict(ok=True, R=bf["R"], t=bf["t"], score=sc, n_cand=total_cand, n_wing=n_wing)
        if best is not None and best["score"] < 0.08:
            break
    if best is None:
        return dict(ok=False, why="no_fit", n_cand=total_cand, n_wing=0)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--D", type=float, default=8.0)
    a = ap.parse_args()
    import cams
    import far_bench as FB
    import ring_bench as RB
    cams.LIDAR_MAX = 60.0
    W = RB.World()
    t0 = time.time()
    ok_n, good_n = 0, 0
    pos_errs, ang_errs = [], []
    for i in range(a.n):
        seed = 7530000 + i
        # Random direction (NOT restricted to ring axis!) so client_p7 can be seen from ANY side at 8 m
        tr = FB._place_far(W, seed, a.D if a.D > 12.0 else 20.0)  # get random attitude & direction
        # Re-place at exact D along random direction u
        km, kd = W.km, W.kd
        rng = tr["rng"]
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
        pL = a.D * u
        q = W.base["quat"][W.lid]
        Rl_b = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        bore_b = -Rl_b[:, 2]
        want = RB._rand_unit_cone(rng, -u, 2.0)
        Rs = RB._rot_between(bore_b, want)
        Rs = Rotation.from_rotvec(want * rng.uniform(-np.pi, np.pi)).as_matrix() @ Rs
        ps = pL - Rs @ W.base["pos"][W.lid]
        kd.qpos[W.cq:W.cq + 7] = [*ps, *RB._q_wxyz(Rs)]
        kd.qpos[W.arm] = W.poses["ready"]
        import mujoco
        mujoco.mj_forward(km, kd)

        P, _ = W.lidar_points()
        Pl_nom, Rl_nom = W.cam_nominal(W.lid)
        R_tr = Rl_nom.T @ tr["Rc"]
        t_tr = Rl_nom.T @ (tr["pc"] - Pl_nom)
        # Simulated FarKF 8 m bus position estimate (+-0.2 m error)
        t_bus = t_tr + rng.normal(0.0, 0.12, 3)
        res = acquire(P, t_bus)
        if res["ok"]:
            ok_n += 1
            pe = float(np.linalg.norm(res["t"] - t_tr)) * 1e3
            ae = float(np.degrees(np.linalg.norm(Rotation.from_matrix(res["R"].T @ R_tr).as_rotvec())))
            pos_errs.append(pe)
            ang_errs.append(ae)
            if ae < 10.0 and pe < 250.0:
                good_n += 1
            if ae >= 10.0:
                print(f"  seed {seed}: MISLOCK ae={ae:.1f} deg pe={pe:.1f} mm score={res['score']:.4f} n_wing={res['n_wing']}")
    print(json.dumps(dict(
        D=a.D, n=a.n, ok=ok_n, good_10deg=good_n,
        pos_mm_p50_95=[round(float(np.percentile(pos_errs, 50)), 1), round(float(np.percentile(pos_errs, 95)), 1)] if pos_errs else None,
        ang_deg_p50_95=[round(float(np.percentile(ang_errs, 50)), 2), round(float(np.percentile(ang_errs, 95)), 2)] if ang_errs else None,
        wall_s=round(time.time() - t0, 1),
    ), indent=1))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
