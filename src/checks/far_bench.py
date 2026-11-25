"""P7 end-to-end calibration: far-end sensor and flight bench on servicer_rev2 + client_p7.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/far_bench.py sensors [--n 25]

Mode `sensors` (ranges 30, 20, 12, 8, 5, 3.2 m; bench seeds 7500000 + i):
  Measures on client_p7 (with rev-2 servicer, arm stowed > 5 m, ready <= 5 m, bench mount error +-10 mm / +-1 deg,
  random client attitude, sun within 70 deg of the approach line, cams.LIDAR_MAX = 60 m):
    - bus_lidar returns and far_nav.lidar_fix centroid error (signed LOS bias -> BLOB_BACK calibration, lateral error,
      3D error, dropout rate)
    - bus_cam bearing (far_nav.cam_bearing): success rate, angular error (deg), lateral error at range D (m)
    - lidar_nav.fit (box fit) and ring_fit.fit at D <= 12 m (with prior error): success rate, pose / ring errors
  Writes runs/far_bench_sensors.json.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
import ring_bench as RB  # noqa: E402
import cams  # noqa: E402
import far_nav  # noqa: E402
import lidar_nav  # noqa: E402
import ring_fit  # noqa: E402

RANGES = (30.0, 20.0, 12.0, 8.0, 5.0, 3.2)
PRIOR_FAR = {12.0: (0.25, 8.0), 8.0: (0.15, 5.0), 5.0: (0.10, 3.0), 3.2: (0.05, 2.0)}
SUN_CONE_DEG = 70.0


def _ang(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 180.0
    return float(np.degrees(np.arccos(np.clip(float(a @ b) / (na * nb), -1.0, 1.0))))


def _rot_err(R1, R2):
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(R1.T @ R2).as_rotvec())))


def _pct(xs, q):
    return round(float(np.percentile(xs, q)), 3) if xs else None


def _place_far(W: RB.World, seed: int, D: float):
    """Place client_p7 at origin (random attitude), servicer at range D along a random direction (at D <= 12 m within
    15 deg of the ring axis, at D > 12 m arbitrary direction so solar wings / bus sides are tested too), sun within
    SUN_CONE_DEG of the approach line, bench mount error and lidar noise as ring_bench."""
    km, kd, rig = W.km, W.kd, W.rig
    rng = np.random.default_rng([seed, int(D * 10)])
    km.cam_pos[:] = W.base["pos"]
    km.cam_quat[:] = W.base["quat"]
    mujoco.mj_resetData(km, kd)

    Rc = Rotation.random(random_state=rng.integers(2**31)).as_matrix()
    pc = np.zeros(3)
    c_w, n_w = pc + Rc @ ring_fit.C_B, Rc @ ring_fit.N_B

    if D <= 12.0:
        u = RB._rand_unit_cone(rng, n_w, RB.CONE_DEG)
    else:
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
    pL = pc + D * u

    q = W.base["quat"][W.lid]
    Rl_b = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    bore_b = -Rl_b[:, 2]
    want = RB._rand_unit_cone(rng, -u, RB.POINT_DEG)
    Rs = RB._rot_between(bore_b, want)
    Rs = Rotation.from_rotvec(want * rng.uniform(-np.pi, np.pi)).as_matrix() @ Rs
    ps = pL - Rs @ W.base["pos"][W.lid]

    kd.qpos[W.tq:W.tq + 7] = [*pc, *RB._q_wxyz(Rc)]
    kd.qpos[W.cq:W.cq + 7] = [*ps, *RB._q_wxyz(Rs)]
    kd.qpos[W.arm] = W.poses["stow" if D > 5.0 else "ready"]

    # Sun direction (light travel = -sun_from): sun within SUN_CONE_DEG of the approach line +u
    sun_from = RB._rand_unit_cone(rng, u, SUN_CONE_DEG)
    lid_sun = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
    if lid_sun >= 0:
        km.light_dir[lid_sun] = -sun_from
        km.light_diffuse[lid_sun] = rng.uniform(0.6, 1.2)
    if km.nlight > 0 and lid_sun < 0:
        km.light_dir[0] = -sun_from

    # Bench mount error (bus_cam + bus_lidar)
    ids = [W.bcam, W.lid]
    o = 0.5 * (W.base["pos"][ids[0]] + W.base["pos"][ids[1]])
    Rb = Rotation.from_rotvec(np.radians(rng.uniform(-RB.MOUNT_DEG, RB.MOUNT_DEG, 3)))
    tb = rng.uniform(-RB.MOUNT_M, RB.MOUNT_M, 3)
    for c in ids:
        Ro = Rotation.from_rotvec(np.radians(rng.uniform(-RB.OWN_DEG, RB.OWN_DEG, 3)))
        km.cam_pos[c] = o + Rb.apply(W.base["pos"][c] - o) + tb + rng.uniform(-RB.OWN_M, RB.OWN_M, 3)
        qq = W.base["quat"][c]
        x, y, z, w = (Rb * Ro * Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]])).as_quat()
        km.cam_quat[c] = [w, x, y, z]

    rig.l_sig0, rig.l_drop = rng.uniform(0.005, 0.02), rng.uniform(0.01, 0.05)
    rig._nrng = np.random.default_rng([seed, int(D * 10), 7])
    rig.gain, rig.noise = rng.uniform(0.75, 1.25), rng.uniform(0.0, 3.0)
    mujoco.mj_forward(km, kd)

    # Client COM in world frame (truth)
    com_w = pc + Rc @ km.body_ipos[W.bid_t]
    return dict(Rc=Rc, pc=pc, com_w=com_w, c_w=c_w, n_w=n_w, u=u, rng=rng)


def lidar_fix_gated(P, Pl, Rl, Pc, b_cam, r_gate=1.2, blob_back=0.42, max_off_deg=20.0):
    """Lidar centroid gated around the camera bearing ray to reject client_p7's asymmetric +Y solar wing.
    Falls back to far_nav.lidar_fix if b_cam is None or too few points lie within r_gate of the ray."""
    if len(P) < 10:
        return None
    W = Pl + P @ Rl.T
    gated = False
    if b_cam is not None:
        v = W - Pc
        s = v @ b_cam
        d_perp = np.linalg.norm(v - s[:, None] * b_cam[None, :], axis=1)
        Wg = W[(s > 1.0) & (d_perp < r_gate)]
        if len(Wg) >= 10:
            W = Wg
            gated = True
    W = W[np.linalg.norm(W - np.median(W, axis=0), axis=1) < (1.2 if gated else 1.5)]
    if len(W) < 10:
        return None
    c = W.mean(axis=0)
    u = c - Pl
    u /= max(float(np.linalg.norm(u)), 1e-12)
    c_est = c + blob_back * u
    cl = Rl.T @ (c_est - Pl)
    if np.degrees(np.arccos(np.clip(-cl[2] / np.linalg.norm(cl), -1.0, 1.0))) > max_off_deg:
        return None
    return c_est, int(len(W)), gated, c


def run_sensors(n: int, ranges=RANGES, out_path: Path | None = None):
    cams.LIDAR_MAX = 60.0
    W = RB.World()
    rows = []
    t0 = time.time()
    for D in ranges:
        for i in range(n):
            seed = 7500000 + i
            tr = _place_far(W, seed, D)
            rng = tr["rng"]
            P, n_self = W.lidar_points()
            Pl_nom, Rl_nom = W.cam_nominal(W.lid)
            Pc_nom, Rc_nom = W.cam_nominal(W.bcam)
            los = tr["pc"] - Pl_nom
            los_u = los / np.linalg.norm(los)

            row = dict(seed=seed, D=D, n_pts=int(len(P)), n_self=n_self)

            # 1) Far lidar blob fix (raw mean + BLOB_BACK=0.4 from far_nav)
            lf = far_nav.lidar_fix(P, Pl_nom, Rl_nom)
            if lf is not None:
                c_est, n_in = lf
                # Undo far_nav.BLOB_BACK to also measure raw near-face centroid bias
                c_raw = c_est - far_nav.BLOB_BACK * ((c_est - Pl_nom) / np.linalg.norm(c_est - Pl_nom))
                e_raw = c_raw - tr["pc"]
                e_est = c_est - tr["pc"]
                los_bias_raw = float(e_raw @ los_u)     # negative = centroid is closer than client origin
                los_err_est = float(e_est @ los_u)
                lat_err = float(np.linalg.norm(e_est - los_err_est * los_u))
                row["lidar"] = dict(
                    ok=True, n_in=int(n_in),
                    los_bias_raw_m=round(los_bias_raw, 4),
                    los_err_m=round(los_err_est, 4),
                    lat_err_m=round(lat_err, 4),
                    err_3d_m=round(float(np.linalg.norm(e_est)), 4),
                )
            else:
                row["lidar"] = dict(ok=False, n_in=0)

            # 2) Bus camera bearing (far_nav.cam_bearing)
            img = W.rig.render(W.kd)["bus_cam"]
            # Predicted client centre with realistic 30 m prior error (up to ~1 m lateral)
            c_pred = tr["pc"] + rng.normal(0.0, min(0.5, 0.03 * D), 3)
            b_est = far_nav.cam_bearing(img, Pc_nom, Rc_nom, c_pred, float(W.km.cam_fovy[W.bcam]), win_m=3.0)
            if b_est is not None:
                b_true = (tr["pc"] - Pc_nom) / np.linalg.norm(tr["pc"] - Pc_nom)
                ang_deg = _ang(b_est, b_true)
                lat_m = D * np.radians(ang_deg)
                row["cam"] = dict(ok=True, ang_deg=round(ang_deg, 3), lat_err_m=round(float(lat_m), 4))
            else:
                row["cam"] = dict(ok=False)

            # 2b) Camera-ray-gated lidar blob (rejects client_p7's +Y solar wing)
            lg = lidar_fix_gated(P, Pl_nom, Rl_nom, Pc_nom, b_est)
            if lg is not None:
                cg_est, ng_in, was_gated, cg_raw = lg
                eg_raw = cg_raw - tr["pc"]
                eg_est = cg_est - tr["pc"]
                glos_bias = float(eg_raw @ los_u)
                glos_err = float(eg_est @ los_u)
                glat_err = float(np.linalg.norm(eg_est - glos_err * los_u))
                row["lidar_g"] = dict(
                    ok=True, gated=was_gated, n_in=ng_in,
                    los_bias_raw_m=round(glos_bias, 4),
                    los_err_m=round(glos_err, 4),
                    lat_err_m=round(glat_err, 4),
                    err_3d_m=round(float(np.linalg.norm(eg_est)), 4),
                )
            else:
                row["lidar_g"] = dict(ok=False, gated=False, n_in=0)

            # 3) Box fit and Ring fit at D <= 12 m
            if D in PRIOR_FAR and len(P) >= 20:
                dp_s, da_s = PRIOR_FAR[D]
                # Prior in nominal lidar frame: truth in nominal frame + prior error
                R_tr_nom = Rl_nom.T @ tr["Rc"]
                t_tr_nom = Rl_nom.T @ (tr["pc"] - Pl_nom)
                R0 = Rotation.from_rotvec(np.radians(rng.normal(0, da_s / np.sqrt(3), 3))).as_matrix() @ R_tr_nom
                t0_ = t_tr_nom + rng.normal(0, dp_s / np.sqrt(3), 3)
                bf = lidar_nav.fit(P, R0, t0_)
                if bf and bf.get("ok"):
                    R_b, t_b = bf["R"], bf["t"]
                    p_b_err_mm = float(np.linalg.norm(t_b - t_tr_nom)) * 1e3
                    a_b_err_deg = _rot_err(R_b, R_tr_nom)
                    row["box"] = dict(ok=True, pos_mm=round(p_b_err_mm, 1), deg=round(a_b_err_deg, 2))
                    rf = ring_fit.fit(P, R_b, t_b)
                    if rf and rf.get("ok"):
                        c_tr_nom = Rl_nom.T @ (tr["c_w"] - Pl_nom)
                        n_tr_nom = Rl_nom.T @ tr["n_w"]
                        row["ring"] = dict(
                            ok=True,
                            pos_mm=round(float(np.linalg.norm(rf["c"] - c_tr_nom)) * 1e3, 1),
                            deg=round(_ang(rf["n"], n_tr_nom), 2),
                            n_top=int(rf.get("n_top", 0)),
                        )
                    else:
                        row["ring"] = dict(ok=False, why=rf.get("why", "fail") if rf else "none")
                else:
                    row["box"] = dict(ok=False)
                    row["ring"] = dict(ok=False, why="no_box")
            rows.append(row)

    # Summarize per range
    summ = []
    for D in ranges:
        rs = [r for r in rows if r["D"] == D]
        pts = [r["n_pts"] for r in rs]
        l_ok = [r["lidar"] for r in rs if r["lidar"]["ok"]]
        lg_ok = [r["lidar_g"] for r in rs if r["lidar_g"]["ok"]]
        c_ok = [r["cam"] for r in rs if r["cam"]["ok"]]
        s = dict(
            D=D, n=len(rs),
            pts_med=int(np.median(pts)), pts_min=int(min(pts)),
            lidar_ok=len(l_ok),
            lidar_lat_p50_95=[_pct([x["lat_err_m"] for x in l_ok], 50), _pct([x["lat_err_m"] for x in l_ok], 95)],
            lg_ok=len(lg_ok),
            lg_blob_back_opt_m=round(float(-np.median([x["los_bias_raw_m"] for x in lg_ok])), 3) if lg_ok else None,
            lg_los_p50_95=[_pct([abs(x["los_err_m"]) for x in lg_ok], 50), _pct([abs(x["los_err_m"]) for x in lg_ok], 95)],
            lg_lat_p50_95=[_pct([x["lat_err_m"] for x in lg_ok], 50), _pct([x["lat_err_m"] for x in lg_ok], 95)],
            lg_3d_p50_95=[_pct([x["err_3d_m"] for x in lg_ok], 50), _pct([x["err_3d_m"] for x in lg_ok], 95)],
            cam_ok=len(c_ok),
            cam_lat_p50_95=[_pct([x["lat_err_m"] for x in c_ok], 50), _pct([x["lat_err_m"] for x in c_ok], 95)],
        )
        if D in PRIOR_FAR:
            b_ok = [r["box"] for r in rs if r.get("box", {}).get("ok")]
            r_ok = [r["ring"] for r in rs if r.get("ring", {}).get("ok")]
            s["box_ok"] = len(b_ok)
            s["box_mm_p50_95"] = [_pct([x["pos_mm"] for x in b_ok], 50), _pct([x["pos_mm"] for x in b_ok], 95)]
            s["box_deg_p50_95"] = [_pct([x["deg"] for x in b_ok], 50), _pct([x["deg"] for x in b_ok], 95)]
            s["ring_ok"] = len(r_ok)
            s["ring_mm_p50_95"] = [_pct([x["pos_mm"] for x in r_ok], 50), _pct([x["pos_mm"] for x in r_ok], 95)]
            s["ring_deg_p50_95"] = [_pct([x["deg"] for x in r_ok], 50), _pct([x["deg"] for x in r_ok], 95)]
        summ.append(s)
        print(json.dumps(s))

    meta = dict(mode="sensors", n=n, ranges=list(ranges), lidar_max_m=cams.LIDAR_MAX, wall_s=round(time.time() - t0, 1))
    out = out_path or (ensure_runs_dir() / "far_bench_sensors.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(meta=meta, summary=summ, rows=rows), indent=1, default=float))
    print("META:", json.dumps(meta))


# ====================================================================================================== flight mode
def _far_R_cov(los_u, sig_los, sig_lat):
    """Anisotropic 3x3 measurement covariance: sig_los along los_u, sig_lat perpendicular."""
    u = los_u / max(float(np.linalg.norm(los_u)), 1e-12)
    P_par = np.outer(u, u)
    P_perp = np.eye(3) - P_par
    return (sig_los ** 2) * P_par + (sig_lat ** 2) * P_perp


def _point_R(bore_b, want_w, roll_ref_w=None):
    """Servicer attitude putting body unit vector bore_b onto world unit vector want_w, minimizing roll drift."""
    w = want_w / max(float(np.linalg.norm(want_w)), 1e-12)
    R0 = RB._rot_between(bore_b, w)
    if roll_ref_w is None:
        return R0
    # Align servicer +z (or a perpendicular reference) smoothly to roll_ref_w projected perp to w
    z0 = R0[:, 2]
    zt = roll_ref_w - float(roll_ref_w @ w) * w
    if np.linalg.norm(zt) < 1e-6:
        return R0
    zt /= np.linalg.norm(zt)
    c = float(np.clip(z0 @ zt, -1.0, 1.0))
    s = float(np.cross(z0, zt) @ w)
    return Rotation.from_rotvec(w * np.arctan2(s, c)).as_matrix() @ R0


class FarMission:
    """Closed-loop 30 m -> 8 m far leg + staged arm unfold on servicer_rev2 + client_p7 (truth-free control)."""

    def __init__(self):
        cams.LIDAR_MAX = 60.0
        import ring_mission as RM  # noqa: E402
        self.RM = RM
        self.M = RM.Mission()
        q = self.M.base["quat"][self.M.lid]
        Rl_b = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        self.bore_b = -Rl_b[:, 2]

    def run_episode(self, seed: int, d_start=30.0, d_end=8.0, v_max=0.10):
        t_wall = time.time()
        M, RM = self.M, self.RM
        km, kd, rig = M.km, M.kd, M.rig
        rng = np.random.default_rng([seed, 75])
        km.cam_pos[:] = M.base["pos"]
        km.cam_quat[:] = M.base["quat"]
        mujoco.mj_resetData(km, kd)
        M._vff = np.zeros(6)
        M.bad = set()
        M.arm_guard(False)

        # Client at origin, random attitude, spinning 0-2 deg/s
        Rc = Rotation.random(random_state=rng.integers(2**31)).as_matrix()
        spin = float(rng.uniform(0.0, 2.0))
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        w_c = np.radians(spin) * (Rc @ ax)
        com_w = Rc @ M.com_true
        kd.qpos[M.tq:M.tq + 7] = [0.0, 0.0, 0.0, *RB._q_wxyz(Rc)]
        kd.qvel[M.tv:M.tv + 3] = -np.cross(w_c, com_w)
        kd.qvel[M.tv + 3:M.tv + 6] = Rc.T @ w_c

        # Sun direction & sun-side approach line u (within 30 deg of sun_from)
        sun_from = rng.normal(size=3)
        sun_from /= np.linalg.norm(sun_from)
        lid_sun = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
        if lid_sun >= 0:
            km.light_dir[lid_sun] = -sun_from
            km.light_diffuse[lid_sun] = float(rng.uniform(0.6, 1.2))
        u = RB._rand_unit_cone(rng, sun_from, 30.0)

        # Servicer at d_start along u, pointing within 2 deg of client, arm STOWED
        p_s = d_start * u
        want0 = RB._rand_unit_cone(rng, -u, 2.0)
        R_s = RB._rot_between(self.bore_b, want0)
        R_s = Rotation.from_rotvec(want0 * float(rng.uniform(-np.pi, np.pi))).as_matrix() @ R_s
        kd.qpos[M.cq:M.cq + 7] = [*p_s, *RB._q_wxyz(R_s)]
        kd.qvel[M.cv:M.cv + 6] = 0.0
        q_stow = M.poses["stow"].copy()
        q_ready = M.poses["ready"].copy()
        kd.qpos[M.arm] = q_stow
        kd.qpos[M.gq] = M.q_open

        # Bench mount error (bus_cam + bus_lidar) and sensor noise
        ids = [M.bcam, M.lid]
        o = 0.5 * (M.base["pos"][ids[0]] + M.base["pos"][ids[1]])
        Rb = Rotation.from_rotvec(np.radians(rng.uniform(-RB.MOUNT_DEG, RB.MOUNT_DEG, 3)))
        tb = rng.uniform(-RB.MOUNT_M, RB.MOUNT_M, 3)
        for c in ids:
            Ro = Rotation.from_rotvec(np.radians(rng.uniform(-RB.OWN_DEG, RB.OWN_DEG, 3)))
            km.cam_pos[c] = o + Rb.apply(M.base["pos"][c] - o) + tb + rng.uniform(-RB.OWN_M, RB.OWN_M, 3)
            qq = M.base["quat"][c]
            x, y, z, w = (Rb * Ro * Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]])).as_quat()
            km.cam_quat[c] = [w, x, y, z]
        rig.l_sig0, rig.l_drop = float(rng.uniform(0.005, 0.02)), float(rng.uniform(0.01, 0.05))
        rig._nrng = np.random.default_rng([seed, 75, 7])
        rig.gain, rig.noise = float(rng.uniform(0.75, 1.25)), float(rng.uniform(0.0, 3.0))
        mujoco.mj_forward(km, kd)

        # FarKF: x = [r, v] = client_centre - servicer_com (world), start prior error <= 3 m / 10 mm/s
        r_true0 = kd.xpos[M.bid_t] - kd.subtree_com[M.bid_c]
        dp0 = np.clip(rng.normal(0.0, 1.0, 3), -3.0, 3.0)
        dv0 = np.clip(rng.normal(0.0, 0.003, 3), -0.01, 0.01)
        kf = far_nav.FarKF(r_true0 + dp0, dv0, sp=3.0, sv=0.01, n=0.0)
        lat_rng = np.random.default_rng([seed, 75, 9])

        # Bus actuators (rev-2)
        pulses = RM.Pulses(M.rcs, seed=seed)
        wheels = RM.Wheels(M.busd)
        roll_ref = R_s[:, 2].copy()
        q_tgt = q_stow.copy()
        dt = float(km.opt.timestep)
        u_thr = np.zeros(M.rcs.n)
        T_cmd = np.zeros(3)
        tau_thr = np.zeros(3)
        a_own = np.zeros(3)
        unloading = False
        pend = []
        t_sens, t_ctrl = 0.0, 0.0
        n_lid, n_gated, n_cam, n_rej = 0, 0, 0, 0
        F_MAX = RM.F_MAX

        def sense_at(t):
            P, _ = M.lidar_points()
            Pl_nom, Rl_nom = M.cam_nominal(M.lid)
            Pc_nom, Rc_nom = M.cam_nominal(M.bcam)
            img = rig.render(kd)["bus_cam"]
            _, _, _, _, com_s = M.bus_state()
            c_pred = com_s + kf.x[:3]
            b_est = far_nav.cam_bearing(img, Pc_nom, Rc_nom, c_pred, float(km.cam_fovy[M.bcam]), win_m=3.5)
            lg = lidar_fix_gated(P, Pl_nom, Rl_nom, Pc_nom, b_est, r_gate=1.2, blob_back=0.48)
            lat = RM.LAT[0] + float(lat_rng.uniform(-RM.LAT_J[0], RM.LAT_J[0]))
            pend.append((t + lat, t, com_s.copy(), Pc_nom.copy(), b_est, lg))

        def deliver(now):
            nonlocal n_lid, n_gated, n_cam, n_rej
            while pend and pend[0][0] <= now + 1e-9:
                _, t_m, com_m, Pc_m, b_est, lg = pend.pop(0)
                dt_l = now - t_m
                D_est = max(float(np.linalg.norm(kf.x[:3])), 2.0)
                if lg is not None:
                    c_est, _, was_gated, _ = lg
                    # Propagate relative measurement r(t_m) = c_est - com_m forward by latency dt_l
                    z = (c_est - com_m) + kf.x[3:] * dt_l
                    los_u = z / max(float(np.linalg.norm(z)), 1e-9)
                    sig_los = 0.15 + 0.005 * D_est
                    sig_lat = (0.30 + 0.015 * D_est) if was_gated else 1.5
                    ok, _ = kf.update(z, _far_R_cov(los_u, sig_los, sig_lat), gate=far_nav.GATE)
                    if ok:
                        n_lid += 1
                        if was_gated:
                            n_gated += 1
                        kf.t_fix = t_m
                    else:
                        n_rej += 1
                if b_est is not None:
                    # Bearing-only update: infinite variance along b_est, tight perpendicular
                    r_proj = float(np.dot(kf.x[:3] + (com_m - Pc_m), b_est))
                    if r_proj > 1.0:
                        z_c = (Pc_m + r_proj * b_est - com_m) + kf.x[3:] * dt_l
                        sig_c = 0.35 + 0.015 * D_est
                        ok, _ = kf.update(z_c, _far_R_cov(b_est, 100.0, sig_c), gate=25.0)
                        if ok:
                            n_cam += 1

        def step_sim(r_des_rel, v_des_rel, a_des_rel, q_arm_cmd, n_steps):
            """r_des_rel = desired (servicer_com - client_centre), world frame."""
            nonlocal t_sens, t_ctrl, u_thr, T_cmd, tau_thr, a_own, unloading
            max_att_err = 0.0
            for _ in range(n_steps):
                t = kd.time
                if pend:
                    deliver(t)
                if t + 1e-9 >= t_sens:
                    t_sens = t + 1.0
                    sense_at(t)
                if t + 1e-9 >= t_ctrl:
                    kf.predict(RM.CTRL_DT if t_ctrl > 0 else 0.0, a_own, t)
                    t_ctrl = t + RM.CTRL_DT
                    p, R, v, wb, com = M.bus_state()
                    m, I = M.bus_mass_inertia()
                    # Relative state: servicer_com - client_centre = -kf.x[:3]
                    r_est = -kf.x[:3]
                    v_est = -kf.x[3:]
                    e_p = r_des_rel - r_est
                    e_v = v_des_rel - v_est
                    F = np.clip(m * (a_des_rel + RM.KP_T * e_p + RM.KD_T * e_v), -F_MAX, F_MAX)
                    a_own = F / m
                    R_d = _point_R(self.bore_b, kf.x[:3], roll_ref)
                    e_R = RM._log(R.T @ R_d)
                    max_att_err = max(max_att_err, float(np.degrees(np.linalg.norm(e_R))))
                    T_cmd = I @ (RM.KP_A * e_R - RM.KD_A * wb) + np.cross(wb, I @ wb)
                    fill = wheels.fill()
                    unloading = fill > RM.UNLOAD_ON or (unloading and fill > RM.UNLOAD_OFF)
                    tau_thr = np.clip(-RM.UNLOAD_K * wheels.H(), -RM.UNLOAD_MAX, RM.UNLOAD_MAX) if unloading else np.zeros(3)
                    w_cmd = np.r_[R.T @ F, tau_thr]
                    ref = R.T @ (com - p)
                    u_thr = M.rcs.allocate(w_cmd, ref) if np.any(np.abs(w_cmd) > 1e-9) else np.zeros(M.rcs.n)
                f = pulses.step(u_thr, dt)
                kd.ctrl[M.noz] = f
                p, R, _, wb, _ = M.bus_state()
                dF = f[:, None] * (pulses.D_true - M.rcs.D)
                dT = np.cross(M.rcs.P, dF).sum(0)
                kd.xfrc_applied[M.bid_c, :3] = R @ dF.sum(0)
                tau, _ = wheels.command(T_cmd - tau_thr, wb, dt)
                kd.ctrl[M.rwm] = tau + dT
                # Arm joint PD
                qa, qda = kd.qpos[M.arm], kd.qvel[M.I["arm_v"]]
                ua = M.kp * (q_arm_cmd - qa) - M.kdv * qda
                lim = km.actuator_ctrlrange[M.I["act_arm"], 1]
                kd.ctrl[M.I["act_arm"]] = np.clip(ua, -lim, lim)
                kd.ctrl[M.I["act_u"]] = M.q_open[0]
                kd.ctrl[M.I["act_l"]] = M.q_open[1]
                mujoco.mj_step(km, kd)
                M._contacts()
            return max_att_err

        # ---- Phase 1: far_hold at 30 m until FarKF converges
        r_hold0 = -kf.x[:3].copy()
        n_sub = int(round(1.0 / dt))
        t_conv = None
        for sec in range(120):
            step_sim(r_hold0, np.zeros(3), np.zeros(3), q_tgt, n_sub)
            sp, sv = kf.sig()
            if sec >= 15 and n_lid >= 10 and sp < 0.60 and sv < 0.025:
                t_conv = round(kd.time, 1)
                break
        if t_conv is None:
            return dict(seed=seed, result="abort: far_hold no_converge", t_s=round(kd.time, 1),
                        n_lid=n_lid, n_cam=n_cam, sig_p=round(kf.sig()[0], 3), sig_v=round(kf.sig()[1], 4),
                        wall_s=round(time.time() - t_wall, 1))

        # ---- Phase 2: far_app 30 m -> 8 m (min-jerk FarPath)
        r0 = -kf.x[:3].copy()
        u_hat = r0 / max(float(np.linalg.norm(r0)), 1e-9)
        path = far_nav.FarPath(r0, d_end * u_hat, v_max=v_max)
        path.k = 1.0
        n_ctrl = int(round(RM.CTRL_DT / dt))
        t_app_max = kd.time + path.T + 120.0
        stale_s = 0.0
        while not path.done() and kd.time < t_app_max:
            stale = (kd.time - kf.t_fix) > 10.0
            path.k = 0.0 if stale else 1.0
            if stale:
                stale_s += RM.CTRL_DT
            else:
                path.tau = min(path.T, path.tau + RM.CTRL_DT)
            rp, vp, ap = path.rel()
            step_sim(rp, vp, ap, q_tgt, n_ctrl)
        if not path.done():
            return dict(seed=seed, result="abort: far_app timeout", t_s=round(kd.time, 1),
                        stale_s=round(stale_s, 1), wall_s=round(time.time() - t_wall, 1))

        # ---- Phase 3: hold8_settle (15 s)
        r8_des = d_end * u_hat
        for _ in range(15):
            step_sim(r8_des, np.zeros(3), np.zeros(3), q_tgt, n_sub)

        # Measure 8 m arrival error vs truth
        r_true8 = kd.subtree_com[M.bid_c] - kd.xpos[M.bid_t]
        v_true8 = kd.qvel[M.cv:M.cv + 3] - kd.qvel[M.tv:M.tv + 3]
        est_pos_err8 = float(np.linalg.norm((-kf.x[:3]) - r_true8))
        est_vel_err8 = float(np.linalg.norm((-kf.x[3:]) - v_true8))
        rng_err8 = float(abs(np.linalg.norm(r_true8) - d_end))
        spd8 = float(np.linalg.norm(v_true8))

        # ---- Phase 4: staged arm unfold at 8 m (deploy_check.py stages)
        qa = q_stow.copy()
        qa[1], qa[2] = q_stow[1] + 0.6, q_stow[2] - 0.9
        qb = qa.copy()
        qb[3] = 0.0
        qd_lim = np.radians(10.0)
        unfold_att_max = 0.0
        for q_a, q_b in ((q_stow, qa), (qa, qb), (qb, q_ready)):
            T_seg = max(float(np.max(np.abs(q_b - q_a))) / qd_lim, 1.0)
            n_seg = int(np.ceil(T_seg / RM.CTRL_DT))
            for k in range(1, n_seg + 1):
                q_tgt = q_a + (q_b - q_a) * (k / n_seg)
                att_e = step_sim(r8_des, np.zeros(3), np.zeros(3), q_tgt, n_ctrl)
                unfold_att_max = max(unfold_att_max, att_e)
        # 5 s post-unfold settle
        for _ in range(5):
            att_e = step_sim(r8_des, np.zeros(3), np.zeros(3), q_ready, n_sub)
            unfold_att_max = max(unfold_att_max, att_e)

        # Final true range & attitude pointing error
        p_s_end, R_s_end, _, _, com_end = M.bus_state()
        r_true_end = com_end - kd.xpos[M.bid_t]
        bore_w = R_s_end @ self.bore_b
        to_cli = -r_true_end / max(float(np.linalg.norm(r_true_end)), 1e-9)
        pt_err_deg = _ang(bore_w, to_cli)

        return dict(
            seed=seed,
            result="collision" if M.bad else "ok",
            spin_deg_s=round(spin, 2),
            t_conv_s=t_conv,
            t_total_s=round(kd.time, 1),
            stale_s=round(stale_s, 1),
            n_lid=n_lid, n_gated=n_gated, n_cam=n_cam, n_rej=n_rej,
            arr8_est_pos_m=round(est_pos_err8, 3),
            arr8_est_vel_mm_s=round(est_vel_err8 * 1e3, 2),
            arr8_rng_err_m=round(rng_err8, 3),
            arr8_speed_mm_s=round(spd8 * 1e3, 2),
            unfold_att_max_deg=round(unfold_att_max, 2),
            end_pt_err_deg=round(pt_err_deg, 2),
            prop_g=round(pulses.prop_kg * 1e3, 1),
            wheel_fill_max=round(wheels.fill(), 3),
            contacts_bad=sorted(M.bad),
            wall_s=round(time.time() - t_wall, 1),
        )


def _flight_job(args):
    seed, v_max = args
    global _FM
    if "_FM" not in globals():
        _FM = FarMission()
    r = _FM.run_episode(seed, v_max=v_max)
    print(json.dumps(r), flush=True)
    return r


def run_flight(n: int, workers: int = 1, v_max: float = 0.10, out_path: Path | None = None):
    from multiprocessing import Pool
    t0 = time.time()
    seeds = [(7510000 + i, v_max) for i in range(n)]
    if workers > 1:
        with Pool(min(workers, 3)) as pool:
            rows = pool.map(_flight_job, seeds, chunksize=1)
    else:
        rows = [_flight_job(s) for s in seeds]
    ok = [r for r in rows if r["result"] == "ok"]
    summ = dict(
        mode="flight", n=len(rows), ok=len(ok),
        collisions=sum(1 for r in rows if r.get("contacts_bad")),
        t_conv_p50_95=[_pct([r["t_conv_s"] for r in ok], 50), _pct([r["t_conv_s"] for r in ok], 95)],
        arr8_est_pos_p50_95=[_pct([r["arr8_est_pos_m"] for r in ok], 50), _pct([r["arr8_est_pos_m"] for r in ok], 95)],
        arr8_est_vel_p50_95=[_pct([r["arr8_est_vel_mm_s"] for r in ok], 50), _pct([r["arr8_est_vel_mm_s"] for r in ok], 95)],
        arr8_rng_err_p50_95=[_pct([r["arr8_rng_err_m"] for r in ok], 50), _pct([r["arr8_rng_err_m"] for r in ok], 95)],
        unfold_att_p50_95=[_pct([r["unfold_att_max_deg"] for r in ok], 50), _pct([r["unfold_att_max_deg"] for r in ok], 95)],
        prop_g_p50_95=[_pct([r["prop_g"] for r in ok], 50), _pct([r["prop_g"] for r in ok], 95)],
        wheel_fill_max=max((r["wheel_fill_max"] for r in ok), default=None),
        wall_s=round(time.time() - t0, 1),
    )
    print("SUMMARY:", json.dumps(summ, indent=1))
    out = out_path or (ensure_runs_dir() / "far_bench_flight.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(summary=summ, rows=rows), indent=1, default=float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["sensors", "flight"])
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--vmax", type=float, default=0.10)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    out = Path(a.out) if a.out else None
    if a.mode == "sensors":
        run_sensors(a.n, out_path=out)
    else:
        run_flight(a.n, workers=a.workers, v_max=a.vmax, out_path=out)
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
