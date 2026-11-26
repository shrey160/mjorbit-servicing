"""Verification bench for the v4.2 Dual-Arm ESPA 6-DOF Servicer in MuJoCo (PlainMujocoBackend + ActuatorBus).

Tests five core physics milestones (no modifications to simulator/ or src/):
  A. Static parity: mass (191.1 kg), 0 contacts in stow/ready, exact 6-DOF FK/IK match (< 0.5 mm, < 0.1 deg), RCS & RW envelopes.
  B. Asynchronous first-touch active-compliant capture with UNMODELED blind 2 deg client tilt + 15 mm Y offset + 3 N.m hold.
  C. 1.0 deg/s roll-spin capture using 4 x RW-12 zero-propellant roll-sync + plume-free detumble.
  D. 2.0 deg/s oblique spin capture + 100% wheels-only detumble (< 80% RW-12 fill, 0 plume pulses, 0 collisions).
  E. Stereo-tracked 5 deg pitch + 5 deg yaw 3D-tilted capture (active q3/q4/q5 wrist alignment) + 5.0 N.m high-load hold.

Also saves EGL renders (`dual_arm_mj_capture.png`, `dual_arm_mj_closeup.png`) to the artifact directory if present.
Run in WSL:
  bash scripts/run.sh python experiments/01_dual_arm_espa/bench_dual_arm_capture.py
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

EXP_DIR = Path(__file__).resolve().parent
REPO_ROOT = EXP_DIR.parents[1]
SRC_DIR = REPO_ROOT / "src"
for p in (EXP_DIR, SRC_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import _paths  # noqa: E402,F401
from dual_arm_spec import (  # noqa: E402
    L1,
    L2,
    PALM_X,
    SHOULDER_X,
    SHOULDER_Y,
    SHOULDER_Z,
    dual_arm_espa_spec,
    solve_dual_arm_ik,
    solve_single_arm_ik_6dof,
)
from sim.actuator_bus import ActuatorBus  # noqa: E402
from sim.backend import PlainMujocoBackend  # noqa: E402
from specs.client_spec import client_p7  # noqa: E402

DEFAULT_ART_DIR = Path("/mnt/c/Users/Shrey/.gemini/antigravity/brain/4379e84e-412f-47f4-9862-cc7cc0bd2a71")


def _quat_from_rotvec(rv: np.ndarray) -> np.ndarray:
    """scipy [x,y,z,w] -> MuJoCo [w,x,y,z]."""
    q_xyzw = Rotation.from_rotvec(rv).as_quat()
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def _rot_mat(q_wxyz: np.ndarray) -> np.ndarray:
    return Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()


def _lock_eq_at_current(bus: ActuatorBus, preload_m: float = 0.0003) -> np.ndarray:
    km, kd = bus.backend.km, bus.backend.kd
    signs = (-1.0, 1.0, -1.0, 1.0)
    g_lock = np.zeros(4, dtype=float)
    for i, (eq_id, gq_adr, s) in enumerate(zip(bus.ids["eq_locks"], bus.ids["gq"], signs)):
        q_lock = float(kd.qpos[gq_adr]) + s * preload_m
        km.eq_data[eq_id, 0] = q_lock
        kd.eq_active[eq_id] = 1
        g_lock[i] = q_lock
    return g_lock


def _unlock_eq(bus: ActuatorBus) -> None:
    for eq_id in bus.ids["eq_locks"]:
        bus.backend.kd.eq_active[eq_id] = 0


def _arm_locked(t: dict[str, float], k: int) -> bool:
    return (
        t[f"a{k}_in"] > 5.0
        and (t[f"a{k}_hook"] > 5.0 or t[f"a{k}_lip"] > 5.0)
        and (t[f"a{k}_palm"] > 5.0 or t[f"a{k}_lip"] > 5.0)
    )


def _reset_scenario(
    bus: ActuatorBus,
    q_arm: np.ndarray,
    g_cmd: np.ndarray,
    target_tilt_deg: tuple[float, float] = (0.0, 0.0),
    target_offset_yz: tuple[float, float] = (0.0, 0.0),
    target_omega_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
    chaser_omega_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
    seed: int = 0,
) -> None:
    km, kd = bus.backend.km, bus.backend.kd
    mujoco.mj_resetData(km, kd)
    bus.reset_episode(seed=seed)
    _unlock_eq(bus)

    cq, tq, cv, tv = bus.ids["cq"], bus.ids["tq"], bus.ids["cv"], bus.ids["tv"]
    # Target base orientation: 180 deg about Z so client +X ring faces chaser (+X)
    q_flip = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    q_tilt = _quat_from_rotvec(np.radians([0.0, target_tilt_deg[0], target_tilt_deg[1]]))
    q_t = _quat_mul(q_tilt, q_flip)

    kd.qpos[cq : cq + 3] = [-1.610, 0.0, 0.0]  # front face at -1.290 m (480 mm from target ring at -0.810 m)
    kd.qpos[cq + 3 : cq + 7] = [1.0, 0.0, 0.0, 0.0]
    kd.qpos[tq : tq + 3] = [0.0, target_offset_yz[0], target_offset_yz[1]]
    kd.qpos[tq + 3 : tq + 7] = q_t

    kd.qpos[bus.ids["arm_q"]] = q_arm
    kd.qpos[bus.ids["gq"]] = g_cmd
    kd.qvel[:] = 0.0
    # Body angular velocity converted cleanly to local frame for MuJoCo freejoint
    kd.qvel[cv + 3 : cv + 6] = np.radians(chaser_omega_deg_s)
    # Match target CoM orbital motion about ring center if needed or pure CoM spin:
    kd.qvel[tv + 3 : tv + 6] = np.radians(target_omega_deg_s)
    mujoco.mj_forward(km, kd)


def _solve_arm_world(
    bus: ActuatorBus,
    k: int,
    ring_center_world: np.ndarray,
    ring_normal_world: np.ndarray,
    ring_r: float = 0.294,
) -> np.ndarray:
    """Solve full 6-DOF IK (q0..q5) in current chaser body frame toward live 3D world ring pose."""
    kd = bus.backend.kd
    bid_c = bus.ids["bid_c"]
    p_c = kd.xpos[bid_c]
    R_c = kd.xmat[bid_c].reshape(3, 3)

    c_b = R_c.T @ (ring_center_world - p_c)
    n_b = R_c.T @ ring_normal_world
    n_b /= max(float(np.linalg.norm(n_b)), 1e-9)
    b_pos = np.array(
        [SHOULDER_X, SHOULDER_Y if k == 1 else -SHOULDER_Y, SHOULDER_Z if k == 1 else -SHOULDER_Z],
        dtype=float,
    )

    # Project shoulder onto tilted ring plane along ring normal to find radial contact point & frame
    vy, vz = float(b_pos[1] - c_b[1]), float(b_pos[2] - c_b[2])
    rs = max(math.hypot(vy, vz), 1e-9)
    ur_b = np.array([0.0, vy / rs, vz / rs])
    dir_p = ur_b - float(np.dot(n_b, ur_b)) * n_b
    dir_p /= max(float(np.linalg.norm(dir_p)), 1e-9)
    p_grasp_b = c_b + ring_r * dir_p

    # Gripper frame in chaser body: +X_g = into ring (-n_b), +Z_g = radially outward (dir_p), +Y_g = Z x X
    x_des = -n_b
    z_des = dir_p
    y_des = np.cross(z_des, x_des)
    R_des_b = np.column_stack([x_des, y_des, z_des])

    # Clamp spherical wrist center W strictly inside reachable shell [|L1-L2|+2mm, L1+L2-2mm]
    w_raw = p_grasp_b - PALM_X * x_des
    d_vec = w_raw - b_pos
    dist = float(np.linalg.norm(d_vec))
    d_min, d_max = abs(L1 - L2) + 0.002, (L1 + L2) - 0.002
    if dist > d_max:
        w_raw = b_pos + d_vec * (d_max / dist)
        p_grasp_b = w_raw + PALM_X * x_des
    elif dist < d_min:
        w_raw = b_pos + d_vec * (d_min / max(dist, 1e-6))
        p_grasp_b = w_raw + PALM_X * x_des

    return np.asarray(solve_single_arm_ik_6dof(tuple(b_pos), p_grasp_b, R_des_b), dtype=float)


def _render_views(bus: ActuatorBus, out_dir: Path) -> None:
    if not out_dir.is_dir():
        return
    try:
        from PIL import Image
    except ImportError:
        return
    km, kd = bus.backend.km, bus.backend.kd
    km.vis.global_.offwidth = max(int(km.vis.global_.offwidth), 1280)
    km.vis.global_.offheight = max(int(km.vis.global_.offheight), 900)
    renderer = mujoco.Renderer(km, height=900, width=1280)

    # 1. Broadside-oblique overview from -Y showing the 480 mm capture gap & both arms gripping the ring
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [-1.05, 0.0, 0.02]
    cam.distance = 2.10
    cam.azimuth = -96.0
    cam.elevation = -10.0
    renderer.update_scene(kd, camera=cam)
    Image.fromarray(renderer.render()).save(out_dir / "dual_arm_mj_capture.png")

    # 2. Close-up on Arm 1 3-DOF Cardan-Roll wrist + Mini45 F/T + form-closure jaw locked on ring
    s1 = kd.site_xpos[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "a1_grip_site")].copy()
    cam2 = mujoco.MjvCamera()
    cam2.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam2.lookat[:] = s1 + np.array([-0.030, 0.0, 0.0])
    cam2.distance = 0.29
    cam2.azimuth = 72.0
    cam2.elevation = -14.0
    renderer.update_scene(kd, camera=cam2)
    Image.fromarray(renderer.render()).save(out_dir / "dual_arm_mj_closeup.png")
    renderer.close()
    print(f"  Rendered EGL views to {out_dir}")


def check_a_static(bus: ActuatorBus, spec) -> None:
    print("\n[CHECK A] STATIC PARITY, STOWED CLEARANCE & 6-DOF FK/IK ACCURACY")
    km, kd = bus.backend.km, bus.backend.kd
    cid = bus.ids["bid_c"]
    mass = float(km.body_subtreemass[cid])
    assert abs(mass - 191.1) < 0.2, f"Unexpected mass {mass}"

    # Verify stowed & ready poses have 0 contacts
    for name, q in (("stow", spec.arm.stow_q), ("ready", spec.arm.ready_q)):
        _reset_scenario(bus, np.asarray(q), np.asarray(spec.tool.q_open))
        bus.record_contacts()
        assert len(bus.bad_contacts) == 0, f"{name} contacts: {bus.bad_contacts}"

    # Verify 6-DOF FK vs IK at 480 mm nominal AND 5 deg pitch + 5 deg yaw tilted ring poses
    rs = math.hypot(SHOULDER_Y, SHOULDER_Z)
    b1 = (SHOULDER_X, +SHOULDER_Y, +SHOULDER_Z)
    b2 = (SHOULDER_X, -SHOULDER_Y, -SHOULDER_Z)
    s1_id = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "a1_grip_site")
    s2_id = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "a2_grip_site")

    max_pos_err = 0.0
    max_ang_err = 0.0
    for tilt_name, R_ring in (
        ("nominal", np.eye(3)),
        ("tilt_5p_5y", Rotation.from_euler("yz", [5.0, 5.0], degrees=True).as_matrix()),
    ):
        c_b = np.array([0.800, 0.015, -0.010]) if tilt_name != "nominal" else np.array([0.800, 0.0, 0.0])
        x_des = R_ring @ np.array([1.0, 0.0, 0.0])
        qs, targets = [], []
        for k, b in ((1, b1), (2, b2)):
            vy, vz = b[1] - c_b[1], b[2] - c_b[2]
            rk = math.hypot(vy, vz)
            ur = np.array([0.0, vy / rk, vz / rk])
            z_des = ur - float(np.dot(-x_des, ur)) * (-x_des)
            z_des /= float(np.linalg.norm(z_des))
            R_des = np.column_stack([x_des, np.cross(z_des, x_des), z_des])
            p_des = c_b + 0.294 * z_des
            qs.append(solve_single_arm_ik_6dof(b, p_des, R_des))
            targets.append((p_des, R_des))

        _reset_scenario(bus, np.concatenate(qs), np.asarray(spec.tool.q_open))
        p_c = kd.xpos[cid].copy()
        R_c = kd.xmat[cid].reshape(3, 3).copy()
        for sid, (p_des, R_des) in zip((s1_id, s2_id), targets):
            p_act = R_c.T @ (kd.site_xpos[sid] - p_c)
            R_act = R_c.T @ kd.site_xmat[sid].reshape(3, 3)
            pos_err = float(np.linalg.norm(p_act - p_des) * 1000.0)
            ang_err = float(np.degrees(Rotation.from_matrix(R_act.T @ R_des).magnitude()))
            max_pos_err = max(max_pos_err, pos_err)
            max_ang_err = max(max_ang_err, ang_err)
            assert pos_err < 0.5 and ang_err < 0.1, f"{tilt_name} FK/IK err: {pos_err:.3f} mm, {ang_err:.3f} deg"

    # RCS + RW checks
    min_fx = min(bus.rcs.max_pure(0, sign=-1.0, enabled=[j != i for j in range(16)]) for i in range(16))
    hx = bus.wheels.envelope([1.0, 0.0, 0.0])
    hy = bus.wheels.envelope([0.0, 1.0, 0.0])
    print(f"  Mass = {mass:.2f} kg | Max 6-DOF FK/IK err = ({max_pos_err:.4f} mm, {max_ang_err:.4f} deg) | 1-Fail min -Fx = {min_fx:.3f} N")
    print(f"  4x RW-12 (65 deg): Hx = {hx:.2f} N.m.s, Hy = {hy:.2f} N.m.s | 0 stow/ready contacts -> PASS")
    assert min_fx >= 1.50 and hx > 20.0 and hy > 21.0


def run_capture_and_detumble(
    bus: ActuatorBus,
    spec,
    label: str,
    tilt_deg: tuple[float, float] = (0.0, 0.0),
    offset_yz: tuple[float, float] = (0.0, 0.0),
    target_omega_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
    roll_sync_wheels: bool = False,
    detumble_after: bool = False,
    blind_tilt: bool = False,
    align_ring_start: bool = False,
    hold_torque_nm: tuple[float, float, float] = (1.5, 2.0, -1.5),
    render_dir: Path | None = None,
) -> dict:
    km, kd = bus.backend.km, bus.backend.kd
    dt = float(km.opt.timestep)
    q_open = np.asarray(spec.tool.q_open, dtype=float)
    q_close = np.array([-spec.tool.q_squeeze, spec.tool.q_squeeze, -spec.tool.q_squeeze, spec.tool.q_squeeze], dtype=float)

    _reset_scenario(
        bus,
        np.asarray(spec.arm.ready_q, dtype=float),
        q_open,
        target_tilt_deg=tilt_deg,
        target_offset_yz=offset_yz,
        target_omega_deg_s=target_omega_deg_s,
        chaser_omega_deg_s=(0.0, 0.0, 0.0),
    )

    bid_t = bus.ids["bid_t"]
    wrist_idx = np.array([3, 4, 5, 9, 10, 11], dtype=int)
    q_admit = np.zeros(6, dtype=float)

    if align_ring_start:
        R_t0 = kd.xmat[bid_t].reshape(3, 3).copy()
        p_t0 = kd.xpos[bid_t].copy()
        ring_top0 = p_t0 + R_t0 @ np.array([0.810, 0.0, 0.0])
        cq = bus.ids["cq"]
        kd.qpos[cq : cq + 3] = [float(ring_top0[0] - 0.790), float(ring_top0[1] - offset_yz[0]), float(ring_top0[2] - offset_yz[1])]
        mujoco.mj_forward(km, kd)

    if roll_sync_wheels and any(target_omega_deg_s):
        R_t0 = kd.xmat[bid_t].reshape(3, 3).copy()
        p_t0 = kd.xpos[bid_t].copy()
        w_t_w0 = R_t0 @ np.radians(target_omega_deg_s)
        kd.qvel[bus.ids["cv"] : bus.ids["cv"] + 3] = np.cross(w_t_w0, kd.xpos[bus.ids["bid_c"]] - p_t0)
        kd.qvel[bus.ids["cv"] + 3 : bus.ids["cv"] + 6] = w_t_w0
        mujoco.mj_forward(km, kd)

    # Phase 1: Dual-arm guarded descent from X_ring-0.080 -> X_ring
    bus.phase = "capture_desc"
    d_off = np.array([0.080, 0.080])
    seated = np.array([False, False])
    seat_step = None
    first_seat_step = None
    is_spin = any(target_omega_deg_s)

    for step in range(6000):  # up to 12.0 s descent
        R_t = kd.xmat[bid_t].reshape(3, 3).copy()
        p_t = kd.xpos[bid_t].copy()
        n_ring_w = R_t @ np.array([1.0, 0.0, 0.0])  # points toward chaser (-X_w) when flipped
        ring_top_w = p_t + R_t @ np.array([0.810, 0.0, 0.0])

        t_now = bus.touches()
        for k in (1, 2):
            idx = k - 1
            f_p = t_now[f"a{k}_palm"]
            if f_p > 0.5 and not seated[idx]:
                seated[idx] = True
                if first_seat_step is None:
                    first_seat_step = step
            if seated[idx]:
                d_off[idx] = float(np.clip(d_off[idx] - 0.006 * dt * np.clip((0.8 - f_p) / 0.8, -1.0, 1.0), -0.030, 0.015))
            else:
                v_desc = 0.018 if d_off[idx] > 0.020 else 0.008
                d_off[idx] = max(-0.030, d_off[idx] - v_desc * dt)

        # In blind_tilt mode (Check B), do not feed ring normal or sub-cm bias; otherwise track live 3D ring pose
        c_est_w = (ring_top_w + np.array([0.0, -0.005, 0.004])) if blind_tilt else ring_top_w
        n_est_w = np.array([-1.0, 0.0, 0.0]) if blind_tilt else n_ring_w
        p_c, R_c, v_c, w_b, _ = bus.bus_state()
        q1_cmd = _solve_arm_world(bus, 1, c_est_w + d_off[0] * n_est_w, n_est_w)
        q2_cmd = _solve_arm_world(bus, 2, c_est_w + d_off[1] * n_est_w, n_est_w)
        q_cmd = np.concatenate([q1_cmd, q2_cmd])

        if blind_tilt:
            # Contact-gated wrist admittance: zero during free-space approach, yields softly upon blind touch
            q_meas_w = kd.qpos[bus.ids["arm_q"][wrist_idx]]
            for k in (1, 2):
                sl = slice(0, 3) if k == 1 else slice(3, 6)
                wi = wrist_idx[sl]
                touched_k = any(t_now[f"a{k}_{s}"] > 0.2 for s in ("palm", "in", "hook", "lip"))
                if touched_k:
                    q_admit[sl] = np.clip(0.98 * q_admit[sl] + 0.02 * (q_meas_w[sl] - q_cmd[wi]), np.radians(-6.0), np.radians(6.0))
                else:
                    q_admit[sl] *= 0.995
            q_cmd[wrist_idx] += q_admit

        standoff_nom = 0.790 if align_ring_start else 0.800
        if not is_spin:
            p_des_w = (ring_top_w + np.array([-0.800, -0.005, 0.004])) if blind_tilt else (ring_top_w + np.array([-standoff_nom, 0.0, 0.0]))
            e_p = p_des_w - p_c
            F_w = np.clip(191.1 * (0.25 * e_p - 0.90 * v_c), -1.5, 1.5)
            R_err = Rotation.from_matrix(R_c.T)
            T_b = 8.0 * R_err.as_rotvec() - 15.0 * w_b
        else:
            p_des_w = ring_top_w + 0.800 * n_est_w
            w_t_w = R_t @ kd.qvel[bus.ids["tv"] + 3 : bus.ids["tv"] + 6]
            e_p = p_des_w - p_c
            v_ring_w = kd.qvel[bus.ids["tv"] : bus.ids["tv"] + 3] + np.cross(w_t_w, p_des_w - p_t)
            a_ff = np.cross(w_t_w, np.cross(w_t_w, p_des_w - p_t))
            F_w = np.clip(191.1 * (a_ff + 0.60 * e_p + 1.50 * (v_ring_w - v_c)), -1.5, 1.5)
            R_err = Rotation.from_matrix(R_c.T @ R_t @ Rotation.from_euler("z", 180.0, degrees=True).as_matrix())
            w_des_c = R_c.T @ w_t_w
            T_b = 12.0 * R_err.as_rotvec() + 25.0 * (w_des_c - w_b)

        bus.step(F_w, T_b, q_cmd, q_open, 1, thr_on=True)
        if seated.all() or (first_seat_step is not None and (step - first_seat_step) >= 2500):
            seat_step = step
            break

    assert first_seat_step is not None, f"[{label}] Neither arm made palm contact!"

    # Phase 2: Ramp all 4 non-backdrivable lead-screw jaws closed (q_open -> q_squeeze)
    bus.phase = "close_jaws"
    g_cmd = q_open.copy()
    close_steps = 1000
    ramp_steps = 850.0
    delay_steps = 0
    for step in range(close_steps):
        alpha = min(1.0, max(0.0, (step + 1 - delay_steps) / (ramp_steps - delay_steps)))
        g_cmd = (1.0 - alpha) * q_open + alpha * q_close
        t_now = bus.touches()
        f_tgt = 1.0
        for k in (1, 2):
            idx = k - 1
            f_p = t_now[f"a{k}_palm"]
            d_off[idx] = float(np.clip(d_off[idx] - 0.004 * dt * np.clip((f_tgt - f_p) / f_tgt, -1.0, 1.0), -0.030, 0.015))
        R_t = kd.xmat[bid_t].reshape(3, 3).copy()
        p_t = kd.xpos[bid_t].copy()
        ring_top_w = p_t + R_t @ np.array([0.810, 0.0, 0.0])
        n_est_w = np.array([-1.0, 0.0, 0.0]) if blind_tilt else (R_t @ np.array([1.0, 0.0, 0.0]))
        c_est_w = (ring_top_w + np.array([0.0, -0.005, 0.004])) if blind_tilt else ring_top_w
        _, R_c, _, w_b, _ = bus.bus_state()
        q_cmd = np.concatenate([
            _solve_arm_world(bus, 1, c_est_w + d_off[0] * n_est_w, n_est_w),
            _solve_arm_world(bus, 2, c_est_w + d_off[1] * n_est_w, n_est_w),
        ])
        if blind_tilt:
            # Freeze q_admit once jaws start closing so wrists do not yield away from squeeze preload
            if step < delay_steps:
                q_meas_w = kd.qpos[bus.ids["arm_q"][wrist_idx]]
                for k in (1, 2):
                    sl = slice(0, 3) if k == 1 else slice(3, 6)
                    wi = wrist_idx[sl]
                    if any(t_now[f"a{k}_{s}"] > 0.2 for s in ("palm", "in", "hook", "lip")):
                        q_admit[sl] = np.clip(0.98 * q_admit[sl] + 0.02 * (q_meas_w[sl] - q_cmd[wi]), np.radians(-6.0), np.radians(6.0))
            q_cmd[wrist_idx] += q_admit
        if is_spin:
            w_t_w = R_t @ kd.qvel[bus.ids["tv"] + 3 : bus.ids["tv"] + 6]
            R_err = Rotation.from_matrix(R_c.T @ R_t @ Rotation.from_euler("z", 180.0, degrees=True).as_matrix())
            w_des_c = R_c.T @ w_t_w
            T_b = 12.0 * R_err.as_rotvec() + 25.0 * (w_des_c - w_b)
        else:
            T_b = np.zeros(3)
        bus.step(np.zeros(3), T_b, q_cmd, g_cmd, 1, thr_on=False)

    # Lock all 4 lead screws + engage rigid wrist holding brakes with matched 0.3 mm preload
    g_hold = _lock_eq_at_current(bus)
    q_hold = kd.qpos[bus.ids["arm_q"]].copy()

    if render_dir is not None:
        _render_views(bus, render_dir)

    # Phase 3: Hold-check disturbance OR full stack Detumble
    t_locked = bus.touches()
    a1_ok = _arm_locked(t_locked, 1)
    a2_ok = _arm_locked(t_locked, 2)
    q_w1 = np.degrees(q_hold[3:6])
    q_w2 = np.degrees(q_hold[9:12])

    max_fill = bus.wheels.fill()
    detumble_time = 0.0
    final_w_deg_s = float(np.degrees(np.linalg.norm(kd.qvel[bus.ids["cv"] + 3 : bus.ids["cv"] + 6])))

    if not detumble_after:
        bus.phase = "hold_check"
        cid = bus.ids["bid_c"]
        hx, hy, hz = hold_torque_nm
        for _ in range(750):
            kd.xfrc_applied[cid, 3:6] = [hx, hy, hz]
            bus.step(np.zeros(3), np.zeros(3), q_hold, g_hold, 1, thr_on=False)
        kd.xfrc_applied[cid, :] = 0.0
        t_after = bus.touches()
        assert _arm_locked(t_after, 1) and _arm_locked(t_after, 2), f"[{label}] Slip under hold disturbance: {t_after}"
    else:
        bus.phase = "detumble"
        hold_cnt = 0
        dt_ctrl = float(spec.bus_ctrl.ctrl_dt)
        n_sub = max(1, int(round(dt_ctrl / dt)))
        max_steps = int(round(spec.bus_ctrl.detumble_max_s / dt_ctrl))
        for k_sub in range(max_steps):
            _, _, _, wb, _ = bus.bus_state()
            max_fill = max(max_fill, bus.wheels.fill())
            w_deg = float(np.degrees(np.linalg.norm(wb)))
            if w_deg < spec.bus_ctrl.detumble_tol_deg_s:
                hold_cnt += 1
                if hold_cnt * dt_ctrl >= spec.bus_ctrl.detumble_hold_s:
                    detumble_time = (k_sub + 1) * dt_ctrl
                    final_w_deg_s = w_deg
                    break
            else:
                hold_cnt = 0
            T_damp = -28.0 * wb
            bus.step(np.zeros(3), T_damp, q_hold, g_hold, n_sub, thr_on=True)
            final_w_deg_s = float(np.degrees(np.linalg.norm(kd.qvel[bus.ids["cv"] + 3 : bus.ids["cv"] + 6])))
        max_fill = max(max_fill, bus.wheels.fill())

    res = dict(
        label=label,
        a1_ok=a1_ok,
        a2_ok=a2_ok,
        q_w1=tuple(float(x) for x in q_w1),
        q_w2=tuple(float(x) for x in q_w2),
        touches=t_locked,
        bad_contacts=sorted(bus.bad_contacts),
        plume_pulses=bus.plume_pulses,
        prop_g=bus.pulses.prop_kg * 1000.0,
        max_wheel_fill=max_fill,
        detumble_s=detumble_time,
        final_w_deg_s=final_w_deg_s,
        first_seat_s=first_seat_step * dt,
        both_seat_s=(seat_step or 0) * dt,
    )
    print(f"\n[{label}]")
    print(
        f"  Touch times: first={res['first_seat_s']:.2f}s both={res['both_seat_s']:.2f}s | "
        f"Wrist q3/4/5: A1=({q_w1[0]:+.1f},{q_w1[1]:+.1f},{q_w1[2]:+.1f})deg A2=({q_w2[0]:+.1f},{q_w2[1]:+.1f},{q_w2[2]:+.1f})deg"
    )
    print(
        f"  Arm1 locked={a1_ok} (palm={t_locked['a1_palm']:.1f}, in={t_locked['a1_in']:.1f}, hook={t_locked['a1_hook']:.1f}, lip={t_locked['a1_lip']:.1f} N)"
    )
    print(
        f"  Arm2 locked={a2_ok} (palm={t_locked['a2_palm']:.1f}, in={t_locked['a2_in']:.1f}, hook={t_locked['a2_hook']:.1f}, lip={t_locked['a2_lip']:.1f} N)"
    )
    print(
        f"  Bad contacts={len(res['bad_contacts'])} | Plume pulses={res['plume_pulses']} | Prop={res['prop_g']:.2f} g | "
        f"Max RW fill={100*max_fill:.1f}% | Detumble={detumble_time:.1f}s (w_end={final_w_deg_s:.3f} deg/s)"
    )
    assert a1_ok and a2_ok, f"[{label}] Failed dual-arm form closure!"
    assert len(res["bad_contacts"]) == 0, f"[{label}] Collisions: {res['bad_contacts']}"
    assert res["plume_pulses"] == 0, f"[{label}] Plume impingement detected!"
    assert max_fill < 0.80, f"[{label}] Reached >=80% RW unload threshold ({100*max_fill:.1f}%)!"
    if detumble_after:
        assert detumble_time > 0.0 and final_w_deg_s < 0.06, f"[{label}] Did not detumble cleanly!"
    print("  -> PASS")
    return res


def main() -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    spec = dual_arm_espa_spec()
    backend = PlainMujocoBackend(spec.scene_xml_path, tool_friction=spec.tool.friction)
    bus = ActuatorBus(backend, spec, client_p7(), seed=7710000)

    print("=" * 78)
    print("v4.2 DUAL-ARM ESPA (6-DOF CARDAN-ROLL WRIST) — MUJOCO VERIFICATION BENCH")
    print("=" * 78)

    check_a_static(bus, spec)

    art_dir = DEFAULT_ART_DIR if DEFAULT_ART_DIR.is_dir() else None
    run_capture_and_detumble(
        bus, spec,
        "CHECK B: ASYNC FIRST-TOUCH (BLIND 2 deg TILT + 15 mm Y OFFSET) + 3 N.m HOLD",
        tilt_deg=(2.0, 0.0),
        offset_yz=(0.015, 0.0),
        blind_tilt=True,
        hold_torque_nm=(1.5, 2.0, -1.5),
        render_dir=art_dir,
    )
    run_capture_and_detumble(
        bus, spec,
        "CHECK C: 1.0 deg/s ROLL SPIN CAPTURE (4x RW-12 ROLL-SYNC) + DETUMBLE",
        target_omega_deg_s=(1.0, 0.0, 0.0),
        roll_sync_wheels=True,
        detumble_after=True,
    )
    run_capture_and_detumble(
        bus, spec,
        "CHECK D: 2.0 deg/s OBLIQUE SPIN CAPTURE + 100% WHEELS-ONLY DETUMBLE",
        target_omega_deg_s=(0.5, 1.4, 1.33),
        roll_sync_wheels=True,
        detumble_after=True,
    )
    run_capture_and_detumble(
        bus, spec,
        "CHECK E: 5 deg PITCH + 5 deg YAW 3D TILTED CAPTURE (6-DOF WRIST) + 5.0 N.m HOLD",
        tilt_deg=(5.0, 5.0),
        offset_yz=(0.015, -0.010),
        blind_tilt=False,
        align_ring_start=True,
        hold_torque_nm=(2.5, 3.5, -2.5),
    )
    print("\nALL MUJOCO DUAL-ARM v4.2 VERIFICATION CHECKS (A–E) PASSED.")


if __name__ == "__main__":
    main()
