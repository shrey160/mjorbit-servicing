"""ServicerSpec & exact 6-DOF analytical IK for the v4.2 Dual-Arm Recessed 15" ESPA Servicer."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

EXP_DIR = Path(__file__).resolve().parent
ASSETS = EXP_DIR / "assets"
SRC_DIR = EXP_DIR.parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
from specs.servicer_spec import ArmSpec, BusControlSpec, SensorRigSpec, ServicerSpec, ToolSpec  # noqa: E402

SHOULDER_X = 0.265
SHOULDER_Y = 0.070
SHOULDER_Z = 0.245
L1 = 0.248
L2 = 0.208
PALM_X = 0.128


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def solve_single_arm_ik_6dof(
    base_pos: Tuple[float, float, float],
    p_grasp_b: np.ndarray,
    R_des_b: Optional[np.ndarray] = None,
) -> Tuple[float, float, float, float, float, float]:
    """Solve exact intersecting spherical-wrist 6-DOF IK (q0..q5) in chaser frame.

    R_des_b columns: [x_approach (into ring), y_tan_mj, z_rad_out].
    """
    p_g = np.asarray(p_grasp_b, dtype=float)
    b = np.asarray(base_pos, dtype=float)
    if R_des_b is None:
        vy, vz = b[1], b[2]
        rs = max(math.hypot(vy, vz), 1e-9)
        z_rad = np.array([0.0, vy / rs, vz / rs])
        x_app = np.array([1.0, 0.0, 0.0])
        R_des_b = np.column_stack([x_app, np.cross(z_rad, x_app), z_rad])

    W = p_g - PALM_X * R_des_b[:, 0]
    d = W - b
    dy, dz = float(d[1]), float(d[2])
    r_d = math.hypot(dy, dz)
    if r_d > 1e-7:
        e_yz = np.array([dy / r_d, dz / r_d])
        ur_yz = e_yz if float(np.dot(e_yz, R_des_b[1:, 2])) >= 0.0 else -e_yz
    else:
        n_rad = R_des_b[1:, 2]
        nr = max(float(np.linalg.norm(n_rad)), 1e-9)
        ur_yz = n_rad / nr

    sign_z = 1.0 if b[2] >= 0.0 else -1.0
    q0 = math.atan2(-sign_z * float(ur_yz[0]), sign_z * float(ur_yz[1]))

    dx = float(d[0])
    dr = float(dy * ur_yz[0] + dz * ur_yz[1])
    dist = math.hypot(dx, dr)
    if not (abs(L1 - L2) + 1e-4 < dist < (L1 + L2) - 1e-4):
        raise ValueError(f"Target unreachable: dist={dist:.4f} m outside ({abs(L1-L2):.3f}, {L1+L2:.3f})")

    cos_q2 = max(-1.0, min(1.0, (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)))
    q2 = -math.acos(cos_q2)
    phi = math.atan2(dr, dx)
    psi = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q1 = phi - psi

    R_base = np.eye(3) if sign_z > 0.0 else np.diag([1.0, -1.0, -1.0])
    R_fore = R_base @ _rot_x(q0) @ _rot_y(-(q1 + q2))
    R_w = R_fore.T @ R_des_b

    q4 = math.asin(max(-1.0, min(1.0, float(R_w[1, 0]))))
    q3 = math.atan2(float(R_w[2, 0]), float(R_w[0, 0]))
    q5 = math.atan2(-float(R_w[1, 2]), float(R_w[1, 1]))
    return float(q0), float(q1), float(q2), float(q3), float(q4), float(q5)


def solve_single_arm_ik(
    base_pos: Tuple[float, float, float],
    target_center_yz: Tuple[float, float],
    ring_r: float,
    ring_x: float,
) -> Tuple[float, float, float, float, float, float]:
    """Solve 6-DOF (q0..q5) in the chaser frame toward an un-tilted concentric/offset ring."""
    vy = base_pos[1] - target_center_yz[0]
    vz = base_pos[2] - target_center_yz[1]
    rs = max(math.hypot(vy, vz), 1e-9)
    z_rad = np.array([0.0, vy / rs, vz / rs])
    x_app = np.array([1.0, 0.0, 0.0])
    R_des_b = np.column_stack([x_app, np.cross(z_rad, x_app), z_rad])
    p_grasp = np.array([ring_x, target_center_yz[0], target_center_yz[1]]) + ring_r * z_rad
    return solve_single_arm_ik_6dof(base_pos, p_grasp, R_des_b)


def solve_dual_arm_ik(
    ring_center_yz_bus: Tuple[float, float] = (0.0, 0.0),
    ring_r: float = 0.294,
    ring_x_bus: float = 0.800,
) -> np.ndarray:
    """Return 12-element joint vector [a1_q0..q5, a2_q0..q5] for dual-arm ring grasp."""
    q_a1 = solve_single_arm_ik((SHOULDER_X, +SHOULDER_Y, +SHOULDER_Z), ring_center_yz_bus, ring_r, ring_x_bus)
    q_a2 = solve_single_arm_ik((SHOULDER_X, -SHOULDER_Y, -SHOULDER_Z), ring_center_yz_bus, ring_r, ring_x_bus)
    return np.asarray([*q_a1, *q_a2], dtype=float)


def dual_arm_espa_spec() -> ServicerSpec:
    """v4.2 Dual-Arm Recessed 15" ESPA 6-DOF ServicerSpec compatible with ActuatorBus."""
    kp = np.array([260.0, 260.0, 200.0, 90.0, 70.0, 50.0, 260.0, 260.0, 200.0, 90.0, 70.0, 50.0])
    arm_m = np.array([2.5, 2.2, 1.4, 0.50, 0.40, 0.30, 2.5, 2.2, 1.4, 0.50, 0.40, 0.30])
    kdv = 2.0 * 0.95 * np.sqrt(kp * arm_m)

    stow_single = (0.0, -0.5 * math.pi, 0.0, 0.0, 0.0, 0.0)
    ready_single = solve_single_arm_ik((SHOULDER_X, SHOULDER_Y, SHOULDER_Z), (0.0, 0.0), 0.294, 0.720)
    stow_q = (*stow_single, *stow_single)
    ready_q = (*ready_single, *ready_single)

    return ServicerSpec(
        name="dual_arm_espa_v4_2",
        body_name="chaser",
        bus_json_path=ASSETS / "dual_arm_espa_bus.json",
        scene_xml_path=ASSETS / "dual_arm_espa_scene.xml",
        bus_ctrl=BusControlSpec(),
        arm=ArmSpec(
            joint_names=(
                "a1_q0", "a1_q1", "a1_q2", "a1_q3", "a1_q4", "a1_q5",
                "a2_q0", "a2_q1", "a2_q2", "a2_q3", "a2_q4", "a2_q5",
            ),
            act_names=(
                "a1_tau0", "a1_tau1", "a1_tau2", "a1_tau3", "a1_tau4", "a1_tau5",
                "a2_tau0", "a2_tau1", "a2_tau2", "a2_tau3", "a2_tau4", "a2_tau5",
            ),
            kp=tuple(float(x) for x in kp),
            kdv=tuple(float(x) for x in kdv),
            qd_max_rad_s=float(np.radians(10.0)),
            stow_q=stow_q,
            ready_q=ready_q,
            unfold_qa_delta=(0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0),
            unfold_qb_j4=0.0,
            ik_seeds=(ready_q,),
        ),
        tool=ToolSpec(
            tool_bodies=("a1_wrist_roll", "a1_jaw_u", "a1_jaw_l", "a2_wrist_roll", "a2_jaw_u", "a2_jaw_l"),
            cam_geoms=("a1_cam_unit", "a2_cam_unit"),
            grip_joints=("a1_grip_u", "a1_grip_l", "a2_grip_u", "a2_grip_l"),
            grip_acts=("a1_grip_u_pos", "a1_grip_l_pos", "a2_grip_u_pos", "a2_grip_l_pos"),
            lock_eqs=("a1_lock_u", "a1_lock_l", "a2_lock_u", "a2_lock_l"),
            q_open=(0.018, -0.012, 0.018, -0.012),
            q_squeeze=0.012,
            jaw_close_signs=(-1.0, 1.0),
            ee_site="a1_grip_site",
            touch_names=("a1_palm", "a1_in", "a1_hook", "a1_lip", "a2_palm", "a2_in", "a2_hook", "a2_lip"),
            wrist_ft_name="a1_wrist_force",
            friction=0.4,
        ),
        sensors=SensorRigSpec(
            wrist_cams=("a1_cam1", "a1_cam2"),
        ),
    )
