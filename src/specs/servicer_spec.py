"""Servicer specification: bus actuation, control gains, arm kinematics, tool, and optical/contact rig."""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import ASSETS_DIR  # noqa: E402
from .client_spec import ClientSpec


@dataclass(frozen=True)
class BusControlSpec:
    ctrl_dt: float = 0.1
    f_max: float = 1.5
    kp_t: float = 0.09
    kd_t: float = 0.54
    kp_a: float = 0.25
    kd_a: float = 0.90
    unload_on: float = 0.8
    unload_off: float = 0.5
    unload_k: float = 0.05
    unload_max: float = 0.2
    plume_dist_m: float = 3.0
    plume_half_cone_deg: float = 30.0
    eta_spin: float = 0.85
    spin_max_deg_s: float = 2.5
    detumble_gain: float = 12.0
    detumble_max_s: float = 180.0
    detumble_tol_deg_s: float = 0.05
    detumble_hold_s: float = 3.0


@dataclass(frozen=True)
class ArmSpec:
    joint_names: Tuple[str, ...]
    act_names: Tuple[str, ...]
    kp: Tuple[float, ...]
    kdv: Tuple[float, ...]
    qd_max_rad_s: float
    stow_q: Tuple[float, ...]
    ready_q: Tuple[float, ...]
    unfold_qa_delta: Tuple[float, ...]
    unfold_qb_j4: float
    ik_seeds: Tuple[Tuple[float, ...], ...]

    @property
    def kp_arr(self) -> np.ndarray:
        return np.asarray(self.kp, dtype=float)

    @property
    def kdv_arr(self) -> np.ndarray:
        return np.asarray(self.kdv, dtype=float)


@dataclass(frozen=True)
class ToolSpec:
    tool_bodies: Tuple[str, ...]
    cam_geoms: Tuple[str, ...]
    grip_joints: Tuple[str, ...]
    grip_acts: Tuple[str, ...]
    lock_eqs: Tuple[str, ...]
    q_open: Tuple[float, ...]
    q_squeeze: float
    jaw_close_signs: Tuple[float, float]
    ee_site: str
    touch_names: Tuple[str, ...]
    wrist_ft_name: str
    friction: float = 0.4


@dataclass(frozen=True)
class SensorRigSpec:
    bus_cam: str = "bus_cam"
    bus_lidar: str = "bus_lidar"
    wrist_cams: Tuple[str, ...] = ("wrist_cam", "wrist_cam2")
    wrist_lamp: str = "wrist_lamp"
    lidar_max_m: float = 60.0
    mount_m: float = 0.010
    mount_deg: float = 1.0
    own_m: float = 0.001
    own_deg: float = 0.1
    bus_cam_res: int = 512
    wrist_res: int = 128


@dataclass(frozen=True)
class ServicerSpec:
    name: str
    body_name: str
    bus_json_path: Path
    scene_xml_path: Path
    bus_ctrl: BusControlSpec
    arm: ArmSpec
    tool: ToolSpec
    sensors: SensorRigSpec

    def load_bus_dict(self) -> dict:
        return json.loads(self.bus_json_path.read_text(encoding="utf-8"))

    def max_linear_accel(self, mass_kg: float) -> float:
        return float(self.bus_ctrl.f_max / max(mass_kg, 1e-6))

    def spin_limit_rad_s(self, client: ClientSpec, mass_kg: float, v_app: float = 0.020) -> float:
        """Conservative analytic spin-rate ceiling at max hold distance for a given client & servicer mass."""
        r_eff = float(client.ring.c_arr[0] + client.ring.h_top + client.r_hold3_m)
        a_lim = self.bus_ctrl.eta_spin * self.max_linear_accel(mass_kg)
        # Solve w^2 * r_eff + 2 * v_app * w = a_lim for w >= 0
        disc = v_app * v_app + r_eff * a_lim
        w_accel = (-v_app + np.sqrt(max(disc, 0.0))) / max(r_eff, 1e-6)
        return float(min(np.radians(self.bus_ctrl.spin_max_deg_s), w_accel))

    def _qa_qb(self, q_stow: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        qa = np.asarray(q_stow, dtype=float).copy() + np.asarray(self.arm.unfold_qa_delta, dtype=float)
        qb = qa.copy()
        qb[3] = float(self.arm.unfold_qb_j4)
        return qa, qb

    def unfold_waypoints(
        self, q_stow: np.ndarray, q_ready: np.ndarray, q_app: np.ndarray
    ) -> Tuple[Tuple[np.ndarray, np.ndarray], ...]:
        qa, qb = self._qa_qb(q_stow)
        qs = np.asarray(q_stow, dtype=float).copy()
        qr = np.asarray(q_ready, dtype=float).copy()
        qp = np.asarray(q_app, dtype=float).copy()
        return ((qs, qa), (qa, qb), (qb, qr), (qr, qp))

    def restow_waypoints(
        self, q_cur: np.ndarray, q_ready: np.ndarray, q_stow: np.ndarray
    ) -> Tuple[Tuple[np.ndarray, np.ndarray], ...]:
        qa, qb = self._qa_qb(q_stow)
        qc = np.asarray(q_cur, dtype=float).copy()
        qr = np.asarray(q_ready, dtype=float).copy()
        qs = np.asarray(q_stow, dtype=float).copy()
        return ((qc, qr), (qr, qb), (qb, qa), (qa, qs))


def servicer_rev2() -> ServicerSpec:
    """Default ESPA-class P7 rev-2 servicer with narrow-elbow 6-DOF arm and ring tool v3."""
    kp = np.array([400.0, 400.0, 300.0, 60.0, 60.0, 60.0])
    arm_m = np.array([3.0, 3.0, 1.5, 0.2, 0.2, 0.2])
    kdv = 2.0 * 0.9 * np.sqrt(kp * arm_m)
    return ServicerSpec(
        name="servicer_rev2",
        body_name="chaser",
        bus_json_path=ASSETS_DIR / "servicer_rev2" / "servicer_rev2_bus.json",
        scene_xml_path=ASSETS_DIR / "servicer_rev2" / "servicer_rev2_scene.xml",
        bus_ctrl=BusControlSpec(),
        arm=ArmSpec(
            joint_names=("j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll"),
            act_names=("tau1", "tau2", "tau3", "tau4", "tau5", "tau6"),
            kp=tuple(float(x) for x in kp),
            kdv=tuple(float(x) for x in kdv),
            qd_max_rad_s=float(np.radians(10.0)),
            stow_q=(0.0, -1.5708, 3.1416, 3.1416, 0.0, 0.0),
            ready_q=(0.0, 0.3, -0.6, 0.3, 0.0, 0.0),
            unfold_qa_delta=(0.0, 0.6, -0.9, 0.0, 0.0, 0.0),
            unfold_qb_j4=0.0,
            ik_seeds=(
                (0.0, 0.3, -0.6, 0.3, 0.0, 0.0),
                (0.0, 0.962, -2.093, 1.131, 0.0, 3.142),
            ),
        ),
        tool=ToolSpec(
            tool_bodies=("wrist3", "jaw_u", "jaw_l"),
            cam_geoms=("cam_unit",),
            grip_joints=("grip_u", "grip_l"),
            grip_acts=("grip_u_pos", "grip_l_pos"),
            lock_eqs=("lock_u", "lock_l"),
            q_open=(0.018, -0.012),
            q_squeeze=0.012,
            jaw_close_signs=(-1.0, 1.0),
            ee_site="grip_site",
            touch_names=("palm", "in", "hook", "lip"),
            wrist_ft_name="wrist_force",
            friction=0.4,
        ),
        sensors=SensorRigSpec(),
    )
