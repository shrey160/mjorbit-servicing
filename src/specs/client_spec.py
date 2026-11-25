"""Client spacecraft specification: bus/wing geometry, capture interface, smooth keep-out, perception priors."""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import ASSETS_DIR  # noqa: E402


@dataclass(frozen=True)
class RingFeature:
    """Launch adapter ring geometry in client body frame."""

    c_b: Tuple[float, float, float]
    n_b: Tuple[float, float, float]
    r_in: float
    r_wall: float
    r_lip: float
    r_grasp: float
    h_top: float
    h_lip: float
    face_half_size: float
    rho_min: float
    ring_seg_n: int = 144
    seat_nominal_rad_mm: float = 1.2
    seat_nominal_h_mm: float = 0.0

    @property
    def c_arr(self) -> np.ndarray:
        return np.asarray(self.c_b, dtype=float)

    @property
    def n_arr(self) -> np.ndarray:
        v = np.asarray(self.n_b, dtype=float)
        return v / max(float(np.linalg.norm(v)), 1e-12)

    @property
    def r_top(self) -> float:
        return 0.5 * (self.r_in + self.r_lip)


@dataclass(frozen=True)
class WingSpec:
    """Appendage / solar wing geometry for smooth directional keep-out and grasp exclusion."""

    name: str
    axis_b: Tuple[float, float, float]
    span_m: float
    half_angle_deg: float = 55.0
    blend_deg: float = 15.0
    grasp_exclude_deg: float = 40.0
    keepout_clear_m: float = 2.16

    @property
    def axis_arr(self) -> np.ndarray:
        v = np.asarray(self.axis_b, dtype=float)
        return v / max(float(np.linalg.norm(v)), 1e-12)

    @property
    def keepout_radius_m(self) -> float:
        return float(self.span_m + self.keepout_clear_m)


@dataclass(frozen=True)
class ClientPerceptionSpec:
    """Sensor-gating and pose-acquisition priors tailored to the client class."""

    blob_back_m: float = 0.48
    r_gate_m: float = 1.2
    cam_win_m: float = 3.5
    acq_strategy: str = "single_wing_pca"  # "single_wing_pca" | "multi_wing_pca" | "box_grid"
    acq_shell_min_m: float = 1.15
    acq_shell_max_m: float = 4.5
    box_min_m: float = 2.2


@dataclass(frozen=True)
class ClientSpec:
    """Complete client definition for mission planning, perception, keep-out, and scoring."""

    name: str
    body_name: str
    xml_path: Path
    bus_radius_m: float
    r_bus_ko_m: float
    r_hold3_m: float
    r_hold18_m: float
    r_station_m: float
    ring: RingFeature
    wings: Tuple[WingSpec, ...]
    perception: ClientPerceptionSpec
    r_rel_mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
    com_err_m: float = 0.010
    i_err_frac: float = 0.05

    @property
    def r_rel(self) -> np.ndarray:
        return np.asarray(self.r_rel_mat, dtype=float)

    def keepout_radius(self, dir_b: np.ndarray, extra_margin: float = 0.0) -> float:
        """Smooth C^1 cosine-blended keep-out radius along client-body direction dir_b."""
        base = float(self.r_bus_ko_m + extra_margin)
        nrm = float(np.linalg.norm(dir_b))
        if nrm < 1e-12 or not self.wings:
            return base
        u = np.asarray(dir_b, dtype=float) / nrm
        r_out = base
        for wing in self.wings:
            ang = float(np.degrees(np.arccos(np.clip(float(u @ wing.axis_arr), -1.0, 1.0))))
            a0 = max(0.0, wing.half_angle_deg - wing.blend_deg)
            a1 = min(180.0, wing.half_angle_deg + wing.blend_deg)
            w_rad = float(wing.keepout_radius_m + extra_margin)
            if ang <= a0:
                val = w_rad
            elif ang >= a1 or a1 <= a0 + 1e-9:
                val = base
            else:
                t = (ang - a0) / (a1 - a0)
                s = 0.5 * (1.0 + float(np.cos(np.pi * t)))
                val = base + s * (w_rad - base)
            if val > r_out:
                r_out = val
        return r_out

    def wing_axes_world(self, Rw: np.ndarray) -> list[np.ndarray]:
        """Return world-frame wing axis unit vectors for client attitude Rw."""
        return [Rw @ w.axis_arr for w in self.wings]

    def grasp_azimuth_ok(self, rad_w: np.ndarray, Rw: np.ndarray, n_w: np.ndarray) -> bool:
        """Check that candidate grasp radial direction stays clear of all wing keep-out wedges."""
        rn = rad_w - float(rad_w @ n_w) * n_w
        if float(np.linalg.norm(rn)) < 1e-9:
            return False
        rn /= float(np.linalg.norm(rn))
        for w in self.wings:
            wa = Rw @ w.axis_arr
            wp = wa - float(wa @ n_w) * n_w
            wp_n = float(np.linalg.norm(wp))
            if wp_n < 1e-6:
                continue
            wp /= wp_n
            ang_deg = float(np.degrees(np.arccos(np.clip(float(rn @ wp), -1.0, 1.0))))
            if ang_deg < w.grasp_exclude_deg:
                return False
        return True


def _mat_tuple(R: np.ndarray) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    return tuple(tuple(float(x) for x in row) for row in R)  # type: ignore[return-value]


def client_p7() -> ClientSpec:
    """Default P7 asymmetric single-wing client (client_p7, 300.1 kg, 610 mm ring on +X)."""
    r_rel = (Rotation.from_rotvec([-np.pi / 2.0, 0.0, 0.0]) * Rotation.from_quat([0.0, 0.0, 1.0, 0.0])).as_matrix()
    return ClientSpec(
        name="client_p7",
        body_name="target",
        xml_path=ASSETS_DIR / "clients" / "client_p7" / "client_p7.xml",
        bus_radius_m=1.03,
        r_bus_ko_m=3.5,
        r_hold3_m=3.2,
        r_hold18_m=2.0,
        r_station_m=1.0,
        ring=RingFeature(
            c_b=(0.75, 0.0, 0.0),
            n_b=(1.0, 0.0, 0.0),
            r_in=0.283,
            r_wall=0.305,
            r_lip=0.317,
            r_grasp=0.294,
            h_top=0.060,
            h_lip=0.045,
            face_half_size=0.5,
            rho_min=0.12,
            ring_seg_n=144,
        ),
        wings=(
            WingSpec(
                name="wing_y_pos",
                axis_b=(0.0, 1.0, 0.0),
                span_m=3.84,
                half_angle_deg=55.0,
                blend_deg=15.0,
                grasp_exclude_deg=40.0,
                keepout_clear_m=2.16,
            ),
        ),
        perception=ClientPerceptionSpec(
            blob_back_m=0.48,
            r_gate_m=1.2,
            cam_win_m=3.5,
            acq_strategy="single_wing_pca",
            acq_shell_min_m=1.15,
            acq_shell_max_m=4.5,
            box_min_m=2.2,
        ),
        r_rel_mat=_mat_tuple(r_rel),
        com_err_m=0.010,
        i_err_frac=0.05,
    )


def client_double_wing(span_m: float = 3.84) -> ClientSpec:
    """Template symmetric two-wing (+Y / -Y) client preset for modular testing & future scenes."""
    base = client_p7()
    # Shoulder along -Z looks away from +Y/-Y wings symmetrically
    r_rel = (Rotation.from_rotvec([-np.pi / 2.0, 0.0, 0.0]) * Rotation.from_quat([0.0, 0.0, 1.0, 0.0])).as_matrix()
    return ClientSpec(
        name="client_double_wing",
        body_name=base.body_name,
        xml_path=base.xml_path,
        bus_radius_m=base.bus_radius_m,
        r_bus_ko_m=base.r_bus_ko_m,
        r_hold3_m=base.r_hold3_m,
        r_hold18_m=base.r_hold18_m,
        r_station_m=base.r_station_m,
        ring=base.ring,
        wings=(
            WingSpec(name="wing_y_pos", axis_b=(0.0, 1.0, 0.0), span_m=span_m, half_angle_deg=55.0, blend_deg=15.0, grasp_exclude_deg=40.0),
            WingSpec(name="wing_y_neg", axis_b=(0.0, -1.0, 0.0), span_m=span_m, half_angle_deg=55.0, blend_deg=15.0, grasp_exclude_deg=40.0),
        ),
        perception=ClientPerceptionSpec(
            blob_back_m=0.48,
            r_gate_m=1.2,
            cam_win_m=3.5,
            acq_strategy="multi_wing_pca",
            acq_shell_min_m=1.15,
            acq_shell_max_m=span_m + 0.7,
            box_min_m=2.2,
        ),
        r_rel_mat=_mat_tuple(r_rel),
        com_err_m=base.com_err_m,
        i_err_frac=base.i_err_frac,
    )
