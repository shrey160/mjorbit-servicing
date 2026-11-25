"""Typed MissionConfig replacing mutable module globals across P7 runners."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class MissionConfig:
    backend: str = "plain"  # "plain" | "mjorbit"
    earth_bg: bool = False
    orbit_alt_km: float = 550.0
    orbit_inc_deg: float = 97.6
    sun_phase_deg: float = 0.0
    optical_fx: bool = False
    d_start: float = 30.0
    d_obs: float = 8.0
    v_far: float = 0.10
    spin_range: Tuple[float, float] = (0.0, 2.0)
    stereo_tilt: bool = True
    cam_mode: str = "stereo"  # "stereo" | "net" | "shift" | ""
    lidar_dt: float = 1.0
    lat_s: float = 0.15
    lat_j_s: float = 0.02
    d_stage: float = 6.0
    v_xfer: float = 0.08
    max_xfer_accel_frac: float = 0.65
    pull_in_max_s: float = 60.0
    pull_in_att_handover_m: float = 0.30
    retract_m: float = 0.20
    retract_v: float = 0.025
    retreat_m: float = 2.2
    retreat_v: float = 0.05
    arm_clear_m: float = 0.8
    open_t_s: float = 4.0
    g3_policy_path: Optional[str] = None
