"""Staged arm trajectories, 500 Hz release contact auditing, and client COM kinematics."""
from __future__ import annotations

from typing import Iterable, List, Optional, Set, Tuple

import mujoco
import numpy as np


def build_staged_segments(
    waypoints: Iterable[Tuple[np.ndarray, np.ndarray]], qd_lim: float, min_seg_s: float = 0.5
) -> List[Tuple[np.ndarray, np.ndarray, float]]:
    """Build piecewise-linear joint trajectory segments bounded by qd_lim."""
    segs: List[Tuple[np.ndarray, np.ndarray, float]] = []
    for a_, b_ in waypoints:
        a_arr = np.asarray(a_, dtype=float).copy()
        b_arr = np.asarray(b_, dtype=float).copy()
        T_s = max(float(np.max(np.abs(b_arr - a_arr))) / max(float(qd_lim), 1e-6), min_seg_s)
        segs.append((a_arr, b_arr, T_s))
    return segs


def eval_staged_segments(
    segs: List[Tuple[np.ndarray, np.ndarray, float]], t_el: float, q_final: np.ndarray
) -> np.ndarray:
    """Evaluate piecewise-linear segments at elapsed time t_el."""
    el = float(t_el)
    for a_, b_, T_s in segs:
        if el <= T_s:
            return a_ + (b_ - a_) * (el / T_s)
        el -= T_s
    return np.asarray(q_final, dtype=float).copy()


def any_client_contact(
    km: mujoco.MjModel, kd: mujoco.MjData, srv_b: Set[int], cli_b: Set[int]
) -> List[float]:
    """500 Hz contact audit: normal forces for ANY servicer-client contact."""
    hits: List[float] = []
    f6 = np.zeros(6)
    for i in range(kd.ncon):
        c = kd.contact[i]
        b1, b2 = int(km.geom_bodyid[c.geom1]), int(km.geom_bodyid[c.geom2])
        if (b1 in srv_b and b2 in cli_b) or (b2 in srv_b and b1 in cli_b):
            mujoco.mj_contactForce(km, kd, i, f6)
            hits.append(float(abs(f6[0])))
    return hits


def client_com_vel(
    kd: mujoco.MjData, bid_t: int, tv: int, com_true: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Return world (v_com, w_world) of the client body."""
    Rc = kd.xmat[bid_t].reshape(3, 3)
    w = Rc @ kd.qvel[tv + 3 : tv + 6]
    v = kd.qvel[tv : tv + 3] + np.cross(w, Rc @ com_true)
    return v, w
