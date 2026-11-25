"""Unified optical/lidar sensor suite, bench-mount randomization, and camera-gated far lidar fix."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from sim.backend import SimBackend
from specs.client_spec import ClientPerceptionSpec
from specs.servicer_spec import SensorRigSpec

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _paths  # noqa: E402,F401

import cams  # noqa: E402
import lidar_nav  # noqa: E402


def extract_client_boxes(km: mujoco.MjModel, bid_t: int) -> list[Tuple[np.ndarray, np.ndarray]]:
    """Extract unrotated colliding box geoms attached to the client body."""
    boxes = []
    for g in range(km.ngeom):
        if (
            km.geom_bodyid[g] == bid_t
            and km.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
            and np.allclose(np.abs(km.geom_quat[g]), [1.0, 0.0, 0.0, 0.0], atol=1e-6)
            and km.geom_contype[g] != 0
        ):
            boxes.append((km.geom_pos[g].copy(), km.geom_size[g].copy()))
    return boxes


def randomize_bench_mount(
    backend: SimBackend,
    rig: cams.CamRig,
    cam_ids: list[int],
    sens_spec: SensorRigSpec,
    rng: np.random.Generator,
    noise_seed: int,
) -> None:
    """Apply shared optical bench + per-sensor mount error and sensor noise params."""
    km = backend.km
    base = backend.base_cam
    o = 0.5 * sum(base["pos"][c] for c in cam_ids)
    Rb = Rotation.from_rotvec(np.radians(rng.uniform(-sens_spec.mount_deg, sens_spec.mount_deg, 3)))
    tb = rng.uniform(-sens_spec.mount_m, sens_spec.mount_m, 3)
    for c in cam_ids:
        Ro = Rotation.from_rotvec(np.radians(rng.uniform(-sens_spec.own_deg, sens_spec.own_deg, 3)))
        km.cam_pos[c] = o + Rb.apply(base["pos"][c] - o) + tb + rng.uniform(-sens_spec.own_m, sens_spec.own_m, 3)
        qq = base["quat"][c]
        x, y, z, w = (Rb * Ro * Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]])).as_quat()
        km.cam_quat[c] = [w, x, y, z]
    rig.l_sig0 = float(rng.uniform(0.005, 0.02))
    rig.l_drop = float(rng.uniform(0.01, 0.05))
    rig._nrng = np.random.default_rng([noise_seed, 771, 7])
    rig.gain = float(rng.uniform(0.75, 1.25))
    rig.noise = float(rng.uniform(0.0, 3.0))


def lidar_fix_gated(
    P: np.ndarray,
    Pl: np.ndarray,
    Rl: np.ndarray,
    Pc: np.ndarray,
    b_cam: Optional[np.ndarray],
    perc: Optional[ClientPerceptionSpec] = None,
    r_gate: Optional[float] = None,
    blob_back: Optional[float] = None,
    max_off_deg: float = 20.0,
) -> Optional[Tuple[np.ndarray, int, bool, np.ndarray]]:
    """Lidar centroid gated around camera bearing ray to reject solar wings."""
    if len(P) < 10:
        return None
    rg = r_gate if r_gate is not None else (perc.r_gate_m if perc is not None else 1.2)
    bb = blob_back if blob_back is not None else (perc.blob_back_m if perc is not None else 0.48)
    W = Pl + P @ Rl.T
    gated = False
    if b_cam is not None:
        v = W - Pc
        s = v @ b_cam
        d_perp = np.linalg.norm(v - s[:, None] * b_cam[None, :], axis=1)
        Wg = W[(s > 1.0) & (d_perp < rg)]
        if len(Wg) >= 10:
            W = Wg
            gated = True
    W = W[np.linalg.norm(W - np.median(W, axis=0), axis=1) < (rg if gated else 1.5)]
    if len(W) < 10:
        return None
    c = W.mean(axis=0)
    u = c - Pl
    u /= max(float(np.linalg.norm(u)), 1e-12)
    c_est = c + bb * u
    cl = Rl.T @ (c_est - Pl)
    if float(np.degrees(np.arccos(np.clip(-cl[2] / max(float(np.linalg.norm(cl)), 1e-12), -1.0, 1.0)))) > max_off_deg:
        return None
    return c_est, int(len(W)), gated, c


class SensorSuite:
    """Wraps CamRig + segmentation self-masking + optional render-only Earth/star background model."""

    def __init__(self, backend: SimBackend, rig: cams.CamRig, sens_spec: SensorRigSpec, bid_c: int):
        self.backend = backend
        self.rig = rig
        self.sens_spec = sens_spec
        self.bid_c = int(bid_c)
        km = backend.km
        self.lid = int(rig.lidar_id)
        self.bcam = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, sens_spec.bus_cam)
        self.dirs = lidar_nav.ray_dirs(rig.lres, km.cam_fovy[self.lid])
        root = km.body_rootid[self.bid_c]
        self.self_geoms = np.array([g for g in range(km.ngeom) if km.body_rootid[km.geom_bodyid[g]] == root])
        self.seg = mujoco.Renderer(km, rig.lres, rig.lres)
        self.seg.enable_segmentation_rendering()

        q = backend.base_cam["quat"][self.lid]
        Rl_b = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        self.bore_b = -Rl_b[:, 2]

        # Separate RGB renderer for bus_cam when backend provides a distinct render_km (e.g., earth_bg=True)
        if backend.render_km is not km:
            self._bg_bcam_r = mujoco.Renderer(backend.render_km, sens_spec.bus_cam_res, sens_spec.bus_cam_res)
            self._bg_bcam_id = mujoco.mj_name2id(backend.render_km, mujoco.mjtObj.mjOBJ_CAMERA, sens_spec.bus_cam)
        else:
            self._bg_bcam_r = None
            self._bg_bcam_id = -1

    def cam_nominal(self, cid: int) -> Tuple[np.ndarray, np.ndarray]:
        kd = self.backend.kd
        Rb = kd.xmat[self.bid_c].reshape(3, 3)
        pb = kd.xpos[self.bid_c]
        q = self.backend.base_cam["quat"][cid]
        Rc = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        return pb + Rb @ self.backend.base_cam["pos"][cid], Rb @ Rc

    def lidar_points(self) -> Tuple[np.ndarray, int]:
        img = self.rig.lidar(self.backend.kd)
        self.seg.update_scene(self.backend.kd, camera=self.lid, scene_option=self.rig.opt)
        sg = self.seg.render()
        own = (sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(sg[..., 0], self.self_geoms)
        img = np.where(own, 0, img)
        return lidar_nav.points(img, self.dirs), int(own.sum())

    def bus_cam_rgb(self) -> np.ndarray:
        if self._bg_bcam_r is None:
            return self.rig.render(self.backend.kd)["bus_cam"]
        self.backend.sync_render()
        self._bg_bcam_r.update_scene(self.backend.render_kd, camera=self._bg_bcam_id, scene_option=self.rig.opt)
        gain = float(getattr(self.rig, "gain", 1.0))
        noise = float(getattr(self.rig, "noise", 0.0))
        img = self._bg_bcam_r.render().astype(np.float32) * gain
        if noise > 0 and hasattr(self.rig, "_nrng"):
            img += self.rig._nrng.normal(0.0, noise, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)
