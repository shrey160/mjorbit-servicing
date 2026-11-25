"""Record a complete P7 mission under MjorbitBackend (--earth-bg) with 5 synchronized square views:
  1. Center:      3rd-person orbital view (Earth limb + stars + servicer + client + arm)
  2. Top-Left:    bus_lidar (depth range colormap)
  3. Bottom-Left: bus_cam (orbital Earth + sun + client)
  4. Top-Right:   wrist_cam (stereo left)
  5. Bottom-Right: wrist_cam2 (stereo right)

Usage (WSL pixi rl env):
  MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    bash tools/rlrun.sh python ../projects/p7_unprepared_client/viz/record_replay.py \
      --seed 7710000 --out ../projects/p7_unprepared_client/runs/replay_p7_earth
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR  # noqa: E402

from mission.run_e2e import run_episode  # noqa: E402
from phases import MissionContext  # noqa: E402
from sim.backend import _make_render_assets  # noqa: E402
from sim.optics import apply_space_optics  # noqa: E402
from specs import MissionConfig, client_p7, servicer_rev2  # noqa: E402

SAMPLE_INTERVALS = {
    "far_hold": 5.0,
    "far_app": 6.0,
    "xfer_to_hold3": 6.0,
    "detumble": 5.0,
    "acq8": 2.0,
    "pull_in3": 2.0,
    "hold3": 2.5,
    "to1.8": 2.5,
    "hold1.8": 2.5,
    "to_station": 2.5,
    "settle": 1.5,
    "unfold": 0.5,
    "to pre-reach": 0.5,
    "to pre-close": 0.4,
    "descent": 0.4,
    "close": 0.4,
    "hold": 0.5,
    "unlock_open": 0.4,
    "retract": 0.4,
    "retreat_restow": 1.0,
}

VIEWS = ("third", "lidar", "bus", "stereo_l", "stereo_r")


def _lidar_colormap(rng_mm: np.ndarray, vmax_m: float = 32.0) -> np.ndarray:
    """Map uint16 lidar range [mm] (0 = no return) to a crisp RGB image."""
    h, w = rng_mm.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    m = rng_mm > 0
    if not np.any(m):
        return rgb
    d = rng_mm[m].astype(np.float32) * 1e-3
    # Non-linear normalization emphasizes close-range ring/bus features (0.5 m .. 32 m)
    t = np.clip(np.log1p(d) / np.log1p(vmax_m), 0.0, 1.0)
    # Near (t=0): bright gold/amber -> Mid (t=0.5): cyan -> Far (t=1.0): deep blue
    r = np.clip(np.where(t < 0.45, 255.0 * (1.0 - t / 0.45) + 40.0 * (t / 0.45), 40.0 * (1.0 - (t - 0.45) / 0.55)), 0, 255)
    g = np.clip(np.where(t < 0.45, 210.0 - 10.0 * (t / 0.45), 220.0 * (1.0 - 0.65 * (t - 0.45) / 0.55)), 0, 255)
    b = np.clip(np.where(t < 0.45, 50.0 + 205.0 * (t / 0.45), 255.0 - 90.0 * (t - 0.45) / 0.55), 0, 255)
    rgb[m] = np.stack([r, g, b], axis=-1).astype(np.uint8)
    return rgb


def _set_third_person_cam(cam: mujoco.MjvCamera, p_s: np.ndarray, p_t: np.ndarray, r_eci_hat: np.ndarray) -> None:
    """Position free 3rd-person camera obliquely above horizon so both craft and Earth limb stay in frame."""
    d_vec = p_s - p_t
    dist = float(np.linalg.norm(d_vec))
    dir_sc = d_vec / max(dist, 1e-6)
    close_blend = float(np.clip((4.5 - dist) / 3.0, 0.0, 1.0))
    lookat = (1.0 - 0.65 * close_blend) * (0.5 * (p_s + p_t)) + (0.65 * close_blend) * (p_t + 0.5 * d_vec)

    r_up = r_eci_hat / max(float(np.linalg.norm(r_eci_hat)), 1e-9)
    side = np.cross(r_up, dir_sc)
    if float(np.linalg.norm(side)) < 1e-4:
        ref = np.array([1.0, 0.0, 0.0]) if abs(dir_sc[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        side = np.cross(ref, dir_sc)
    side /= float(np.linalg.norm(side))

    eye_dir = 0.78 * side + 0.46 * dir_sc + 0.42 * r_up
    eye_dir /= float(np.linalg.norm(eye_dir))
    cam_dist = float(np.clip(1.38 * dist + 1.6, 2.7, 44.0))
    fwd = -eye_dir  # lookat - eye

    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = cam_dist
    cam.azimuth = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
    cam.elevation = float(np.degrees(np.arcsin(np.clip(fwd[2], -0.999, 0.999))))


class ReplayRecorder:
    """Captures 5 synchronized square views cleanly via hooks without altering mission phase logic."""

    def __init__(self, ctx: MissionContext, out_dir: Path, jpeg_quality: int = 84):
        self.ctx = ctx
        self.out_dir = out_dir
        self.jpeg_quality = int(jpeg_quality)
        self.frames_dir = out_dir / "frames"
        for v in VIEWS:
            (self.frames_dir / v).mkdir(parents=True, exist_ok=True)
        tex_dir = out_dir / "textures"
        tex_dir.mkdir(parents=True, exist_ok=True)
        if not (tex_dir / "earth_day.jpg").is_file():
            import io
            assets = _make_render_assets()
            for tname in ("mli_gold.png", "solar_cell.png", "brushed_metal.png", "radiator_osr.png"):
                if tname in assets:
                    (tex_dir / tname).write_bytes(assets[tname])
            if "earth_day.png" in assets:
                im_e = Image.open(io.BytesIO(assets["earth_day.png"])).convert("RGB").resize((1024, 512), Image.Resampling.LANCZOS)
                im_e.save(tex_dir / "earth_day.jpg", quality=88)

        be = ctx.backend
        rkm = be.render_km
        self._r_third = mujoco.Renderer(rkm, 384, 384)
        self._r_bus = mujoco.Renderer(rkm, 224, 224)
        self._r_st = mujoco.Renderer(rkm, 192, 192)
        self._vopt = mujoco.MjvOption()
        self._vopt.sitegroup[:] = 0
        self._cam3 = mujoco.MjvCamera()

        self._id_bcam = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_CAMERA, ctx.servicer.sensors.bus_cam)
        self._id_st1 = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_CAMERA, ctx.servicer.sensors.wrist_cams[0])
        self._id_st2 = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_CAMERA, ctx.servicer.sensors.wrist_cams[1])
        self._id_lamp = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_LIGHT, ctx.servicer.sensors.wrist_lamp)
        self._id_sun = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
        self._arm_bids = [
            mujoco.mj_name2id(ctx.M.km, mujoco.mjtObj.mjOBJ_BODY, b)
            for b in ("arm_base", "upper", "fore", "wrist1", "wrist3")
        ]
        self._grip_sid = mujoco.mj_name2id(ctx.M.km, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self._track_bids = [
            b for b in range(1, rkm.nbody)
            if (mujoco.mj_id2name(rkm, mujoco.mjtObj.mjOBJ_BODY, b) or "") not in ("mjo_earth", "mjo_atmo")
        ]
        self._export_scene3d(rkm, out_dir / "scene3d.json")
        # Keep render_km headlight ambient slightly lifted so unlit spacecraft faces remain readable visually
        rkm.vis.headlight.ambient[:] = [0.09, 0.09, 0.10]

        self.frames: List[Dict[str, Any]] = []
        self._last_t = -1e9
        self._idx = 0
        self.optical_fx = bool(ctx.config.optical_fx)
        self._opt_rng = np.random.default_rng(919)

    def _export_scene3d(self, rkm: mujoco.MjModel, path: Path) -> None:
        bid_to_idx = {b: i for i, b in enumerate(self._track_bids)}
        mat_map = {}
        for m in range(rkm.nmat):
            mn = mujoco.mj_id2name(rkm, mujoco.mjtObj.mjOBJ_MATERIAL, m) or ""
            for key, tag in (
                ("mjo_mli_mat", "mli"), ("mjo_solar_mat", "solar"), ("mjo_metal_mat", "metal"),
                ("mjo_radiator_mat", "radiator"), ("mjo_dish_mat", "dish"), ("mjo_nozzle_mat", "nozzle"),
            ):
                if mn == key:
                    mat_map[m] = tag
        geoms, used_meshes = [], set()
        for g in range(rkm.ngeom):
            b = int(rkm.geom_bodyid[g])
            if b not in bid_to_idx:
                continue
            if int(rkm.geom_group[g]) > 2 or float(rkm.geom_rgba[g][3]) <= 0.01:
                continue
            tp = int(rkm.geom_type[g])
            mid = int(rkm.geom_dataid[g]) if tp == int(mujoco.mjtGeom.mjGEOM_MESH) else -1
            if mid >= 0:
                used_meshes.add(mid)
            geoms.append({
                "b": bid_to_idx[b], "t": tp,
                "pos": [round(float(x), 4) for x in rkm.geom_pos[g]],
                "quat": [round(float(x), 4) for x in rkm.geom_quat[g]],
                "size": [round(float(x), 4) for x in rkm.geom_size[g]],
                "rgba": [round(float(x), 3) for x in rkm.geom_rgba[g]],
                "mat": mat_map.get(int(rkm.geom_matid[g])),
                "mesh": mid,
            })
        meshes = {}
        for mid in sorted(used_meshes):
            v0, nv = int(rkm.mesh_vertadr[mid]), int(rkm.mesh_vertnum[mid])
            f0, nf = int(rkm.mesh_faceadr[mid]), int(rkm.mesh_facenum[mid])
            meshes[str(mid)] = {
                "v": np.round(rkm.mesh_vert[v0 : v0 + nv].reshape(-1), 4).tolist(),
                "f": rkm.mesh_face[f0 : f0 + nf].reshape(-1).tolist(),
            }
        names = [mujoco.mj_id2name(rkm, mujoco.mjtObj.mjOBJ_BODY, b) or f"b{b}" for b in self._track_bids]
        path.write_text(json.dumps({"bodies": names, "geoms": geoms, "meshes": meshes}, separators=(",", ":")))

    def maybe_capture(self, force: bool = False) -> None:
        kd = self.ctx.M.kd
        t = float(kd.time)
        phase = str(self.ctx.M.phase or "init")
        interval = SAMPLE_INTERVALS.get(phase, 2.0)
        if not force and (t - self._last_t) < interval - 1e-6:
            return
        self._last_t = t

        M = self.ctx.M
        be = self.ctx.backend
        be.sync_render()
        rkd = be.render_kd
        rkm = be.render_km

        p_s = kd.xpos[M.bid_c].copy()
        p_t = kd.xpos[M.bid_t].copy()
        range_m = float(np.linalg.norm(p_s - p_t))

        orb = be.orbit_info() or {}
        r_eci = np.asarray(orb.get("R_eci_km", [0.0, 0.0, 1.0]), dtype=float)
        _set_third_person_cam(self._cam3, p_s, p_t, r_eci)

        # 1) Third-person orbital view
        self._r_third.update_scene(rkd, camera=self._cam3, scene_option=self._vopt)
        img_third = self._r_third.render()

        # 2) LiDAR colormap (use own-body masked raw range)
        M.rig.rl.update_scene(kd, camera=M.lid, scene_option=M.rig.opt)
        z_lid = M.rig.rl.render().astype(np.float64) * M.rig._ray
        M.seg.update_scene(kd, camera=M.lid, scene_option=M.rig.opt)
        sg = M.seg.render()
        own = (sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(sg[..., 0], self.ctx.sensors.self_geoms)
        z_lid = np.where((z_lid > 0.3) & (z_lid < 60.0) & (~own), np.round(z_lid * 1000.0), 0).astype(np.uint16)
        img_lid = _lidar_colormap(z_lid, vmax_m=32.0)

        # 3) Bus cam
        self._r_bus.update_scene(rkd, camera=self._id_bcam, scene_option=self._vopt)
        img_bus = self._r_bus.render()

        # 4 & 5) Wrist stereo L + R (illuminate wrist lamp at close range < 3.5 m)
        lamp_prev = int(rkm.light_active[self._id_lamp]) if self._id_lamp >= 0 else 0
        if self._id_lamp >= 0 and range_m < 3.5:
            rkm.light_active[self._id_lamp] = 1
            rkm.light_diffuse[self._id_lamp] = [0.9, 0.9, 0.85]
        self._r_st.update_scene(rkd, camera=self._id_st1, scene_option=self._vopt)
        img_stl = self._r_st.render()
        self._r_st.update_scene(rkd, camera=self._id_st2, scene_option=self._vopt)
        img_str = self._r_st.render()
        if self._id_lamp >= 0:
            rkm.light_active[self._id_lamp] = lamp_prev

        if self.optical_fx:
            img_bus = apply_space_optics(img_bus, rng=self._opt_rng)
            img_stl = apply_space_optics(img_stl, rng=self._opt_rng)
            img_str = apply_space_optics(img_str, rng=self._opt_rng)

        fname = f"f_{self._idx:04d}.jpg"
        for name, arr in (
            ("third", img_third),
            ("lidar", img_lid),
            ("bus", img_bus),
            ("stereo_l", img_stl),
            ("stereo_r", img_str),
        ):
            Image.fromarray(arr).save(self.frames_dir / name / fname, quality=self.jpeg_quality)

        touches = {k: round(v, 1) for k, v in self.ctx.bus.touches().items()}
        r_norm = max(float(np.linalg.norm(r_eci)), 1e-9)
        r_hat = (r_eci / r_norm).tolist()
        sun_dir = (-rkm.light_dir[self._id_sun]).tolist() if self._id_sun >= 0 else [1.0, 0.0, 0.0]
        arm_pts = [
            [round(float(x), 4) for x in kd.xpos[bid]]
            for bid in self._arm_bids
            if bid >= 0
        ]
        if self._grip_sid >= 0:
            arm_pts.append([round(float(x), 4) for x in kd.site_xpos[self._grip_sid]])
        pose = dict(
            chaser=[round(float(x), 4) for x in (*kd.xpos[M.bid_c], *kd.xquat[M.bid_c])],
            client=[round(float(x), 4) for x in (*kd.xpos[M.bid_t], *kd.xquat[M.bid_t])],
            bodies=[
                [round(float(x), 4) for x in (*rkd.xpos[b], *rkd.xquat[b])]
                for b in self._track_bids
            ],
            arm=arm_pts,
            r_hat=[round(float(x), 4) for x in r_hat],
            sun=[round(float(x), 4) for x in sun_dir],
        )
        self.frames.append(
            dict(
                i=self._idx,
                file=fname,
                t=round(t, 2),
                phase=phase,
                range_m=round(range_m, 2),
                prop_g=round(self.ctx.bus.pulses.prop_kg * 1e3, 1),
                wheel_fill=round(self.ctx.bus.wheels.fill(), 3),
                touches=touches,
                eclipse=orb.get("eclipse"),
                pose=pose,
            )
        )
        self._idx += 1
        if self._idx % 40 == 0:
            print(f"  [replay] frame {self._idx:4d} t={t:6.1f}s phase={phase} range={range_m:5.2f}m", flush=True)


def record(
    seed: int,
    out_dir: Path,
    orbit_alt_km: float = 550.0,
    orbit_inc_deg: float = 97.6,
    sun_phase_deg: float = 0.0,
    optical_fx: bool = False,
    g3_policy: str | None = None,
) -> Dict[str, Any]:
    t0 = time.time()
    cfg = MissionConfig(
        backend="mjorbit",
        earth_bg=True,
        orbit_alt_km=orbit_alt_km,
        orbit_inc_deg=orbit_inc_deg,
        sun_phase_deg=sun_phase_deg,
        optical_fx=optical_fx,
        stereo_tilt=True,
        g3_policy_path=g3_policy,
    )
    ctx = MissionContext(client_p7(), servicer_rev2(), cfg)
    rec = ReplayRecorder(ctx, out_dir)

    # Non-intrusive wrapper over ctx.bus.step and ctx.M._before_step (during capture)
    orig_bus_step = ctx.bus.step
    orig_m_before = ctx.M._before_step

    def wrapped_bus_step(F_w, T_b, q_arm_cmd, g_cmd, n_steps, *, thr_on=True, on_step=None):
        orig_bus_step(F_w, T_b, q_arm_cmd, g_cmd, n_steps, thr_on=thr_on, on_step=on_step)
        rec.maybe_capture()

    def wrapped_m_before():
        orig_m_before()
        rec.maybe_capture()

    ctx.bus.step = wrapped_bus_step  # type: ignore[method-assign]
    ctx.M._before_step = wrapped_m_before  # type: ignore[method-assign]

    res = run_episode(ctx, seed)
    rec.maybe_capture(force=True)

    manifest = dict(
        meta=dict(
            seed=seed,
            backend="mjorbit",
            earth_bg=True,
            orbit_alt_km=orbit_alt_km,
            orbit_inc_deg=orbit_inc_deg,
            sun_phase_deg=sun_phase_deg,
            optical_fx=optical_fx,
            g3_policy=g3_policy,
            client=ctx.client.name,
            servicer=ctx.servicer.name,
            n_frames=len(rec.frames),
            views=list(VIEWS),
            record_wall_s=round(time.time() - t0, 1),
        ),
        summary=res,
        frames=rec.frames,
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1, default=float), encoding="utf-8")
    print(f"REPLAY SAVED: {out_dir} ({len(rec.frames)} frames, result={res.get('result')}, coll={len(res.get('contacts_bad', []))}, wall={time.time()-t0:.1f}s)")
    return manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7710000)
    ap.add_argument("--orbit-alt-km", type=float, default=550.0)
    ap.add_argument("--orbit-inc-deg", type=float, default=97.6)
    ap.add_argument("--sun-phase-deg", type=float, default=0.0, help="Initial orbital sun-phase offset [deg] toward terminator")
    ap.add_argument("--optical-fx", action="store_true", help="Apply orbital camera bloom, auto-exposure, and shot/read noise")
    ap.add_argument("--g3-policy", default=None, help="Optional G3 RL residual checkpoint for descent control")
    ap.add_argument("--out", default=str(RUNS_DIR / "replay_p7_earth"))
    a = ap.parse_args()
    record(
        a.seed,
        Path(a.out),
        orbit_alt_km=a.orbit_alt_km,
        orbit_inc_deg=a.orbit_inc_deg,
        sun_phase_deg=a.sun_phase_deg,
        optical_fx=bool(a.optical_fx),
        g3_policy=a.g3_policy,
    )
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
