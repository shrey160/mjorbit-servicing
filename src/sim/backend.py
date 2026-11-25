"""Simulation backends: plain zero-g MuJoCo and full mjorbit orbital physics + optional Earth/star visuals."""
from __future__ import annotations

import io
import re
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import PBR_ASSETS_DIR, SIM_SRC  # noqa: E402

EARTH_TEX_5K_PATH = SIM_SRC / "viewer" / "assets" / "earth_day_5400.jpg"
EARTH_TEX_PATH = SIM_SRC / "viewer" / "assets" / "earth_day_2k.jpg"

# Exact upstream mjorbit central body constants (src/cpp/src/gravity.cc & config)
GM_EARTH_KM3_S2 = 398600.4418
R_EARTH_KM = 6378.137
J2_EARTH = 0.00108262668
OMEGA_EARTH_RAD_S = 7.292115e-5


def strip_mjorbit_xml(xml_text: str) -> str:
    """Remove <mjorbit ...> blocks/lines so plain MuJoCo compiles cleanly."""
    cleaned = re.sub(r"<mjorbit\b[^>]*>.*?</mjorbit>", "", xml_text, flags=re.DOTALL)
    cleaned = re.sub(r"<mjorbit\b[^/>]*/>", "", cleaned)
    return "\n".join(
        ln for ln in cleaned.splitlines() if "<mjorbit " not in ln and "use_gravity_gradient" not in ln
    )


def _apply_model_defaults(km: mujoco.MjModel, tool_friction: float = 0.4) -> None:
    km.vis.global_.offwidth = max(int(km.vis.global_.offwidth), 1024)
    km.vis.global_.offheight = max(int(km.vis.global_.offheight), 1024)
    km.vis.map.znear = min(float(km.vis.map.znear), 0.01 / max(float(km.stat.extent), 1e-6))
    prefixes = ("ring_wall", "ring_lip", "pad_u", "pad_w", "pad_l", "hook_finger")
    exact = ("palm", "tool_housing", "tool_neck")
    for g in range(km.ngeom):
        nm = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if nm.startswith(prefixes) or nm in exact:
            km.geom_friction[g, 0] = tool_friction


def _make_uv_sphere_obj(n_lat: int = 72, n_lon: int = 144) -> bytes:
    """Generate an in-memory Wavefront .obj unit sphere with outward normals and equirectangular UVs."""
    lines: List[str] = ["# Equirectangular UV unit sphere"]
    for i in range(n_lat + 1):
        theta = np.pi * i / n_lat
        sin_t = float(np.sin(theta))
        cos_t = float(np.cos(theta))
        v = 1.0 - (i / n_lat)
        for j in range(n_lon + 1):
            lon = -np.pi + 2.0 * np.pi * (j / n_lon)
            x = sin_t * float(np.cos(lon))
            y = sin_t * float(np.sin(lon))
            z = cos_t
            u = j / n_lon
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
            lines.append(f"vt {u:.6f} {v:.6f}")
            lines.append(f"vn {x:.6f} {y:.6f} {z:.6f}")

    stride = n_lon + 1
    for i in range(n_lat):
        for j in range(n_lon):
            v00 = i * stride + j + 1
            v10 = (i + 1) * stride + j + 1
            v01 = v00 + 1
            v11 = v10 + 1
            if i == 0:
                lines.append(f"f {v00}/{v00}/{v00} {v10}/{v10}/{v10} {v11}/{v11}/{v11}")
            elif i == n_lat - 1:
                lines.append(f"f {v00}/{v00}/{v00} {v10}/{v10}/{v10} {v01}/{v01}/{v01}")
            else:
                lines.append(f"f {v00}/{v00}/{v00} {v10}/{v10}/{v10} {v11}/{v11}/{v11}")
                lines.append(f"f {v00}/{v00}/{v00} {v11}/{v11}/{v11} {v01}/{v01}/{v01}")
    return ("\n".join(lines) + "\n").encode("ascii")


def _make_annulus_obj(r_in: float, r_out: float, n_seg: int = 128) -> bytes:
    """Generate an in-memory Wavefront .obj flat annulus in the XY plane with +Z outward normals."""
    lines: List[str] = ["# Atmospheric limb halo annulus"]
    for j in range(n_seg):
        ang = 2.0 * np.pi * j / n_seg
        ca, sa = float(np.cos(ang)), float(np.sin(ang))
        lines.append(f"v {r_in * ca:.6f} {r_in * sa:.6f} 0.0")
        lines.append(f"v {r_out * ca:.6f} {r_out * sa:.6f} 0.0")
    lines.append("vn 0.0 0.0 1.0")
    for j in range(n_seg):
        jn = (j + 1) % n_seg
        vi0 = 2 * j + 1
        vo0 = 2 * j + 2
        vi1 = 2 * jn + 1
        vo1 = 2 * jn + 2
        lines.append(f"f {vi0}//1 {vo0}//1 {vo1}//1")
        lines.append(f"f {vi0}//1 {vo1}//1 {vi1}//1")
    return ("\n".join(lines) + "\n").encode("ascii")


def _encode_png_rgb(arr_u8: np.ndarray) -> bytes:
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(arr_u8, mode="RGB").save(buf, format="PNG")
    return buf.getvalue()


def _bake_pbr_rgb(
    asset_dir: Path,
    size: int,
    tint: Tuple[float, float, float],
    relief: float,
    ao_weight: float,
    bright: float = 1.0,
) -> Optional[bytes]:
    """Bake PBR Color + AO + NormalGL directional relief into a single MuJoCo RGB texture."""
    if not asset_dir.is_dir():
        return None
    color_files = list(asset_dir.glob("*_Color.*"))
    if not color_files:
        return None
    from PIL import Image

    resamp = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS
    col = (
        np.asarray(
            Image.open(io.BytesIO(color_files[0].read_bytes())).convert("RGB").resize((size, size), resamp),
            dtype=np.float32,
        )
        / 255.0
    )

    ao_files = list(asset_dir.glob("*_AmbientOcclusion.*"))
    if ao_files and ao_weight > 0.0:
        ao = (
            np.asarray(
                Image.open(io.BytesIO(ao_files[0].read_bytes())).convert("L").resize((size, size), resamp),
                dtype=np.float32,
            )
            / 255.0
        )
        col *= ((1.0 - ao_weight) + ao_weight * ao)[..., None]

    norm_files = list(asset_dir.glob("*_NormalGL.*"))
    if norm_files and relief > 0.0:
        n_arr = (
            np.asarray(
                Image.open(io.BytesIO(norm_files[0].read_bytes())).convert("RGB").resize((size, size), resamp),
                dtype=np.float32,
            )
            / 255.0
        )
        n_vec = n_arr * 2.0 - 1.0
        n_vec /= np.maximum(np.linalg.norm(n_vec, axis=-1, keepdims=True), 1e-6)
        ldir = np.array([0.42, -0.34, 0.84], dtype=np.float32)
        ldir /= np.linalg.norm(ldir)
        dot = np.clip(np.sum(n_vec * ldir, axis=-1), 0.25, 1.15)
        col *= ((1.0 - relief) + relief * dot)[..., None]

    tint_arr = np.array(tint, dtype=np.float32).reshape(1, 1, 3)
    out = np.clip(col * tint_arr * float(bright) * 255.0, 0.0, 255.0).astype(np.uint8)
    return _encode_png_rgb(out)


def _make_render_assets(earth_dist_m: float = 1500.0, earth_radius_m: float = 1381.4) -> Dict[str, bytes]:
    """Build in-memory textures, UV Earth mesh, and tangent-limb halo rings for rendering."""
    from PIL import Image

    cos_h = min(float(earth_radius_m) / max(float(earth_dist_m), 1e-6), 0.999999)
    sin_h = float(np.sqrt(max(1.0 - cos_h * cos_h, 1e-6)))
    r_limb = float(earth_radius_m) * sin_h
    assets: Dict[str, bytes] = {
        "earth_sphere.obj": _make_uv_sphere_obj(72, 144),
        "atmo_inner.obj": _make_annulus_obj(r_limb * 0.992, r_limb * 1.032, 128),
        "atmo_outer.obj": _make_annulus_obj(r_limb * 1.032, r_limb * 1.085, 128),
    }

    # 1) Power-law magnitude-distributed 1024x1024 starfield (bright stars isolated so CC < 8 px in bus_cam)
    star_rng = np.random.default_rng(42)
    sky = np.zeros((1024, 1024, 3), dtype=np.uint8)
    sky[..., :] = [1, 1, 2]
    tints = np.array(
        [
            [0.78, 0.88, 1.00],  # B/A blue-white
            [0.97, 0.98, 1.00],  # F white
            [1.00, 0.95, 0.84],  # G yellow-white
            [1.00, 0.80, 0.60],  # K warm orange
        ],
        dtype=np.float32,
    )
    tint_probs = np.array([0.22, 0.38, 0.25, 0.15])

    # Faint background stars (< CAM_THR=40)
    n_faint = 1220
    fy = star_rng.integers(0, 1024, size=n_faint)
    fx = star_rng.integers(0, 1024, size=n_faint)
    fb = 10.0 + 24.0 * (star_rng.random(n_faint) ** 2.0)
    ft = tints[star_rng.choice(len(tints), size=n_faint, p=tint_probs)]
    sky[fy, fx] = np.clip(fb[:, None] * ft, 0, 255).astype(np.uint8)

    # Brighter catalog-like stars (45..255) spaced >= 4 px apart so connected components stay < 8 px
    occupied = np.zeros((1024, 1024), dtype=bool)
    n_bright_placed = 0
    for _ in range(1200):
        if n_bright_placed >= 180:
            break
        y = int(star_rng.integers(2, 1022))
        x = int(star_rng.integers(2, 1022))
        if occupied[y - 3 : y + 4, x - 3 : x + 4].any():
            continue
        occupied[y, x] = True
        b = float(45.0 + 210.0 * (star_rng.random() ** 2.4))
        tint = tints[int(star_rng.choice(len(tints), p=tint_probs))]
        # Subtle cross halo capped <= 22 so above-CAM_THR core remains strictly 1 texel
        halo_val = min(22.0, 0.12 * b)
        halo_rgb = np.clip(halo_val * tint, 0, 22).astype(np.uint8)
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            sky[y + dy, x + dx] = np.maximum(sky[y + dy, x + dx], halo_rgb)
        sky[y, x] = np.clip(b * tint, 0, 255).astype(np.uint8)
        n_bright_placed += 1
    assets["stars_sky.png"] = _encode_png_rgb(sky)

    # 2) High-res NASA Blue Marble Earth (prefer 5400x2700 resized cleanly to 4096x2048)
    tex_path = EARTH_TEX_5K_PATH if EARTH_TEX_5K_PATH.is_file() else EARTH_TEX_PATH
    if tex_path.is_file():
        resamp = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS
        im = Image.open(io.BytesIO(tex_path.read_bytes())).convert("RGB").resize((4096, 2048), resamp)
        buf_earth = io.BytesIO()
        im.save(buf_earth, format="PNG")
        assets["earth_day.png"] = buf_earth.getvalue()
    else:
        arr = np.zeros((256, 512, 3), dtype=np.uint8)
        arr[..., :] = [28, 75, 170]
        assets["earth_day.png"] = _encode_png_rgb(arr)

    # 3) Kapton gold MLI foil (client & servicer): baked ambientCG Foil002 or procedural Voronoi fallback
    sz = 512
    g_rad = np.linspace(0.0, 2.0 * np.pi, sz, endpoint=False, dtype=np.float32)
    yy_r, xx_r = np.meshgrid(g_rad, g_rad, indexing="ij")
    baked_mli = _bake_pbr_rgb(PBR_ASSETS_DIR / "Foil002", 512, tint=(0.98, 0.72, 0.22), relief=0.55, ao_weight=0.45, bright=1.12)
    baked_mli_srv = _bake_pbr_rgb(PBR_ASSETS_DIR / "Foil002", 512, tint=(0.90, 0.63, 0.17), relief=0.55, ao_weight=0.45, bright=1.08)
    if baked_mli is not None and baked_mli_srv is not None:
        assets["mli_gold.png"] = baked_mli
        assets["mli_servicer.png"] = baked_mli_srv
    else:
        mli_rng = np.random.default_rng(73)
        nc = 10  # coarse foil pockets
        pts_c = mli_rng.random((nc, nc, 2), dtype=np.float32)
        norms_c = mli_rng.normal(0.0, 1.0, (nc, nc, 3)).astype(np.float32)
        norms_c[..., 2] = np.abs(norms_c[..., 2]) + 1.4
        norms_c /= np.linalg.norm(norms_c, axis=-1, keepdims=True)

        gy, gx = np.meshgrid(
            np.arange(sz, dtype=np.float32) * (nc / sz),
            np.arange(sz, dtype=np.float32) * (nc / sz),
            indexing="ij",
        )
        ci = np.floor(gy).astype(np.int32)
        cj = np.floor(gx).astype(np.int32)
        d1 = np.full((sz, sz), 1e9, dtype=np.float32)
        d2 = np.full((sz, sz), 1e9, dtype=np.float32)
        best_n = np.zeros((sz, sz, 3), dtype=np.float32)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                ni = (ci + di) % nc
                nj = (cj + dj) % nc
                py = ci + di + pts_c[ni, nj, 0]
                px = cj + dj + pts_c[ni, nj, 1]
                dist = np.hypot(gy - py, gx - px)
                m1 = dist < d1
                d2 = np.where(m1, d1, np.minimum(d2, dist))
                d1 = np.where(m1, dist, d1)
                best_n = np.where(m1[..., None], norms_c[ni, nj], best_n)

        # Fine secondary crinkle perturbation inside each pocket
        sub_nx = 0.24 * np.sin(11.0 * xx_r + 7.0 * yy_r) + 0.14 * np.cos(19.0 * xx_r - 13.0 * yy_r)
        sub_ny = 0.24 * np.cos(9.0 * xx_r - 12.0 * yy_r) + 0.14 * np.sin(17.0 * yy_r + 15.0 * xx_r)
        n_jit = best_n.copy()
        n_jit[..., 0] += sub_nx
        n_jit[..., 1] += sub_ny
        n_jit /= np.linalg.norm(n_jit, axis=-1, keepdims=True)

        ldir = np.array([0.45, -0.35, 0.82], dtype=np.float32)
        ldir /= np.linalg.norm(ldir)
        hdir = ldir + np.array([0.0, 0.0, 1.0], dtype=np.float32)
        hdir /= np.linalg.norm(hdir)
        diff = np.clip(np.sum(n_jit * ldir, axis=-1), 0.15, 1.0)
        spec = np.clip(np.sum(n_jit * hdir, axis=-1), 0.0, 1.0) ** 14
        crease = np.clip((d2 - d1) / 0.10, 0.0, 1.0)
        shade = (0.38 + 0.62 * diff) * (0.55 + 0.45 * crease)

        mli = np.empty((sz, sz, 3), dtype=np.float32)
        mli[..., 0] = 224.0 * shade + 30.0 * spec
        mli[..., 1] = 158.0 * shade + 72.0 * spec
        mli[..., 2] = 26.0 * shade + 95.0 * spec
        # Stitched Kapton blanket tape seams on tile edges
        seam = (np.arange(sz) < 3) | (np.arange(sz) >= sz - 2)
        seam_mask = seam[:, None] | seam[None, :]
        mli[seam_mask] = mli[seam_mask] * 0.65 + np.array([45.0, 28.0, 6.0], dtype=np.float32)
        assets["mli_gold.png"] = baked_mli if baked_mli is not None else _encode_png_rgb(np.clip(mli, 0, 255).astype(np.uint8))
        assets["mli_servicer.png"] = (
            baked_mli_srv if baked_mli_srv is not None else _encode_png_rgb(np.clip(mli * 0.88, 0, 255).astype(np.uint8))
        )

    # 4) Photovoltaic solar-cell array texture: baked SolarPanel002 or procedural fallback
    sz_s = 256
    baked_solar = _bake_pbr_rgb(PBR_ASSETS_DIR / "SolarPanel002", 512, tint=(0.92, 0.96, 1.05), relief=0.35, ao_weight=0.35, bright=1.05)
    if baked_solar is not None:
        assets["solar_cell.png"] = baked_solar
    else:
        cell_rng = np.random.default_rng(107)
        solar = np.empty((sz_s, sz_s, 3), dtype=np.float32)
        rows_c, cols_c = 4, 8
        ch, cw = sz_s // rows_c, sz_s // cols_c
        cell_var = cell_rng.uniform(0.88, 1.12, size=(rows_c, cols_c))
        yi = np.arange(sz_s) // ch
        xi = np.arange(sz_s) // cw
        vmap = cell_var[yi[:, None], xi[None, :]]
        solar[..., 0] = 16.0 * vmap
        solar[..., 1] = 28.0 * vmap
        solar[..., 2] = 68.0 * vmap
        fine_bus = (np.arange(sz_s) % (cw // 2)) == 0
        solar[:, fine_bus] = np.array([56.0, 74.0, 114.0])
        gx_s = (np.arange(sz_s) % cw) <= 1
        gy_s = (np.arange(sz_s) % ch) <= 1
        grid = gy_s[:, None] | gx_s[None, :]
        solar[grid] = np.array([152.0, 162.0, 176.0])
        assets["solar_cell.png"] = _encode_png_rgb(np.clip(solar, 0, 255).astype(np.uint8))

    # 5) Brushed aluminum adapter ring & servicer titanium / dark hardware finishes
    baked_ring = _bake_pbr_rgb(PBR_ASSETS_DIR / "Metal049A", 512, tint=(0.92, 0.94, 0.96), relief=0.35, ao_weight=0.30, bright=1.02)
    baked_ti = _bake_pbr_rgb(PBR_ASSETS_DIR / "Metal032", 512, tint=(0.66, 0.69, 0.73), relief=0.40, ao_weight=0.35, bright=1.02)
    baked_dark = _bake_pbr_rgb(PBR_ASSETS_DIR / "Metal032", 512, tint=(0.24, 0.26, 0.30), relief=0.35, ao_weight=0.35, bright=1.05)

    def _proc_metal(seed: int, tint: Tuple[float, float, float]) -> bytes:
        rng_m = np.random.default_rng(seed)
        row_t = rng_m.normal(0.0, 7.5, size=(sz, 1)).astype(np.float32)
        lathe = 4.5 * np.sin(36.0 * yy_r) + 2.5 * np.cos(84.0 * yy_r + 0.4 * np.sin(4.0 * xx_r))
        stk = 4.0 * np.sin(8.0 * yy_r + 1.0 * xx_r) + rng_m.normal(0.0, 2.5, size=(sz, sz)).astype(np.float32)
        lum_m = np.clip(178.0 + row_t + lathe + stk, 120.0, 226.0)
        arr_m = np.stack([lum_m * tint[0], lum_m * tint[1], lum_m * tint[2]], axis=-1)
        return _encode_png_rgb(np.clip(arr_m, 0, 255).astype(np.uint8))

    assets["brushed_metal.png"] = baked_ring if baked_ring is not None else _proc_metal(211, (0.96, 0.98, 1.00))
    assets["hw_titanium.png"] = baked_ti if baked_ti is not None else _proc_metal(223, (0.64, 0.67, 0.71))
    assets["hw_dark.png"] = baked_dark if baked_dark is not None else _proc_metal(227, (0.24, 0.26, 0.30))

    # 6) Seamless 256x256 OSR white/silver thermal radiator tile grid
    osr_rng = np.random.default_rng(313)
    osr = np.empty((sz_s, sz_s, 3), dtype=np.float32)
    rc_o, cc_o = 6, 6
    oh, ow = sz_s // rc_o, sz_s // cc_o
    ov = osr_rng.uniform(0.95, 1.02, size=(rc_o, cc_o))
    omap = ov[np.minimum(np.arange(sz_s) // oh, rc_o - 1)[:, None], np.minimum(np.arange(sz_s) // ow, cc_o - 1)[None, :]]
    osr[..., 0] = 228.0 * omap
    osr[..., 1] = 234.0 * omap
    osr[..., 2] = 242.0 * omap
    ogrid = ((np.arange(sz_s) % oh) <= 1)[:, None] | ((np.arange(sz_s) % ow) <= 1)[None, :]
    osr[ogrid] = np.array([175.0, 182.0, 192.0])
    assets["radiator_osr.png"] = _encode_png_rgb(np.clip(osr, 0, 255).astype(np.uint8))

    return assets


def _attach_geom_material(xml_text: str, geom_name: str, mat_name: str) -> str:
    """Replace inline rgba/material on a named <geom .../> tag in render-only XML."""
    pat = re.compile(rf'<geom\b([^>]*\bname="{re.escape(geom_name)}"[^>]*)/>')

    def _repl(m: re.Match) -> str:
        attrs = m.group(1)
        attrs = re.sub(r'\s*\brgba="[^"]*"', "", attrs)
        attrs = re.sub(r'\s*\bmaterial="[^"]*"', "", attrs)
        return f'<geom{attrs} material="{mat_name}"/>'

    return pat.sub(_repl, xml_text, count=1)


def _inject_earth_visuals(stripped_xml: str, earth_dist_m: float, earth_radius_m: float) -> str:
    """Inject visual-only starfield, UV Earth + atmo halo, Earthshine light, and craft textures into render XML."""
    s = stripped_xml
    if "<quality " in s:
        s = re.sub(r"<quality\b[^>]*/>", '<quality shadowsize="4096" offsamples="4"/>', s, count=1)
    else:
        s = s.replace("<visual>\n", '<visual>\n    <quality shadowsize="4096" offsamples="4"/>\n', 1)
    if "<map " in s:
        s = re.sub(r"<map\b([^>]*)/>", r'<map\1 zfar="2000"/>', s, count=1)
    else:
        s = s.replace("<visual>\n", '<visual>\n    <map zfar="2000"/>\n', 1)

    d_limb = earth_dist_m * (1.0 - (earth_radius_m / max(earth_dist_m, 1e-6)) ** 2)
    asset_inject = (
        f'    <mesh name="mjo_earth_mesh" file="earth_sphere.obj" scale="{earth_radius_m:.3f} {earth_radius_m:.3f} {earth_radius_m:.3f}"/>\n'
        '    <mesh name="mjo_atmo_inner_mesh" file="atmo_inner.obj" inertia="shell"/>\n'
        '    <mesh name="mjo_atmo_outer_mesh" file="atmo_outer.obj" inertia="shell"/>\n'
        '    <texture name="mjo_stars_tex" type="skybox" file="stars_sky.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_earth_tex" type="2d" file="earth_day.png"/>\n'
        '    <texture name="mjo_mli_tex" type="cube" file="mli_gold.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_mli_servicer_tex" type="cube" file="mli_servicer.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_solar_tex" type="cube" file="solar_cell.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_metal_tex" type="cube" file="brushed_metal.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_hw_ti_tex" type="cube" file="hw_titanium.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_hw_dark_tex" type="cube" file="hw_dark.png" gridsize="1 1"/>\n'
        '    <texture name="mjo_radiator_tex" type="cube" file="radiator_osr.png" gridsize="1 1"/>\n'
        '    <material name="mjo_earth_mat" texture="mjo_earth_tex" rgba="1 1 1 1" emission="0.08" specular="0.06" shininess="0.12" reflectance="0"/>\n'
        '    <material name="mjo_atmo_inner_mat" rgba="0.38 0.72 1.0 0.28" emission="0.55" specular="0" shininess="0" reflectance="0"/>\n'
        '    <material name="mjo_atmo_outer_mat" rgba="0.22 0.50 1.0 0.11" emission="0.45" specular="0" shininess="0" reflectance="0"/>\n'
        '    <material name="mjo_mli_mat" texture="mjo_mli_tex" rgba="1 1 1 1" texrepeat="2 2" specular="0.72" shininess="0.68"/>\n'
        '    <material name="mjo_mli_servicer_mat" texture="mjo_mli_servicer_tex" rgba="1 1 1 1" texrepeat="2 2" specular="0.72" shininess="0.68"/>\n'
        '    <material name="mjo_solar_mat" texture="mjo_solar_tex" rgba="1 1 1 1" texrepeat="2 2" specular="0.75" shininess="0.85"/>\n'
        '    <material name="mjo_metal_mat" texture="mjo_metal_tex" rgba="1 1 1 1" texrepeat="2 2" specular="0.85" shininess="0.90"/>\n'
        '    <material name="mjo_hw_titanium_mat" texture="mjo_hw_ti_tex" rgba="1 1 1 1" texrepeat="3 3" specular="0.70" shininess="0.72"/>\n'
        '    <material name="mjo_hw_dark_mat" texture="mjo_hw_dark_tex" rgba="1 1 1 1" texrepeat="3 3" specular="0.45" shininess="0.52"/>\n'
        '    <material name="mjo_radiator_mat" texture="mjo_radiator_tex" rgba="1 1 1 1" texrepeat="3 3" specular="0.35" shininess="0.50"/>\n'
        '    <material name="mjo_dish_mat" rgba="0.90 0.91 0.93 1" specular="0.35" shininess="0.45"/>\n'
        '    <material name="mjo_nozzle_mat" rgba="0.28 0.30 0.34 1" specular="0.78" shininess="0.85"/>\n'
        '    <material name="mjo_mli_solid_mat" rgba="0.82 0.60 0.16 1" specular="0.78" shininess="0.72"/>\n'
        '    <material name="mjo_solar_solid_mat" rgba="0.09 0.16 0.36 1" specular="0.82" shininess="0.88"/>\n'
        '    <material name="mjo_rad_solid_mat" rgba="0.84 0.87 0.90 1" specular="0.45" shininess="0.60"/>\n'
        '    <material name="mjo_ti_solid_mat" rgba="0.56 0.59 0.63 1" specular="0.72" shininess="0.74"/>\n'
        '    <material name="mjo_dark_solid_mat" rgba="0.20 0.22 0.26 1" specular="0.48" shininess="0.55"/>\n'
    )
    s = s.replace("  <asset>\n", "  <asset>\n" + asset_inject, 1)

    body_inject = (
        f'    <body name="mjo_earth" mocap="true" pos="0 0 -{earth_dist_m:.2f}">\n'
        f'      <geom name="mjo_earth_geom" type="mesh" mesh="mjo_earth_mesh" '
        f'material="mjo_earth_mat" rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>\n'
        f"    </body>\n"
        f'    <body name="mjo_atmo" mocap="true" pos="0 0 -{d_limb:.2f}">\n'
        f'      <geom name="mjo_atmo_inner_geom" type="mesh" mesh="mjo_atmo_inner_mesh" '
        f'material="mjo_atmo_inner_mat" rgba="0.38 0.72 1.0 0.28" contype="0" conaffinity="0" group="1"/>\n'
        f'      <geom name="mjo_atmo_outer_geom" type="mesh" mesh="mjo_atmo_outer_mesh" '
        f'material="mjo_atmo_outer_mat" rgba="0.22 0.50 1.0 0.11" contype="0" conaffinity="0" group="1"/>\n'
        f"    </body>\n"
    )
    s = s.replace("  <worldbody>\n", "  <worldbody>\n" + body_inject, 1)

    for g_name, m_name in (
        ("client_bus", "mjo_mli_mat"),
        ("wing_r_panel1", "mjo_solar_mat"),
        ("wing_r_panel2", "mjo_solar_mat"),
        ("wing_r_panel3", "mjo_solar_mat"),
        ("adapter_ring", "mjo_metal_mat"),
        ("radiator_1", "mjo_radiator_mat"),
        ("hga_dish", "mjo_dish_mat"),
        ("thruster_main", "mjo_nozzle_mat"),
        ("rcs_1", "mjo_nozzle_mat"),
        ("rcs_2", "mjo_nozzle_mat"),
        ("rcs_3", "mjo_nozzle_mat"),
        ("rcs_4", "mjo_nozzle_mat"),
    ):
        s = _attach_geom_material(s, g_name, m_name)

    earthshine_light = (
        '    <light name="mjo_earthshine" directional="true" castshadow="false" '
        'pos="0 0 -10" dir="0 0 1" diffuse="0.14 0.20 0.30" specular="0.03 0.05 0.08"/>\n'
        '    <light name="mjo_bounce_rim" directional="true" castshadow="false" '
        'pos="0 0 10" dir="0 0 -1" diffuse="0.09 0.085 0.08" specular="0.04 0.04 0.04"/>\n'
    )
    s = s.replace("  </worldbody>", earthshine_light + "  </worldbody>", 1)
    return s


def _assign_servicer_materials(render_km: mujoco.MjModel) -> None:
    """Assign baked PBR textures on primitives and solid specular materials on UV-less STL meshes."""
    mid_mli = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_mli_servicer_mat")
    mid_solar = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_solar_mat")
    mid_rad = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_radiator_mat")
    mid_noz = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_nozzle_mat")
    mid_ti = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_hw_titanium_mat")
    mid_dark = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_hw_dark_mat")

    mid_solar_s = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_solar_solid_mat")
    mid_rad_s = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_rad_solid_mat")
    mid_ti_s = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_ti_solid_mat")
    mid_dark_s = mujoco.mj_name2id(render_km, mujoco.mjtObj.mjOBJ_MATERIAL, "mjo_dark_solid_mat")
    tex_mids = {mid_mli, mid_solar, mid_rad, mid_ti, mid_dark}

    hw_bodies = ("chaser", "arm_base", "upper", "fore", "wrist1", "wrist2", "wrist3")
    for g in range(render_km.ngeom):
        if int(render_km.geom_matid[g]) >= 0:
            continue
        rgba = render_km.geom_rgba[g].copy()
        if float(rgba[3]) < 0.05:
            continue  # preserve invisible alpha=0 collision boxes (bus_col, cluster bounds, separation ring)
        gname = mujoco.mj_id2name(render_km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        bname = mujoco.mj_id2name(render_km, mujoco.mjtObj.mjOBJ_BODY, int(render_km.geom_bodyid[g])) or ""
        if (
            float(rgba[0]) > 0.85
            and float(rgba[1]) > 0.85
            and float(rgba[2]) > 0.70
            and gname != "srv2_radiator"
            and "gnss" not in gname
        ):
            continue  # preserve emissive wrist lamp face
        if float(np.max(rgba[:3])) < 0.10:
            continue  # preserve dark camera/lidar optics

        is_uvless_mesh = (
            int(render_km.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH)
            and int(render_km.geom_dataid[g]) >= 0
            and int(render_km.mesh_texcoordnum[int(render_km.geom_dataid[g])]) == 0
        )

        target_mid = -1
        if "cells_" in gname:
            target_mid = mid_solar_s if is_uvless_mesh else mid_solar
        elif "radiator" in gname:
            target_mid = mid_rad_s if is_uvless_mesh else mid_rad
        elif "thr_" in gname:
            target_mid = mid_noz
        elif bname == "chaser" and float(rgba[0]) > 0.72 and float(rgba[1]) > 0.50 and float(rgba[2]) < 0.35:
            target_mid = mid_mli
        elif bname in ("jaw_l", "jaw_u") or (
            bname in ("chaser", "upper", "fore", "wrist3") and float(np.max(rgba[:3])) < 0.35
        ):
            target_mid = mid_dark_s if is_uvless_mesh else mid_dark
        elif bname in hw_bodies:
            target_mid = mid_ti_s if is_uvless_mesh else mid_ti
        if target_mid >= 0:
            render_km.geom_matid[g] = target_mid
            if target_mid in tex_mids:
                render_km.geom_rgba[g] = [1.0, 1.0, 1.0, float(rgba[3])]
            else:
                render_km.geom_rgba[g] = [
                    float(render_km.mat_rgba[target_mid, 0]),
                    float(render_km.mat_rgba[target_mid, 1]),
                    float(render_km.mat_rgba[target_mid, 2]),
                    float(rgba[3]),
                ]


def _j2_accel_km_s2(r_eci_km: np.ndarray, gm: float, j2: float, r_eq: float) -> np.ndarray:
    """Exact replica of upstream gravity.cc::j2_accel (km/s^2)."""
    r = float(np.linalg.norm(r_eci_km))
    if r == 0.0:
        return np.zeros(3)
    factor = 1.5 * j2 * gm * (r_eq * r_eq) / (r**5)
    z_r2 = (r_eci_km[2] / r) ** 2
    return np.array(
        [
            factor * r_eci_km[0] * (5.0 * z_r2 - 1.0),
            factor * r_eci_km[1] * (5.0 * z_r2 - 1.0),
            factor * r_eci_km[2] * (5.0 * z_r2 - 3.0),
        ]
    )


def compute_orbital_wrenches(
    km: mujoco.MjModel,
    kd: mujoco.MjData,
    r_chief_km: np.ndarray,
    *,
    use_j2: bool = True,
    use_gravity_gradient: bool = True,
    gm: float = GM_EARTH_KM3_S2,
    j2: float = J2_EARTH,
    r_eq: float = R_EARTH_KM,
) -> np.ndarray:
    """Exact replica of upstream coupling_passive.cc::apply_differential_gravity + apply_gravity_gradient_torques.

    Returns (nbody, 6) world-frame wrenches [Fx, Fy, Fz, Tx, Ty, Tz] applied at each body's COM (xipos).
    """
    out = np.zeros((km.nbody, 6), dtype=float)
    rc2 = float(np.dot(r_chief_km, r_chief_km))
    if rc2 < 1e-9:
        return out
    rc = float(np.sqrt(rc2))
    scale_tide = -gm / (rc2 * rc)
    j2_chief = _j2_accel_km_s2(r_chief_km, gm, j2, r_eq) if use_j2 else np.zeros(3)

    for b in range(1, km.nbody):
        mass = float(km.body_mass[b])
        if mass <= 0.0:
            continue
        rho_km = kd.xipos[b] * 1e-3
        sigma = (2.0 * float(np.dot(rho_km, r_chief_km)) + float(np.dot(rho_km, rho_km))) / rc2
        one_p = 1.0 + sigma
        one_p32 = one_p * float(np.sqrt(one_p))
        f = sigma * (3.0 + 3.0 * sigma + sigma * sigma) / ((1.0 + one_p32) * one_p32)
        r_body_km = r_chief_km + rho_km
        diff_accel_km = scale_tide * (rho_km - f * r_body_km)
        if use_j2:
            diff_accel_km = diff_accel_km + (_j2_accel_km_s2(r_body_km, gm, j2, r_eq) - j2_chief)
        out[b, :3] = mass * diff_accel_km * 1000.0

        if use_gravity_gradient:
            r_mag = float(np.linalg.norm(r_body_km))
            if r_mag > 1e-9:
                r_hat = r_body_km / r_mag
                ximat = kd.ximat[b].reshape(3, 3)
                inertia = km.body_inertia[b]
                J_w = (ximat * inertia[None, :]) @ ximat.T
                coeff = 3.0 * gm / (r_mag * r_mag * r_mag)
                out[b, 3:] = coeff * np.cross(r_hat, J_w @ r_hat)
    return out


class SimBackend(ABC):
    """Abstract interface over MuJoCo zero-g and mjorbit orbital physics backends."""

    km: mujoco.MjModel
    kd: mujoco.MjData
    render_km: mujoco.MjModel
    render_kd: mujoco.MjData
    base_cam: Dict[str, np.ndarray]

    @property
    def dt(self) -> float:
        return float(self.km.opt.timestep)

    @property
    def time(self) -> float:
        return float(self.kd.time)

    @abstractmethod
    def reset(
        self,
        *,
        rng: Optional[np.random.Generator] = None,
        sun_cone_axis: Optional[np.ndarray] = None,
        sun_cone_deg: float = 60.0,
    ) -> np.ndarray:
        """Reset simulation state, configure orbit & lighting, return sun_from unit vector in world frame."""

    @abstractmethod
    def forward(self) -> None:
        """Run forward kinematics/dynamics."""

    @abstractmethod
    def step_physics(self, chaser_bid: Optional[int] = None, chaser_force_w: Optional[np.ndarray] = None) -> None:
        """Advance one physics timestep, injecting orbital wrenches + optional chaser thruster disturbance."""

    def orbital_wrenches(self, kd: Optional[mujoco.MjData] = None) -> np.ndarray:
        return np.zeros((self.km.nbody, 6), dtype=float)

    def advance_orbit_and_apply(self, kd: mujoco.MjData, chaser_bid: int) -> None:
        """Hook for legacy RM.Mission._before_step integration."""

    @abstractmethod
    def sync_render(self) -> None:
        """Synchronize render_kd state (e.g., Earth & sun pose when earth_bg=True) before RGB camera rendering."""

    def orbit_info(self) -> Optional[Dict[str, Any]]:
        return None


class PlainMujocoBackend(SimBackend):
    """Standard zero-g MuJoCo backend (strips <mjorbit>, exact parity with P7 baseline)."""

    def __init__(self, scene_xml_path: Path | str, tool_friction: float = 0.4):
        raw = Path(scene_xml_path).read_text(encoding="utf-8")
        self.km = mujoco.MjModel.from_xml_string(strip_mjorbit_xml(raw))
        _apply_model_defaults(self.km, tool_friction=tool_friction)
        self.kd = mujoco.MjData(self.km)
        self.render_km = self.km
        self.render_kd = self.kd
        self.base_cam = {"pos": self.km.cam_pos.copy(), "quat": self.km.cam_quat.copy()}
        self._lid_sun = mujoco.mj_name2id(self.km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")

    def reset(
        self,
        *,
        rng: Optional[np.random.Generator] = None,
        sun_cone_axis: Optional[np.ndarray] = None,
        sun_cone_deg: float = 60.0,
    ) -> np.ndarray:
        self.km.cam_pos[:] = self.base_cam["pos"]
        self.km.cam_quat[:] = self.base_cam["quat"]
        mujoco.mj_resetData(self.km, self.kd)
        if rng is None:
            sun_from = np.array([1.0, 0.0, 0.0])
            diffuse = 0.9
        else:
            if sun_cone_axis is not None:
                sun_from = _rand_unit_cone(rng, sun_cone_axis, sun_cone_deg)
            else:
                v = rng.normal(size=3)
                sun_from = v / max(float(np.linalg.norm(v)), 1e-12)
            diffuse = float(rng.uniform(0.6, 1.2))
        if self._lid_sun >= 0:
            self.km.light_dir[self._lid_sun] = -sun_from
            self.km.light_diffuse[self._lid_sun] = diffuse
        return sun_from

    def forward(self) -> None:
        mujoco.mj_forward(self.km, self.kd)

    def step_physics(self, chaser_bid: Optional[int] = None, chaser_force_w: Optional[np.ndarray] = None) -> None:
        if chaser_bid is not None and chaser_force_w is not None:
            self.kd.xfrc_applied[chaser_bid, :3] = chaser_force_w
        mujoco.mj_step(self.km, self.kd)

    def sync_render(self) -> None:
        return None


class MjorbitBackend(SimBackend):
    """Full LEO orbital physics backend powered by upstream mjorbit orbit oracle + Encke/J2/GG wrenches.

    Keeps upstream `simulator/` completely untouched while preserving native MuJoCo `eq_active` / `eq_data`
    for ring-tool lead-screw locking and optional Earth/starfield visual rendering for RGB cameras.
    """

    def __init__(
        self,
        scene_xml_path: Path | str,
        orbit_alt_km: float = 550.0,
        orbit_inc_deg: float = 97.6,
        earth_bg: bool = False,
        tool_friction: float = 0.4,
        wrench_refresh_dt: float = 0.02,
        sun_phase_deg: float = 0.0,
    ):
        if str(SIM_SRC) not in sys.path:
            sys.path.insert(0, str(SIM_SRC))
        import mjorbit
        from mjorbit.spec import MjoSpec

        self._mjorbit = mjorbit
        self.orbit_alt_km = float(orbit_alt_km)
        self.orbit_inc_deg = float(orbit_inc_deg)
        self.earth_bg = bool(earth_bg)
        self.wrench_refresh_dt = float(wrench_refresh_dt)
        self.sun_phase_deg = float(sun_phase_deg)

        raw = Path(scene_xml_path).read_text(encoding="utf-8")
        stripped = strip_mjorbit_xml(raw)
        self.km = mujoco.MjModel.from_xml_string(stripped)
        # Match upstream mjorbit integrator promotion (Euler -> implicitfast)
        self.km.opt.integrator = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
        _apply_model_defaults(self.km, tool_friction=tool_friction)
        self.kd = mujoco.MjData(self.km)
        self.base_cam = {"pos": self.km.cam_pos.copy(), "quat": self.km.cam_quat.copy()}
        self._lid_sun = mujoco.mj_name2id(self.km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")

        # 1-body upstream mjorbit oracle (stepped at 0.1 s for exact R_eci, V_eci, sun_vector_eci, eclipse)
        oracle_xml = """<mujoco model="orbit_oracle">
  <option timestep="0.1"/>
  <mjorbit use_j2="true" use_gravity_gradient="false" use_drag="false" use_srp="false" use_magnetic="false"/>
  <worldbody>
    <body name="oracle" pos="0 0 0">
      <freejoint name="oracle_j"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>"""
        self.oracle_model = MjoSpec.from_xml_string(oracle_xml).compile()
        cb = self.oracle_model.central_body
        self._gm = float(cb.gm)
        self._r_eq = float(cb.radius)
        self._j2 = float(cb.j2)
        self._r_mag_km = self._r_eq + self.orbit_alt_km
        self._v_mag_km_s = float(np.sqrt(self._gm / self._r_mag_km))
        orb0 = self._make_sunlit_orbit(raan_rad=0.0, t_epoch=0.0)
        self.oracle_data = self.oracle_model.make_data(orbit=orb0)
        self._oracle_dt = float(self.oracle_model.opt.timestep)
        self._oracle_t = 0.0
        self._orb_wrenches = np.zeros((self.km.nbody, 6), dtype=float)
        self._last_wrench_t = -1e9
        self._t0_epoch = 0.0
        self._sun_diffuse = 0.9

        # Separate render-only MuJoCo model with Earth + starfield skybox if earth_bg=True
        if self.earth_bg:
            earth_dist = 1500.0
            earth_rad = earth_dist * (self._r_eq / self._r_mag_km)
            rxml = _inject_earth_visuals(stripped, earth_dist_m=earth_dist, earth_radius_m=earth_rad)
            self.render_km = mujoco.MjModel.from_xml_string(
                rxml, assets=_make_render_assets(earth_dist_m=earth_dist, earth_radius_m=earth_rad)
            )
            _apply_model_defaults(self.render_km, tool_friction=tool_friction)
            _assign_servicer_materials(self.render_km)
            near_m = 0.01
            far_m = max(3500.0, 2.5 * earth_dist)
            ext_r = max(float(self.render_km.stat.extent), 1e-6)
            self.render_km.vis.map.znear = near_m / ext_r
            self.render_km.vis.map.zfar = far_m / ext_r
            # Keep sun directional shadow frustum centered on spacecraft (~12 m)
            self.render_km.stat.center[:] = self.km.stat.center
            self.render_km.vis.map.shadowclip = min(0.008, 12.0 / ext_r)
            # Minimal bounce floor; directional Earthshine + rim bounce supply 3D shadow gradients
            self.render_km.vis.headlight.active = 1
            self.render_km.vis.headlight.ambient[:] = [0.025, 0.028, 0.036]
            self.render_km.vis.headlight.diffuse[:] = [0.0, 0.0, 0.0]
            self.render_km.vis.headlight.specular[:] = [0.0, 0.0, 0.0]
            self.render_kd = mujoco.MjData(self.render_km)
            earth_bid = mujoco.mj_name2id(self.render_km, mujoco.mjtObj.mjOBJ_BODY, "mjo_earth")
            atmo_bid = mujoco.mj_name2id(self.render_km, mujoco.mjtObj.mjOBJ_BODY, "mjo_atmo")
            self._earth_mocap_id = int(self.render_km.body_mocapid[earth_bid])
            self._atmo_mocap_id = int(self.render_km.body_mocapid[atmo_bid])
            self._render_lid_sun = mujoco.mj_name2id(self.render_km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
            self._render_lid_earthshine = mujoco.mj_name2id(self.render_km, mujoco.mjtObj.mjOBJ_LIGHT, "mjo_earthshine")
            self._render_lid_bounce = mujoco.mj_name2id(self.render_km, mujoco.mjtObj.mjOBJ_LIGHT, "mjo_bounce_rim")
            self._earth_dist_m = earth_dist
            self._atmo_dist_m = earth_dist * (1.0 - (earth_rad / earth_dist) ** 2)
        else:
            self.render_km = self.km
            self.render_kd = self.kd
            self._earth_mocap_id = -1
            self._atmo_mocap_id = -1
            self._render_lid_sun = -1
            self._render_lid_earthshine = -1
            self._render_lid_bounce = -1
            self._earth_dist_m = 0.0
            self._atmo_dist_m = 0.0

    def _make_sunlit_orbit(self, raan_rad: Optional[float] = None, t_epoch: float = 0.0) -> Any:
        # Query analytic sun vector at t_epoch using a temporary orbit, then place chief on dayside hemisphere
        r_tmp = np.array([self._r_mag_km, 0.0, 0.0])
        v_tmp = np.array([0.0, self._v_mag_km_s, 0.0])
        orb = self._mjorbit.OrbitInit(R_eci=r_tmp, V_eci=v_tmp, t=t_epoch)
        if not hasattr(self, "oracle_data"):
            # Initial construction will update after oracle_data exists
            return orb
        self.oracle_data.reset(orb)
        s_eci = np.asarray(self.oracle_data.env.sun_vector_eci, dtype=float)
        s_norm = s_eci / max(float(np.linalg.norm(s_eci)), 1e-12)
        # Pick angular momentum axis h inclined at orbit_inc_deg from +Z (or rotated by raan)
        inc = np.radians(self.orbit_inc_deg)
        raan = 0.0 if raan_rad is None else float(raan_rad)
        h_hat = np.array([np.sin(inc) * np.sin(raan), -np.sin(inc) * np.cos(raan), np.cos(inc)])
        # Project s_norm into the orbital plane so chief starts at sub-solar point (plus optional sun_phase_deg shift)
        r_in_plane = s_norm - float(np.dot(s_norm, h_hat)) * h_hat
        if float(np.linalg.norm(r_in_plane)) < 1e-6:
            r_in_plane = np.cross(h_hat, np.array([1.0, 0.0, 0.0]))
        r_hat = r_in_plane / float(np.linalg.norm(r_in_plane))
        if abs(self.sun_phase_deg) > 1e-6:
            ang = np.radians(self.sun_phase_deg)
            r_hat = r_hat * float(np.cos(ang)) + np.cross(h_hat, r_hat) * float(np.sin(ang))
            r_hat /= max(float(np.linalg.norm(r_hat)), 1e-12)
        v_hat = np.cross(h_hat, r_hat)
        return self._mjorbit.OrbitInit(
            R_eci=self._r_mag_km * r_hat, V_eci=self._v_mag_km_s * v_hat, t=t_epoch
        )

    def reset(
        self,
        *,
        rng: Optional[np.random.Generator] = None,
        sun_cone_axis: Optional[np.ndarray] = None,
        sun_cone_deg: float = 60.0,
    ) -> np.ndarray:
        self.km.cam_pos[:] = self.base_cam["pos"]
        self.km.cam_quat[:] = self.base_cam["quat"]
        mujoco.mj_resetData(self.km, self.kd)
        if self.render_km is not self.km:
            self.render_km.cam_pos[:] = self.base_cam["pos"]
            self.render_km.cam_quat[:] = self.base_cam["quat"]
            mujoco.mj_resetData(self.render_km, self.render_kd)

        t_epoch = float(rng.uniform(0.0, 365.0 * 86400.0)) if rng is not None else 0.0
        raan = float(rng.uniform(0.0, 2.0 * np.pi)) if rng is not None else 0.0
        orb = self._make_sunlit_orbit(raan_rad=raan, t_epoch=t_epoch)
        self.oracle_data.reset(orb)
        self._oracle_t = 0.0
        self._t0_epoch = t_epoch
        self._last_wrench_t = -1e9
        self._orb_wrenches[:] = 0.0

        s_raw = np.asarray(self.oracle_data.env.sun_vector_eci, dtype=float)
        sun_from = s_raw / max(float(np.linalg.norm(s_raw)), 1e-12)
        diffuse = float(rng.uniform(0.6, 1.2)) if rng is not None else 0.9
        self._sun_diffuse = diffuse
        eclipse = float(self.oracle_data.env.eclipse)
        if self._lid_sun >= 0:
            self.km.light_dir[self._lid_sun] = -sun_from
            self.km.light_diffuse[self._lid_sun] = diffuse * eclipse
        if self._render_lid_sun >= 0:
            self.render_km.light_dir[self._render_lid_sun] = -sun_from
            self.render_km.light_diffuse[self._render_lid_sun] = diffuse * eclipse
        return sun_from

    def forward(self) -> None:
        mujoco.mj_forward(self.km, self.kd)
        self._refresh_wrenches(force=True)
        self.sync_render()

    def _advance_oracle(self, target_t: float) -> None:
        while self._oracle_t + self._oracle_dt <= target_t + 1e-9:
            self._mjorbit.mjo_step(self.oracle_model, self.oracle_data)
            self._oracle_t += self._oracle_dt

    def _refresh_wrenches(self, force: bool = False) -> None:
        t_now = float(self.kd.time)
        if force or (t_now - self._last_wrench_t >= self.wrench_refresh_dt - 1e-9):
            self._advance_oracle(t_now)
            r_chief = np.asarray(self.oracle_data.orbit.R_eci, dtype=float)
            self._orb_wrenches = compute_orbital_wrenches(
                self.km,
                self.kd,
                r_chief,
                use_j2=True,
                use_gravity_gradient=True,
                gm=self._gm,
                j2=self._j2,
                r_eq=self._r_eq,
            )
            self._last_wrench_t = t_now

    def orbital_wrenches(self, kd: Optional[mujoco.MjData] = None) -> np.ndarray:
        self._refresh_wrenches(force=False)
        return self._orb_wrenches

    def advance_orbit_and_apply(self, kd: mujoco.MjData, chaser_bid: int) -> None:
        """Preserve freshly written chaser_bid[:3] dF while replacing all orbital wrenches cleanly."""
        self._refresh_wrenches(force=False)
        df_chaser = kd.xfrc_applied[chaser_bid, :3].copy()
        kd.xfrc_applied[:] = self._orb_wrenches
        kd.xfrc_applied[chaser_bid, :3] = df_chaser + self._orb_wrenches[chaser_bid, :3]

    def step_physics(self, chaser_bid: Optional[int] = None, chaser_force_w: Optional[np.ndarray] = None) -> None:
        self._refresh_wrenches(force=False)
        self.kd.xfrc_applied[:] = self._orb_wrenches
        if chaser_bid is not None and chaser_force_w is not None:
            self.kd.xfrc_applied[chaser_bid, :3] += chaser_force_w
        mujoco.mj_step(self.km, self.kd)

    def sync_render(self) -> None:
        if self.render_km is self.km:
            return
        # Mirror randomized camera mounts + light props and robot qpos into render model
        self.render_km.cam_pos[: self.km.ncam] = self.km.cam_pos
        self.render_km.cam_quat[: self.km.ncam] = self.km.cam_quat
        for lid in range(self.km.nlight):
            self.render_km.light_active[lid] = self.km.light_active[lid]
            self.render_km.light_diffuse[lid] = self.km.light_diffuse[lid]
            self.render_km.light_dir[lid] = self.km.light_dir[lid]
        self.render_kd.qpos[:] = self.kd.qpos
        self.render_kd.time = self.kd.time

        r_eci = np.asarray(self.oracle_data.orbit.R_eci, dtype=float)
        r_hat = r_eci / max(float(np.linalg.norm(r_eci)), 1e-12)
        if self._earth_mocap_id >= 0:
            self.render_kd.mocap_pos[self._earth_mocap_id] = -self._earth_dist_m * r_hat
            ang_earth = OMEGA_EARTH_RAD_S * float(self.oracle_data.orbit.t)
            x, y, z, w = Rotation.from_euler("z", ang_earth).as_quat()
            self.render_kd.mocap_quat[self._earth_mocap_id] = [w, x, y, z]
        if self._atmo_mocap_id >= 0:
            dot_zr = float(np.clip(r_hat[2], -1.0, 1.0))
            if dot_zr > 0.999999:
                q_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
            elif dot_zr < -0.999999:
                q_xyzw = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                ax = np.cross(np.array([0.0, 0.0, 1.0]), r_hat)
                ax /= float(np.linalg.norm(ax))
                q_xyzw = Rotation.from_rotvec(ax * float(np.arccos(dot_zr))).as_quat()
            self.render_kd.mocap_pos[self._atmo_mocap_id] = -self._atmo_dist_m * r_hat
            self.render_kd.mocap_quat[self._atmo_mocap_id] = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

        sun_eci = np.asarray(self.oracle_data.env.sun_vector_eci, dtype=float)
        s_hat = sun_eci / max(float(np.linalg.norm(sun_eci)), 1e-12)
        ecl = float(self.oracle_data.env.eclipse)
        if self._render_lid_sun >= 0:
            self.render_km.light_dir[self._render_lid_sun] = -s_hat
            self.render_km.light_diffuse[self._render_lid_sun] = np.array([1.0, 0.985, 0.95]) * (
                self._sun_diffuse * ecl
            )
            self.render_km.light_specular[self._render_lid_sun] = np.array([0.65, 0.63, 0.58]) * ecl
        if self._render_lid_earthshine >= 0:
            self.render_km.light_dir[self._render_lid_earthshine] = r_hat
            es_scale = 0.15 + 0.85 * max(0.0, float(np.dot(r_hat, s_hat)))
            self.render_km.light_diffuse[self._render_lid_earthshine] = np.array([0.18, 0.26, 0.38]) * es_scale
            self.render_km.light_specular[self._render_lid_earthshine] = np.array([0.05, 0.08, 0.12]) * es_scale
        if self._render_lid_bounce >= 0:
            b_dir = 0.75 * s_hat + 0.35 * r_hat
            b_dir /= max(float(np.linalg.norm(b_dir)), 1e-12)
            self.render_km.light_dir[self._render_lid_bounce] = b_dir
            self.render_km.light_diffuse[self._render_lid_bounce] = np.array([0.10, 0.095, 0.085]) * ecl
            self.render_km.light_specular[self._render_lid_bounce] = np.array([0.04, 0.04, 0.04]) * ecl
        mujoco.mj_kinematics(self.render_km, self.render_kd)
        mujoco.mj_camlight(self.render_km, self.render_kd)

    def orbit_info(self) -> Dict[str, Any]:
        sun = np.asarray(self.oracle_data.env.sun_vector_eci, dtype=float)
        sun_u = sun / max(float(np.linalg.norm(sun)), 1e-12)
        return {
            "R_eci_km": [round(float(x), 3) for x in self.oracle_data.orbit.R_eci],
            "V_eci_km_s": [round(float(x), 4) for x in self.oracle_data.orbit.V_eci],
            "eclipse": round(float(self.oracle_data.env.eclipse), 3),
            "sun_from": [round(float(x), 4) for x in sun_u],
            "t_orbit_s": round(float(self.oracle_data.orbit.t), 2),
        }


def _rand_unit_cone(rng: np.random.Generator, axis: np.ndarray, deg: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / max(float(np.linalg.norm(a)), 1e-12)
    ct = float(rng.uniform(np.cos(np.radians(deg)), 1.0))
    ph = float(rng.uniform(0.0, 2.0 * np.pi))
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, ref)
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    st = float(np.sqrt(max(1.0 - ct * ct, 0.0)))
    return ct * a + st * (np.cos(ph) * u + np.sin(ph) * v)
