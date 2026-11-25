"""Headless Blender 5.3 Cycles (OptiX) orbital renderer for mjorbit replays.

Reconstructs the exact 10-body / 134-geom / 89-mesh scene from `scene3d.json`
and per-frame poses (`bodies`, `sun`, `r_hat`, `eclipse`) from `manifest.json`,
then renders photorealistic Cycles PBR keyframes and exact `wrist_cam` views.

Usage (Windows Blender 5.3 CLI):
  "C:/Program Files/Blender Foundation/Blender 5.3/blender.exe" --background --python \
    projects/p7_unprepared_client/viz/render_blender.py -- \
      --replay projects/p7_unprepared_client/runs/replay_p7_earth_g3 \
      --out projects/p7_unprepared_client/runs/blender_proto \
      --frames 45,360,415 --wrist-frame 415 --samples 64 --save-blend
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import SIM_SRC  # noqa: E402


def parse_args() -> argparse.Namespace:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    ap = argparse.ArgumentParser(description="Render mjorbit replay in Blender 5.3 Cycles")
    ap.add_argument("--replay", required=True, help="Path to replay directory with scene3d.json and manifest.json")
    ap.add_argument("--out", required=True, help="Output directory for PNG renders and optional .blend")
    ap.add_argument("--pbr-dir", default="", help="Optional ambientCG PBR folder (defaults to <replay>/../pbr_assets)")
    ap.add_argument("--frames", default="45,360,415", help="Comma-separated frame indices for 3rd-person render")
    ap.add_argument("--wrist-frame", type=int, default=415, help="Frame index for wrist_cam render (-1 to skip)")
    ap.add_argument("--samples", type=int, default=64, help="Cycles samples per pixel")
    ap.add_argument("--width", type=int, default=960, help="3rd-person render width")
    ap.add_argument("--height", type=int, default=540, help="3rd-person render height")
    ap.add_argument("--wrist-res", type=int, default=512, help="Square resolution for wrist_cam render")
    ap.add_argument("--closeup-wrist", action="store_true", help="Also render diagnostic wrist & gripper hardware close-up")
    ap.add_argument("--save-blend", action="store_true", help="Save mjorbit_p7_proto.blend in output directory")
    ap.add_argument("--no-optics", action="store_true", help="Disable ISS-style optical compositor stack")
    ap.add_argument("--seq-start-t", type=float, default=-1.0, help="Start simulation time (s) for interpolated sequence")
    ap.add_argument("--seq-end-t", type=float, default=-1.0, help="End simulation time (s) for interpolated sequence")
    ap.add_argument("--seq-fps", type=float, default=15.0, help="Output FPS for interpolated sequence")
    ap.add_argument("--frame-sleep", type=float, default=0.0, help="Seconds to sleep after each sequence frame for steady GPU duty cycle")
    ap.add_argument("--max-gpu-temp", type=int, default=77, help="Pause sequence rendering above this GPU temp (C)")
    ap.add_argument("--cool-gpu-temp", type=int, default=70, help="Resume sequence rendering below this GPU temp (C)")
    return ap.parse_args(argv)


def clear_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def configure_cycles(scene: bpy.types.Scene, samples: int) -> None:
    scene.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    backend = "OPTIX"
    optix_devs = prefs.get_devices_for_type("OPTIX")
    gpu_devs = [d for d in optix_devs if d.type == "OPTIX"]
    if not gpu_devs:
        backend = "CUDA"
        cuda_devs = prefs.get_devices_for_type("CUDA")
        gpu_devs = [d for d in cuda_devs if d.type == "CUDA"]
    prefs.compute_device_type = backend
    for dev in prefs.get_devices_for_type(backend):
        dev.use = dev.type != "CPU"

    scene.cycles.device = "GPU" if gpu_devs else "CPU"
    scene.cycles.samples = int(samples)
    scene.cycles.use_adaptive_sampling = True
    scene.cycles.adaptive_threshold = 0.03 if int(samples) <= 16 else (0.025 if int(samples) <= 32 else 0.015)
    scene.cycles.use_denoising = True
    if gpu_devs and backend == "OPTIX":
        scene.cycles.denoiser = "OPTIX"
    else:
        scene.cycles.denoiser = "OPENIMAGEDENOISE"
    scene.cycles.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    scene.cycles.use_animated_seed = False
    scene.cycles.sample_clamp_direct = 8.0
    scene.cycles.sample_clamp_indirect = 4.0
    scene.cycles.filter_width = 1.8
    scene.cycles.max_bounces = 8
    scene.cycles.diffuse_bounces = 4
    scene.cycles.glossy_bounces = 4
    scene.cycles.transmission_bounces = 4
    scene.cycles.transparent_max_bounces = 8

    scene.view_settings.view_transform = "AgX"
    try:
        scene.view_settings.look = "AgX - Base Contrast"
    except Exception:
        try:
            scene.view_settings.look = "None"
        except Exception:
            pass
    scene.view_settings.exposure = -0.25
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"


def setup_optical_compositor(scene: bpy.types.Scene) -> None:
    """Attach a Blender 5.3 ISS-style optical compositor (bloom, streaks, dispersion, grade, vignette)."""
    scene.render.use_compositing = True
    ng = bpy.data.node_groups.new("OrbitalOptics", "CompositorNodeTree")
    scene.compositing_node_group = ng
    ng.interface.new_socket(name="Image", in_out="OUTPUT", socket_type="NodeSocketColor")

    rl = ng.nodes.new("CompositorNodeRLayers")
    rl.location = (-760, 0)

    gl_fog = ng.nodes.new("CompositorNodeGlare")
    gl_fog.location = (-500, 80)
    gl_fog.inputs["Type"].default_value = "Fog Glow"
    gl_fog.inputs["Quality"].default_value = "High"
    gl_fog.inputs["Threshold"].default_value = 1.35
    gl_fog.inputs["Smoothness"].default_value = 0.25
    gl_fog.inputs["Strength"].default_value = 0.24
    gl_fog.inputs["Size"].default_value = 0.65
    gl_fog.inputs["Saturation"].default_value = 0.90

    gl_str = ng.nodes.new("CompositorNodeGlare")
    gl_str.location = (-260, 80)
    gl_str.inputs["Type"].default_value = "Streaks"
    gl_str.inputs["Quality"].default_value = "High"
    gl_str.inputs["Threshold"].default_value = 6.0
    gl_str.inputs["Strength"].default_value = 0.06
    gl_str.inputs["Streaks"].default_value = 4
    gl_str.inputs["Streaks Angle"].default_value = math.radians(15.0)
    gl_str.inputs["Fade"].default_value = 0.82

    ld = ng.nodes.new("CompositorNodeLensdist")
    ld.location = (-20, 80)
    ld.inputs["Type"].default_value = "Radial"
    ld.inputs["Distortion"].default_value = -0.005
    ld.inputs["Dispersion"].default_value = 0.003
    ld.inputs["Fit"].default_value = True

    cb = ng.nodes.new("CompositorNodeColorBalance")
    cb.location = (220, 80)
    cb.inputs["Type"].default_value = "Lift/Gamma/Gain"
    cb.inputs["Factor"].default_value = 1.0
    cb.inputs[4].default_value = (0.990, 0.995, 1.006, 1.0)
    cb.inputs[6].default_value = (0.996, 0.999, 1.002, 1.0)
    cb.inputs[8].default_value = (1.03, 1.02, 1.00, 1.0)

    em = ng.nodes.new("CompositorNodeEllipseMask")
    em.location = (-20, -240)
    em.inputs[3].default_value = (0.5, 0.5)
    em.inputs[4].default_value = (0.92, 0.92)

    bl = ng.nodes.new("CompositorNodeBlur")
    bl.location = (220, -240)
    bl.inputs["Type"].default_value = "Gaussian"
    bl.inputs["Size"].default_value = (240.0, 240.0)

    mix_v = ng.nodes.new("ShaderNodeMix")
    mix_v.location = (480, 0)
    mix_v.data_type = "RGBA"
    mix_v.blend_type = "MULTIPLY"
    mix_v.inputs["Factor"].default_value = 0.14

    out = ng.nodes.new("NodeGroupOutput")
    out.location = (700, 0)

    ng.links.new(rl.outputs["Image"], gl_fog.inputs["Image"])
    ng.links.new(gl_fog.outputs["Image"], gl_str.inputs["Image"])
    ng.links.new(gl_str.outputs["Image"], ld.inputs["Image"])
    ng.links.new(ld.outputs["Image"], cb.inputs["Image"])
    ng.links.new(em.outputs["Mask"], bl.inputs["Image"])
    ng.links.new(cb.outputs["Image"], mix_v.inputs[6])
    ng.links.new(bl.outputs["Image"], mix_v.inputs[7])
    ng.links.new(mix_v.outputs[2], out.inputs["Image"])


def _load_image(path: Path, colorspace: str = "sRGB") -> Optional[bpy.types.Image]:
    if not path.is_file():
        return None
    img = bpy.data.images.load(str(path), check_existing=True)
    if colorspace:
        try:
            img.colorspace_settings.name = colorspace
        except Exception:
            pass
    return img


def _load_pbr_set(pbr_dir: Optional[Path], asset_id: str) -> Dict[str, Optional[bpy.types.Image]]:
    out: Dict[str, Optional[bpy.types.Image]] = {}
    if pbr_dir is None:
        return out
    sub = pbr_dir / asset_id
    if not sub.is_dir():
        return out
    for ch, cs in (
        ("Color", "sRGB"),
        ("Roughness", "Non-Color"),
        ("Metalness", "Non-Color"),
        ("NormalGL", "Non-Color"),
        ("Displacement", "Non-Color"),
        ("AmbientOcclusion", "Non-Color"),
    ):
        matches = list(sub.glob(f"*_{ch}.*"))
        if matches:
            out[ch] = _load_image(matches[0], colorspace=cs)
    return out


def _ensure_nodes(obj_with_tree: Any) -> bpy.types.NodeTree:
    if obj_with_tree.node_tree is None:
        obj_with_tree.use_nodes = True
    return obj_with_tree.node_tree


def _new_mat(name: str) -> Tuple[bpy.types.Material, bpy.types.NodeTree, bpy.types.Node, bpy.types.Node]:
    mat = bpy.data.materials.new(name=name)
    nt = _ensure_nodes(mat)
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (540, 0)
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (220, 0)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat, nt, bsdf, out


def _make_uv_mapper(nt: bpy.types.NodeTree, scale: Tuple[float, float, float]) -> bpy.types.Node:
    tc = nt.nodes.new("ShaderNodeTexCoord")
    tc.location = (-920, 120)
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.location = (-730, 120)
    mp.inputs["Scale"].default_value = scale
    nt.links.new(tc.outputs["UV"], mp.inputs["Vector"])
    return mp


def _add_uv_tex(
    nt: bpy.types.NodeTree,
    mp: bpy.types.Node,
    img: Optional[bpy.types.Image],
    y_pos: float,
) -> Optional[bpy.types.Node]:
    if img is None:
        return None
    tx = nt.nodes.new("ShaderNodeTexImage")
    tx.location = (-480, y_pos)
    tx.image = img
    tx.projection = "FLAT"
    nt.links.new(mp.outputs["Vector"], tx.inputs["Vector"])
    return tx


def _add_box_tex(
    nt: bpy.types.NodeTree,
    img: Optional[bpy.types.Image],
    scale: Tuple[float, float, float],
) -> Optional[bpy.types.Node]:
    if img is None:
        return None
    tc = nt.nodes.new("ShaderNodeTexCoord")
    tc.location = (-750, 120)
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.location = (-560, 120)
    mp.inputs["Scale"].default_value = scale
    tx = nt.nodes.new("ShaderNodeTexImage")
    tx.location = (-320, 120)
    tx.image = img
    tx.projection = "BOX"
    tx.projection_blend = 0.2
    nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
    nt.links.new(mp.outputs["Vector"], tx.inputs["Vector"])
    return tx


def _build_kapton_mli_mat(
    name: str,
    tex_img: Optional[bpy.types.Image],
    base_tint: Tuple[float, float, float],
    pbr: Dict[str, Optional[bpy.types.Image]],
) -> bpy.types.Material:
    mat, nt, bsdf, _ = _new_mat(name)
    tc_obj = nt.nodes.new("ShaderNodeTexCoord")
    tc_obj.location = (-920, -280)

    # Macro quilted thermal blanket folds (visible from 1.8 m .. 15 m)
    vor = nt.nodes.new("ShaderNodeTexVoronoi")
    vor.location = (-660, -280)
    vor.feature = "SMOOTH_F1"
    vor.inputs["Scale"].default_value = 9.5
    vor.inputs["Smoothness"].default_value = 0.40
    nt.links.new(tc_obj.outputs["Object"], vor.inputs["Vector"])

    macro_bump = nt.nodes.new("ShaderNodeBump")
    macro_bump.location = (-110, -240)
    macro_bump.inputs["Strength"].default_value = 0.18
    macro_bump.inputs["Distance"].default_value = 0.008
    nt.links.new(vor.outputs["Distance"], macro_bump.inputs["Height"])

    if pbr.get("NormalGL") is not None:
        mp = _make_uv_mapper(nt, (1.2, 1.2, 1.2))
        tx_c = _add_uv_tex(nt, mp, pbr.get("Color"), 280)
        tx_r = _add_uv_tex(nt, mp, pbr.get("Roughness"), 40)
        tx_n = _add_uv_tex(nt, mp, pbr.get("NormalGL"), -140)

        nmap = nt.nodes.new("ShaderNodeNormalMap")
        nmap.location = (-280, -140)
        nmap.space = "TANGENT"
        nmap.inputs["Strength"].default_value = 0.35
        nt.links.new(tx_n.outputs["Color"], nmap.inputs["Color"])
        nt.links.new(nmap.outputs["Normal"], macro_bump.inputs["Normal"])
        nt.links.new(macro_bump.outputs["Normal"], bsdf.inputs["Normal"])

        if tx_r is not None:
            rmp = nt.nodes.new("ShaderNodeMapRange")
            rmp.location = (-160, 40)
            rmp.inputs["From Min"].default_value = 0.0
            rmp.inputs["From Max"].default_value = 1.0
            rmp.inputs["To Min"].default_value = 0.26
            rmp.inputs["To Max"].default_value = 0.44
            nt.links.new(tx_r.outputs["Color"], rmp.inputs["Value"])
            nt.links.new(rmp.outputs["Result"], bsdf.inputs["Roughness"])
        else:
            bsdf.inputs["Roughness"].default_value = 0.32

        if tx_c is not None:
            mix_c = nt.nodes.new("ShaderNodeMix")
            mix_c.data_type = "RGBA"
            mix_c.blend_type = "MULTIPLY"
            mix_c.location = (-80, 260)
            mix_c.inputs["Factor"].default_value = 0.65
            mix_c.inputs[6].default_value = (*base_tint, 1.0)
            nt.links.new(tx_c.outputs["Color"], mix_c.inputs[7])
            nt.links.new(mix_c.outputs[2], bsdf.inputs["Base Color"])
        else:
            bsdf.inputs["Base Color"].default_value = (*base_tint, 1.0)
    else:
        noi = nt.nodes.new("ShaderNodeTexNoise")
        noi.location = (-660, -440)
        noi.inputs["Scale"].default_value = 38.0
        noi.inputs["Detail"].default_value = 6.0
        noi.inputs["Roughness"].default_value = 0.65
        nt.links.new(tc_obj.outputs["Object"], noi.inputs["Vector"])

        micro_bump = nt.nodes.new("ShaderNodeBump")
        micro_bump.location = (-280, -340)
        micro_bump.inputs["Strength"].default_value = 0.30
        micro_bump.inputs["Distance"].default_value = 0.008
        nt.links.new(noi.outputs["Fac"], micro_bump.inputs["Height"])
        nt.links.new(micro_bump.outputs["Normal"], macro_bump.inputs["Normal"])
        nt.links.new(macro_bump.outputs["Normal"], bsdf.inputs["Normal"])

        bsdf.inputs["Roughness"].default_value = 0.32
        tx = _add_box_tex(nt, tex_img, (1.6, 1.6, 1.6))
        if tx is not None:
            mix_c = nt.nodes.new("ShaderNodeMix")
            mix_c.data_type = "RGBA"
            mix_c.blend_type = "MULTIPLY"
            mix_c.location = (-60, 140)
            mix_c.inputs["Factor"].default_value = 0.55
            mix_c.inputs[6].default_value = (*base_tint, 1.0)
            nt.links.new(tx.outputs["Color"], mix_c.inputs[7])
            nt.links.new(mix_c.outputs[2], bsdf.inputs["Base Color"])
        else:
            bsdf.inputs["Base Color"].default_value = (*base_tint, 1.0)

    bsdf.inputs["Metallic"].default_value = 0.94
    return mat


def _build_adapter_ring_mat(
    tex_img: Optional[bpy.types.Image],
    color: Tuple[float, float, float],
    pbr: Dict[str, Optional[bpy.types.Image]],
) -> bpy.types.Material:
    mat, nt, bsdf, _ = _new_mat("mat_adapter_metal")

    # Radial lathe-turned anisotropic brushing around ring X symmetry axis
    tan = nt.nodes.new("ShaderNodeTangent")
    tan.location = (-240, 80)
    tan.direction_type = "RADIAL"
    tan.axis = "X"
    if "Tangent" in bsdf.inputs:
        nt.links.new(tan.outputs["Tangent"], bsdf.inputs["Tangent"])
    if "Anisotropic" in bsdf.inputs:
        bsdf.inputs["Anisotropic"].default_value = 0.65

    # Hairline concentric CNC lathe tool-path grooves + machined metal micro-grain
    tc_obj = nt.nodes.new("ShaderNodeTexCoord")
    tc_obj.location = (-920, -340)
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.location = (-680, -340)
    wave.wave_type = "RINGS"
    wave.rings_direction = "X"
    wave.inputs["Scale"].default_value = 1400.0
    wave.inputs["Distortion"].default_value = 1.2
    wave.inputs["Detail"].default_value = 2.0
    nt.links.new(tc_obj.outputs["Object"], wave.inputs["Vector"])

    grain = nt.nodes.new("ShaderNodeTexNoise")
    grain.location = (-680, -520)
    grain.inputs["Scale"].default_value = 450.0
    grain.inputs["Detail"].default_value = 10.0
    grain.inputs["Roughness"].default_value = 0.72
    nt.links.new(tc_obj.outputs["Object"], grain.inputs["Vector"])

    mix_g = nt.nodes.new("ShaderNodeMath")
    mix_g.operation = "MULTIPLY_ADD"
    mix_g.location = (-440, -400)
    mix_g.inputs[1].default_value = 0.65
    mul_w = nt.nodes.new("ShaderNodeMath")
    mul_w.operation = "MULTIPLY"
    mul_w.location = (-560, -360)
    mul_w.inputs[1].default_value = 0.35
    nt.links.new(wave.outputs["Fac"], mul_w.inputs[0])
    nt.links.new(grain.outputs["Fac"], mix_g.inputs[0])
    nt.links.new(mul_w.outputs["Value"], mix_g.inputs[2])

    lathe_bump = nt.nodes.new("ShaderNodeBump")
    lathe_bump.location = (-100, -240)
    lathe_bump.inputs["Strength"].default_value = 0.06
    lathe_bump.inputs["Distance"].default_value = 0.0005
    nt.links.new(mix_g.outputs["Value"], lathe_bump.inputs["Height"])
    nt.links.new(lathe_bump.outputs["Normal"], bsdf.inputs["Normal"])

    if pbr.get("Color") is not None:
        mp = _make_uv_mapper(nt, (4.5, 4.5, 4.5))
        tx_c = _add_uv_tex(nt, mp, pbr.get("Color"), 280)
        tx_r = _add_uv_tex(nt, mp, pbr.get("Roughness"), 40)
        tx_n = _add_uv_tex(nt, mp, pbr.get("NormalGL"), -140)

        if tx_n is not None:
            nmap = nt.nodes.new("ShaderNodeNormalMap")
            nmap.location = (-320, -140)
            nmap.space = "TANGENT"
            nmap.inputs["Strength"].default_value = 1.15
            nt.links.new(tx_n.outputs["Color"], nmap.inputs["Color"])
            nt.links.new(nmap.outputs["Normal"], lathe_bump.inputs["Normal"])

        if tx_r is not None:
            rmp = nt.nodes.new("ShaderNodeMapRange")
            rmp.location = (-160, 20)
            rmp.inputs["From Min"].default_value = 0.0
            rmp.inputs["From Max"].default_value = 1.0
            rmp.inputs["To Min"].default_value = 0.26
            rmp.inputs["To Max"].default_value = 0.48
            nt.links.new(tx_r.outputs["Color"], rmp.inputs["Value"])
            nt.links.new(rmp.outputs["Result"], bsdf.inputs["Roughness"])
        else:
            bsdf.inputs["Roughness"].default_value = 0.21

        mix_c = nt.nodes.new("ShaderNodeMix")
        mix_c.data_type = "RGBA"
        mix_c.blend_type = "MULTIPLY"
        mix_c.location = (-80, 260)
        mix_c.inputs["Factor"].default_value = 0.45
        mix_c.inputs[6].default_value = (*color, 1.0)
        nt.links.new(tx_c.outputs["Color"], mix_c.inputs[7])
        nt.links.new(mix_c.outputs[2], bsdf.inputs["Base Color"])
    else:
        tx = _add_box_tex(nt, tex_img, (4.0, 4.0, 4.0))
        if tx is not None:
            mix_c = nt.nodes.new("ShaderNodeMix")
            mix_c.data_type = "RGBA"
            mix_c.blend_type = "MULTIPLY"
            mix_c.location = (-60, 120)
            mix_c.inputs["Factor"].default_value = 0.5
            mix_c.inputs[6].default_value = (*color, 1.0)
            nt.links.new(tx.outputs["Color"], mix_c.inputs[7])
            nt.links.new(mix_c.outputs[2], bsdf.inputs["Base Color"])
        else:
            bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.21

    bsdf.inputs["Metallic"].default_value = 0.96
    _attach_ao_crevices(nt, bsdf, distance=0.045, factor=0.55)
    return mat


def _build_solar_mat(
    tex_img: Optional[bpy.types.Image],
    pbr: Dict[str, Optional[bpy.types.Image]],
) -> bpy.types.Material:
    mat, nt, bsdf, _ = _new_mat("mat_solar")
    if pbr.get("Color") is not None:
        mp = _make_uv_mapper(nt, (1.1, 1.1, 1.1))
        tx_c = _add_uv_tex(nt, mp, pbr.get("Color"), 240)
        tx_r = _add_uv_tex(nt, mp, pbr.get("Roughness"), 20)
        tx_m = _add_uv_tex(nt, mp, pbr.get("Metalness"), -120)
        tx_n = _add_uv_tex(nt, mp, pbr.get("NormalGL"), -280)
        nt.links.new(tx_c.outputs["Color"], bsdf.inputs["Base Color"])
        if tx_r is not None:
            nt.links.new(tx_r.outputs["Color"], bsdf.inputs["Roughness"])
        else:
            bsdf.inputs["Roughness"].default_value = 0.16
        if tx_m is not None:
            nt.links.new(tx_m.outputs["Color"], bsdf.inputs["Metallic"])
        else:
            bsdf.inputs["Metallic"].default_value = 0.42
        if tx_n is not None:
            nmap = nt.nodes.new("ShaderNodeNormalMap")
            nmap.location = (-160, -240)
            nmap.space = "TANGENT"
            nmap.inputs["Strength"].default_value = 0.65
            nt.links.new(tx_n.outputs["Color"], nmap.inputs["Color"])
            nt.links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
    else:
        tx = _add_box_tex(nt, tex_img, (1.1, 1.1, 1.1))
        if tx is not None:
            nt.links.new(tx.outputs["Color"], bsdf.inputs["Base Color"])
        else:
            bsdf.inputs["Base Color"].default_value = (0.04, 0.08, 0.22, 1.0)
        bsdf.inputs["Metallic"].default_value = 0.38
        bsdf.inputs["Roughness"].default_value = 0.16
    if "Coat Weight" in bsdf.inputs:
        bsdf.inputs["Coat Weight"].default_value = 0.90
        bsdf.inputs["Coat Roughness"].default_value = 0.04
    return mat


def _build_radiator_mat(tex_img: Optional[bpy.types.Image]) -> bpy.types.Material:
    mat, nt, bsdf, _ = _new_mat("mat_radiator")
    tx = _add_box_tex(nt, tex_img, (1.4, 1.4, 1.4))
    if tx is not None:
        nt.links.new(tx.outputs["Color"], bsdf.inputs["Base Color"])
    else:
        bsdf.inputs["Base Color"].default_value = (0.82, 0.86, 0.90, 1.0)
    bsdf.inputs["Metallic"].default_value = 0.22
    bsdf.inputs["Roughness"].default_value = 0.12
    if "Coat Weight" in bsdf.inputs:
        bsdf.inputs["Coat Weight"].default_value = 0.75
        bsdf.inputs["Coat Roughness"].default_value = 0.04
    return mat


def _attach_ao_crevices(
    nt: bpy.types.NodeTree,
    bsdf: bpy.types.Node,
    distance: float = 0.04,
    factor: float = 0.55,
) -> None:
    # Fast zero-raytrace cavity/edge shading using mesh Pointiness (avoids OptiX ShaderNodeAO kernel stall)
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    geo.location = (-260, 420)
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.location = (-40, 420)
    ramp.color_ramp.elements[0].position = 0.46
    ramp.color_ramp.elements[0].color = (0.45, 0.45, 0.45, 1.0)
    mid = ramp.color_ramp.elements.new(0.50)
    mid.color = (1.0, 1.0, 1.0, 1.0)
    ramp.color_ramp.elements[2].position = 0.55
    ramp.color_ramp.elements[2].color = (1.18, 1.18, 1.18, 1.0)
    nt.links.new(geo.outputs["Pointiness"], ramp.inputs["Fac"])

    base_in = bsdf.inputs["Base Color"]
    mix_ao = nt.nodes.new("ShaderNodeMix")
    mix_ao.data_type = "RGBA"
    mix_ao.blend_type = "MULTIPLY"
    mix_ao.location = (110, 260)
    mix_ao.inputs["Factor"].default_value = factor
    if base_in.is_linked:
        src = base_in.links[0].from_socket
        nt.links.remove(base_in.links[0])
        nt.links.new(src, mix_ao.inputs[6])
    else:
        mix_ao.inputs[6].default_value = tuple(base_in.default_value)
    nt.links.new(ramp.outputs["Color"], mix_ao.inputs[7])
    nt.links.new(mix_ao.outputs[2], base_in)


def _build_hardware_metal_mat(
    name: str,
    rgba: Tuple[float, float, float, float],
    metallic: float,
    roughness: float,
    pbr: Dict[str, Optional[bpy.types.Image]],
    uv_scale: float = 14.0,
    grain_scale: float = 650.0,
    grain_strength: float = 0.07,
    ao_dist: float = 0.04,
    ao_factor: float = 0.60,
) -> bpy.types.Material:
    mat, nt, bsdf, _ = _new_mat(name)
    tc_obj = nt.nodes.new("ShaderNodeTexCoord")
    tc_obj.location = (-900, -340)
    grit = nt.nodes.new("ShaderNodeTexNoise")
    grit.location = (-660, -340)
    grit.inputs["Scale"].default_value = grain_scale
    grit.inputs["Detail"].default_value = 10.0
    grit.inputs["Roughness"].default_value = 0.75
    nt.links.new(tc_obj.outputs["Object"], grit.inputs["Vector"])

    grit_bump = nt.nodes.new("ShaderNodeBump")
    grit_bump.location = (-40, -240)
    grit_bump.inputs["Strength"].default_value = grain_strength
    grit_bump.inputs["Distance"].default_value = 0.0004
    nt.links.new(grit.outputs["Fac"], grit_bump.inputs["Height"])
    nt.links.new(grit_bump.outputs["Normal"], bsdf.inputs["Normal"])

    if pbr.get("Roughness") is not None:
        mp = _make_uv_mapper(nt, (uv_scale, uv_scale, uv_scale))
        tx_c = _add_uv_tex(nt, mp, pbr.get("Color"), 220)
        tx_r = _add_uv_tex(nt, mp, pbr.get("Roughness"), 20)
        tx_n = _add_uv_tex(nt, mp, pbr.get("NormalGL"), -180)
        if tx_c is not None:
            mix_c = nt.nodes.new("ShaderNodeMix")
            mix_c.data_type = "RGBA"
            mix_c.blend_type = "MULTIPLY"
            mix_c.location = (-80, 220)
            mix_c.inputs["Factor"].default_value = 0.45
            mix_c.inputs[6].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
            nt.links.new(tx_c.outputs["Color"], mix_c.inputs[7])
            nt.links.new(mix_c.outputs[2], bsdf.inputs["Base Color"])
        else:
            bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
        rmp = nt.nodes.new("ShaderNodeMapRange")
        rmp.location = (-160, 20)
        rmp.inputs["From Min"].default_value = 0.0
        rmp.inputs["From Max"].default_value = 1.0
        rmp.inputs["To Min"].default_value = max(0.12, roughness - 0.08)
        rmp.inputs["To Max"].default_value = min(0.70, roughness + 0.16)
        nt.links.new(tx_r.outputs["Color"], rmp.inputs["Value"])
        nt.links.new(rmp.outputs["Result"], bsdf.inputs["Roughness"])
        if tx_n is not None:
            nmap = nt.nodes.new("ShaderNodeNormalMap")
            nmap.location = (-240, -180)
            nmap.space = "TANGENT"
            nmap.inputs["Strength"].default_value = 0.80
            nt.links.new(tx_n.outputs["Color"], nmap.inputs["Color"])
            nt.links.new(nmap.outputs["Normal"], grit_bump.inputs["Normal"])
    else:
        bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
        bsdf.inputs["Roughness"].default_value = roughness

    bsdf.inputs["Metallic"].default_value = metallic
    _attach_ao_crevices(nt, bsdf, distance=ao_dist, factor=ao_factor)
    return mat


def _build_simple_mat(
    name: str,
    rgba: Tuple[float, float, float, float],
    metallic: float,
    roughness: float,
    emission_strength: float = 0.0,
) -> bpy.types.Material:
    mat, _, bsdf, _ = _new_mat(name)
    bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if emission_strength > 0.0 and "Emission Color" in bsdf.inputs:
        bsdf.inputs["Emission Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    return mat


def build_materials(tex_dir: Path, pbr_dir: Optional[Path] = None) -> Dict[str, Any]:
    imgs = {
        "mli": _load_image(tex_dir / "mli_gold.png"),
        "metal": _load_image(tex_dir / "brushed_metal.png"),
        "solar": _load_image(tex_dir / "solar_cell.png"),
        "radiator": _load_image(tex_dir / "radiator_osr.png"),
    }
    pbr_foil = _load_pbr_set(pbr_dir, "Foil002")
    pbr_ring = _load_pbr_set(pbr_dir, "Metal049A")
    pbr_hw = _load_pbr_set(pbr_dir, "Metal032")
    pbr_solar = _load_pbr_set(pbr_dir, "SolarPanel002")

    named = {
        "mli": _build_kapton_mli_mat("mat_mli_client", imgs["mli"], (0.94, 0.67, 0.17), pbr_foil),
        "mli_servicer": _build_kapton_mli_mat("mat_mli_servicer", imgs["mli"], (0.88, 0.59, 0.14), pbr_foil),
        "metal": _build_adapter_ring_mat(imgs["metal"], (0.86, 0.88, 0.90), pbr_ring),
        "solar": _build_solar_mat(imgs["solar"], pbr_solar),
        "radiator": _build_radiator_mat(imgs["radiator"]),
        "dish": _build_simple_mat("mat_dish", (0.84, 0.85, 0.86, 1.0), metallic=0.05, roughness=0.58),
        "nozzle": _build_hardware_metal_mat("mat_nozzle", (0.14, 0.15, 0.17, 1.0), 0.90, 0.34, pbr_hw),
        # Role-specific wrist / arm / ring-tool aerospace hardware finishes
        "jaw_anodized": _build_hardware_metal_mat(
            "mat_jaw_anodized", (0.09, 0.10, 0.12, 1.0), 0.70, 0.46, pbr_hw, uv_scale=18.0, grain_scale=700.0, grain_strength=0.09, ao_dist=0.035, ao_factor=0.65
        ),
        "stainless_rail": _build_hardware_metal_mat(
            "mat_stainless_rail", (0.78, 0.80, 0.83, 1.0), 0.98, 0.17, pbr_ring, uv_scale=24.0, grain_scale=900.0, grain_strength=0.03, ao_dist=0.03, ao_factor=0.50
        ),
        "cam_black_anodized": _build_hardware_metal_mat(
            "mat_cam_black_anodized", (0.06, 0.065, 0.075, 1.0), 0.55, 0.50, pbr_hw, uv_scale=18.0, grain_scale=750.0, grain_strength=0.08, ao_dist=0.035, ao_factor=0.65
        ),
        "wrist_titanium": _build_hardware_metal_mat(
            "mat_wrist_titanium", (0.48, 0.50, 0.54, 1.0), 0.88, 0.34, pbr_hw, uv_scale=16.0, grain_scale=680.0, grain_strength=0.08, ao_dist=0.04, ao_factor=0.60
        ),
        "arm_carbon": _build_hardware_metal_mat(
            "mat_arm_carbon", (0.08, 0.09, 0.11, 1.0), 0.45, 0.34, pbr_hw, uv_scale=20.0, grain_scale=320.0, grain_strength=0.10, ao_dist=0.05, ao_factor=0.55
        ),
    }
    rgba_cache: Dict[Tuple[float, ...], bpy.types.Material] = {}

    def get_mat(geom: Dict[str, Any], body_name: str = "") -> bpy.types.Material:
        tag = geom.get("mat")
        if tag in named:
            return named[tag]
        rgba = tuple(round(float(x), 3) for x in geom["rgba"])
        sz = [float(x) for x in geom.get("size", [0.1, 0.1, 0.1])]

        if rgba[0] > 0.72 and rgba[1] > 0.50 and rgba[2] < 0.35:
            return named["mli_servicer"]
        if rgba[0] > 0.85 and rgba[1] > 0.85 and rgba[2] > 0.70:
            if "lamp_face" not in rgba_cache:
                rgba_cache["lamp_face"] = _build_simple_mat("mat_wrist_lamp_face", rgba, 0.0, 0.15, emission_strength=4.0)
            return rgba_cache["lamp_face"]
        if max(rgba[:3]) < 0.10:
            if "lens" not in rgba_cache:
                rgba_cache["lens"] = _build_simple_mat("mat_lens_optic", rgba, 0.15, 0.04)
            return rgba_cache["lens"]

        # Distinct component role shaders for wrist tool, gripper jaws & arm links
        if body_name in ("jaw_l", "jaw_u"):
            return named["jaw_anodized"]
        if body_name == "wrist3":
            if max(rgba[:3]) < 0.22:
                return named["cam_black_anodized"]
            if max(sz) < 0.032 and min(sz) < 0.005:
                return named["stainless_rail"]
            return named["wrist_titanium"]
        if body_name in ("wrist1", "wrist2"):
            return named["wrist_titanium"]
        if body_name in ("upper", "fore") and max(rgba[:3]) < 0.35:
            return named["arm_carbon"]

        if rgba in rgba_cache:
            return rgba_cache[rgba]
        lum = 0.3 * rgba[0] + 0.59 * rgba[1] + 0.11 * rgba[2]
        mat = _build_hardware_metal_mat(
            f"mat_rgba_{len(rgba_cache)}",
            rgba,
            metallic=0.86 if lum > 0.35 else 0.80,
            roughness=0.28 if lum > 0.45 else 0.36,
            pbr=pbr_hw,
        )
        rgba_cache[rgba] = mat
        return mat

    return {"get_mat": get_mat}


def _apply_auto_smooth(mesh: bpy.types.Mesh, angle_deg: float = 36.0, bevel_offset: float = 0.0) -> None:
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.normal_update()
    bm.edges.ensure_lookup_table()
    thresh = math.radians(angle_deg)

    # Geometric micro-chamfer on sharp CAD edges for Cycles OptiX specular edge highlights
    if bevel_offset > 0.0 and len(bm.faces) <= 4000:
        sharp_pre = [e for e in bm.edges if len(e.link_faces) == 2 and e.calc_face_angle(0.0) > thresh]
        if sharp_pre:
            try:
                bmesh.ops.bevel(
                    bm,
                    geom=sharp_pre,
                    offset=bevel_offset,
                    offset_type="OFFSET",
                    segments=2,
                    profile=0.5,
                    affect="EDGES",
                    clamp_overlap=True,
                )
                bm.normal_update()
            except Exception:
                pass

    # Metric triplanar UVs (1 UV unit = 1 m) after chamfer so even bevel strips map cleanly
    uv_layer = bm.loops.layers.uv.verify()
    for f in bm.faces:
        n = f.normal
        ax, ay, az = abs(n.x), abs(n.y), abs(n.z)
        for loop in f.loops:
            v = loop.vert.co
            if ax >= ay and ax >= az:
                loop[uv_layer].uv = (v.y, v.z)
            elif ay >= az:
                loop[uv_layer].uv = (v.x, v.z)
            else:
                loop[uv_layer].uv = (v.x, v.y)

    bm.edges.ensure_lookup_table()
    sharp = [e for e in bm.edges if len(e.link_faces) == 2 and e.calc_face_angle(0.0) > thresh]
    if sharp:
        bmesh.ops.split_edges(bm, edges=sharp)
    for f in bm.faces:
        f.smooth = True
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()


def _make_geom_mesh(geom: Dict[str, Any], meshes: Dict[str, Any], idx: int, bevel_offset: float = 0.0) -> bpy.types.Mesh:
    t = int(geom["t"])
    sz = [float(x) for x in geom["size"]]
    bm = bmesh.new()
    if t == 7:
        md = meshes[str(geom["mesh"])]
        verts = np.asarray(md["v"], dtype=float).reshape(-1, 3)
        faces = np.asarray(md["f"], dtype=int).reshape(-1, 3)
        me = bpy.data.meshes.new(f"mesh_g{idx}")
        me.from_pydata(verts.tolist(), [], faces.tolist())
        me.update()
        _apply_auto_smooth(me, 36.0, bevel_offset=bevel_offset)
        return me
    if t == 6:
        bmesh.ops.create_cube(bm, size=2.0)
        bmesh.ops.scale(bm, vec=(sz[0], sz[1], sz[2]), verts=bm.verts)
    elif t == 5:
        bmesh.ops.create_cone(
            bm, cap_ends=True, cap_tris=False, segments=32, radius1=sz[0], radius2=sz[0], depth=2.0 * sz[1]
        )
    elif t == 3:
        r, hl = sz[0], sz[1]
        bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=16, radius=r)
        for v in bm.verts:
            if v.co.z >= 0.0:
                v.co.z += hl
            else:
                v.co.z -= hl
    else:
        bmesh.ops.create_icosphere(bm, subdivisions=2, radius=max(sz[0], 0.01))

    me = bpy.data.meshes.new(f"geom_{idx}")
    bm.to_mesh(me)
    bm.free()
    _apply_auto_smooth(me, 36.0, bevel_offset=bevel_offset)
    return me


BEVEL_BODIES = {"chaser", "arm_base", "upper", "fore", "wrist1", "wrist2", "wrist3", "jaw_l", "jaw_u"}


def build_spacecraft(
    scene3d: Dict[str, Any],
    tex_dir: Path,
    pbr_dir: Optional[Path],
    coll: bpy.types.Collection,
) -> Dict[str, Any]:
    mat_api = build_materials(tex_dir, pbr_dir=pbr_dir)
    body_names = list(scene3d["bodies"])
    body_empties: List[bpy.types.Object] = []
    body_map: Dict[str, bpy.types.Object] = {}
    for bname in body_names:
        emp = bpy.data.objects.new(f"body_{bname}", None)
        emp.empty_display_size = 0.08
        emp.rotation_mode = "QUATERNION"
        coll.objects.link(emp)
        body_empties.append(emp)
        body_map[bname] = emp

    meshes = scene3d["meshes"]
    for idx, g in enumerate(scene3d["geoms"]):
        bname = body_names[int(g["b"])]
        bev = 0.0009 if (bname in BEVEL_BODIES or g.get("mat") == "metal") else 0.0
        me = _make_geom_mesh(g, meshes, idx, bevel_offset=bev)
        obj = bpy.data.objects.new(f"geom_{idx:03d}", me)
        obj.data.materials.append(mat_api["get_mat"](g, bname))
        obj.parent = body_empties[int(g["b"])]
        obj.location = tuple(float(x) for x in g["pos"])
        obj.rotation_mode = "QUATERNION"
        obj.rotation_quaternion = Quaternion(tuple(float(x) for x in g["quat"]))
        coll.objects.link(obj)

    return {"empties": body_empties, "by_name": body_map}


def build_orbital_environment(
    scene: bpy.types.Scene,
    tex_dir: Path,
    coll: bpy.types.Collection,
) -> Dict[str, bpy.types.Object]:
    # World: true black space + crisp camera-ray-only Voronoi stars (zero ambient bounce)
    world = bpy.data.worlds.new("OrbitalSpace")
    scene.world = world
    nt = _ensure_nodes(world)
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    out.location = (420, 0)
    bg = nt.nodes.new("ShaderNodeBackground")
    bg.location = (80, 90)
    bg_bounce = nt.nodes.new("ShaderNodeBackground")
    bg_bounce.location = (80, -90)
    bg_bounce.inputs["Color"].default_value = (0.016, 0.022, 0.034, 1.0)
    bg_bounce.inputs["Strength"].default_value = 1.0
    mix_w = nt.nodes.new("ShaderNodeMixShader")
    mix_w.location = (260, 0)
    nt.links.new(bg_bounce.outputs["Background"], mix_w.inputs[1])
    nt.links.new(bg.outputs["Background"], mix_w.inputs[2])
    nt.links.new(mix_w.outputs["Shader"], out.inputs["Surface"])

    lp = nt.nodes.new("ShaderNodeLightPath")
    lp.location = (-620, 220)
    nt.links.new(lp.outputs["Is Camera Ray"], mix_w.inputs["Fac"])
    tc = nt.nodes.new("ShaderNodeTexCoord")
    tc.location = (-840, -40)
    vor = nt.nodes.new("ShaderNodeTexVoronoi")
    vor.location = (-620, -40)
    vor.inputs["Scale"].default_value = 340.0
    cr = nt.nodes.new("ShaderNodeValToRGB")
    cr.location = (-360, -40)
    cr.color_ramp.elements[0].position = 0.0
    cr.color_ramp.elements[0].color = (1.0, 1.0, 1.0, 1.0)
    cr.color_ramp.elements[1].position = 0.085
    cr.color_ramp.elements[1].color = (0.0, 0.0, 0.0, 1.0)
    mul_str = nt.nodes.new("ShaderNodeMath")
    mul_str.operation = "MULTIPLY"
    mul_str.location = (-80, 90)
    mul_str.inputs[1].default_value = 2.6

    nt.links.new(tc.outputs["Generated"], vor.inputs["Vector"])
    nt.links.new(vor.outputs["Distance"], cr.inputs["Fac"])
    nt.links.new(cr.outputs["Color"], mul_str.inputs[0])
    nt.links.new(mul_str.outputs["Value"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 1.0

    # High-res Earth sphere (1381 m radius at 1500 m nadir, matching MjorbitBackend)
    earth_img_path = SIM_SRC / "viewer" / "assets" / "earth_day_5400.jpg"
    if not earth_img_path.is_file():
        earth_img_path = tex_dir / "earth_day.jpg"
    earth_img = _load_image(earth_img_path)

    bpy.ops.mesh.primitive_uv_sphere_add(segments=128, ring_count=64, radius=1381.0, calc_uvs=True)
    earth = bpy.context.active_object
    earth.name = "mjo_earth"
    for c_old in list(earth.users_collection):
        c_old.objects.unlink(earth)
    coll.objects.link(earth)
    for poly in earth.data.polygons:
        poly.use_smooth = True
    earth.visible_shadow = False

    emat, ent, ebsdf, _ = _new_mat("mat_earth")
    ebsdf.inputs["Roughness"].default_value = 0.88
    if "Specular IOR Level" in ebsdf.inputs:
        ebsdf.inputs["Specular IOR Level"].default_value = 0.30
    etc = ent.nodes.new("ShaderNodeTexCoord")
    if earth_img is not None:
        etx = ent.nodes.new("ShaderNodeTexImage")
        etx.image = earth_img
        etx.projection = "FLAT"
        ent.links.new(etc.outputs["UV"], etx.inputs["Vector"])

        # Lift dark deep-ocean texture subtly and overlay procedural orbital cloud wisps
        emp = ent.nodes.new("ShaderNodeMapping")
        emp.inputs["Scale"].default_value = (6.0, 14.0, 6.0)
        cnoi = ent.nodes.new("ShaderNodeTexNoise")
        cnoi.inputs["Scale"].default_value = 2.4
        cnoi.inputs["Detail"].default_value = 10.0
        cnoi.inputs["Roughness"].default_value = 0.62
        cnoi.inputs["Distortion"].default_value = 1.4
        cr_cld = ent.nodes.new("ShaderNodeValToRGB")
        cr_cld.color_ramp.elements[0].position = 0.49
        cr_cld.color_ramp.elements[0].color = (0.0, 0.0, 0.0, 1.0)
        cr_cld.color_ramp.elements[1].position = 0.72
        cr_cld.color_ramp.elements[1].color = (0.85, 0.85, 0.85, 1.0)
        mix_earth = ent.nodes.new("ShaderNodeMix")
        mix_earth.data_type = "RGBA"
        mix_earth.inputs[7].default_value = (0.92, 0.94, 0.98, 1.0)
        bc = ent.nodes.new("ShaderNodeBrightContrast")
        bc.inputs["Bright"].default_value = 0.08
        bc.inputs["Contrast"].default_value = 0.15

        ent.links.new(etc.outputs["Generated"], emp.inputs["Vector"])
        ent.links.new(emp.outputs["Vector"], cnoi.inputs["Vector"])
        ent.links.new(cnoi.outputs["Fac"], cr_cld.inputs["Fac"])
        ent.links.new(etx.outputs["Color"], bc.inputs["Color"])
        ent.links.new(cr_cld.outputs["Color"], mix_earth.inputs["Factor"])
        ent.links.new(bc.outputs["Color"], mix_earth.inputs[6])
        ent.links.new(mix_earth.outputs[2], ebsdf.inputs["Base Color"])
    else:
        ebsdf.inputs["Base Color"].default_value = (0.15, 0.35, 0.65, 1.0)
    earth.data.materials.append(emat)

    # Atmospheric limb shell (1398 m radius: soft 3-stop Fresnel halo modulated by day/night solar terminator)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=128, ring_count=64, radius=1398.0, calc_uvs=True)
    atmo = bpy.context.active_object
    atmo.name = "mjo_atmo"
    for c_old in list(atmo.users_collection):
        c_old.objects.unlink(atmo)
    coll.objects.link(atmo)
    for poly in atmo.data.polygons:
        poly.use_smooth = True
    atmo.visible_shadow = False

    amat = bpy.data.materials.new("mat_atmo")
    ant = _ensure_nodes(amat)
    ant.nodes.clear()
    aout = ant.nodes.new("ShaderNodeOutputMaterial")
    mixs = ant.nodes.new("ShaderNodeMixShader")
    trans = ant.nodes.new("ShaderNodeBsdfTransparent")
    emis = ant.nodes.new("ShaderNodeEmission")
    emis.inputs["Color"].default_value = (0.20, 0.52, 1.0, 1.0)
    emis.inputs["Strength"].default_value = 1.45

    lw = ant.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.55
    ramp = ant.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.65
    ramp.color_ramp.elements[0].color = (0.0, 0.0, 0.0, 1.0)
    mid_stop = ramp.color_ramp.elements.new(0.89)
    mid_stop.color = (0.58, 0.58, 0.58, 1.0)
    ramp.color_ramp.elements[2].position = 0.985
    ramp.color_ramp.elements[2].color = (0.0, 0.0, 0.0, 1.0)

    # Solar terminator attenuation so Earth's nightside limb stays dark
    geo = ant.nodes.new("ShaderNodeNewGeometry")
    sun_vec = ant.nodes.new("ShaderNodeCombineXYZ")
    sun_vec.name = "SunDirVec"
    dot = ant.nodes.new("ShaderNodeVectorMath")
    dot.operation = "DOT_PRODUCT"
    day_rng = ant.nodes.new("ShaderNodeMapRange")
    day_rng.inputs["From Min"].default_value = -0.14
    day_rng.inputs["From Max"].default_value = 0.35
    day_rng.inputs["To Min"].default_value = 0.02
    day_rng.inputs["To Max"].default_value = 1.0
    mul_atmo = ant.nodes.new("ShaderNodeMath")
    mul_atmo.operation = "MULTIPLY"

    ant.links.new(lw.outputs["Facing"], ramp.inputs["Fac"])
    ant.links.new(geo.outputs["Normal"], dot.inputs[0])
    ant.links.new(sun_vec.outputs["Vector"], dot.inputs[1])
    ant.links.new(dot.outputs["Value"], day_rng.inputs["Value"])
    ant.links.new(ramp.outputs["Color"], mul_atmo.inputs[0])
    ant.links.new(day_rng.outputs["Result"], mul_atmo.inputs[1])
    ant.links.new(mul_atmo.outputs["Value"], mixs.inputs["Fac"])
    ant.links.new(trans.outputs["BSDF"], mixs.inputs[1])
    ant.links.new(emis.outputs["Emission"], mixs.inputs[2])
    ant.links.new(mixs.outputs["Shader"], aout.inputs["Surface"])
    atmo.data.materials.append(amat)

    # Collimated solar disc light (0.53 deg apparent angular diameter)
    sun_data = bpy.data.lights.new("OrbitalSun", type="SUN")
    sun_data.energy = 4.5
    sun_data.angle = math.radians(0.53)
    sun_data.color = (1.0, 0.985, 0.96)
    sun_obj = bpy.data.objects.new("OrbitalSun", sun_data)
    coll.objects.link(sun_obj)

    # Soft blue Earthshine nadir fill light
    esh_data = bpy.data.lights.new("EarthshineFill", type="SUN")
    esh_data.energy = 0.25
    esh_data.angle = math.radians(55.0)
    esh_data.color = (0.45, 0.70, 1.0)
    esh_data.use_shadow = False
    esh_obj = bpy.data.objects.new("EarthshineFill", esh_data)
    coll.objects.link(esh_obj)

    return {"earth": earth, "atmo": atmo, "sun": sun_obj, "earthshine": esh_obj, "atmo_sun_node": sun_vec}


def setup_cameras(wrist3_obj: bpy.types.Object, coll: bpy.types.Collection) -> Dict[str, bpy.types.Object]:
    # 3rd-person orbital camera
    cam3_data = bpy.data.cameras.new("ThirdPersonCam")
    cam3_data.sensor_fit = "VERTICAL"
    cam3_data.angle_y = math.radians(42.0)
    cam3_data.clip_start = 0.02
    cam3_data.clip_end = 5000.0
    cam3_obj = bpy.data.objects.new("ThirdPersonCam", cam3_data)
    coll.objects.link(cam3_obj)

    # Exact wrist_cam mounted on wrist3 (servicer_rev2_scene.xml L384):
    # pos = (0.0870, -0.0550, -0.0450)
    # xyaxes = X_c (0, -1, 0), Y_c (0.1736, 0, 0.9848), -Z_c = +(X_c x Y_c)? Check:
    # MuJoCo camera looks along -Z_c where Z_c = X_c x Y_c:
    # X_c x Y_c = (0,-1,0) x (0.1736,0,0.9848) = (-0.9848, 0, 0.1736) -> -Z_c = (+0.9848, 0, -0.1736) (forward +X, 10 deg down!)
    wcam_data = bpy.data.cameras.new("WristCam")
    wcam_data.sensor_fit = "VERTICAL"
    wcam_data.angle_y = math.radians(75.0)
    wcam_data.clip_start = 0.01
    wcam_data.clip_end = 5000.0
    wcam_obj = bpy.data.objects.new("WristCam", wcam_data)
    wcam_obj.parent = wrist3_obj
    wcam_obj.location = (0.0870, -0.0550, -0.0450)
    xc = Vector((0.0, -1.0, 0.0))
    yc = Vector((0.1736, 0.0, 0.9848)).normalized()
    zc = xc.cross(yc).normalized()
    rot_m = Matrix((xc, yc, zc)).transposed()  # columns = local X, Y, Z axes in wrist3 frame
    wcam_obj.rotation_mode = "QUATERNION"
    wcam_obj.rotation_quaternion = rot_m.to_quaternion()
    coll.objects.link(wcam_obj)

    # Active wrist LED spot lamp (servicer_rev2_scene.xml: pos=(0.088,-0.055,-0.020), dir=(0.9848,0,-0.1736))
    wl_data = bpy.data.lights.new("WristLamp", type="SPOT")
    wl_data.energy = 14.0
    wl_data.spot_size = math.radians(88.0)
    wl_data.spot_blend = 0.35
    wl_data.color = (0.96, 0.95, 0.88)
    wl_obj = bpy.data.objects.new("WristLamp", wl_data)
    wl_obj.parent = wrist3_obj
    wl_obj.location = (0.088, -0.055, -0.020)
    wl_dir = Vector((0.9848, 0.0, -0.1736)).normalized()
    wl_obj.rotation_mode = "QUATERNION"
    wl_obj.rotation_quaternion = wl_dir.to_track_quat("-Z", "Y")
    coll.objects.link(wl_obj)

    return {"third": cam3_obj, "wrist": wcam_obj, "wrist_lamp": wl_obj}


def set_frame_state(
    frame: Dict[str, Any],
    bodies: Dict[str, Any],
    env: Dict[str, Any],
    cams: Dict[str, Any],
) -> None:
    pose = frame["pose"]
    for emp, bp in zip(bodies["empties"], pose["bodies"]):
        emp.location = (float(bp[0]), float(bp[1]), float(bp[2]))
        emp.rotation_quaternion = Quaternion((float(bp[3]), float(bp[4]), float(bp[5]), float(bp[6])))

    p_s = Vector(pose["chaser"][:3])
    p_t = Vector(pose["client"][:3])
    mid = 0.5 * (p_s + p_t)
    r_hat = Vector(pose["r_hat"]).normalized()
    sun_dir = Vector(pose["sun"]).normalized()

    # Keep Earth centered relative to spacecraft midpoint and orient oceans/continents toward sunlit hemisphere
    earth_pos = mid - 1500.0 * r_hat
    env["earth"].location = earth_pos
    env["atmo"].location = earth_pos
    env["earth"].rotation_euler = (0.35, 1.15, 0.95)

    # Sunlight direction (rays propagate along -sun_dir)
    ecl = float(frame.get("eclipse", 1.0) or 1.0)
    env["sun"].data.energy = 4.2 * ecl
    env["sun"].rotation_mode = "QUATERNION"
    env["sun"].rotation_quaternion = (-sun_dir).to_track_quat("-Z", "Y")
    sun_node = env.get("atmo_sun_node")
    if sun_node is not None:
        sun_node.inputs["X"].default_value = sun_dir.x * ecl
        sun_node.inputs["Y"].default_value = sun_dir.y * ecl
        sun_node.inputs["Z"].default_value = sun_dir.z * ecl

    # Earthshine nadir fill (stronger over sunlit Earth hemisphere)
    dayside = max(0.0, float(r_hat.dot(sun_dir)) + 0.15)
    env["earthshine"].data.energy = 0.18 + 0.36 * min(dayside, 1.0)
    env["earthshine"].rotation_mode = "QUATERNION"
    env["earthshine"].rotation_quaternion = r_hat.to_track_quat("-Z", "Y")

    # Wrist lamp active at close range < 3.5 m (calibrated close-range LED brightness)
    range_m = float(frame.get("range_m", 10.0))
    cams["wrist_lamp"].data.energy = 1.8 if range_m < 3.5 else 0.0

    # Oblique third-person orbital camera showing spacecraft + curving Earth horizon
    d_vec = p_s - p_t
    dist = max(float(d_vec.length), 1e-6)
    dir_sc = d_vec / dist
    close_blend = max(0.0, min(1.0, (4.5 - dist) / 3.0))
    lookat = (1.0 - 0.65 * close_blend) * mid + (0.65 * close_blend) * (p_t + 0.5 * d_vec)
    side = r_hat.cross(dir_sc)
    if side.length < 1e-4:
        ref = Vector((1.0, 0.0, 0.0)) if abs(dir_sc.x) < 0.9 else Vector((0.0, 1.0, 0.0))
        side = ref.cross(dir_sc)
    side.normalize()
    eye_dir = (0.78 * side + 0.46 * dir_sc + 0.42 * r_hat).normalized()
    cam_dist = max(2.7, min(44.0, 1.38 * dist + 1.6))
    eye = lookat + cam_dist * eye_dir

    cam3 = cams["third"]
    cam3.location = eye
    fwd = (lookat - eye).normalized()
    cam3.rotation_mode = "QUATERNION"
    cam3.rotation_quaternion = fwd.to_track_quat("-Z", "Y")


def _query_gpu_temp() -> Optional[int]:
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if res.returncode == 0 and res.stdout.strip():
            return int(res.stdout.strip().splitlines()[0].strip())
    except Exception:
        pass
    return None


def _thermal_guard(max_temp: int, cool_temp: int) -> None:
    temp = _query_gpu_temp()
    if temp is None or temp < max_temp:
        return
    print(f"THERMAL_GUARD pause: GPU {temp}C >= {max_temp}C, cooling to <= {cool_temp}C", flush=True)
    for _ in range(24):
        time.sleep(5)
        cur = _query_gpu_temp()
        if cur is None or cur <= cool_temp:
            print(f"THERMAL_GUARD resume: GPU {cur}C", flush=True)
            return
    print("THERMAL_GUARD timeout reached, resuming", flush=True)


def interpolate_frame(frames: List[Dict[str, Any]], t_query: float, ts: Optional[List[float]] = None) -> Dict[str, Any]:
    """Interpolate spacecraft poses and orbital lighting at time t_query using shortest-arc SLERP."""
    if not frames:
        raise ValueError("Manifest contains no frames")
    if len(frames) == 1 or t_query <= float(frames[0]["t"]):
        return frames[0]
    if t_query >= float(frames[-1]["t"]):
        return frames[-1]

    times = ts if ts is not None else [float(fr["t"]) for fr in frames]
    idx = max(0, min(len(frames) - 2, bisect.bisect_right(times, t_query) - 1))
    f0, f1 = frames[idx], frames[idx + 1]
    t0, t1 = float(f0["t"]), float(f1["t"])
    alpha = max(0.0, min(1.0, (t_query - t0) / max(t1 - t0, 1e-9)))
    inv = 1.0 - alpha

    p0, p1 = f0["pose"], f1["pose"]
    bodies_interp: List[List[float]] = []
    for bp0, bp1 in zip(p0["bodies"], p1["bodies"]):
        px = inv * float(bp0[0]) + alpha * float(bp1[0])
        py = inv * float(bp0[1]) + alpha * float(bp1[1])
        pz = inv * float(bp0[2]) + alpha * float(bp1[2])
        q0 = Quaternion((float(bp0[3]), float(bp0[4]), float(bp0[5]), float(bp0[6]))).normalized()
        q1 = Quaternion((float(bp1[3]), float(bp1[4]), float(bp1[5]), float(bp1[6]))).normalized()
        if q0.dot(q1) < 0.0:
            q1 = -q1
        qm = q0.slerp(q1, alpha)
        bodies_interp.append([px, py, pz, qm.w, qm.x, qm.y, qm.z])

    ch0, ch1 = Vector(p0["chaser"][:3]), Vector(p1["chaser"][:3])
    cl0, cl1 = Vector(p0["client"][:3]), Vector(p1["client"][:3])
    r_hat = Vector(p0["r_hat"]).lerp(Vector(p1["r_hat"]), alpha).normalized()
    sun = Vector(p0["sun"]).lerp(Vector(p1["sun"]), alpha).normalized()
    ecl = inv * float(f0.get("eclipse", 1.0) or 1.0) + alpha * float(f1.get("eclipse", 1.0) or 1.0)
    rng = inv * float(f0.get("range_m", 10.0)) + alpha * float(f1.get("range_m", 10.0))

    return {
        "t": t_query,
        "phase": f0["phase"],
        "range_m": rng,
        "eclipse": ecl,
        "pose": {
            "bodies": bodies_interp,
            "chaser": list(ch0.lerp(ch1, alpha)),
            "client": list(cl0.lerp(cl1, alpha)),
            "r_hat": list(r_hat),
            "sun": list(sun),
        },
    }


def main() -> None:
    args = parse_args()
    replay_dir = Path(args.replay).resolve()
    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    scene3d = json.loads((replay_dir / "scene3d.json").read_text(encoding="utf-8"))
    manifest = json.loads((replay_dir / "manifest.json").read_text(encoding="utf-8"))
    frames = manifest["frames"]

    clear_scene()
    scene = bpy.context.scene
    configure_cycles(scene, args.samples)
    if not args.no_optics:
        setup_optical_compositor(scene)
    coll = bpy.data.collections.new("mjorbit_P7")
    scene.collection.children.link(coll)

    pbr_dir = Path(args.pbr_dir).resolve() if args.pbr_dir else (replay_dir.parent / "pbr_assets")
    bodies = build_spacecraft(scene3d, replay_dir / "textures", pbr_dir if pbr_dir.is_dir() else None, coll)
    env = build_orbital_environment(scene, replay_dir / "textures", coll)
    cams = setup_cameras(bodies["by_name"]["wrist3"], coll)

    render_ids = [int(x.strip()) for x in args.frames.split(",") if x.strip()]
    for fi in render_ids:
        fr = frames[fi]
        set_frame_state(fr, bodies, env, cams)
        scene.camera = cams["third"]
        scene.render.resolution_x = args.width
        scene.render.resolution_y = args.height
        out_png = out_dir / f"frame_{fi:04d}_third.png"
        scene.render.filepath = str(out_png)
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {out_png.name} (phase={fr['phase']}, range={fr['range_m']}m)", flush=True)

    if args.wrist_frame >= 0:
        fi = int(args.wrist_frame)
        fr = frames[fi]
        set_frame_state(fr, bodies, env, cams)
        scene.camera = cams["wrist"]
        scene.render.resolution_x = args.wrist_res
        scene.render.resolution_y = args.wrist_res
        out_png = out_dir / f"frame_{fi:04d}_wrist.png"
        scene.render.filepath = str(out_png)
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {out_png.name} (wrist_cam @ phase={fr['phase']}, range={fr['range_m']}m)", flush=True)

        if args.closeup_wrist:
            bpy.context.view_layer.update()
            wpos = bodies["by_name"]["wrist3"].matrix_world.translation.copy()
            cam3 = cams["third"]
            cam3.location = wpos + Vector((0.28, -0.34, 0.22))
            cam3.rotation_mode = "QUATERNION"
            cam3.rotation_quaternion = (wpos - cam3.location).normalized().to_track_quat("-Z", "Y")
            scene.camera = cam3
            scene.render.resolution_x = 640
            scene.render.resolution_y = 480
            out_cu = out_dir / f"frame_{fi:04d}_wrist_closeup.png"
            scene.render.filepath = str(out_cu)
            bpy.ops.render.render(write_still=True)
            print(f"RENDERED {out_cu.name} (wrist closeup)", flush=True)
            set_frame_state(fr, bodies, env, cams)

    if args.seq_start_t >= 0.0 and args.seq_end_t > args.seq_start_t:
        frames_dir = out_dir / "frames"
        frames_dir.mkdir(parents=True, exist_ok=True)
        ts = [float(fr["t"]) for fr in frames]
        n_seq = int(round((args.seq_end_t - args.seq_start_t) * args.seq_fps)) + 1
        scene.camera = cams["third"]
        scene.render.resolution_x = args.width
        scene.render.resolution_y = args.height
        rendered_cnt = 0
        for k in range(n_seq):
            out_png = frames_dir / f"seq_{k:04d}.png"
            if out_png.is_file():
                continue
            t_q = min(args.seq_end_t, args.seq_start_t + k / args.seq_fps)
            fr_interp = interpolate_frame(frames, t_q, ts=ts)
            set_frame_state(fr_interp, bodies, env, cams)
            scene.render.filepath = str(out_png)
            bpy.ops.render.render(write_still=True)
            rendered_cnt += 1
            if k == 0 or (k + 1) % 25 == 0 or k == n_seq - 1:
                temp = _query_gpu_temp()
                t_str = f"{temp}C" if temp is not None else "N/A"
                print(
                    f"SEQ {k + 1}/{n_seq} t={t_q:.2f}s phase={fr_interp['phase']} gpu={t_str}",
                    flush=True,
                )
            if args.frame_sleep > 0.0:
                time.sleep(args.frame_sleep)
            if rendered_cnt % 2 == 0:
                _thermal_guard(args.max_gpu_temp, args.cool_gpu_temp)

    if args.save_blend:
        scene.camera = cams["third"]
        scene.render.resolution_x = args.width
        scene.render.resolution_y = args.height
        blend_path = out_dir / "mjorbit_p7_proto.blend"
        bpy.ops.file.pack_all()
        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
        print(f"SAVED {blend_path}", flush=True)


if __name__ == "__main__":
    main()
