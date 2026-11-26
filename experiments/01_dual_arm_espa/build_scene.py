"""Build MuJoCo dual-arm 15" ESPA v4.2 scene XML (experiments/01_dual_arm_espa/assets/dual_arm_espa_scene.xml).

Run via WSL pixi:
  bash scripts/run.sh python experiments/01_dual_arm_espa/build_scene.py
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

EXP_DIR = Path(__file__).resolve().parent
ASSETS = EXP_DIR / "assets"
PARTS = ASSETS / "parts"
OUT_XML = ASSETS / "dual_arm_espa_scene.xml"
BUS_JSON = ASSETS / "dual_arm_espa_bus.json"

REPO_ROOT = EXP_DIR.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
import _paths  # noqa: E402,F401
from _paths import ASSETS_DIR  # noqa: E402
from servicer.make_scene_rev2 import fmt, stl  # noqa: E402


def _extract_client_p7_from_rev2():
    """Extract client_p7 `<body name="target">` (incl. 144-seg ring) and its inline meshes from servicer_rev2_scene.xml."""
    rev2_xml = ASSETS_DIR / "servicer_rev2" / "servicer_rev2_scene.xml"
    root = ET.parse(rev2_xml).getroot()
    target_body = next(b for b in root.find("worldbody").findall("body") if b.get("name") == "target")
    needed_meshes = {g.get("mesh") for g in target_body.iter("geom") if g.get("mesh")}
    meshes = [
        "    " + ET.tostring(m, encoding="unicode").strip()
        for m in root.find("asset").findall("mesh")
        if m.get("name") in needed_meshes
    ]
    c_txt = "    " + ET.tostring(target_body, encoding="unicode").strip() + "\n"
    return c_txt, meshes


def _build_arm_xml(k: int, pos_str: str, quat_str: str) -> str:
    """Build one symmetric 6-DOF arm (q0..q5) with intersecting Cardan-Roll wrist + ATI Mini45 F/T + v3-inspired tool."""
    p = f"a{k}"
    # Gripper shifted +0.054 m past PALM_X = 0.074 -> PALM_X = 0.128 m (preserving verified v4.1 jaw/palm/wedge offsets)
    # Passive wedge pad (20 deg undercut behind 15 mm lip at x=0.143, z=+0.011..+0.023)
    q_wedge = fmt([math.cos(math.radians(-10.0)), 0.0, math.sin(math.radians(-10.0)), 0.0])
    return f"""
      <!-- Dual-Arm v4.2 Arm {k} (248 + 208 mm, PALM_X = 128 mm, 3-DOF CSD-14/TBM2G Cardan-Roll wrist + Mini45 F/T + Marman jaws) -->
      <body name="{p}_base_roll" pos="{pos_str}" quat="{quat_str}">
        <inertial pos="-0.02 0 0" mass="1.40" diaginertia="0.0025 0.0020 0.0020"/>
        <joint name="{p}_q0" type="hinge" axis="1 0 0" range="-1.5708 1.5708" armature="0.35" damping="1.5" frictionloss="0.15"/>
        <geom name="{p}_turntable" class="arm_col" type="cylinder" size="0.038 0.016" pos="-0.068 0 0" zaxis="1 0 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_ear_p" class="arm_col" type="box" size="0.025 0.00425 0.021" pos="-0.025 0.02775 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_ear_p_ring" class="arm_col" type="cylinder" size="0.0415 0.00425" pos="0 0.02775 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_ear_n" class="arm_col" type="box" size="0.025 0.00425 0.021" pos="-0.025 -0.02775 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_ear_n_boss" class="arm_col" type="cylinder" size="0.026 0.0055" pos="0 -0.029 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_sh_stator" class="arm_col" type="cylinder" size="0.037 0.014" pos="0 0.0265 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_sh_cap" class="arm_col" type="cylinder" size="0.0315 0.0035" pos="0 0.0440 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
        <geom name="{p}_sh_gland" class="arm_col" type="cylinder" size="0.014 0.001" pos="0 0.0485 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
        <body name="{p}_upper" pos="0 0 0">
          <inertial pos="0.124 0 0" mass="1.65" diaginertia="0.0011 0.0100 0.0100"/>
          <joint name="{p}_q1" type="hinge" axis="0 -1 0" range="-2.4 2.4" armature="0.30" damping="1.2" frictionloss="0.12"/>
          <geom name="{p}_sh_collar" class="arm_col" type="cylinder" size="0.0378 0.00135" pos="0 0.01085 0" zaxis="0 1 0" rgba="0.94 0.52 0.10 1"/>
          <geom name="{p}_sh_rotor" class="arm_col" type="cylinder" size="0.035 0.0161" pos="0 -0.0069 0" zaxis="0 1 0" rgba="0.74 0.76 0.80 1"/>
          <geom name="{p}_sh_cheek" class="arm_col" type="box" size="0.022 0.0126 0.020" pos="0.022 -0.0034 0" rgba="0.74 0.76 0.80 1"/>
          <geom name="{p}_sh_sock" class="arm_col" type="cylinder" size="0.0225 0.00775" pos="0.04625 0 0" zaxis="1 0 0" rgba="0.74 0.76 0.80 1"/>
          <geom name="{p}_upper_tube" class="arm_col" type="capsule" fromto="0.0542 0 0 0.1958 0 0" size="0.018" rgba="0.58 0.62 0.68 1"/>
          <geom name="{p}_el_sock_in" class="arm_col" type="cylinder" size="0.022 0.009" pos="0.205 0 0" zaxis="1 0 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_trans" class="arm_col" type="box" size="0.004 0.027 0.018" pos="0.210 0 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_ear_p" class="arm_col" type="box" size="0.018 0.00375 0.018" pos="0.230 0.02325 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_ear_p_ring" class="arm_col" type="cylinder" size="0.0365 0.00375" pos="0.248 0.02325 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_ear_n" class="arm_col" type="box" size="0.018 0.00375 0.018" pos="0.230 -0.02325 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_ear_n_boss" class="arm_col" type="cylinder" size="0.022 0.005" pos="0.248 -0.0245 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_stator" class="arm_col" type="cylinder" size="0.0325 0.0125" pos="0.248 0.025 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_cap" class="arm_col" type="cylinder" size="0.027 0.00325" pos="0.248 0.04075 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
          <geom name="{p}_el_gland" class="arm_col" type="cylinder" size="0.012 0.001" pos="0.248 0.045 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
          <body name="{p}_fore" pos="0.248 0 0">
            <inertial pos="0.095 0 0" mass="1.25" diaginertia="0.0008 0.0055 0.0055"/>
            <joint name="{p}_q2" type="hinge" axis="0 -1 0" range="-2.8 2.8" armature="0.20" damping="0.8" frictionloss="0.08"/>
            <geom name="{p}_el_collar" class="arm_col" type="cylinder" size="0.0332 0.00135" pos="0 0.01085 0" zaxis="0 1 0" rgba="0.94 0.52 0.10 1"/>
            <geom name="{p}_el_rotor" class="arm_col" type="cylinder" size="0.0305 0.0141" pos="0 -0.0049 0" zaxis="0 1 0" rgba="0.74 0.76 0.80 1"/>
            <geom name="{p}_el_cheek" class="arm_col" type="box" size="0.020 0.0111 0.017" pos="0.020 -0.0019 0" rgba="0.74 0.76 0.80 1"/>
            <geom name="{p}_el_sock" class="arm_col" type="cylinder" size="0.019 0.007" pos="0.041 0 0" zaxis="1 0 0" rgba="0.74 0.76 0.80 1"/>
            <geom name="{p}_fore_tube" class="arm_col" type="capsule" fromto="0.0482 0 0 0.1738 0 0" size="0.015" rgba="0.58 0.62 0.68 1"/>
            <geom name="{p}_fore_clevis" class="arm_col" type="box" size="0.0027 0.028 0.014" pos="0.1767 0 0" rgba="0.34 0.36 0.40 1"/>
            <geom name="{p}_fore_ear" class="arm_col" type="cylinder" size="0.014 0.0029" pos="0.208 -0.0211 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
            <body name="{p}_wrist_pitch" pos="0.208 0 0">
              <inertial pos="0 0.028 0" mass="0.62" diaginertia="0.0005 0.0005 0.0005"/>
              <joint name="{p}_q3" type="hinge" axis="0 -1 0" range="-2.8 2.8" armature="0.06" damping="0.35"/>
              <geom name="{p}_wr_pitch_drum" class="arm_col" type="cylinder" size="0.0285 0.019" pos="0 0.037 0" zaxis="0 1 0" rgba="0.34 0.36 0.40 1"/>
              <body name="{p}_wrist_yaw" pos="0 0 0">
                <inertial pos="0 0 -0.035" mass="0.60" diaginertia="0.0005 0.0005 0.0005"/>
                <joint name="{p}_q4" type="hinge" axis="0 0 1" range="-1.5708 1.5708" armature="0.05" damping="0.30"/>
                <geom name="{p}_wr_yoke" class="arm_col" type="box" size="0.013 0.017 0.0195" pos="0 0 -0.0095" rgba="0.74 0.76 0.80 1"/>
                <geom name="{p}_wr_yaw_drum" class="arm_col" type="cylinder" size="0.0285 0.019" pos="0 0 -0.049" zaxis="0 0 1" rgba="0.34 0.36 0.40 1"/>
                <body name="{p}_wrist_roll" pos="0 0 0">
                  <inertial pos="0.088 0 0" mass="1.93" diaginertia="0.0018 0.0022 0.0018"/>
                  <joint name="{p}_q5" type="hinge" axis="1 0 0" range="-3.1416 3.1416" armature="0.04" damping="0.25"/>
                  <geom name="{p}_wr_yaw_bridge" class="arm_col" type="box" size="0.0039 0.014 0.029" pos="0.0259 0 -0.015" rgba="0.34 0.36 0.40 1"/>
                  <geom name="{p}_wr_roll_drum" class="arm_col" type="cylinder" size="0.0285 0.016" pos="0.046 0 0" zaxis="1 0 0" rgba="0.34 0.36 0.40 1"/>
                  <geom name="{p}_wr_roll_collar" class="arm_col" type="cylinder" size="0.0298 0.003" pos="0.057 0 0" zaxis="1 0 0" rgba="0.94 0.52 0.10 1"/>
                  <geom name="{p}_ft_wafer" class="arm_col" type="cylinder" size="0.0225 0.00785" pos="0.070 0 0" zaxis="1 0 0" rgba="0.22 0.64 0.60 1"/>
                  <site name="{p}_wrist_ft" pos="0.070 0 0" size="0.008" rgba="0.2 0.45 0.75 0.4" group="4"/>
                  <geom name="{p}_housing" class="tool" type="box" size="0.006 0.026 0.042" pos="0.084 0 0.002" rgba="0.74 0.76 0.80 1"/>
                  <geom name="{p}_neck" class="tool" type="box" size="0.013 0.022 0.014" pos="0.103 0 0" rgba="0.74 0.76 0.80 1"/>
                  <geom name="{p}_palm" class="tool" type="box" size="0.006 0.026 0.018" pos="0.122 0 0.007" rgba="0.74 0.76 0.80 1"/>
                  <site name="{p}_grip_site" pos="0.128 0 0" size="0.010" rgba="0.1 0.9 0.2 1"/>
                  <site name="{p}_touch_palm" type="box" pos="0.1285 0 0.007" size="0.0020 0.026 0.018" rgba="0.2 0.8 0.3 0.35" group="4"/>
                  <!-- Stereo camera bar on -Y_mj tangential face (clears q3 drum on +Y_mj) -->
                  <geom name="{p}_cam_unit" class="arm_col" type="box" size="0.011 0.011 0.031" pos="0.089 -0.037 0" rgba="0.10 0.11 0.14 1"/>
                  <camera name="{p}_cam1" mode="fixed" pos="0.100 -0.037 -0.025" xyaxes="0.0000 -1.0000 0.0000 0.1736 0.0000 0.9848" fovy="75"/>
                  <camera name="{p}_cam2" mode="fixed" pos="0.100 -0.037 0.025" xyaxes="0.0000 -1.0000 0.0000 0.1736 0.0000 0.9848" fovy="75"/>
                  <!-- Inboard sliding jaw: closes +Z (from z=-0.012 to contact at q_l=+0.008 -> body offset -0.008) -->
                  <body name="{p}_jaw_l" pos="0 0 -0.008">
                    <inertial pos="0.129 0 -0.020" mass="0.38" diaginertia="0.0002 0.0002 0.0002"/>
                    <joint name="{p}_grip_l" type="slide" axis="0 0 1" range="-0.03 0.03" damping="5"/>
                    <geom name="{p}_car_l" class="tool" type="box" size="0.010 0.022 0.0095" pos="0.105 0 -0.0285" rgba="0.88 0.42 0.12 1"/>
                    <geom name="{p}_pad_l" class="tool" type="box" size="0.022 0.022 0.005" pos="0.146 0 -0.016" rgba="0.88 0.42 0.12 1"/>
                    <site name="{p}_touch_in" type="box" pos="0.146 0 -0.011" size="0.022 0.022 0.0025" rgba="0.2 0.8 0.3 0.35" group="4"/>
                  </body>
                  <!-- Outboard sliding hook jaw: closes -Z (contact at q_u=-0.008 -> body offset +0.008) -->
                  <body name="{p}_jaw_u" pos="0 0 0.008">
                    <inertial pos="0.132 0 0.026" mass="0.42" diaginertia="0.0002 0.0002 0.0002"/>
                    <joint name="{p}_grip_u" type="slide" axis="0 0 1" range="-0.03 0.03" damping="5"/>
                    <geom name="{p}_car_u" class="tool" type="box" size="0.010 0.022 0.010" pos="0.105 0 0.030" rgba="0.88 0.42 0.12 1"/>
                    <geom name="{p}_hook_finger" class="tool" type="box" size="0.023 0.022 0.005" pos="0.147 0 0.034" rgba="0.88 0.42 0.12 1"/>
                    <geom name="{p}_pad_u" class="tool" type="box" size="0.0035 0.022 0.0092" pos="0.1525 0 0.0203" rgba="0.88 0.42 0.12 1"/>
                    <geom name="{p}_pad_w" class="tool" type="box" size="0.0020 0.022 0.0072" pos="0.1446 0 0.0225" quat="{q_wedge}" rgba="0.94 0.52 0.10 1"/>
                    <site name="{p}_touch_hook" type="box" pos="0.1525 0 0.0115" size="0.0035 0.022 0.0025" rgba="0.2 0.8 0.3 0.35" group="4"/>
                    <site name="{p}_touch_lip" type="box" pos="0.1435 0 0.0220" size="0.006 0.022 0.007" rgba="0.2 0.8 0.3 0.35" group="4"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>"""


def build_scene() -> Path:
    bus = json.loads(BUS_JSON.read_text(encoding="utf-8"))
    I = np.array(bus["inertia_kgm2"], dtype=float)

    # 1. Visual meshes for dual-arm bus + client_p7 meshes
    mesh_lines = []
    bus_vis_geoms = []
    for p in bus["parts"]:
        v, f = stl(PARTS / f'{p["name"]}.stl')
        mesh_lines.append(f'    <mesh name="da_{p["name"]}" vertex="{fmt(v, 6)}" face="{fmt(f, 0)}"/>')
        bus_vis_geoms.append(
            f'      <geom name="da_vis_{p["name"]}" class="vis" type="mesh" mesh="da_{p["name"]}" '
            f'contype="0" conaffinity="0" mass="0" rgba="{fmt(p["rgba"])}"/>'
        )

    c_txt, c_meshes = _extract_client_p7_from_rev2()
    mesh_lines.extend(c_meshes)

    bus_col_geoms = []
    for cb in bus["collision_boxes"]:
        nm = "servicer_bus" if cb["name"] == "bus_col" else f'da_col_{cb["name"]}'
        bus_col_geoms.append(
            f'      <geom name="{nm}" class="bus_col" type="box" pos="{fmt(cb["pos"])}" '
            f'size="{fmt(cb["size"])}" rgba="0.85 0.7 0.2 0"/>'
        )

    sites = []
    for n in bus["nozzles"]:
        sites.append(
            f'      <site name="{n["name"]}" pos="{fmt(n["pos_m"])}" zaxis="{fmt(n["force_dir"])}" '
            f'size="0.008" rgba="1 0.3 0.1 0.6" group="4"/>'
        )
    for w in bus["wheels"]:
        sites.append(
            f'      <site name="rw_{w["name"][-1]}" pos="{fmt(w["pos_m"])}" zaxis="{fmt(w["axis"])}" '
            f'size="0.01" rgba="0.2 0.6 1 0.6" group="4"/>'
        )

    # Sensors & Actuators & Equalities
    act_lines = []
    sensor_lines = []
    eq_lines = []
    for k in (1, 2):
        p = f"a{k}"
        act_lines.extend([
            f'    <motor name="{p}_tau0" joint="{p}_q0" ctrlrange="-60 60" ctrllimited="true" gear="1"/>',
            f'    <motor name="{p}_tau1" joint="{p}_q1" ctrlrange="-50 50" ctrllimited="true" gear="1"/>',
            f'    <motor name="{p}_tau2" joint="{p}_q2" ctrlrange="-35 35" ctrllimited="true" gear="1"/>',
            f'    <motor name="{p}_tau3" joint="{p}_q3" ctrlrange="-25 25" ctrllimited="true" gear="1"/>',
            f'    <motor name="{p}_tau4" joint="{p}_q4" ctrlrange="-20 20" ctrllimited="true" gear="1"/>',
            f'    <motor name="{p}_tau5" joint="{p}_q5" ctrlrange="-18 18" ctrllimited="true" gear="1"/>',
            f'    <position name="{p}_grip_u_pos" joint="{p}_grip_u" kp="30000" kv="1200" forcerange="-300 300" ctrlrange="-0.03 0.03"/>',
            f'    <position name="{p}_grip_l_pos" joint="{p}_grip_l" kp="10000" kv="400" forcerange="-150 150" ctrlrange="-0.03 0.03"/>',
        ])
        sensor_lines.extend([
            f'    <touch name="touch_{p}_palm" site="{p}_touch_palm"/>',
            f'    <touch name="touch_{p}_in" site="{p}_touch_in"/>',
            f'    <touch name="touch_{p}_hook" site="{p}_touch_hook"/>',
            f'    <touch name="touch_{p}_lip" site="{p}_touch_lip"/>',
            f'    <force name="{p}_wrist_force" site="{p}_wrist_ft"/>',
            f'    <torque name="{p}_wrist_torque" site="{p}_wrist_ft"/>',
            f'    <jointpos name="{p}_jaw_u_pos" joint="{p}_grip_u"/>',
            f'    <jointpos name="{p}_jaw_l_pos" joint="{p}_grip_l"/>',
        ])
        eq_lines.extend([
            f'    <joint name="{p}_lock_u" joint1="{p}_grip_u" polycoef="0 0 0 0 0" active="false" solref="0.004 1"/>',
            f'    <joint name="{p}_lock_l" joint1="{p}_grip_l" polycoef="0 0 0 0 0" active="false" solref="0.004 1"/>',
        ])

    for n in bus["nozzles"]:
        act_lines.append(
            f'    <motor name="test_rcs_{n["name"][4:]}" site="{n["name"]}" gear="0 0 1 0 0 0" '
            f'ctrllimited="true" ctrlrange="0 {1.2 * bus["thrust_n"]:g}"/>'
        )
    for ax, g in (("x", "1 0 0"), ("y", "0 1 0"), ("z", "0 0 1")):
        act_lines.append(
            f'    <motor name="test_rwm_{ax}" joint="chaser_joint" gear="0 0 0 {g}" ctrllimited="true" ctrlrange="-2 2"/>'
        )

    arm1_xml = _build_arm_xml(1, "0.265 0.070 0.245", "1 0 0 0")
    arm2_xml = _build_arm_xml(2, "0.265 -0.070 -0.245", "0 1 0 0")

    xml = f"""<!--
  GENERATED by experiments/01_dual_arm_espa/build_scene.py (do not edit manually).
  v4.2 Dual-Arm Recessed-Cavity 15" ESPA Servicer + client_p7 (144-segment 610 mm ring).
-->
<mujoco model="dual_arm_espa_v4_2">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="0.002" gravity="0 0 0" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="0">
    <flag contact="enable"/>
  </option>

  <default>
    <geom condim="3" friction="0.5 0.005 0.0001"/>
    <default class="bus_col">
      <geom contype="2" conaffinity="4" condim="3" friction="0.5 0.005 0.0001" solref="0.008 1" solimp="0.95 0.99 0.001"/>
    </default>
    <default class="arm_col">
      <geom contype="2" conaffinity="4" condim="3" friction="0.4 0.01 0.0005" solref="0.006 1" solimp="0.95 0.99 0.001"/>
    </default>
    <default class="tool">
      <geom contype="2" conaffinity="4" condim="4" friction="0.4 0.02 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/>
    </default>
    <default class="vis">
      <geom contype="0" conaffinity="0" group="1"/>
    </default>
    <default class="client_vis">
      <geom contype="0" conaffinity="0" group="1"/>
    </default>
    <default class="client_col">
      <geom contype="4" conaffinity="2" condim="4" friction="0.4 0.02 0.001" solref="0.008 1" solimp="0.95 0.99 0.001"/>
    </default>
  </default>

  <visual>
    <global offwidth="1024" offheight="1024"/>
    <headlight ambient="0.08 0.08 0.10" diffuse="0.55 0.55 0.58" specular="0.3 0.3 0.3"/>
  </visual>

  <asset>
{chr(10).join(mesh_lines)}
  </asset>

  <worldbody>
    <light name="sun" directional="true" pos="4 2 3" dir="-0.7 -0.35 -0.45" diffuse="0.95 0.93 0.90" specular="0.4 0.4 0.4"/>
    <light name="fill" directional="true" pos="-3 -3 -2" dir="0.6 0.6 0.4" diffuse="0.20 0.24 0.30" specular="0 0 0" castshadow="false"/>

    <body name="chaser" pos="-1.610 0 0">
      <freejoint name="chaser_joint"/>
      <inertial pos="{fmt(bus["com_m"])}" mass="{bus["mass_kg"]:.6g}" fullinertia="{fmt([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>
{chr(10).join(bus_col_geoms)}
{chr(10).join(sites)}
{chr(10).join(bus_vis_geoms)}
      <camera name="bus_cam" mode="fixed" pos="0.332 0.221 -0.075" xyaxes="0 -1 0 0 0 1" fovy="40"/>
      <camera name="bus_lidar" mode="fixed" pos="0.332 0.221 0.005" xyaxes="0 -1 0 0 0 1" fovy="45"/>
{arm1_xml}
{arm2_xml}
    </body>

{c_txt}  </worldbody>

  <actuator>
{chr(10).join(act_lines)}
  </actuator>

  <sensor>
{chr(10).join(sensor_lines)}
  </sensor>

  <equality>
{chr(10).join(eq_lines)}
  </equality>
</mujoco>
"""
    OUT_XML.write_text(xml, encoding="utf-8", newline="\n")
    return OUT_XML


if __name__ == "__main__":
    import mujoco

    out = build_scene()
    m = mujoco.MjModel.from_xml_path(str(out))
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chaser")
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "target")
    print(
        f"Built {out} ({out.stat().st_size / 1024:.1f} KB): nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom} | "
        f"servicer subtree={m.body_subtreemass[cid]:.2f} kg (bus={m.body_mass[cid]:.2f}) | client={m.body_mass[tid]:.2f} kg"
    )
