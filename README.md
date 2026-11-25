# mjorbit-servicing

Autonomous orbital servicing, non-cooperative satellite capture, and spacecraft co-design experiments built on **[mjorbit](https://github.com/johnzhang3/mjorbit)**.

---

## 1. Built on `mjorbit`

**[mjorbit](https://github.com/johnzhang3/mjorbit)** ([arXiv:2609.08010](https://arxiv.org/abs/2609.08010)) extends MuJoCo (`3.7.0`) and `mujoco-warp` with coupled orbital and multibody contact physics. It propagates a spacecraft's Earth-Centered Inertial (ECI) reference orbit alongside local inertial multibody dynamics—modeling orbital tides, $J_2$ oblateness, gravity-gradient torque, solar direction and eclipses, reaction wheels, thrusters, and GPU-batched rollouts for thousands of parallel worlds.

This repository treats upstream `mjorbit` (`simulator/`) as an unmodified physics engine and builds a modular orbital servicing stack on top of it: typed spacecraft specs, orbital rendering backends, truth-free perception and GNC, GPU residual RL, and interactive 3D replay tooling.

---

## 2. Baseline Experiment: P7 (Unprepared Ring Capture)

**P7** was our first experiment to see how far an onboard-only (truth-free) autonomy stack could go—taking an ESPA-class 6-DOF servicer from 30 m rendezvous through launch-adapter-ring capture, detumble, and safe release on an unprepared spinning satellite in Low Earth Orbit.

### What worked

- **Zero collisions across 25 orbital runs:** `16 / 25` missions completed end-to-end (`16 / 16` once within capture range), with `0` collisions, `0` crashes, and clean detumble + release.
- **Onboard perception against Earth & stars:** Fine-tuned orbital vision (`BusNet_orb` + wrist stereo `StereoNet_orb`) locked onto the target on all `25 / 25` runs and tightened close-range ring accuracy to `~1.25 mm`.
- **GPU residual RL (`mjorbit_warp`):** Boosted hard-case capture from `80.3%` to `85.0%` (`97.2%` nominal, `0` collisions) while cutting peak contact forces by `4.7x`.
- **Replay & photorealistic inspection:** Synchronized 5-camera + Three.js 3D CAD browser console ([`src/viz/index.html`](src/viz/index.html)) and Blender Cycles PBR renderer ([`src/viz/render_blender.py`](src/viz/render_blender.py)).

### What didn't & limitations

- **Pre-capture aborts (`9 / 25`):** Safely aborted before contact due to fast target spin (`> 2 °/s`), missed 8 m ring lock, or solar-wing keep-out geometry.
- **Packaging & leverage:** Folding a long single arm in front of the bus pushed stowed depth slightly past the standard 15" ESPA envelope (`727 mm` vs `711 mm`) and increased detumble momentum.
- **Narrow tumble tolerance:** Thrust limits cap spin near `2–2.5 °/s`, transverse tumble triggers aborts above `0.25 °/s`, and the rigid gripper palm struggles past `2°` of roll tilt.
- **Sim-to-real gaps:** OpenGL rendering lacks real orbital glare/LiDAR dropouts, pose fitting assumes known client CAD geometry, and rigid-body physics omits joint/solar-wing flex and mid-capture hardware faults ([full critique](docs/EVOLUTION_AND_CRITIQUE.md)).

*Detailed hardware sizing and benchmark logs: [docs/P7_SPEC.md](docs/P7_SPEC.md) · [docs/SERVICER_REV2_RESEARCH.md](docs/SERVICER_REV2_RESEARCH.md).*

---

## 3. Quickstart

> **Requirements:** Linux x86-64 (or Windows **WSL2 Ubuntu** with an NVIDIA GPU + CUDA for Warp/RL and EGL rendering) and [Pixi](https://pixi.sh/latest/installation/). Pre-trained orbital weights (`weights/`) and compiled MuJoCo scenes (`assets/`) are bundled.

### Option A — Clone with the `mjorbit` Submodule

```bash
git clone --recurse-submodules https://github.com/shrey160/mjorbit-servicing.git
cd mjorbit-servicing

# Install and build upstream mjorbit's pinned rl environment (first run compiles C++/CUDA extensions)
cd simulator
pixi install -e rl --locked
pixi run -e rl --locked sync-package
cd ..
```

### Option B — Reuse an Existing Local `mjorbit` Checkout

If you already installed `mjorbit`, skip building `simulator/` and point `MJORBIT_DIR` to it:

```bash
git clone https://github.com/shrey160/mjorbit-servicing.git
cd mjorbit-servicing
export MJORBIT_DIR=/path/to/mjorbit
```

### Run & Visualize

Use [`scripts/run.sh`](scripts/run.sh), which sets `PYTHONPATH`, `MJORBIT_DIR`, `MUJOCO_GL=egl`, and Pixi's `--locked` `rl` environment:

```bash
# 1. Verify specs, keep-out geometry, LiDAR acquisition, and 10 s orbital parity
bash scripts/run.sh python src/checks/test_modular_specs.py

# 2. Run a full 1-seed orbital mission (BusNet_orb + StereoNet_orb + G3 RL policy)
bash scripts/run.sh python src/mission/run_e2e.py \
  --backend mjorbit --earth-bg --seeds 7720000 \
  --g3-policy weights/g3_ppo_orb.pt \
  --out runs/smoke_7720000.json

# 3. Record a synchronized 5-view + 3D CAD replay and inspect it in your browser
bash scripts/run.sh python src/viz/record_replay.py \
  --seed 7720000 --g3-policy weights/g3_ppo_orb.pt \
  --out runs/replay_7720000
python3 -m http.server 8765 --bind 127.0.0.1
# Open: http://127.0.0.1:8765/src/viz/index.html?dir=../../runs/replay_7720000
```

<details>
<summary>Optional: GPU RL evaluation, MP4 export, and Blender Cycles PBR renders</summary>

```bash
# GPU RL: record 25 CPU pre-reach starting states, then evaluate the orbital G3 policy on mjorbit_warp
bash scripts/run.sh python src/rl/g1_check.py record --n 25
bash scripts/run.sh python src/rl/g3_ppo.py eval --backend mjorbit_warp --policy weights/g3_ppo_orb.pt --per 20

# Export a 25 fps 5-view MP4 from a recorded replay (requires uv, pillow, imageio-ffmpeg)
uv run --with pillow --with imageio-ffmpeg python src/viz/export_video.py --dir runs/replay_7720000 --fps 25

# Render Cycles (OptiX) PBR keyframes in Blender 5.3
blender --background --python src/viz/render_blender.py -- \
  --replay runs/replay_7720000 --out runs/blender_7720000 --frames 45,360,415 --samples 64
```

</details>

---

## 4. Next Experiments & Future Configurations

P7 gives us a modular testbed (`src/specs/` + `src/sim/` + `src/phases/`) to swap spacecraft geometries, sensors, and controllers cleanly without touching `simulator/` (see [`experiments/README.md`](experiments/README.md)):

1. **Dual-Arm Recessed ESPA Servicer** ([FreeCAD Model](cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) · [Build Script](cad/concepts/dual_arm_espa/build_dual_arm_concept.py) · [Research](docs/DUAL_ARM_CONCEPT_RESEARCH.md))
   A centerline (`y = +/-70 mm`) recessed dual-arm concept (`600 x 600 x 900 mm` bus, **`695.0 x 606.0 x 951.1 mm`** stowed inside standard 15" ESPA `711 x 610 x 965 mm`) supporting **`8"`–`37"`** adapter rings, soft impedance dual-arm closing, and ~59% lower reduced-mass detumble momentum for **plume-free wheels-only detumble**.
2. **Multi-Client & High-Tumble Envelopes** ([`src/specs/client_spec.py`](src/specs/client_spec.py))
   Symmetric double-wing (`client_double_wing`), larger inertias, passive roll compliance, and compliant `3–6 °/s` tumbling/nutating capture policies trained on `mjorbit_warp`.
3. **Photorealistic PBR Data & Dark-Room HIL Anchor** ([Data Research](docs/COSMOS_SYNTHETIC_DATA_RESEARCH.md))
   Decoupled MuJoCo/Warp trajectories re-rendered with Cycles PBR, space optics, and simulated event-camera streams, anchored against a low-cost physical dark-room stereo/LiDAR testbed.
4. **Unknown-Object 3DGS & Multi-Agent Swarms** ([Evolution & Critique](docs/EVOLUTION_AND_CRITIQUE.md))
   Online 3D Gaussian Splatting reconstruction for unknown/damaged targets, in-loop thruster/sensor fault survival, and Inspector + Servicer distributed CBF swarms.

---

## 5. Repository Layout

```text
mjorbit-servicing/
├── simulator/            # Pinned upstream johnzhang3/mjorbit submodule (unmodified)
├── scripts/run.sh        # Portable launcher (MJORBIT_DIR or ./simulator + pixi rl env)
├── assets/               # Pre-compiled servicer_rev2 & client_p7 MuJoCo XMLs and JSON specs
├── weights/              # Pre-trained BusNet_orb, StereoNet_orb, and G3 PPO checkpoints
├── src/
│   ├── _paths.py         # Centralized repo, asset, weight, runs, and MJORBIT_DIR resolver
│   ├── specs/            # Typed ClientSpec, ServicerSpec, and MissionConfig dataclasses
│   ├── sim/              # PlainMujocoBackend, MjorbitBackend (orbital physics + PBR), ActuatorBus
│   ├── gnc/              # FarNavTracker, SensorSuite, ArmIK, LiDAR/optical helpers
│   ├── perception/       # Geometric Marman ring fitter and truth-free 8 m LiDAR acquisition
│   ├── servicer/         # Rev-2 thruster allocation, reaction wheels, scene builder
│   ├── phases/           # Modular mission phases (approach -> acquisition -> capture -> detumble -> release)
│   ├── mission/          # End-to-end mission runner (run_e2e.py)
│   ├── nets/             # BusNet, StereoNet, and RingCamNet models & training scripts
│   ├── rl/               # Batched GPU reach environment (mjorbit_warp), Torch IK, G3 PPO
│   ├── checks/           # Parity tests, sensor/grasp benches, fault injection, truth audits
│   └── viz/              # 5-view replay recorder, HTML/Three.js console, MP4 & Blender Cycles
├── cad/                  # FreeCAD (.FCStd), STEP (.step) models & build scripts (Rev-2, client_p7, dual-arm concept)
├── experiments/          # Configuration roadmap & new experiment workspace
└── docs/                 # P7 spec, hardware studies, synthetic data report, evolution critique
```
