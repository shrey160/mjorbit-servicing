# mjorbit-servicing

Autonomous orbital servicing, non-cooperative satellite capture, and spacecraft co-design experiments built on **[mjorbit](https://github.com/johnzhang3/mjorbit)**.

> **13-Page Technical Draft PDF:** [`docs/servicer_2_draft/servicer_2_draft.pdf`](docs/servicer_2_draft/servicer_2_draft.pdf)
> **Independent Verification Guide:** [`REPRODUCE.md`](REPRODUCE.md) (exact commands mapping every table, figure, MuJoCo benchmark, and thermal/FEM run)
> **Bundled Agent Skills:** [`skills/README.md`](skills/README.md) (FreeCAD CAD workflow & `thermal-sims`)

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

## 4. Dual-Arm `v4.2` Servicer, Hosted IOD Demo & Next Experiments

P7 gives us a modular testbed (`src/specs/` + `src/sim/` + `src/phases/`) to swap spacecraft geometries, sensors, and controllers cleanly without touching `simulator/` (see [`experiments/README.md`](experiments/README.md) and [`REPRODUCE.md`](REPRODUCE.md)):

1. **Dual-Arm Recessed 15" ESPA Servicer (`v4.2`)** ([FreeCAD Model](cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) · [Build](cad/concepts/dual_arm_espa/build_dual_arm_concept.py) · [Verification](cad/concepts/dual_arm_espa/verify_v4_engineering.py) · [MuJoCo Bench](experiments/01_dual_arm_espa/bench_dual_arm_capture.py))
   Centerline (`Y = +/-70 mm`) recessed dual-arm baseline (`640 x 600 x 910 mm` core bus, **`704.3 x 606.0 x 937.4 mm`** stowed inside standard 15" ESPA `711 x 610 x 965 mm`, `188.5 kg` Spec-MGA, `CG = 357.7 mm`), two 6-DOF Cardan-Roll arms (`0.00 mm^3` solid overlap across **`8"`–`37"`** rings past `350 mm` GEO nozzles), and closed-loop MuJoCo capture Checks A–E passing (**2.0 deg/s wheels-only detumble at 69.3% RW fill, 0 bad contacts, 0 braking plume pulses**, active `5p+5y` wrist alignment).
2. **Ultra-Shallow Hosted-Payload IOD Demo (`arm_iod_demo`)** ([FreeCAD Model](cad/concepts/arm_iod_demo/arm_iod_demo.FCStd) · [Build](cad/concepts/arm_iod_demo/build_arm_iod_demo.py) · [Verification](cad/concepts/arm_iod_demo/verify_arm_iod_demo.py))
   A **`273.0 x 520.0 x 840.0 mm`** (`38.0 kg` base / `45.6 kg` +20% MGA) hosted-payload testbed exercising identical 6-DOF arm hardware against a 15" target ring deployed on four rollable bistable CFRP booms (`8.2 Hz` first mode, `+5 deg` differential pitch tilt, wrist-stereo + Mini45 F/T capture).
3. **Preliminary Thermal Audit (`Orin NX 25 W` + Rad-Hard OBC `4–6 W`)** ([Orbital TMM](cad/concepts/thermal_audit.py) · [Unit Smoke Test](cad/concepts/verify_freecad_ccx_units.py))
   Calibrated to ECSS-E-ST-31C, NASA MSFC TM-2001-211221, NASA GSFC GOLD Rules, and Gilmore (`0.195 m^2` `v4.2` / `0.101 m^2` IOD `-Z` OSR, bolted DragonEye CCHP strap). Zero hard-limit failures across LEO, GEO, and conductive/isolated IOD cases (`<= 59.7%` cold heater duty), verified in coupled 3D `Gmsh` + `CalculiX v2.22` FEM (`run_fem_thermal_*.py`, max CTE shift `0.0464 mm`).
4. **Next Simulation & Control Milestones**
   - Build `experiments/02_arm_iod_demo/` (compliant 4-boom dynamics, independent `+5 deg` tilt, wrist-stereo + F/T capture/retract).
   - 12-DOF coordinated free-floating dual-arm control, `v4.2` orbital autonomy (`far_leg` -> `release` w/ `Orin NX` burst gating), and 12-DOF dual-arm residual RL on `mjorbit_warp`.

---

## 5. Repository Layout

```text
mjorbit-servicing/
├── simulator/            # Pinned upstream johnzhang3/mjorbit submodule (unmodified)
├── scripts/run.sh        # Portable launcher (MJORBIT_DIR or ./simulator + pixi rl env)
├── REPRODUCE.md          # Independent verification guide & claim-to-command traceability
├── skills/               # Bundled FreeCAD CAD workflow & thermal-sims agent skills
├── assets/               # Pre-compiled servicer_rev2 & client_p7 MuJoCo XMLs and JSON specs
├── weights/              # Pre-trained BusNet_orb, StereoNet_orb, and G3 PPO checkpoints
├── cad/
│   ├── concepts/         # v4.2 dual-arm ESPA, arm_iod_demo, thermal_audit.py & 3D CalculiX FEM
│   ├── servicer_rev2/    # P7 Rev-2 FreeCAD/STEP models & build scripts
│   └── clients/          # client_p7 FreeCAD/STEP model
├── experiments/
│   └── 01_dual_arm_espa/ # v4.2 dual-arm MuJoCo scene, spec & Checks A-E benchmark
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
└── docs/
    ├── servicer_2_draft/ # 13-page Technical Design Draft PDF & cross-platform builder
    ├── media/            # Verified CAD, MuJoCo, and thermal/FEM renders
    └── *.md              # P7 spec, hardware studies, synthetic data report, evolution critique
```
