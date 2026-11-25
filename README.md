# mjorbit-servicing

Modular autonomous orbital servicing, unprepared satellite capture, GPU residual reinforcement learning, and spacecraft configuration experiments built on **[mjorbit](https://github.com/johnzhang3/mjorbit)**.

---

## 1. About `mjorbit`

**[mjorbit](https://github.com/johnzhang3/mjorbit)** ([arXiv:2609.08010](https://arxiv.org/abs/2609.08010)) extends MuJoCo (`3.7.0`) and `mujoco-warp` with coupled orbital and multibody dynamics:

- Propagated Chief ECI orbit (`R_eci`, `V_eci` in km and km/s) alongside local chief-centered inertial MuJoCo multibody coordinates (metres, SI wrenches)
- Analytical/numerical orbital perturbations: Clohessy–Wiltshire / Encke two-body tidal acceleration, $J_2$ oblateness, gravity-gradient torque, solar direction/eclipse state, drag, and SRP
- Native reaction wheels, thrusters, magnetorquers, contact-rich rigid-body dynamics, and GPU-batched simulation (`mjorbit_warp`) for thousands of parallel worlds

This repository consumes upstream `mjorbit` unmodified (either via the pinned `simulator/` submodule or an existing local checkout pointed to via `MJORBIT_DIR`) and adds a modular autonomy, perception, hardware-spec, RL, and visualization stack for **non-cooperative orbital servicing**.

---

## 2. Baseline Experiment: P7 Unprepared Ring-Capture Servicer

**P7** was our experimental proving ground to test how far a truth-free, classical + learned orbital servicing stack could go when capturing an unprepared, tumbling **300.1 kg asymmetric single-wing client** (`client_p7`) by its **24" (609.6 mm) Marman launch adapter ring** in a **550 km, 97.6° SSO orbit** (`MjorbitBackend` + Earth limb + starfield + orbital sun).

- **Servicer Rev 2 hardware class:** `196.5 kg` wet mass, `16 x 1 N` canted corner RCS thrusters (`50 ms` minimum impulse bit, plume keep-out cone inside `3 m`), `4 x 12 N·m·s / 0.2 N·m` reaction wheels in a pyramid, narrow-elbow 6-DOF manipulator (`1.18 m` reach), form-closure `ring_tool` v3 (`0.4` / `0.2` friction, dual lead-screw lock), `60 m` flash LiDAR, `512 px` `bus_cam`, and `50 mm`-baseline `128 px` `wrist_cam` / `wrist_cam2` stereo with LED lamp.
- **Full mission loop (`30 m -> capture -> detumble -> release`):** Zero simulator truth in the control/estimation path (`FarKF` + `BusNet_orb` + PCA/LiDAR 6-DOF acquisition + Euler `RelNav` + C¹ wing keep-out transfer + `StereoNet_orb` wrist refinement + guarded descent / lead-screw lock + wheels/thruster detumble + 500 Hz post-open release audit).

### What Succeeded

- **25-seed Orbital E2E Benchmark (`v5_g3`, `MjorbitBackend`, `earth_bg=True`, seeds `7720000–24`, `G3` policy):**
  - **16 / 25 `complete`** — **16 / 16 (100%)** of seeds reaching final capture succeeded
  - **0 / 25 collisions** and **0 / 25 crashes**
  - **0** `no grip (sensed)` and **0** `lost grip in detumble`
  - Propellant `p50 / p95 = 322.1 / 428.5 g` (lowest across `v2–v5` runs); release push `dv p50 / p95 = 0.123 / 1.058 mm/s`; peak clamp/contact force median `143.8 N`
- **Truth-Free Perception under Earth Limb & Stars:**
  - `BusNet_orb` (`bus_cam` far bearing) locked on **25 / 25** orbital seeds (`23 / 25` with full `16/16` fixes) despite bright Earth-limb backgrounds.
  - `StereoNet_orb` reduced median in-plane ring estimate error `E0 -> E1` from **10.00 mm → 1.25 mm** (`max 4.80 mm`, axis tilt `0.79 deg`).
- **GPU Residual RL on `mjorbit_warp` (`G3` PPO):**
  - Trained on `mjorbit_warp` with calibrated `StereoNet_orb` mixture noise and Encke/$J_2$/gravity-gradient tides (`0` training collisions).
  - Raised `N=360` orbital capture rate from **80.3% → 85.0%** (`289 -> 306 / 360`; **93.9% → 97.2%** at nominal 1× error scale, **0 collisions**) and cut peak contact force **4.7×** (`390.9 N -> 82.6 N`) on 1-seed smoke `7720000`.
- **Orbital Dynamics Parity & Visualization:**
  - `MjorbitBackend` matches upstream `mjorbit.mjo_step` over 10 s free-drift within **0.0003 mm** (chaser) and **0.0000 mm** (client) while keeping MuJoCo equalities intact.
  - Interactive **5-view + telemetry + Three.js 3D CAD replay console** (`src/viz/index.html`) plus headless **Blender 5.3 Cycles (OptiX) PBR renderer** (`src/viz/render_blender.py`) and 5-view MP4 exporter (`src/viz/export_video.py`).

### Failures, Limits & Honest Audit

P7 worked well within its design point, and simultaneously exposed clear engineering limits that motivate our next experiments:

1. **9 / 25 Pre-Capture Safe Aborts (Seeds `7720000–24`):** All 9 aborted safely with zero collisions, showing where the classical fly-around and rate limits saturate:
   - `4` `acq8` spin-gate aborts (`> 2.0 deg/s` rated ceiling on high-spin seeds)
   - `2` `xfer_to_hold3: no ring at 8m` (8 m ring lock missed during transfer)
   - `2` `pull_in3: wing_keepout` (approach geometry intersected solar-wing keep-out margin)
   - `1` `hold3: no fix`
2. **ESPA Packaging Miss:** Folding the `1.18 m` 6-DOF single arm against the front face resulted in a **727 mm** stowed depth from the aft port face — **16 mm deeper** than the **711 mm** standard 15" ESPA slot envelope (`711 x 610 x 965 mm`), and the long capture reach amplified detumble momentum (`~2.3 m` CoM-to-CoM lever).
3. **Hard Dynamic Cliffs:**
   - **Axial spin ceiling:** Thrust-limited (`1.5 N / 196.5 kg = 7.65 mm/s^2`) to **2.0 °/s rated / 2.5 °/s max**.
   - **Transverse tumble & roll tilt:** Safe aborts trigger at **≥ 0.25 °/s** transverse rate, and the rigid gripper palm fails to seat cleanly past **≥ 2°** roll tilt.
4. **Sim-to-Real & Subsystem Blind Spots (`docs/EVOLUTION_AND_CRITIQUE.md`):**
   - **Optical domain gap:** MuJoCo OpenGL lacks >120 dB LEO HDR, specular Kapton/aluminum saturation, lens flare, and flash-LiDAR multipath/dropout.
   - **Known-CAD assumption:** Acquisition and ring fit rely on known `client_p7` box dimensions and `609.6 mm` ring geometry.
   - **Rigid-body physics:** Omits harmonic-drive torsional compliance, vacuum friction (`mu = 0.4/0.2` assumed), `0.2–2 Hz` flexible solar-wing modes, and experimental Marman flange load limits (`1.3–2.1 kN` peak clamp spikes seen).
   - **No in-loop fault injection yet:** Stuck thrusters, failed wheels, or star-tracker sun blinding are not yet injected during capture.

---

## 3. Quickstart: Clone & Run with `mjorbit`

> **Platform:** Linux x86-64 (or Windows **WSL2 Ubuntu** with NVIDIA GPU + CUDA for Warp/RL/EGL) using [Pixi](https://pixi.sh/latest/installation/). Pre-trained P7 orbital weights (`bus_net_orb.pt`, `stereo_net_in+out_orb.pt`, `stereo_net_in_orb.pt`, `g3_ppo_orb.pt`) and the compiled scene XMLs are bundled under `weights/` and `assets/`.

### Option A: Clone with the Pinned `mjorbit` Submodule

```bash
git clone --recurse-submodules https://github.com/shrey160/mjorbit-servicing.git mjorbit-servicing
cd mjorbit-servicing

# Install and compile mjorbit's pinned RL/GPU + MuJoCo environment (first run builds C++/CUDA extensions)
cd simulator
pixi install -e rl --locked
pixi run -e rl --locked sync-package
cd ..
```

### Option B: Reuse an Existing Local `mjorbit` Checkout

If you already built `mjorbit` (`johnzhang3/mjorbit`) locally, you don't need to rebuild `simulator/` — just point `MJORBIT_DIR` to it:

```bash
git clone https://github.com/shrey160/mjorbit-servicing.git mjorbit-servicing
cd mjorbit-servicing
export MJORBIT_DIR=/path/to/mjorbit   # e.g. /mnt/c/Shrey_Projs/robotics/mjorbit/simulator
```

### Run Verification & Missions

All commands use `scripts/run.sh`, which configures `PYTHONPATH`, `MJORBIT_DIR`, `MUJOCO_GL=egl`, and Pixi's `--locked` `rl` environment automatically:

1. **Modular Specs, Keep-Out, LiDAR Acq & 10 s `mjorbit` Orbital Parity Test:**
   ```bash
   bash scripts/run.sh python src/checks/test_modular_specs.py
   ```
   Expected: `ALL MODULAR TESTS PASSED` (verifying `<= 0.001 mm` 10 s free-drift parity against upstream `mjorbit.mjo_step` and `earth_bg=True` RGB + LiDAR isolation).

2. **1-Seed Full Orbital E2E Smoke (with pre-trained `BusNet_orb` + `StereoNet_orb` + `G3`):**
   ```bash
   bash scripts/run.sh python src/mission/run_e2e.py \
     --backend mjorbit --earth-bg --seeds 7720000 \
     --g3-policy weights/g3_ppo_orb.pt \
     --out runs/smoke_7720000.json
   ```

3. **Record & Inspect an Interactive 5-View + 3D CAD Replay:**
   ```bash
   bash scripts/run.sh python src/viz/record_replay.py \
     --seed 7720000 --g3-policy weights/g3_ppo_orb.pt \
     --out runs/replay_7720000
   ```
   Serve from the repo root and open in your browser:
   ```bash
   python3 -m http.server 8765 --bind 127.0.0.1
   # Open: http://127.0.0.1:8765/src/viz/index.html?dir=../../runs/replay_7720000
   ```
   Optional offline 5-view MP4 composite (requires `uv` or `pillow` + `imageio-ffmpeg`):
   ```bash
   uv run --with pillow --with imageio-ffmpeg python src/viz/export_video.py --dir runs/replay_7720000 --fps 25
   ```

4. **GPU Batch Evaluation & PPO Training on `mjorbit_warp`:**
   ```bash
   # Record 25 CPU pre-reach starting states (one-time prerequisite for RL sweeps)
   bash scripts/run.sh python src/rl/g1_check.py record --n 25
   # Evaluate orbital baseline & pre-trained G3 policy
   bash scripts/run.sh python src/rl/g3_ppo.py eval --backend mjorbit_warp --policy weights/g3_ppo_orb.pt --per 20
   ```

5. **Blender 5.3 Cycles (OptiX) PBR Keyframes (optional, requires Blender 5.3 + PBR textures):**
   ```bash
   blender --background --python src/viz/render_blender.py -- \
     --replay runs/replay_7720000 --out runs/blender_7720000 --frames 45,360,415 --samples 64
   ```

---

## 4. Repository Layout

```text
mjorbit-servicing/
├── simulator/            # Git submodule -> johnzhang3/mjorbit (unmodified)
├── scripts/
│   └── run.sh            # Portable runner (MJORBIT_DIR or ./simulator + pixi rl env)
├── assets/
│   ├── servicer_rev2/    # Pre-compiled MuJoCo XML scene, bus & tool JSON specs
│   └── clients/client_p7/# Target spacecraft XML & mass/inertia JSON
├── weights/              # Pre-trained P7 orbital weights (BusNet_orb, StereoNet_orb, G3 PPO)
├── src/
│   ├── _paths.py         # Centralized repo, asset, weight, runs, and MJORBIT_DIR resolver
│   ├── specs/            # Typed ClientSpec, ServicerSpec, MissionConfig dataclasses
│   ├── sim/              # PlainMujocoBackend, MjorbitBackend (Encke/J2/GG + PBR), ActuatorBus, optics
│   ├── gnc/              # SensorSuite, FarNavTracker, ArmIK, vendored cams/lidar_nav/far_nav/fastrot
│   ├── perception/       # Geometric ring_fit & truth-free 8 m lidar_acq
│   ├── servicer/         # Rev-2 RCS pulse-frequency allocation, reaction wheels, scene builder
│   ├── phases/           # Modular episode phases (far_leg -> acq8 -> xfer -> capture -> detumble -> release)
│   ├── mission/          # Modular run_e2e.py orchestrator & ring_mission harness
│   ├── nets/             # BusNet, StereoNet, RingCamNet architectures & trainers
│   ├── rl/               # mjorbit_warp GpuReachEnv, ArmFK torch IK, G2 envelope, G3 PPO
│   ├── checks/           # Unit parity tests, sensor/flight benches, grasp_map, fault & truth audits
│   └── viz/              # Synchronized 5-view + 3D CAD replay recorder, HTML console, MP4 & Blender Cycles
├── cad/
│   ├── servicer_rev2/    # FreeCAD 1.1 parametric Rev-2 bus, narrow-elbow arm, and ring tool scripts
│   └── concepts/         # Centerline dual-arm recessed-cavity ESPA concept CAD script
├── experiments/          # Guide & workspace for upcoming configuration/algorithm experiments
└── docs/                 # P7 spec, Rev-2 & dual-arm studies, Cosmos PBR research, evolution critique
```

---

## 5. Future Experiments & Next Configurations

P7 establishes our modular baseline; upcoming branches and `experiments/` will test new hardware, perception, and multi-agent architectures on top of `src/specs/` and `MjorbitBackend` / `mjorbit_warp`:

1. **Dual-Arm Recessed-Cavity ESPA Servicer (`docs/DUAL_ARM_CONCEPT_RESEARCH.md`, `cad/concepts/dual_arm_espa/`):**
   - Centerline (`y = +/-70 mm`) dual recessed cavities (`600 x 600 x 900 mm` core bus; **`695.0 x 606.0 x 951.1 mm`** stowed strictly inside standard 15" ESPA `711 x 610 x 965 mm`).
   - Unlocks **`8"` (`203 mm`) through `37"` (`940 mm`)** adapter rings (`15"` and `24"` zero-overlap verified), ~59% lower reduced-mass detumble momentum enabling **plume-free wheels-only detumble**, and soft impedance symmetric closing to eliminate first-touch kick.
2. **Multi-Client & Higher-Tumble Envelopes (`src/specs/client_spec.py`):**
   - Symmetric two-wing (`client_double_wing`), larger inertias, passive roll compliance, and compliant `3–6 deg/s` tumbling/nutating capture policies trained on `mjorbit_warp`.
3. **Photorealistic PBR Data Factory & Dark-Room HIL Anchor (`docs/COSMOS_SYNTHETIC_DATA_RESEARCH.md`, `docs/EVOLUTION_AND_CRITIQUE.md`):**
   - Decoupled MuJoCo/Warp contact trajectories re-rendered with Cycles PBR + >120 dB space optics + simulated event-camera streams, calibrated against a low-cost physical dark-room stereo/LiDAR HIL ring bench.
4. **Unknown-Object 3DGS & Multi-Agent Swarms (`docs/EVOLUTION_AND_CRITIQUE.md`):**
   - Online primitive-initialized 3D Gaussian Splatting for arbitrary damaged targets, in-loop thruster/sensor FDIR fault survival, and Inspector + Servicer distributed Clohessy–Wiltshire CBF swarms.
