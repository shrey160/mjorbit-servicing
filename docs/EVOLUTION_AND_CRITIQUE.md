# mjorbit Beyond the Demo: Critical Audit, Synthetic Data Factory, and Multi-Agent Evolution

**Date:** 23 November 2026  
**Scope:** Post-P7 strategic and technical critique of the `mjorbit` servicer rev-2 programme — identifying hard sim-to-real gaps, evaluating Starship-era cost/hardware trade-offs, and defining a roadmap to scale into a photorealistic synthetic data factory, unknown-object tumbling capture, and multi-agent orbital swarms.

---

## Executive Summary

Through Projects 1–7, `mjorbit` has built a verified, truth-free classical-plus-learned autonomy kernel for capturing an unprepared spinning satellite by its launch adapter ring. On the rev-2 ESPA-class servicer (196.2 kg wet, 16 × 1 N corner thrusters, 4 × 12 N·m·s wheels, narrow-elbow 6-DOF arm, form-closure ring tool v3, 50 mm wrist stereo + 60 m flash LiDAR, 150 ms latency), the current stack achieves **25 / 25 tune captures**, **24 / 25 frozen-test captures**, **25 / 25 CPU reach-bench grips**, and **0 collisions** across all rated conditions (0–2 °/s spin, max 2.5 °/s thrust-limited).

As a demonstration and Phase-0/A sizing tool, this is strong. As a path to real flight hardware or general orbital autonomy, it has **severe structural blind spots**: synthetic OpenGL rendering hides a massive optical domain gap, perception relies on known CAD geometry, rigid-body contact hides joint/wing compliance and ring-lip stress, and the dynamic envelope breaks down under true multi-axis tumbling.

This study audits those flaws ruthlessly and lays out a four-pillar evolution:
1. **Decouple physics from rendering** to turn `mjorbit` (MuJoCo + Warp) into a **photorealistic, procedural synthetic data factory** (RGB, stereo, LiDAR, event camera, F/T wrenches).
2. **Replace known-CAD fitting** with online **3D Gaussian Splatting (3DGS)** reconstruction and apply **learned visuomotor policies (Diffusion / GPU RL)** *inside* verifiable classical safety shields specifically for compliant tumbling capture and assembly.
3. **Scale from single-agent to multi-agent / swarms** (Inspector + Servicer pairs → Cooperative Multi-Tug capture → Heterogeneous assembly swarms) guarded by **Distributed Control Barrier Functions (CBFs)** with passive-safety guarantees.
4. **Anchor sim-to-real** with a low-cost (\$5k–\$15k) dark-room Hardware-in-the-Loop (HIL) bench before trusting any synthetic dataset or learned controller.

---

## 1. Harsh Technical Critique: Seven Flaws Beyond the Demo

### 1.1 The Optical & Sensor Sim-to-Real Cliff (Highest Risk)
`stereo_net` and `ringcam_net` are trained and validated exclusively on MuJoCo's basic OpenGL renderer. OpenGL produces clean Lambertian/Phong shading, soft shadows, and ideal pinhole geometry. Real Low Earth Orbit (LEO) illumination is radically more hostile:
- **Extreme High Dynamic Range (>120 dB):** With no atmosphere to scatter light, sunlit anodized aluminum and crinkled gold Multi-Layer Insulation (MLI) clip standard CMOS sensors while adjacent shadowed surfaces drop to pitch black.
- **Stray light, lens flare, and Earth albedo:** Specular glints off solar arrays and bright Earth-limb background corrupt edge-based shift searches and CNN features.
- **LiDAR multipath and specular dropout:** Real flash LiDARs experience range bias, saturation, and missing returns on specular metal rings and Kapton blankets—none of which appear in MuJoCo depth-buffer raycasts.

Stanford SLAB's **SPEED+** and **SHIRT** benchmarks demonstrated that models achieving sub-centimeter accuracy on synthetic OpenGL imagery routinely collapse on Hardware-in-the-Loop (HIL) sunlamp imagery unless explicitly trained with physically-based rendering and domain adaptation. Our current perception nets would almost certainly fail zero-shot on real orbital images.

### 1.2 The Known-CAD Crutch (Not Truly "Unprepared")
Although P7 removed simulation truth from the control path, it still accepts **design geometry from the model** (`CONTEXT.md`): the box-fit, ring-fit, and grasp-section logic know the exact client box dimensions and 609.6 mm Marman ring diameter/lip profile. Real defunct satellites and rocket bodies can have torn MLI, bent antennas, unknown mass/CoM offsets, or non-standard separation rings (15", 24", 37", Ariane/Vega/Falcon adapters). If the target is not in a preloaded CAD library, the current perception pipeline cannot initialize or select a grasp point.

### 1.3 Rigid-Body Physics Hides Contact & Structural Failures
MuJoCo models the arm links, harmonic-drive joints, ring lip, and client solar arrays as rigid bodies (and MuJoCo Warp runs with `noslip=0`). In flight:
- **Joint compliance and vacuum friction:** Real space manipulators (e.g., DLR ROKVISS) exhibit harmonic-drive torsional stiffness around ~4,900 N·m/rad, backlash, and ~50% higher joint friction in orbital thermal-vacuum conditions.
- **Flexible appendages:** Client solar wings have 0.2–2 Hz structural modes that couple into post-capture detumble and can excite control instabilities or snap under abrupt torque steps.
- **Unverified ring-lip loads:** P7.5 measured 1.3–2.1 kN peak contact/clamp spikes on the ring lip. We have no verified structural allowable for a real Marman flange and no measured vacuum friction coefficient ($\mu = 0.4 / 0.2$ is an engineering assumption; cold welding and stick-slip are unmodelled).

### 1.4 Narrow Dynamic Envelope (Benign Spin vs. Real Tumbling)
The current mission is rated for 0–2 °/s (2.5 °/s max, thrust-limited by $F_{\max}/m = 7.65\text{ mm/s}^2$). Real derelict upper stages and dead satellites frequently tumble/nutate at 3–10+ °/s with significant transverse rates. P7.5's grasp map already exposed two hard cliffs:
- **Transverse rate $\ge 0.25\text{ deg/s}$** triggers safe aborts.
- **Roll-axis tilt $\ge 2^\circ$** fails to seat because the gripper palm has no passive or active roll compliance.
- **Incomplete end-to-end integration:** `ring_mission.py` currently covers 3.2 m hold → capture. The far leg (30 → 3.2 m), post-capture detumble/ID/move (where rev-2's 0.2 N·m wheels already missed the 0.5° attitude criterion), and release are still on the frozen P6 bar-client path.

### 1.5 Brittle Classical State Machine & Missing In-Loop Faults
The mission executes a fixed sequential state machine (`hold → approach → settle → pre-reach → descent → close → lock`) governed by hand-tuned thresholds (`SWEEP_MIN = 4°`, `ETA_SPIN = 0.85`, `CATCH_MM = 1.0`, fixed 150 ± 20 ms latency). This is transparent and verifiable for one narrow capture task, but does not generalize to partial occlusion, re-grasping, or multi-step manipulation. Furthermore, **in-loop hardware faults**—a stuck-open thruster, dead reaction wheel, frozen camera frame, radiation single-event upset (SEU), or star-tracker blinding inside its 45° sun keep-out cone—are not yet injected or survived during capture.

### 1.6 Unmodelled Non-Mechanical Spacecraft Subsystems
Thermal distortion (which can shift the 50 mm wrist-stereo extrinsic calibration in eclipse vs. sun), radiation tolerance of COTS vision compute and flash LiDAR, battery/solar power gating during peak wheel + arm + lidar draw, and long-horizon orbital mechanics ($J_2$, drag, phasing burns) remain design notes rather than coupled simulation models.

### 1.7 Summary Risk Matrix

| # | Flaw | Current Sim Assumption | Real-Orbit Failure Mode | Severity | Primary Fix |
|---|---|---|---|---|---|
| 1 | Optical domain gap | MuJoCo OpenGL depth & RGB | Sun glare, shadow blackout, LiDAR dropout | **Critical** | PBR ray-tracing + event cams + HIL bench |
| 2 | Known-CAD crutch | Exact 609.6 mm ring & box known | Cannot track unknown/damaged debris | **High** | Online 3DGS reconstruction + hardpoint segmentation |
| 3 | Rigid-body physics | Rigid joints, wings, lip; `noslip=0` | Contact bounce, wing resonance, lip buckling | **High** | Joint compliance model, FEA lip check, compliant control |
| 4 | Benign spin only | 0–2 °/s, thrust-limited to 2.5 °/s | Tumbling targets (>3 °/s) uncapturable | **Medium-High** | Passive roll compliance, higher thrust/closer hold, RL residual |
| 5 | Brittle state machine | Fixed phases, no mid-capture faults | Stuck thruster / occlusion causes collision | **Medium-High** | CBF safety filter + fault injection + retry logic |
| 6 | Unmodelled subsystems | Thermal, rad, power as design notes | Stereo drift, COTS GPU upset, brownout | **Medium** | Coupled thermal/power ledger, rad shielding |

---

## 2. Cost Reality & Starship-Era Economics

Our rev-2 sizing is credible as a **Phase-0/A hardware-class reference**, but real procurement cost depends heavily on heritage tier and launch constraints.

### 2.1 Recurring Parts BOM vs. Mission Cost

| Tier | Propulsion & ADCS | Sensors, Arm & Bus | Parts BOM (Unit #2+) | Est. First-Unit Mission (w/ NRE & Launch) |
|---|---|---|---|---|
| **Heritage Aerospace Catalog** (rev-2 report baseline) | 16× ECAPS 1N HPGP (\$1.5M–\$2.5M), 4× RW-12 (\$400k–\$480k) | RVS/ASC LiDAR (\$0.8M–\$2.5M), space arm + ring tool (\$0.7M–\$1.8M) | **\$4.5M – \$10.2M** | **\$20M – \$45M** |
| **Lean NewSpace / COTS-Screened** | 16× 1N cold gas (\$200k–\$350k), NSS wheels + COTS ST (\$150k–\$280k) | Screened ToF/LiDAR + CMOS stereo (\$40k–\$150k), in-house arm (\$150k–\$400k) | **\$0.8M – \$1.8M** | **\$4M – \$8M** (incl. \$1.4M Transporter launch) |
| **Ground Dark-Room HIL Testbed** | Air-bearing / fixed mount | Industrial ToF/LiDAR, stereo CMOS, cobot arm, ATI Mini40, CNC ring | **\$50k – \$150k** (full) / **\$5k – \$15k** (minimal) | N/A (ground lab) |

**Market comparison:** Current operational or contracted servicers sit at **\$37.5M–\$54.5M** (Starfish Space Otter contracts), **\$80M–\$90M** (Astroscale ADRAS-J / J2 program), **\$112M** (ESA/OHB ClearSpace-1 prime contract), and **\$65M+ per 5-yr service** (Northrop Grumman MEV). A lean \$3M–\$5M recurring vehicle is **10×–30× cheaper**, which is the threshold required to make Active Debris Removal (ADR) commercially viable.

### 2.2 What Changes When Starship Drops Launch Cost
If Starship reduces LEO launch cost from ~\$7,000/kg (Transporter) to \$200–\$1,000/kg with a large fairing:
1. **ESPA volume torture vanishes:** Half of P7's CAD complexity came from folding a 1.18 m arm into 710 mm depth and 200 kg mass. With cheap mass and volume, **cold gas wins outright** (a 30–50 kg, 300-bar N₂ tank eliminates \$1M+ in monoprop hardware and all plume contamination).
2. **Spin and wheel bottlenecks disappear:** Adding 30–50 kg for 2–4 N thrusters and 25–50 N·m·s wheels enables wheels-only detumble and pushes rated spin well past 3–4 °/s.
3. **Heavier = cheaper & radiation-tolerant:** Stiffer, heavier industrial-derived arm joints cut structural flex and cost, while thick aluminum shielding allows cheap COTS GPUs and LiDARs to survive LEO radiation.
4. **Market shift:** Cheap launch *shrinks* LEO smallsat life extension (operators will replace cheap dead satellites), but *explodes* demand for **Active Debris Removal** (more launches + stricter FCC 5-year / ESA Zero Debris mandates), **GEO/high-value servicing**, and **in-space assembly** (Starship delivers bulk modules; autonomous robotic tugs assemble them).

---

## 3. Turning `mjorbit` into a Realistic Synthetic Data Factory

Rather than discarding MuJoCo/Warp, the highest-leverage next architectural step is to **decouple contact physics from photorealistic rendering**:

```
[Procedural Client & Orbit Generator]
          │  (randomized CAD, MLI, rings, wings, damage, tumble states)
          ▼
[MuJoCo / MuJoCo Warp Physics Engine]  (100k+ steps/s on GPU)
          │  (exact trajectories, contacts, F/T wrenches, joint torques, momentum)
          ▼
[Photorealistic PBR Rendering Pass]    (Blender Cycles / NVIDIA Isaac Sim / Unreal)
          │  (Kapton MLI BRDFs, 0-ambient sun, Earthshine, HDR bloom,
          │   ray-traced LiDAR multipath, simulated Event Camera streams)
          ▼
[Multimodal Orbital Manipulation Dataset]
          └─► RGB + Stereo + Flash LiDAR + Event Stream + 6-DOF Pose
              + Dense Normals + Grasp Affordances + Contact F/T Time-Series
```

### Key Components of the Data Factory
- **Procedural Spacecraft Generator:** Programmatically assemble thousands of diverse targets—varying bus aspect ratios, solar array counts/hinge angles, dish/antenna clutter, crinkled MLI normal maps, thermal blankets, surface scorching/damage, and multiple interface standards (15"/24"/37" Marman rings, apogee engine nozzles, grapple fixtures, trusses).
- **Physically-Based Orbital Lighting:** Model collimated solar irradiance (~1,361 W/m²), hard umbra shadows, Earth albedo bounce, and sensor response curves (auto-exposure lag, saturation bloom, read/shot noise, motion blur).
- **Neuromorphic Event Camera Channel:** Add a simulated Dynamic Vision Sensor (DVS) stream alongside RGB/stereo. Event cameras provide **>120–140 dB dynamic range** and microsecond latency—directly defeating solar glare and motion blur on fast-tumbling targets—while Recurrent Vision Transformers (RVTs) handle the low-event-rate final approach (down to 1 mm/s).
- **Benchmark Value:** Public datasets like SPEED+ and SHIRT cover *monocular pose estimation* of a single satellite. Almost no public dataset pairs **photorealistic orbital vision + flash LiDAR + event streams + contact-rich 6-DOF grasping forces**. Building this generator makes `mjorbit` a reusable foundation for future space-robotics models.

---

## 4. Evolving the Autonomy Stack

### 4.1 Unknown-Object Perception (Online 3DGS + Hardpoint Segmentation)
Replace the known-CAD assumption with a two-stage perception pipeline during the 30 m → 8 m inspection phase:
1. **Primitive-Initialized 3D Gaussian Splatting (3DGS):** Unlike NeRFs (which require heavy ray-marching and are too slow for space avionics), 3DGS represents the unknown tumbling target as explicit anisotropic 3D Gaussians that train and render orders of magnitude faster. Using a CNN primitive initializer plus sun-aware illumination models and photometric pose refinement, the servicer reconstructs the target's 3D geometry and tracks its 6-DOF tumble simultaneously.
2. **Grasp-Affordance Segmentation:** Run a geometry/vision segmentation head on the reconstructed 3DGS/point cloud to locate structurally load-bearing features (adapter rings, engine bells, structural trusses) and exclude fragile zones (solar cells, MLI, antennas), feeding candidate grasp frames directly into the classical reach planner.

### 4.2 Hybrid Control: Classical Safety Shields + Learned Visuomotor Policies
Our P2 and P7 experiments already proved a vital principle: **do not use RL where classical control already gets >90% (G3 PPO stayed flat because there was no gradient signal).** Instead:
- **Keep classical physics for safety and transport:** Orbital guidance, passive-safety CW free-drift checks, momentum budgeting, and Control Barrier Function (CBF) safety filters stay mathematically verifiable.
- **Deploy Diffusion Policies / VLA / GPU RL on genuine classical cliffs:**
  1. *High-rate tumbling & nutating capture (3–10 °/s):* Closed-loop visuomotor + force-torque policies using wrist stereo, wrist F/T, and joint torques to accommodate roll tilt and transverse rates passively/actively during contact.
  2. *Dexterous in-space assembly:* Multi-step contact-rich tasks (aligning connectors, mating truss nodes, re-grasping) where hand-coded state machines become unmaintainable.

---

## 5. Scaling to Multi-Agent Systems & Swarms

A single 200 kg servicer is fundamentally limited by one viewpoint, one thrust-to-mass ratio, and one grasp lever arm. Scaling `mjorbit` to multi-agent orbital robotics unlocks three progressive tiers:

| Tier | Architecture | Core Capability Unlocked | Technical Requirements |
|---|---|---|---|
| **Tier 1: Inspector + Servicer Pair** (2 agents, highest near-term ROI) | 1× 200 kg servicer + 1× 10–20 kg free-flying inspector | Eliminates self-shadowing, occluded-side blind spots, and **unobserved roll** (P7 F0–F1); continuous 360° 3DGS mapping | Inter-agent RF/optical ranging, shared EKF/factor-graph belief state, synchronized sun-angle phasing |
| **Tier 2: Cooperative Multi-Tug Capture** (2–4 agents) | 2–4 small Starship-era servicers approaching a 3+ ton tumbling upper stage | Splits momentum and thrust across agents; captures fast-tumbling, massive debris without a \$100M+ mega-servicer | Synchronized multi-body impedance control, symmetric grasp allocation, coupled stack momentum management |
| **Tier 3: Orbital Assembly & Logistics Swarms** (5–20+ agents) | Heterogeneous fleet: bulk tugs + nimble inspectors + dexterous assemblers | Autonomous assembly of large telescopes, power arrays, and orbital depots from Starship-delivered modules | Decentralized task allocation (market/consensus), inter-agent collision avoidance, multi-step planning |

### Safety Backbone for Swarms: Distributed CBFs + Passive Safety
In orbit, a failed thruster on one agent can turn it into a projectile through the swarm. Multi-agent `mjorbit` should enforce **Distributed Control Barrier Functions (CBFs)** formulated as lightweight Quadratic Program (QP) safety filters on each agent. Each CBF encodes **fixed-horizon Clohessy-Wiltshire passive safety**: an agent's commanded thrust is only admissible if, under complete loss of control at the next timestep, its natural free-drift trajectory remains outside the keep-out spheres of both the client and every other swarm agent. Because `mjorbit` already runs 4,096 worlds in MuJoCo Warp, extending the GPU environment to multi-chaser scenes (2–8 agents per world) is a direct architectural extension.

---

## 6. Ground Validation & Phased Evolution Roadmap

### 6.1 The Dark-Room HIL Anchor (\$5k–\$15k)
Before investing months in synthetic datasets or swarm RL, build a minimal physical validation loop inspired by Stanford TRON and DLR EPOS:
- **Setup:** One real stereo camera pair (50 mm baseline), one industrial ToF/flash LiDAR, a desktop 6-DOF arm (or linear/rotary stage), a machined anodized-aluminum ring section wrapped with crinkled Kapton foil, and a harsh collimated studio spotlight in a blackened room.
- **Goal:** Capture **100–200 real labeled frames and 20–50 physical grasps**. Running sim-trained nets on this small real test set will immediately quantify the true optical and contact domain gap and calibrate the PBR rendering pipeline.

### 6.2 Phased Roadmap

| Phase | Horizon | Objective | Exit Gate |
|---|---|---|---|
| **Phase A: Close P7** | Weeks 1–4 | Full GPU re-sweep (scales 1–2); unify far-leg + capture + post-capture + release on rev 2; add roll compliance & test 3–4 °/s tumbling/nutating clients | Unified mission ≥ 90% capture, 0 collisions on fresh test split `7700000–24`; measured tumbling gap documented |
| **Phase B: Data Factory + HIL** | Months 2–4 | Build Blender Cycles / Isaac Sim PBR replay pass over MuJoCo trajectories + procedural client generator + event-cam simulator; build \$5k–\$15k dark-room HIL bench | Sim-trained stereo/pose net transfers to real dark-room anodized ring within <5 mm radial error |
| **Phase C: Unknown Objects + Compliant Learning** | Months 5–9 | Online 3DGS target reconstruction during inspection; hardpoint segmentation; Diffusion/RL residual policy for 3–6 °/s compliant tumbling capture with CBF shield | Zero-shot capture on unseen procedural clients at ≥ 85% success, 0 collisions, with injected sensor/actuator faults |
| **Phase D: Multi-Agent / Swarm Warp** | Months 10–15 | Tier-1 Inspector + Servicer pair in `mjorbit`; Distributed CBF passive-safety filter; Tier-2 dual-tug cooperative detumble of a 3-ton tumbling stage in MuJoCo Warp | 0 inter-agent or client collisions under single-agent thruster failure; measurable perception/momentum gain over 1-agent baseline |

---

## 7. References

1. **Stanford SLAB — SPEED+ & SHIRT:** Park et al., *SPEED+: Next-Generation Dataset for Spacecraft Pose Estimation across Domain Gap* (IEEE Aerospace, 2022); *SHIRT: Satellite Hardware-in-the-loop Rendezvous Trajectories Dataset* (Stanford SLAB).
2. **3DGS / NeRF for Non-Cooperative Spacecraft:** Recent 2024–2026 literature on primitive-initialized 3D Gaussian Splatting and lighting-aware photometric pose refinement for tumbling space objects (Stanford SLAB / arXiv).
3. **Event Cameras for Space RPO:** Neuromorphic DVS vision (>120 dB HDR, microsecond latency) and Recurrent Vision Transformers (RVTs) for slow-approach and high-glare orbital pose estimation (ESA / NASA / arXiv).
4. **Multi-Agent Passive Safety & CBFs:** Distributed Control Barrier Functions for fixed-horizon Clohessy-Wiltshire passive safety and cooperative tethered/multi-agent debris capture (Schaub AVS Lab / AMOS / IEEE).
5. **Commercial & Institutional Servicing Benchmarks:** Starfish Space Otter contracts (USSF/SDA, 2024–2026); Astroscale ELSA-d & ADRAS-J / ADRAS-J2 (MEXT/JAXA); ESA / OHB ClearSpace-1; Northrop Grumman MEV; SpaceX Transporter Rideshare pricing; ECAPS 1N HPGP, Rocket Lab RW-12, Dawn/VACCO 1N cold gas, NewSpace Systems ADCS.
6. **Internal `mjorbit` Records:** `CONTEXT.md`, `projects/p7_unprepared_client/SPEC_7.md` (§12–26), `reports/Servicer rev 2 research for P7.md`, and `learning/RESEARCH_SERVICING.md`.
