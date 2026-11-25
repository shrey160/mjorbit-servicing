# Agentic mjorbit Workflow Research & Gap Analysis

Date: 24 November 2026
Context: Lessons from building `p1_detumble` through `p7_unprepared_client` on `mjorbit` (MuJoCo 3.7 + MuJoCo Warp).

## 1. The Core Gap
Most "AI + space robotics" frameworks (e.g. *Space Robotics Bench*, *Isaac Lab*) assume the spacecraft CAD, sensors, actuator layout, and environment already exist and focus purely on parallel RL training. In practice, across `p1`–`p7`, the hardest failures came from the **co-design and verification loop** between CAD, contact physics, orbital dynamics, perception, and FDIR monitors.

An agentic `mjorbit` workflow fills this gap by turning a high-level mission specification (*"capture an unprepared 300 kg spinning ring client in 550 km LEO"*) into a **closed-loop engineering & verification pipeline** with automated physical gates.

## 2. Four Pillars of an Agentic `mjorbit` System

### A. Parametric CAD-to-MJCF Compiler + Static Geometry Gates
- **Observed pain points:** FreeCAD scripts, STL exports, and MuJoCo XMLs drifted out of sync (`wrist_cam` 47 mm inside bus at stow, shoulder housings 10 mm inside bus plate, `znear = 69 mm` clipping `adapter_ring` during `descent`).
- **Agentic solution:**
  - Edit typed specs (`ServicerSpec`, `ClientSpec`, `MissionConfig`) -> rebuild CAD via FreeCAD MCP -> compile MJCF -> run **automated static gates** before any dynamics run:
    1. Launch envelope & mass margin check (e.g. ESPA `61 × 71 × 97 cm`, `<= 181 kg`).
    2. Self-collision & clearance check (`mj_geomDistance`) across `stow -> unfold -> ready -> pre-reach -> grasp`.
    3. Camera FOV, occlusion, and `znear` check at closest approach.

### B. Physics-First Calibration & Feasibility Harness
- **Observed pain points:** Simulator settings were repeatedly mistaken for controller bugs (pyramidal friction cones under-reporting grip by 4×, `noslip=0` creep, `condim=3` missing twist, `inertia="shell"` on planar meshes).
- **Agentic solution:** Mandatory pre-flight benches before training RL or running Monte Carlo batches:
  1. **Contact gate:** Ramp-wrench grip test (`condim=4`, elliptic cone, `noslip`) to certify the capture envelope (`grasp_map`).
  2. **Authority gate:** Derive `spin_limit` from `0.85 × thrust/mass` and wheel momentum budgets.
  3. **Dynamics parity gate:** Verify `MjorbitBackend` vs. upstream `mjorbit.mjo_step` (`10 s` Encke / J₂ / gravity-gradient parity).

### C. Hypothesis-Driven Experiment & Split-Hygiene Loop
- **Observed pain points:** Easy for agents to tune on held-out test seeds, leak simulator truth into control, or launch thermal/OOM-inducing sweeps.
- **Agentic solution:** Enforce our 5-rule working loop as tool-level guardrails:
  1. **Locked split registry:** Enforce `tune` / `val` / frozen `test` seed separation.
  2. **Truth-leak & FDIR audit:** Auto-run `truth_audit` and fault injection (kicks, stuck/weak thrusters) on every candidate controller.
  3. **Compute & thermal guardrails:** Mandatory 1-seed smoke before batches, half GPU duty (`DUTY = 0.5`, thermal pause `90 °C`), `<= 3` CPU workers, 1 sim job at a time.
  4. **Failure-frame triage:** Auto-render failure windows (`third`, `bus`, `stereo_l/r`, telemetry) whenever a seed aborts or collides.

### D. Closed-Loop Perception & Domain-Gap Pipeline
- **Observed pain points:** `stereo_net` trained on flat-grey `plain` MuJoCo degraded under `earth_bg=True` (Kapton MLI, brushed metal, Earth limb, stars), causing `4 / 25` `no grip (sensed)` in the `7720000–24` orbital benchmark.
- **Agentic solution:** Compare `E0 -> E1` accuracy across `plain`, `earth_bg=True`, and `optical_fx=True`, and trigger targeted `gen_stereo --earth-bg` dataset generation + fine-tuning only on the failing visual/lighting regimes.
