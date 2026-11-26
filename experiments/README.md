# Experiments

Add new spacecraft configurations, client presets, control policies, and rendering/HIL benchmarks on top of the modular `src/specs/`, `src/sim/`, `src/phases/`, and `src/rl/` abstractions without modifying upstream `simulator/`.

## Adding a Configuration

1. Define a new `ClientSpec` (bus radius, keep-out geometry, `RingFeature` or alternate grapple interface, `WingSpec` appendages, perception priors) in `src/specs/client_spec.py` or an experiment module.
2. Define a `ServicerSpec` (bus JSON / MJCF scene path, thruster/wheel `BusControlSpec`, `ArmSpec`, `ToolSpec`, `SensorRigSpec`) in `src/specs/servicer_spec.py` or a new builder script under `experiments/<name>/`.
3. Evaluate safely before sweeping:
   - Single-seed smoke (`--seeds 7710000`) and static checks (`src/checks/test_modular_specs.py`, `src/checks/phys_rev2.py`, `src/checks/tool_gate.py`) before any multi-seed batch.
   - Keep `<= 3` CPU workers and half GPU duty (`--duty 0.5`, thermal guard `<= 85 C`).
   - Keep `tune` (`7600000+`), `val` (`7710000+`), and frozen `test` (`7720000–24`) seed splits strictly separate.

## Planned Configuration Series

- **`01_dual_arm_espa`** — Verified baseline (`Checks A–E PASS`): centerline (`Y = +/-70 mm`) recessed-cavity dual-arm 15" ESPA servicer (`640 x 600 x 910 mm` bus, `704.3 x 606.0 x 937.4 mm` stowed, `188.5 kg` Spec-MGA / `191.1 kg` MJCF wet mass, 6-DOF Cardan-Roll wrist, `8"`–`37"` ring capture, active F/T compliance, `2 deg/s` wheels-only detumble at `69.3%` RW fill). CAD in [`cad/concepts/dual_arm_espa/`](../cad/concepts/dual_arm_espa/build_dual_arm_concept.py); MuJoCo bench in [`01_dual_arm_espa/bench_dual_arm_capture.py`](01_dual_arm_espa/bench_dual_arm_capture.py).
- **`02_arm_iod_demo`** *(Next)* — `273.0 x 520.0 x 840.0 mm` (`45.6 kg`) hosted-payload IOD MuJoCo scene modeling the compliant 4-boom target plate (`8.2 Hz`), independent `+5 deg` differential pitch tilt, wrist-stereo + `Mini45` F/T admittance capture and retract cycles (CAD in [`cad/concepts/arm_iod_demo/`](../cad/concepts/arm_iod_demo/build_arm_iod_demo.py)).
- **`02_multi_client`** — Symmetric double-wing (`client_double_wing`), alternate mass/inertia classes, and tumbling/nutating target envelopes beyond `2.5 deg/s`.
- **`03_pbr_sim2real`** — Cycles PBR (`src/viz/render_blender.py`) stereo domain-gap evaluation (`200–500` pairs across high-glare Kapton and Earth-limb lighting) paired with a dark-room HIL bench (`docs/COSMOS_SYNTHETIC_DATA_RESEARCH.md`, `docs/EVOLUTION_AND_CRITIQUE.md`).
- **`04_tumbling_and_swarms`** — Compliant 3–6 deg/s tumbling capture, online 3D Gaussian Splatting (3DGS) unknown-object reconstruction, mid-capture FDIR fault injection, and Inspector + Servicer distributed CBF swarms.
