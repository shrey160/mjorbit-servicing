# Dual-Arm Recessed-Cavity Servicer Concept — Engineering Research & Sizing Study

**Date:** 26 November 2026 (Updated v3: Compact `240+200 mm` Arms + `z₀ = ±230 mm` + Swept Arc Reliefs + Bulkhead `q₀`)  
**CAD Assembly:** [dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) ([build_dual_arm_concept.py](../cad/concepts/dual_arm_espa/build_dual_arm_concept.py))  
**Scope:** Engineering trade and FreeCAD solid-kernel validation of a **dual diagonally opposed, compact-arm recessed servicer** with **bulkhead-mounted Base-Roll (`q₀`) drives** and **swept circular arc relief pockets** packed strictly inside a **standard 15" ESPA** rideshare envelope (`711 × 610 × 965 mm`), supporting `8"`–`37"` adapter rings and large `(ΔY, ΔZ)` station offsets at **`0.0°` jaw roll misalignment**.

---

## 1. Executive Summary & Why Compact Arms + Arc Reliefs + Bulkhead `q₀` Win

Shortening the arms by `~20%` (**`L1 = 240 mm, L2 = 200 mm, GRIP_LEN = 105 mm`**) and moving the shoulders inward to **`(248, ±70, ±230) mm`** paired with **`R = 115 mm` swept circular arc relief pockets** and the vertically centered `+Y` RPO optical bench delivers a major upgrade over both Rev 2 and our earlier v1/v2 layouts:

1. **Eliminates the `24"` cylindrical singularity (`rₛ = 240.4 mm` sits cleanly between `15"` and `24"` rings):**
   - Moving the shoulders inward from `z₀ = ±300 → ±230 mm` places the Base-Roll axis radius at **`rₛ = √(70² + 230²) = 240.4 mm`** (equivalent to an `18.9"` diameter) — clear of **`8"` (`rₚ = -138.8 mm`)**, **`15"` (`rₚ = -49.9 mm`)**, **`24"` (`rₚ = +64.4 mm`)**, and **`37"` (`rₚ = +229.5 mm`)**.
   - On `24"` targets (`client_p7`), `q₀` stays non-singular and rotates jaws in the same directional sense as the ring tangent, while **`4× RW-12` reaction wheels** provide zero-propellant `ωₓ` roll-sync during capture.
2. **Swept circular arc pockets (`R_ARC = 115 mm`) unlock `~90°+` outboard `q₀` swing:**
   - Shorter stowed length (`545 mm` vs. `655 mm`) trims the cavities from `800 → 690 mm` (`z ∈ [-345, +345] mm`), restoring `105 mm` of solid bus end structure around the corner RCS thrusters.
   - Moving the `+Y` RPO optical bench (`bus_lidar` + `bus_cam`) down to mid-height (`y = +215 mm, z ∈ [-83, +83] mm`) frees both outboard shoulder corners for `R = 115 mm` swept circular arc pockets (clipped to `|Y| ≥ 10 mm`), keeping `22–32 mm` clearance from the sensor bench and corner RCS clusters while letting jaws track large **`(+80 Y, -50 Z) mm`** ring offsets (`q₀ = -39.8°`).
3. **100% Standard 15" ESPA Compliant (`695.0 × 606.0 × 951.1 mm`) & Exact `0.0°` Jaw Roll Error:**
   - Fits strictly inside `711 × 610 × 965 mm` with **`+16.0 X, +4.0 Y, +13.9 Z mm`** margin while retaining a `600 × 600 × 900 mm` core (`20 mm` structural bulkhead behind cavities, `X ∈ [-275, +150] mm` equipment bay, `R = 44 mm` internal Base-Roll drives at `X = 94..170 mm`).
4. **~64% lower detumble leverage momentum (`μ d² ω`) vs. Rev 2:**
   - At `350 mm` nominal bus-face standoff (`RING_X = 650 mm`), CoM-to-CoM lever arm drops to `d ≈ 1.40 m` (`~7.9 N·m·s` reduced-mass momentum at `2 °/s`, total stack `~10.7 N·m·s`), comfortably inside `4× RW-12` capacity (`19.6 N·m·s`) for **plume-free wheels-only detumble**.

---

## 2. Standard 15" ESPA Limits & Architecture Comparison

| Feature / Limit | Standard 15" ESPA | Servicer Rev 2 (Baseline) | Dual-Arm v3 (Compact + Arc + `q₀`) |
|---|---|---|---|
| **Stowed Envelope (`X × Y × Z`)** | `711 × 610 × 965 mm` | `727 × 606 × 935 mm` *(`+16 mm` over X)* | **`695.0 × 606.0 × 951.1 mm`** *(`+16 / +4 / +13.9 mm` OK)* |
| **Core bus shell** | — | `500 × 600 × 900 mm` | **`600 × 600 × 900 mm`** (`20 mm` bulkhead + hollowed bay) |
| **Shoulder mount `(x, y, z)`** | — | `(313, 0, -250) mm` | **`(248, +70, +230)` & `(248, -70, -230) mm`** (`rₛ = 240.4 mm`) |
| **Arm kinematics per arm** | — | 6-DOF single (`550 + 450 + 130 mm`) | **4-DOF `Rₓ – Rᵧ – Rᵧ – Rᵧ` (`q₀` + `240+200` + `q₃` + `105 mm` jaw)** |
| **Cavity & relief geometry** | — | None (proud of `+X`) | `130 (X) × 120 (Y) × 690 (Z) mm` + **`R = 115 mm` swept arc** + `10 mm` bevel |
| **RPO optical bench (`bus_lidar`+`cam`)** | — | `(250, +200, +225..300)` | **`(300, +215, -83..+83) mm`** (vertically centered `+Y` cheek) |
| **Supported adapter rings** | — | `24"` (`609.6 mm`) single-arc | **`8"`, `15"`, `24"`, `37"`** + large `(ΔY, ΔZ)` offsets at **`0.0°` roll error** |
| **Detumble leverage (`d`, `2 °/s`)** | — | `d ≈ 2.30 m → 21.9 N·m·s` *(RCS unload)* | **`d ≈ 1.40 m → ~7.9 N·m·s`** *(wheels-only)* |
| **Grip stereo modules** | — | `1×` (`50 mm`, `128×128`) | **`2×` (`50 mm` outboard per jaw + `203–940 mm` cross-ring)** |

---

## 3. Kinematics, Swept Arc Pocket & Capture Protocol

### 3.1 `Rₓ – Rᵧ – Rᵧ – Rᵧ` Radial Alignment + Swept Arc Relief
- **Zero-twisting `q₀` Base Roll:** Fixed harmonic drives (`R = 44 mm` barrel, `R = 36 mm` encoder, `X = 94..170 mm`) drive `R = 38 mm` turntables + twin-ear clevis yokes (`X = 171..269 mm`), aiming both `L1 + L2` arm planes directly through any estimated ring center `(cᵧ, c_z)`.
- **Why the swept arc pocket matters:** Clipping an `R = 115 mm` bore centered at `(±70, ±230)` to `|Y| ≥ 10 mm` gives continuous conical clearance up to **`|q₀| > 45°`**, blending tangentially into `z = ±345 mm` while keeping the `20 mm` central spine (`10 mm` front bevel bridge), corner thrusters, and mid-cheek LiDAR/camera bench untouched.
- **No-singularity tracking:** With `rₛ = 240.4 mm`, every standard adapter (`8"`, `15"`, `24"`, `37"`) stays `≥ 50 mm` away from `rₚ = 0`. During capture, `4× RW-12` wheels execute zero-propellant `X`-axis roll-sync while `q₁, q₂, q₃` keep the gripper parallel to `+X` under soft impedance (`K ≈ 50–150 N/m`) before dual lead-screw rigidization.

---

## 4. Verified FreeCAD v3 Results & Views

All 6 configurations below pass `Part` solid boolean assertions in [dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) with **`0.00 mm³` arm/bus, arm/arm, arm/ring, and internal overlap**:

| Configuration | Ring `X` (Standoff) | Base Roll `q₀` (RH `+X`) | `q₁` / `q₂` / `q₃` (Arm 1, Arm 2) | Reach / `440 mm` | Jaw Roll Err | Overlaps (`mm³`) |
|---|---|---|---|---|---|---|
| **`Arms_Stowed`** | — | `0.0°` | Flush inside `690 mm` bays | `695.0 × 606.0 × 951.1 mm` | N/A | **`0.00`** *(+`16 / +4 / +13.9 mm` ESPA)* |
| **`24"` Ring (`609.6 mm`)** | `650 mm` (`+350`) | `-16.9°` / `-16.9°` | `+53.3° / -93.1° / +39.8°` | `303.9 mm` (`136.1 mm` margin) | **`0.0°`** | **`0.00`** |
| **`15"` Ring (`381.0 mm`)** | `650 mm` (`+350`) | `-16.9°` / `-16.9°` | `+31.9° / -94.1° / +62.2°` | `301.2 mm` (`138.8 mm` margin) | **`0.0°`** | **`0.00`** |
| **`8"` Ring (`203.2 mm`)** | `650 mm` (`+350`) | `-16.9°` / `-16.9°` | `+12.3° / -84.1° / +71.8°` | `327.8 mm` (`112.2 mm` margin) | **`0.0°`** | **`0.00`** |
| **`37"` Ring (`939.8 mm`)** | `690 mm` (`+390`) | `-16.9°` / `-16.9°` | `+54.3° / -44.4° / -9.9°` | `407.7 mm` (`32.3 mm` margin) | **`0.0°`** | **`0.00`** |
| **`24"` Large Offset (`+80 Y, -50 Z`)** | `650 mm` (`+350`) | **`+2.0°`** / **`-39.8°`** | `+46.7° / -95.3° / +48.6°` *(A1)* · `+54.2° / -92.6° / +38.4°` *(A2)* | `298.0` / `305.3 mm` | **`0.0°`** | **`0.00`** |

> *Inspection tip:* Open [dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) in FreeCAD and toggle `Bus_SideSlice_Cutaway`, `Bus_Internal_Ref`, `ESPA_Envelope_Ref`, `Arms_Stowed`, `Arms_Deployed_24in`, `Arms_Deployed_15in`, `Arms_Deployed_8in`, `Arms_Deployed_37in`, and `Arms_Deployed_24in_LargeOff` to inspect each configuration interactively.

---

## 5. Next Step: MuJoCo Dual-Arm Capture Benchmark (`experiments/01_dual_arm_espa`)
1. Export per-part `.step`/`.stl` geometry (`mm → m`), inertia tensors, and actuator limits (`q₀..q₃` per arm + `4× RW-12` roll-sync) into a `DualArmServicerSpec` / MJCF scene under `experiments/01_dual_arm_espa/`.
2. Benchmark asynchronous first-touch soft impedance catch + wheel roll-sync on `client_p7` (`0.5–2 °/s` spin) without modifying upstream `simulator/`.
