# Dual-Arm Recessed-Cavity Servicer Concept — Engineering Research & Sizing Study

**Date:** 25 November 2026 (Updated: Centerline `y = ±70 mm` + Standard 15" ESPA Depth-Expanded Layout)  
**CAD Assembly:** [dual_arm_concept.FCStd](file:///c:/Shrey_Projs/robotics/mjorbit/models/concepts/dual_arm_concept/dual_arm_concept.FCStd) ([build_dual_arm_concept.py](file:///c:/Shrey_Projs/robotics/mjorbit/models/concepts/dual_arm_concept/build_dual_arm_concept.py))  
**Scope:** Engineering trade and FreeCAD geometric validation of a **dual diagonally opposed, recessed-cavity short-arm servicer** packed strictly inside a **standard 15" ESPA** rideshare envelope (`711 × 610 × 965 mm`), supporting small-to-medium adapter rings (`8"`–`37"`).

---

## 1. Executive Summary & Why Centerline + Depth-Expansion Wins

Moving the two diagonal arm cavities inward to flank the vertical centerline (**`y = ±70 mm`**, `20 mm` central spine) while extending the bus depth along `+X` from **`500 → 600 mm`** yields a **strict upgrade over both Servicer Rev 2 and our initial wide-bus (`760 mm`) dual-arm layout**:

1. **Fits 100% inside Standard 15" ESPA (`695 × 606 × 951.1 mm` vs. `711 × 610 × 965 mm` limit):**
   - Keeps bus width at `600 mm` (`606 mm` w/ body solar cells, `4 mm` margin).
   - Uses the freed `+X` overhang (Rev 2 wasted `~227 mm` outside the core shell on its folded arm, hitting `727 mm > 711 mm`) to grow the structural bus shell from `500 → 600 mm` deep (`+54.0 L` gross volume). Even after carving out both `130 × 120 × 800 mm` arm cavities (`-25.0 L`), **net internal bus volume is `299.0 L` (+10.7% more internal volume than Rev 2's `270.0 L` shell)**, with **`16.0 mm` depth margin** left inside `711 mm`.
2. **Grabs small rings (`15"` / `381 mm`, even `8"` / `203 mm`) as well as `24"` (`client_p7`) & `37"`:**
   - At `y = ±70 mm`, both grippers stay well inside even smallsat adapters while grasping across the full ring diameter (`381 mm` on `15"`, `609.6 mm` on `24"`).
   - Rim tangents stay nearly flat (`13.3°` on `24"`, `21.6°` on `15"`), allowing a single jaw pre-canted `~15°` with passive `±10°` roll compliance to fit both ring classes without an active wrist-roll motor.
3. **59% lower detumble leverage momentum (`μ d² ω`):**
   - At `420 mm` bus-face standoff (`x_ring = 720 mm`), CoM-to-CoM distance is `d = 0.30 + 0.42 + 0.75 = 1.47 m` (vs `~2.30 m` on Rev 2). At `2 °/s`, `μ d² ω` drops from **`~21.9 N·m·s`** to **`8.9 N·m·s`** (total stack `~11.7 N·m·s`), fitting completely inside four RW-12 reaction wheels (`19.6 N·m·s` X/Y capacity) for **plume-free wheels-only detumble**.
4. **Wide-baseline + local dual stereo perception:**
   - Each gripper carries a `50 mm`-baseline, `128×128` stereo module on its outboard side (`±Y`) so the two grippers clear each other on tight `15"` rings while giving a `~360–610 mm` cross-ring stereo baseline and shadow/glare redundancy.

---

## 2. Standard 15" ESPA Limits & Volumetric Budget (Exact Trade)

Standard Moog 15" ESPA payload interface limits (radial = servicer `X` depth from aft port face; tangential = `Y` width; axial = `Z` height):

| ESPA Axis / Limit | Standard 15" ESPA Spec | Servicer Rev 2 | Centerline Dual-Arm Concept | Margin / Status |
|---|---|---|---|---|
| **Depth `X` (radial from port face)** | **`711 mm`** (`28 in`) | `727 mm` *(exceeds by `16 mm`)* | **`695.0 mm`** (`50` ring + `600` core + `45` cheek lens) | **`+16.0 mm` OK** |
| **Width `Y` (tangential)** | **`610 mm`** (`24 in`) | `606.0 mm` | **`606.0 mm`** (`600` shell + `2 × 3 mm` solar) | **`+4.0 mm` OK** |
| **Height `Z` (axial)** | **`965 mm`** (`38 in`) | `935.0 mm` | **`951.1 mm`** (`900` shell + radiator/STs) | **`+13.9 mm` OK** |
| **CG offset from port face** | **`<= 508 mm`** (`20 in`) | `~310 mm` | **`~365–390 mm`** (core centered at `350 mm` from port) | **`+118–143 mm` OK** |
| **Mass capacity** | `220 kg` heritage / `257 kg` MAC ([Moog](https://moog.com/products/spacecraft-payload-interfaces/espa.html)) | `196.2–196.5 kg` | **`~190–200 kg`** (est.) | **`+20–30 kg` OK** |
| **Net internal bus volume** | — | `270.0 L` (`500×600×900`) | **`299.0 L`** (`600×600×900` minus `25.0 L` bays) | **`+29.0 L` (+10.7%)** |

> **Key packaging insight:** Extending bus depth `+100 mm` along `X` (up to `600 mm` structural core) adds `54.0 L` of interior volume — **`2.16×`** the volume lost to the two `130 × 120 × 800 mm` arm cavities — while keeping the stowed spacecraft smaller along `X` than Rev 2.

---

## 3. Head-to-Head Architecture Comparison

| Feature | Servicer Rev 2 (Baseline) | Centerline Dual-Arm Concept |
|---|---|---|
| **Core bus (`X × Y × Z`)** | `500 × 600 × 900 mm` | **`600 × 600 × 900 mm`** |
| **Arm mounting `(x, y, z)`** | Single shoulder `(313, 0, -250) mm` | **Diagonal centerline:** `(235, +70, +300)` & `(235, -70, -300) mm` |
| **Arm kinematics (`L1 + L2 + jaw`)** | `550 + 450 + 130 = 1180 mm` (6-DOF) | **`300 + 250 + 105 = 655 mm` each** (see Section 5 DOF trade) |
| **Cavity dimensions (`× 2`)** | None (folds flat proud of `+X` face) | `130 (X) × 120 (Y) × 800 (Z) mm`, `20 mm` central spine |
| **RPO Optical Bench (`bus_lidar` + `bus_cam`)** | `+X` cheek at `(250, +200, +225..300)` | **`+X` cheek at `(300, +215, +165..335)`** (outside arm plane) |
| **Supported adapter rings** | `24"` (`609.6 mm`) single-arc `60 mm` grip | **`8"`, `15"`, `24"`, `37"`** antipodal dual-jaw capture |
| **Detumble leverage (`d`, `2 °/s`)** | `d ≈ 2.30 m` → `μd²ω ≈ 21.9 N·m·s` *(RCS unload)* | **`d ≈ 1.47 m` → `μd²ω ≈ 8.9 N·m·s`** *(wheels-only)* |
| **Grip stereo modules** | `1×` (`50 mm` baseline, `128×128`) | **`2×` (`50 mm` outboard per jaw + `360–610 mm` cross-ring)** |

---

## 4. Multi-Ring Geometry & Chord Angles (`y₀ = ±70 mm`)

Moving the cavities from `y₀ = ±215 mm` inward to **`y₀ = ±70 mm`** unlocks multi-class target compatibility with shallow, uniform jaw angles (`RING_X = 720 mm`, `+420 mm` ahead of the `+300 mm` bus face, `L1 = 300 mm`, `L2 = 250 mm`, `GRIP_LEN = 105 mm`):

| Ring Standard | Diameter (`R`) | Contact `z = ±√(R² − 70²)` | Rim Angle `φ = arcsin(70 / R)` | IK Reach / Max (`550 mm`) | Reach Margin | Status |
|---|---|---|---|---|---|---|
| **`8"` Lightband (microsat)** | `203.2 mm` (`101.6`) | `±73.6 mm` | `43.5°` | `441 mm` | `109 mm` | **Grabbable** *(tight cheek clearance)* |
| **`15"` ESPA / Smallsat** | `381.0 mm` (`190.5`) | **`±177.2 mm`** | **`21.6°`** | **`399.4 mm`** | **`150.6 mm`** | **Verified in FreeCAD (`0.00 mm³` ov)** |
| **`24"` Marman (`client_p7`)** | `609.6 mm` (`304.8`) | **`±296.7 mm`** | **`13.3°`** | **`380.0 mm`** | **`170.0 mm`** | **Verified in FreeCAD (`0.00 mm³` ov)** |
| **`37"` Medium-sat** | `939.8 mm` (`469.9`) | `±464.7 mm` | `8.6°` | `414 mm` | `136 mm` | **Grabbable** |

*Practical payoff:* Between `15"`, `24"`, and `37"` satellites, the rim angle `φ` varies only **`8.6°–21.6°`** (`±6.5°` around a `15°` jaw nominal tilt!). A simple **spring-centered passive compliant wrist** handles all three sizes without an active wrist-roll actuator.

---

## 5. Prior Art, Kinematics DOF Trade & Closed-Chain Contact

### 5.1 Literature & Flight Precedents
- **ClearSpace-1 (ESA / ClearSpace):** Symmetrical multi-arm claw (4 arms) wrapping around an uncooperative adapter/satellite; relies on coordinated compliant closing before rigidization ([ESA](https://esa.int/), [ClearSpace](https://clearspace.today/)).
- **Northrop Grumman MRV / DARPA RSGS:** Dual dexterous 7-DOF arms on `+X` deck ([NASA](https://nasa.gov/)); sized for GEO servicing rather than compact ESPA capture.
- **NASA OSAM-1 / FREND & NRL US 10,414,053; ESA e.Deorbit / SENER LAR:** Two-stage Marman ring clamping (soft contact/draw-in + non-backdrivable mechanical lock constraining all 6 DOF).
- **Free-floating dual-arm closed-chain control:** Literature highlights two hazards when grabbing spinning targets with two arms: (a) **asynchronous first-touch kick** deflecting the opposite rim before the second arm latches, and (b) **closed-chain internal strain** (`Chaser → Arm 1 → Ring → Arm 2 → Chaser`) under stiff position control ([arXiv](https://arxiv.org/), [DLR](https://dlr.de/)). Both motivate **impedance/torque-limited soft capture followed by lead-screw rigidization**.

### 5.2 Arm DOF Options
- **Option A — Strict 2-DOF (`Base Pitch + Elbow Pitch`, 4 actuators total):** Couples tip `(X, Z)` with jaw pitch `θ = q₁ + q₂`, leaving no lateral (`Y`) actuation for `±25 mm` station errors. Insufficient alone.
- **Option B — 2-DOF + Parallelogram / Passive Compliant Wrist (`4` actuators total):** Parallelogram links hold palm parallel to `X`; compliant wrist absorbs `±15°` tilt/roll. Minimal actuator count, relies on wide jaw throat (`±20 mm`) + bus RCS for `Y` error.
- **Option C — 3-DOF (`Base Yaw + Base Pitch + Elbow Pitch`) + Passive Compliant Wrist (`6` actuators total) *(Recommended)*:** Same actuator count as Servicer Rev 2 (`3` per arm), absorbs full 3D `±40 mm` station error independently per gripper, and easily adapts across `8"`–`37"` ring diameters while letting the passive wrist eliminate closed-chain fighting.
- **Option D — Full 4-DOF (`Base Yaw/Pitch + Elbow + Active Wrist`, 8 actuators):** Highest mass/cost; reserve only if active multi-angle alignment (`8"`/`43.5°` through `37"`/`8.6°`) is required.

### 5.3 Two-Arm Capture Protocol (Recommended for MuJoCo Validation)
1. **Simultaneous Soft-Impedance Catch:** Both arms track antipodal ring points (`G3` velocity feed-forward `v + ω × r`), dropping joint stiffness (`K ≈ 50–150 N/m`, torque cap `≤ 5–10 N·m`) upon touch detection so Arm 1 cushions impact instead of kicking the ring away from Arm 2.
2. **Synchronized Draw-In & Rigidize:** Once both palms confirm contact (`touch > 5 N`), both non-backdrivable lead-screw jaws clamp simultaneously onto the lip; Arm 1 locks wrist compliance for detumble while Arm 2 holds low-stiffness force regulation (`≤ 20 N`) to prevent closed-chain thermal/calibration binding.

---

## 6. Verified FreeCAD Concept Views & Geometric Checks

All 3 configurations below are built in [dual_arm_concept.FCStd](file:///c:/Shrey_Projs/robotics/mjorbit/models/concepts/dual_arm_concept/dual_arm_concept.FCStd) (`Part` solid kernel checks):
- **`Arms_Stowed`:** `695.0 X × 606.0 Y × 951.1 Z mm` inside `711 × 610 × 965 mm` ESPA box (`0.00 mm³` overlap).
- **`Arms_Deployed_24in` (`609.6 mm` ring):** `q₁ = ±40.6°`, `q₂ = ∓93.1°`, reach `380.0 / 550 mm` (`170.0 mm` margin), **`0.00 mm³` bus/arm overlap**.
- **`Arms_Deployed_15in` (`381.0 mm` ring):** `q₁ = ±20.8°`, `q₂ = ∓87.3°`, reach `399.4 / 550 mm` (`150.6 mm` margin), **`0.00 mm³` bus/arm overlap**.

| Stowed in Centerline Bays + `711×610×965` ESPA Box | Deployed Capturing `24"` (`609.6 mm`) Ring | Deployed Capturing `15"` (`381.0 mm`) Ring |
|---|---|---|
| ![Stowed inside ESPA box](file:///C:/Users/Shrey/.gemini/antigravity/brain/4379e84e-412f-47f4-9862-cc7cc0bd2a71/dual_arm_stowed.png) | ![Deployed on 24in ring](file:///C:/Users/Shrey/.gemini/antigravity/brain/4379e84e-412f-47f4-9862-cc7cc0bd2a71/dual_arm_deployed_24in.png) | ![Deployed on 15in ring](file:///C:/Users/Shrey/.gemini/antigravity/brain/4379e84e-412f-47f4-9862-cc7cc0bd2a71/dual_arm_deployed_15in.png) |

---

## 7. Open Trade Before MuJoCo Prototype
1. **Standoff vs. Central Engine Nozzle Clearance:** Current `L1=300, L2=250 mm` gives `420 mm` between `+X` face and ring plane (great for flat/recessed centers; clients with a `>400 mm` protruding apogee bell need `~350 + 300 mm` links inside an `~840 mm` cavity for `550–600 mm` standoff, which still keeps `μ d² ω ≈ 11 N·m·s` inside wheels-only capacity).
2. **Jaw Throat & `8"` Ring Support:** Confirm minimum target ring size (`15"` smallsats fit effortlessly with `y₀ = ±70 mm`; `8"` requires `43.5°` rim tilt or passive swivel).
3. **Next Verification Step (when ready):** Build a `DualArmServicerSpec` inside `projects/p7_unprepared_client/` (without modifying `simulator/` or `servicer_rev_2/`) and benchmark asynchronous first-touch impedance closing on a `0.5–2 °/s` spinning `client_p7`.
