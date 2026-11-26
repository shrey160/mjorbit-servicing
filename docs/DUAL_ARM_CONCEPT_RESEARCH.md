# Dual-Arm Recessed-Cavity Servicer Concept (`v4.1`) — Engineering Research & Verification Study

**Date:** 26 November 2026 (`v4.1` Locked Baseline: Hybrid `275+235 mm` Arms + `480–500 mm` Standoff + Recessed Sensors + Sunk `55°` RCS)  
**CAD Assembly:** [dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) ([build_dual_arm_concept.py](../cad/concepts/dual_arm_espa/build_dual_arm_concept.py))  
**Verification Suite:** [verify_v4_engineering.py](../cad/concepts/dual_arm_espa/verify_v4_engineering.py)  
**Scope:** Hardware-grounded sizing and solid-kernel / analytical verification of an experimental **dual diagonally opposed, compact-arm recessed 15" ESPA servicer** using flight-heritage COTS subsystems (`MLB15.000`, `RW-12`, `ECAPS 1N`, `ASC DragonEye`, `3D PLUS IRIS`, `ST-16RT2`, `NEA 9102`) and custom-buildable hollow-shaft `CSD-20/17` joints + passive `Ti-6Al-4V` wrist flexure compliance.

---

## 1. Executive Summary (`v4.1` Locked Architecture)

Recessing the forward RPO sensors and top-deck star trackers inside the bus skin unlocks enough depth and height inside the **standard 15" ESPA envelope (`711 × 610 × 965 mm`)** to pair a mild link stretch (`275 + 235 mm`) with adaptive `480–500 mm` standoff, solving every `v3` critique cleanly:

1. **Sensor Recessing Expands Core Bus (`600×600×900` → `640×600×910 mm`) on `42.2 mm` `MLB15.000`:**
   - **Recessed `+Y` Optical Bay (`X = 185..332 mm`):** Embeds an **ASC DragonEye** `128×128` flash LiDAR (`3.6 kg`) and **3D PLUS IRIS** camera (`0.5 kg`) into the `+Y` cheek ahead of the intact `20 mm` front shear bulkhead (`X = 165..185 mm`), with only a `+12 mm` bezel proud of `+320 mm`.
   - **Chamfer/Deck-Recessed `+Z` Star Trackers (`2× ST-16RT2` / `Auriga`):** Optical barrels embedded at `(-130, ±175, 455) mm` tilted `±20°` outward (`+14 mm` baffle rim proud, `70°` clear of `≥ 34°` sun exclusion cones, fitting cleanly inside symmetric `Z ∈ [-482.5, +482.5] mm`).
   - **Sunk Forward RCS Pods (`5 mm` deeper):** Recesses the `55°` canted corner nozzle bells inside the front corners (`XMax = +342.1 mm`), giving a **`704.3 × 606.0 × 937.4 mm`** total stowed envelope (**`+6.7 X, +4.0 Y, +27.6 Z mm`** ESPA margins; `460 mm` internal equipment bay).
2. **Hybrid Compact Arms (`275 + 235 + 105 mm`) & `480–500 mm` Adaptive Standoff:**
   - Centerline-flanking shoulders at **`(265, ±70, ±245) mm`** (`rₛ = 254.8 mm`, `20.1"` eq. diameter) eliminate the `24"` `rₚ ≈ 0` singularity while leaving `75 mm` solid top/bottom end structure outside `760 mm` cavities (`z ∈ [-380, +380]`) and `R = 115 mm` swept arc reliefs.
   - Reaches **`8"`, `15"`, `24"`, `37"`, and `(+80 Y, -50 Z)` `24"` offset** rings at **`480 mm` nominal standoff** (`RING_X = +800 mm`, up to `500 mm` on `24"` GEO targets), leaving **`+130 to +150 mm` axial clearance** past protruding `350 mm` GEO apogee engine bells (`ArianeGroup S400 / LEROS` class), **`39°–65°` elbow dexterity**, and **`0.0°` jaw roll error**.
3. **100% Plume-Free Wheels-Only Detumble & Full-Rank Fault-Tolerant `16 × 1 N` RCS:**
   - **RCS (`55°` fwd / `30°` aft cant):** Rank `6/6` nominally (`+Fx 3.464 N, -Fx 2.294 N, ±Fy 2.822 N, ±Fz 3.247 N`) and across **all 16 single-thruster failures** (`min -Fx = 1.721 N > 1.50 N` `F_MAX`), with braking plumes **`32.5°–74.8°` clear** of `client_p7` (`≥ 30°` NASA keep-out).
   - **Wheels (`4 × RW-12`, `β = 65°` skew):** `Hx = 20.29 N·m·s (0.338 N·m)`, `Hy = Hz = 21.75 N·m·s (0.363 N·m)`. At `2 °/s` (`d = 1.624 m`), total stack detumble demand is **`14.16 N·m·s` (`65.1%` fill)** — below the `80%` unload threshold for plume-free detumble.
4. **Launch-Ready Mass (`188.5 kg` Spec-MGA), CG (`348.4 mm`) & Passive Wrist Compliance:**
   - Under `15 g` ESPA loads (`956 N` per `6.5 kg` arm), **`2× EBAD NEA 9102`** separation nuts (`17.8 kN` preload) + cup-cone nests inside the cavities bypass launch loads into the `20 mm` bulkhead with **`12.4×` FoS**.
   - A `14 mm` monolithic `Ti-6Al-4V` tangential flexure collar (`±5°` hard-stopped) + `5 mm` knitted Inconel wire-mesh jaw pads absorb client nutation without vacuum-outgassing elastomers or a 5th motor.

---

## 2. Standard 15" ESPA Limits & Rev-2 vs. `v4.1` Comparison

| Feature / Limit | Standard 15" ESPA | Servicer Rev 2 (Baseline) | Dual-Arm `v4.1` (Verified & Locked) |
|---|---|---|---|
| **Stowed Envelope (`X × Y × Z`)** | `711 × 610 × 965 mm` | `727 × 606 × 935 mm` *(`+16 mm` over X)* | **`704.3 × 606.0 × 937.4 mm`** *(`+6.7 / +4.0 / +27.6 mm`)* |
| **Core Bus & Separation Ring** | `CG ≤ 508 mm`, `≤ 220 kg` | `500 × 600 × 900 mm`, `50 mm` ring, `196.2 kg` | **`640 × 600 × 910 mm`**, `42.2 mm` `MLB15`, **`188.5 kg`**, **`CG = 348.4 mm`** |
| **Shoulder Mount `(x, y, z)`** | — | `(313, 0, -250) mm` | **`(265, ±70, ±245) mm`** (`rₛ = 254.8 mm`) |
| **Arm Kinematics** | — | 6-DOF single (`550+450+130 mm`) | **2× 4-DOF `Rₓ–Rᵧ–Rᵧ–Rᵧ` (`q₀` + `275+235` + `105 mm` jaw + Ti flex)** |
| **Cavity & Launch Locks** | — | None (exposed on `+X`) | `135 × 120 × 760 mm` + **`R = 115 mm` swept arc** + **`2× NEA 9102`** |
| **RPO Optics & Star Trackers** | — | Proud `+X` bench + proud `+Z` STs | **Recessed `DragonEye` + `IRIS` (`+12 mm`) & `±20°` STs (`+14 mm`)** |
| **Nominal Standoff & Cant** | — | `~600 mm` / `50°` fwd RCS | **`480–500 mm`** (`+130–150 mm` past `350 mm` GEO LAE) / **`55°` fwd RCS** |
| **Wheels (`4× RW-12`) & Detumble** | — | `β = 70°`, `d ≈ 2.3 m → ~22–26 N·m·s` *(unload)* | **`β = 65°`**, `d = 1.624 m →` **`14.16 N·m·s` (`65.1%`, wheels-only)** |

---

## 3. Subsystem Mass, CG & Hardware Basis Rollup (`v4.1`)

*(Measured from aft `MLB15.000` port plane `X_port = 0`; `CG` limit `≤ 508.0 mm`; 15" ESPA limit `≤ 220.0 kg` Heritage / `≤ 200.0 kg` SpaceX full-plate.)*

| Subsystem | Hardware Basis | Base (kg) | MGA | Total (kg) | `X_CG` (mm) |
|---|---|---:|---:|---:|---:|
| Structure shell + `20 mm` front bulkhead + decks | Custom Al-7075 / honeycomb | `48.0` | `1.20` | `57.6` | `355.0` |
| MLI + `-Z` radiator + thermal straps/heaters | Kapton MLI + OSR radiator | `9.5` | `1.20` | `11.4` | `360.0` |
| Separation ring (spacecraft half, `42.2 mm`) | Rocket Lab `MLB15.000` | `2.2` | `1.10` | `2.4` | `21.1` |
| `16 × 1 N` thrusters + `8` corner pods | ECAPS 1N HPGP class | `8.9` | `1.15` | `10.2` | `362.0` |
| COPV tank + lines/valves (dry) | Overwrapped tank + valves | `6.0` | `1.20` | `7.2` | `240.0` |
| Propellant (`~54 m/s` RPO `Δv`) | `LMP-103S` green monoprop | `4.5` | `1.00` | `4.5` | `240.0` |
| `4 × RW-12` (`65°`) + wheel electronics | Rocket Lab `RW-12` | `21.5` | `1.10` | `23.7` | `270.0` |
| `2×` Star Trackers + tactical IMU | `ST-16RT2` / `Auriga` + `STIM300` | `1.3` | `1.10` | `1.4` | `250.0` |
| RPO Optics (recessed bay) | `ASC DragonEye` + `3D PLUS IRIS` | `6.0` | `1.15` | `6.9` | `595.0` |
| Power (`±Y` arrays + battery) & Avionics/Harness | Body cells + Li-ion + OBC/GPU | `37.8` | `1.16` | `44.0` | `282.0` |
| `2×` Bulkhead `q₀` Base-Roll drives | `CSD-20` + `TBM2G-060` + `Ø20 mm` bore | `2.8` | `1.20` | `3.4` | `495.0` |
| `2×` Compact 4-DOF arms + Ti flex + Marman jaws + stereo | `CSD-17/14` + CFRP + `2× 50 mm` stereo | `11.6` | `1.20` | `13.9` | `610.0` |
| `2×` Non-pyro HDRM launch locks | `EBAD NEA 9102` + cup-cone nests | `1.6` | `1.20` | `1.9` | `620.0` |
| **TOTAL `v4.1` SERVICER** | **Heritage ESPA margin: `+31.5 kg`** | **`161.7`** | **`1.17`** | **`188.5`** | **`348.4`** |

*(Even with a conservative flat `20%` dry MGA across every subsystem, wet mass is `193.1 kg` — `+26.9 kg` under `220 kg` and `+6.9 kg` under `200 kg`.)*

---

## 4. Analytical & FreeCAD Solid-Kernel Verification Results

### 4.1 Analytical Verification ([verify_v4_engineering.py](../cad/concepts/dual_arm_espa/verify_v4_engineering.py))

| Check | Verified Quantitative Result | Status |
|---|---|---|
| **6-DOF RCS (Nominal)** | Rank `6/6`; `+Fx 3.464`, `-Fx 2.294`, `±Fy 2.822`, `±Fz 3.247 N`; `Mx 1.578`, `My 1.144`, `Mz 0.765 N·m` | **PASS** |
| **6-DOF RCS (All 16 1-Fail)** | Worst single-thruster failure stays Rank `6/6`; min `-Fx = 1.721 N` (`> 1.50 N` `F_MAX`), `Fy 0.894 N`, `Fz 1.808 N` | **PASS** |
| **Close-Range Plumes (`480 mm`)** | `55°` forward cant: `74.79°` to Client CoM, `73.64°` to ring rim, `32.49°` to nearest `1×1 m` client corner (`≥ 30°`) | **PASS** |
| **`4 × RW-12` (`65°`) Detumble** | `Hx 20.29 / Hy=Hz 21.75 N·m·s`; `2 °/s` stack = `14.16 N·m·s` (`65.1%` fill `< 80%` → **plume-free**) | **PASS** |
| **Spin-Rate Gate (`hold3 = 3.2 m`)** | Analytic worst-axis ceiling `ω_max = 2.08 °/s` at `F_MAX = 1.5 N` (`1.40 N` needed for `2.00 °/s`) | **PASS** |
| **Sensor FoV & GEO Nozzle** | `+130.0 mm` (`+150.0 mm` @ `500 mm`) past `350 mm` LAE; `cam_40deg` clears `24"/37"/15"/LargeOff` (`26.2–46.8°`); `lidar_45deg` clears `24"/37"/LargeOff` (`25.3–41.8°`), edging `8"/15"` (`17.8–19.5°`) inside close-range stereo handover; STs `70° ≥ 34°` sun | **PASS** |

### 4.2 FreeCAD `Part` Solid-Kernel Verification ([dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd))

All solid intersections across `Bus`, `Bus_Internal_Ref` (`COPV` + `65° RW` + avionics), `Arms_Stowed`, `GEO_LAE_Nozzle_350mm`, and all 5 deployed configurations report **`0.00 mm³` overlap**:

| Configuration | Ring `X` (Standoff) | `q₀` (`+X`) | `q₁` / `q₂` / `q₃` (Arm 1, Arm 2) | Reach / `510 mm` | Roll Err | Overlaps (`mm³`) |
|---|---|---|---|---|---|---|
| **`Arms_Stowed`** | — | `0.0°` | Flush inside `760 mm` bays | `704.3 × 606.0 × 937.4 mm` | N/A | **`0.00`** *(all checks)* |
| **`24"` + `350 mm` GEO Nozzle** | `800 mm` (`+480`) | `-15.9°` | `+35.9° / -64.1° / +28.2°` | `432.9 mm` (`+77.1 mm`) | **`0.0°`** | **`0.00`** *(incl. nozzle)* |
| **`15"` Ring (`381.0 mm`)** | `800 mm` (`+480`) | `-15.9°` | `+20.4° / -63.2° / +42.9°` | `434.8 mm` (`+75.2 mm`) | **`0.0°`** | **`0.00`** |
| **`8"` Ring (`203.2 mm`)** | `800 mm` (`+480`) | `-15.9°` | `+4.7° / -53.1° / +48.4°` | `456.5 mm` (`+53.5 mm`) | **`0.0°`** | **`0.00`** |
| **`37"` + `350 mm` GEO Nozzle** | `800 mm` (`+480`) | `-15.9°` | `+44.5° / -39.1° / -5.4°` | `480.8 mm` (`+29.2 mm`) | **`0.0°`** | **`0.00`** *(incl. nozzle)* |
| **`24"` LargeOff (`+80 Y, -50 Z`)** | `800 mm` (`+480`) | `+1.9°` / **`-37.6°`** | `+31.0° / -65.2° / +34.2°` *(A1)* · `+36.8° / -63.6° / +26.8°` *(A2)* | `430.1` / `434.0 mm` | **`0.0°`** | **`0.00`** |

*(Additional GEO-nozzle mode verified analytically: `24"` @ `500 mm` standoff → `q = (-15.9°, +31.5°, -55.0°, +23.5°)`, `reach = 452.8 / 510 mm (+57.2 mm)`, `+150.0 mm` nozzle clearance.)*

---

## 5. Next Step: MuJoCo Dual-Arm Prototype (`experiments/01_dual_arm_espa`)

1. Export per-part `.step`/`.stl` geometry (`mm → m`), `v4.1` inertia tensors, `55°/30°` thruster table, and `65° RW-12` pyramid into a `DualArmServicerSpec` / MJCF scene under `experiments/01_dual_arm_espa/`.
2. Benchmark asynchronous first-touch soft impedance catch + passive `±5°` wrist flexure compliance + `4× RW-12` roll-sync on `client_p7` and GEO-nozzle variants without modifying upstream `simulator/`.
