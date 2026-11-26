# Dual-Arm Recessed-Cavity Servicer Concept (`v4.2`) — Engineering Research & Verification Study

**Date:** 26 November 2026 (`v4.2` Verified Baseline: 6-DOF Compact Intersecting Cardan-Roll Wrist + `ATI Mini45` F/T + `248+208+128 mm` Arms + `480–500 mm` Standoff + Recessed Sensors + Sunk `55°` RCS)  
**CAD Assembly:** [dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd) ([build_dual_arm_concept.py](../cad/concepts/dual_arm_espa/build_dual_arm_concept.py))  
**Engineering Verification Suite:** [verify_v4_engineering.py](../cad/concepts/dual_arm_espa/verify_v4_engineering.py)  
**MuJoCo Simulation Suite:** [bench_dual_arm_capture.py](../experiments/01_dual_arm_espa/bench_dual_arm_capture.py) ([build_scene.py](../experiments/01_dual_arm_espa/build_scene.py), [dual_arm_spec.py](../experiments/01_dual_arm_espa/dual_arm_spec.py))  
**Scope:** Hardware-grounded sizing, FreeCAD solid-kernel verification, and MuJoCo physics benchmarking of an experimental **dual diagonally opposed, compact 6-DOF arm recessed 15" ESPA servicer** combining flight-heritage COTS subsystems (`MLB15.000`, `RW-12`, `ECAPS 1N`, `ASC DragonEye`, `3D PLUS IRIS`, `ST-16RT2`, `NEA 9102`) with hollow-shaft `CSD-20/17/14` + `TBM2G-05008` actuators, a compact intersecting **3-DOF Cardan-Roll wrist** + `ATI Mini45 Titanium` F/T wafer, and a `servicer_rev2` v3-inspired form-closure Marman gripper.

---

## 1. Executive Summary (`v4.2` Locked Architecture)

Upgrading from `v4.1` (4-DOF + 1-axis passive flexure) to `v4.2` replaces the passive distal flexure with an **intersecting 3-DOF Hollow-Shaft Serial Cardan-Roll Wrist (`q₃/q₄/q₅`) + 6-axis F/T wafer** while keeping the exact same `704.3 × 606.0 × 937.4 mm` 15" ESPA stowed envelope and `480–500 mm` standoff by trading limb link lengths (`275+235 → 248+208 mm`) into the dexterous wrist/palm stack (`PALM_X = 74 → 128 mm`):

1. **Sensor Recessing Expands Core Bus (`600×600×900` → `640×600×910 mm`) on `42.2 mm` `MLB15.000`:**
   - **Recessed `+Y` Optical Bay (`X = 185..332 mm`):** Embeds an **ASC DragonEye** `128×128` flash LiDAR (`3.6 kg`) and **3D PLUS IRIS** camera (`0.5 kg`) into the `+Y` cheek ahead of the intact `20 mm` front shear bulkhead (`X = 165..185 mm`), with only a `+12 mm` bezel proud of `+320 mm`.
   - **Chamfer/Deck-Recessed `+Z` Star Trackers (`2× ST-16RT2` / `Auriga`):** Optical barrels embedded at `(-130, ±175, 455) mm` tilted `±20°` outward (`+14 mm` baffle rim proud, `70°` clear of `≥ 34°` sun exclusion cones, fitting cleanly inside symmetric `Z ∈ [-482.5, +482.5] mm`).
   - **Sunk Forward RCS Pods (`5 mm` deeper):** Recesses the `55°` canted corner nozzle bells inside the front corners (`XMax = +342.1 mm`), giving a **`704.3 × 606.0 × 937.4 mm`** total stowed envelope (**`+6.7 X, +4.0 Y, +27.6 Z mm`** ESPA margins; `460 mm` internal equipment bay).
2. **Compact 6-DOF Arm (`248 + 208 mm`) + Intersecting 3-DOF Cardan-Roll Wrist (`PALM_X = 128 mm`):**
   - Centerline-flanking shoulders at **`(265, ±70, ±245) mm`** (`rₛ = 254.8 mm`, `20.1"` eq. diameter) eliminate the `24"` `rₚ ≈ 0` singularity while leaving `75 mm` solid top/bottom end structure outside `760 mm` cavities (`z ∈ [-380, +380]`) and `R = 115 mm` swept arc reliefs.
   - Each distal forearm terminates in a compact **intersecting 3-DOF serial Cardan-Roll spherical wrist** (`q₃` pitch `-Y`, `q₄` yaw `+Z`, `q₅` roll `+X` intersecting at `W`) built around **`Harmonic Drive CSD-14-2A`** (100:1, `Ø55 × ~15 mm`, `5.4 N·m` rated / `19 N·m` repeated peak / `31 N·m` momentary peak, `< 1.5 arcmin` zero backlash) + **`Kollmorgen TBM2G-05008`** frameless motors (`Ø50.04 × 19.84 mm`, `Ø24.75 mm` thru-bore, `~23 N·m` @ 100:1) with integral power-off friction holding brakes and an **`ATI Mini45 Titanium`** 6-axis F/T wafer (`Ø45 × 15.7 mm`, `0.092 kg`, `±580 N` shear, `±1,160 N` axial, `±20 N·m` torques).
   - **DLR ROKVISS / CAESAR & DARPA RSGS / NASA OSAM-1 Control Paradigm:** Rather than passive elastomer/flexure compliance (which lowers structural stiffness during post-capture detumble), `v4.2` pairs **zero-backlash active F/T admittance compliance** during asynchronous contact with **power-off mechanical holding brakes + non-backdrivable lead-screw jaw locks** after capture.
   - **`servicer_rev2` v3-Inspired Form-Closure Marman Gripper (`PALM_X = 128 mm`):** Retains pure-radial lead-screw jaw slides (`inboard` crowned multi-diameter finger + `outboard` `20°` undercut wedge hook behind the `15 mm` ring lip) and shifts the `50 mm` global-shutter stereo bar + LED lamp to the `-Y_mj` housing side face so it clears the `+Y_mj` `q₃` pitch drum.
   - Reaches **`8"`, `15"`, `24"`, and `(+80 Y, -50 Z)` `24"` offset** rings at **`480 mm` nominal standoff** (`RING_X = +800 mm`, up to `500 mm` on `24"` GEO targets) and **`37"`** rings at **`450 mm` standoff** (`RING_X = +770 mm`), preserving **`+130 to +150 mm` axial clearance** past protruding `350 mm` GEO apogee engine bells (`+100 mm` on `37"`), **`≥ +21.1 mm` reach margin**, and **`0.0000 deg` 6-DOF orientation error**.
3. **100% Plume-Free Wheels-Only Detumble & Full-Rank Fault-Tolerant `16 × 1 N` RCS:**
   - **RCS (`55°` fwd / `30°` aft cant):** Rank `6/6` nominally (`+Fx 3.464 N, -Fx 2.294 N, ±Fy 2.854 N, ±Fz 3.264 N`) and across **all 16 single-thruster failures** (`min -Fx = 1.721 N > 1.50 N` `F_MAX`), with braking plumes **`32.49°–74.79°` clear** of `client_p7` (`≥ 30°` NASA keep-out).
   - **Wheels (`4 × RW-12`, `β = 65°` skew):** `Hx = 20.29 N·m·s (0.338 N·m)`, `Hy = Hz = 21.75 N·m·s (0.363 N·m)`. At `2 °/s` (`d = 1.620 m`, `μ = 116.8 kg`), total stack detumble demand is **`14.20 N·m·s` (`65.3%` fill)** — verified in MuJoCo at **`69.3%` peak fill, `71.5 s` detumble, `0` plume pulses**.
4. **Launch-Ready Mass (`191.1 kg` Spec-MGA), CG (`352.0 mm`) & Verified MuJoCo Physics (Checks A–E):**
   - Under `15 g` ESPA loads (`1,104 N` per `7.5 kg` arm), **`2× EBAD NEA 9102`** separation nuts (`17.8 kN` preload) + cup-cone nests at `Z = ∓145.0 mm` (clear of `q₄` yaw drums) bypass launch loads into the `20 mm` bulkhead with **`10.8×` FoS**.
   - All five MuJoCo closed-loop physics benchmarks (`A–E`) pass cleanly: exact `(0.0000 mm, 0.0000 deg)` 6-DOF FK/IK parity, blind `2°` async first-touch capture + `3 N·m` hold, `1 °/s` roll-sync detumble (`21.4 s`), `2 °/s` oblique wheels-only detumble (`71.5 s`, `69.3%` RW fill), and `5° pitch + 5° yaw` 3D-tilted capture + `5.0 N·m` hold.

---

## 2. Standard 15" ESPA Limits & Rev-2 vs. `v4.1` vs. `v4.2` Comparison

| Feature / Limit | Standard 15" ESPA | Servicer Rev 2 (Baseline) | Dual-Arm `v4.1` (4-DOF + Flex) | Dual-Arm `v4.2` (6-DOF Cardan Wrist) |
|---|---|---|---|---|
| **Stowed Envelope (`X × Y × Z`)** | `711 × 610 × 965 mm` | `727 × 606 × 935 mm` *(`+16 mm` over X)* | `704.3 × 606.0 × 937.4 mm` | **`704.3 × 606.0 × 937.4 mm`** *(`+6.7 / +4.0 / +27.6 mm`)* |
| **Core Bus & Separation Ring** | `CG ≤ 508 mm`, `≤ 220 kg` | `500 × 600 × 900 mm`, `50 mm` ring, `196.2 kg` | `640 × 600 × 910 mm`, `188.5 kg`, `CG 348.4 mm` | **`640 × 600 × 910 mm`**, `42.2 mm` `MLB15`, **`191.1 kg`**, **`CG = 352.0 mm`** |
| **Arm Kinematics** | — | 6-DOF single (`550+450+130 mm`) | 2× 4-DOF (`275+235` + `74 mm` palm + passive Ti flex) | **2× 6-DOF (`248+208` + intersecting `q₃/q₄/q₅` Cardan wrist + `Mini45` F/T + `128 mm` palm)** |
| **3D Orientation & Compliance** | — | Active 6-DOF ball wrist | Passive `±5°` tangential flexure only | **Full `q₃/q₄/q₅` (`±160° / ±90° / ±180°`) + active F/T admittance + holding brakes** |
| **Cavity & Launch Locks** | — | None (exposed on `+X`) | `2× NEA 9102` (`12.4×` FoS) | `135 × 120 × 760 mm` + `R = 115 mm` arc + **`2× NEA 9102` @ `Z = ∓145 mm` (`10.8×` FoS)** |
| **Nominal Standoff & Cant** | — | `~600 mm` / `50°` fwd RCS | `480–500 mm` (`455 mm` on `37"`) / `55°` | **`480–500 mm`** (`450 mm` on `37"`; `+100–150 mm` past `350 mm` LAE) / **`55°` fwd RCS** |
| **Wheels (`4× RW-12`) & Detumble** | — | `β = 70°`, `~22–26 N·m·s` *(unload)* | `β = 65°`, `14.16 N·m·s` (`65.1%`) | **`β = 65°`**, `d = 1.620 m →` **`14.20 N·m·s` (`65.3%` analytic / `69.3%` MuJoCo)** |

---

## 3. Subsystem Mass, CG & Hardware Basis Rollup (`v4.2`)

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
| `2×` Compact 6-DOF arms (`q₁–q₅`) + `Mini45` F/T + jaws + stereo | `CSD-17/14-2A` + `TBM2G-05008` + CFRP + Marman jaws | `13.8` | `1.20` | `16.6` | `610.0` |
| `2×` Non-pyro HDRM launch locks | `EBAD NEA 9102` + cup-cone nests | `1.6` | `1.20` | `1.9` | `620.0` |
| **TOTAL `v4.2` SERVICER** | **SpaceX margin: `+8.9 kg` · Heritage: `+28.9 kg`** | **`163.9`** | **`1.17`** | **`191.1`** | **`352.0`** |

*(Even with a conservative flat `20%` dry MGA across every subsystem, wet mass is `195.8 kg` — `+24.2 kg` under `220 kg` and `+4.2 kg` under `200 kg`, with `CG = 352.0 mm` providing `+156.0 mm` margin inside `508.0 mm`.)*

### 3.2 Hardware Options Matrix (COTS & Custom-Buildable Alternatives)

| Subsystem | Primary `v4.2` Baseline | COTS / Custom Alternatives & Trade Notes |
|---|---|---|
| **3-DOF Cardan-Roll Wrist (`q₃/q₄/q₅`)** *(NEW in `v4.2`)* | **Custom Ti/Al Housed `Harmonic Drive CSD-14-2A`** (100:1, `Ø55 × ~15 mm`, `0.06 kg` core, `5.4 / 19 / 31 N·m`, `< 1.5 arcmin` zero backlash) + **`Kollmorgen TBM2G-05008`** (`Ø50.04 × 19.84 mm`, `Ø24.75 mm` thru-bore, `~23 N·m` @ 100:1) + cross-roller bearing + power-off holding brake (`Ø57 × 38 mm` `q₃/q₄`, `Ø57 × 32 mm` `q₅`, `~0.52 kg`/DOF) | COTS housed **Harmonic Drive `SHF-14-2UH`** (`0.41 kg`, `23/47 N·m` w/ integrated cross-roller bearing — heavier/longer housing); **Maxon EC Frameless DT 50**; sequential inline roll-pitch-yaw alternatives rejected for longer lever arm and inability to center intersecting axes behind `PALM_X = 128 mm`. |
| **Wrist F/T Sensing & Compliance Control** *(NEW in `v4.2`)* | **`ATI Mini45 Titanium`** 6-axis F/T wafer (`Ø45 × 15.7 mm`, `0.0917 kg`, thru-hole routing, `±580 N` shear, `±1,160 N` axial, `±20 N·m` torques) running **DLR ROKVISS/CAESAR-style active F/T impedance** at touch + **DARPA RSGS/OSAM-1 power-off brake lock** post-capture | Replaces `v4.1` passive `Ti-6Al-4V` `±5°` flexure, giving programmable softness at asynchronous contact without sacrificing post-capture structural stiffness under `5 N·m` hold torques and `2 °/s` stack detumble. |
| **Form-Closure Marman Gripper** *(`servicer_rev2` v3 Heritage)* | Pure-radial twin linear rails + non-backdrivable lead screws (`q_u, q_l`), `PALM_X = 128 mm` datum, crowned inboard finger (`8"`–`37"` clearance), outboard `20°` undercut wedge pad (`4–8 mm` Marman draw), `-Y_mj` `50 mm` global-shutter stereo PCB (`2× OV9281/PYTHON 300`, `10°` inward pitch) + LED lamp | Keeps verified `servicer_rev2` v3 contact ratios while shifting gripper `+54 mm` past the `q₃/q₄/q₅` wrist + `Mini45` stack (`X_w = 78..166 mm`). Camera bar sits on `-Y_mj` opposite the `+Y_mj` `q₃` drum with zero self-occlusion. |
| **15" Separation Ring** | **Rocket Lab `MLB15.000`** (`42.2 mm` height, `~2.2 kg` sep half) | **Beyond Gravity `PAS 381S`** (`79 mm`, `0.98 kg` sep half, `<100 g` clampband; trims bus `X` depth by `37 mm`). |
| **RPO Flash LiDAR & Far Camera** *(recessed `+Y`)* | **ASC DragonEye** (`128×128`, `45°`, `3.6 kg`) + **3D PLUS IRIS** (`86×72×69 mm`, `12.5°–80°`) | **Custom `128×128` SPAD Flash LiDAR** (`1.8–2.5 kg`); **Redwire Argus V2**; **Malin ECAM-M50**. |
| **Star Trackers (`2×`)** *(deck-recessed `±20°`)* | **Rocket Lab `ST-16RT2`** (`185 g`, `34°` sun exclusion, `70°` clear) | **Sodern Auriga** (`225 g`); **Jena-Optronik ASTRO CL** (`310 g`). |
| **Reaction Wheels (`4×`)** *(`65°` pyramid)* | **Rocket Lab `RW-12`** (`12 N·m·s`, `0.2 N·m`, `~5 kg`) | **Bradford W18** (`18 N·m·s`, `0.265 N·m`); **Honeywell HR14-25** (`25 N·m·s`). |
| **RCS Thrusters (`16 × 1 N`)** | **ECAPS `1N HPGP`** (`LMP-103S`, `204–231 s`, `0.38 kg`) | **Nammo `MHT-1N`** (hydrazine, `228 s`); **Moog `0.9 N GN2`** cold gas. |
| **Hollow-Shaft `q₀` & `q₁–q₂` Drives** | **Custom Ti/Al `CSD-20/17` + Kollmorgen `TBM2G`** (`1.4 kg` `q₀`, `Ø20 mm` bore) | **Harmonic Drive `SHF-20/17-2UH`** (`+1.5 kg` steel penalty). |
| **Launch Locks / HDRMs (`2×`)** | **EBAD `NEA 9102`** (`130 g`, `Ø43×39 mm`, `17.8 kN`, `10.8×` FoS at `Z = ∓145 mm`) | **`2–3×` EBAD Frangibolt `FD04`** per arm (`10 g`, `667 N`); **EBAD TiNi Pin Puller `2545`** (`182 g`). |

---

## 4. Analytical & FreeCAD Solid-Kernel Verification Results

### 4.1 Analytical Verification ([verify_v4_engineering.py](../cad/concepts/dual_arm_espa/verify_v4_engineering.py))

| Check | Verified Quantitative Result | Status |
|---|---|---|
| **Mass, CG & `15 g` Launch** | Base `163.9 kg`, Spec-MGA `191.1 kg`, Flat-20% MGA `195.8 kg`; `CG = 352.0 mm ≤ 508.0 mm`; `1,104 N` per-arm `15 g` load → `NEA 9102` `10.8×` FoS | **PASS** |
| **6-DOF RCS (Nominal)** | Rank `6/6`; `+Fx 3.464`, `-Fx 2.294`, `±Fy 2.854`, `±Fz 3.264 N`; `Mx 1.578`, `My 1.144`, `Mz 0.765 N·m` | **PASS** |
| **6-DOF RCS (All 16 1-Fail)** | Worst single-thruster failure stays Rank `6/6`; min `-Fx = 1.721 N` (`> 1.50 N` `F_MAX`), `Fy 0.904 N`, `Fz 1.818 N`, `M_min = 0.269 N·m` | **PASS** |
| **Close-Range Plumes (`480 mm`)** | `55°` forward cant: `74.79°` to Client CoM, `73.64°` to ring rim, `32.49°` to nearest `1×1 m` client corner (`≥ 30°`) | **PASS** |
| **`4 × RW-12` (`65°`) Detumble** | `Hx 20.29 / Hy=Hz 21.75 N·m·s`; `2 °/s` stack (`d = 1.620 m, μ = 116.8 kg`) = `14.20 N·m·s` (`65.3%` fill `< 80%`) | **PASS** |
| **Spin-Rate Gate (`hold3 = 3.2 m`)** | Analytic worst-axis ceiling `ω_max = 2.07 °/s` at `F_MAX = 1.5 N` (`1.42 N` needed for `2.00 °/s`) | **PASS** |
| **Sensor FoV & GEO Nozzle** | `+130.0 mm` (`+150.0 mm` @ `500 mm`) past `350 mm` LAE; `cam_40deg` clears `24"/37"/15"/LargeOff` (`26.2–48.7°`); `lidar_45deg` clears `24"/37"/LargeOff` (`25.3–43.7°`), handing off to wrist stereo at close range on `8"/15"` (`17.8–19.5°`); STs `Ymax=208.0≤305, Zmax=478.4≤482.5, 70°≥34°` sun | **PASS** |

### 4.2 FreeCAD `Part` Solid-Kernel Verification ([dual_arm_concept.FCStd](../cad/concepts/dual_arm_espa/dual_arm_concept.FCStd))

All solid intersections across `Bus`, `Bus_Internal_Ref` (`COPV` + `65° RW` + avionics), `Arms_Stowed`, `GEO_LAE_Nozzle_350mm`, internal wrist sub-parts, and all 5 deployed configurations report **`0.00 mm³` overlap** (`L1 = 248.0 mm`, `L2 = 208.0 mm`, `456.0 mm` shoulder-to-wrist reach, `PALM_X = 128.0 mm`):

| Configuration | Ring `X` (Standoff) | `q₀` (`+X`) | `q₁` / `q₂` / `q₃` (Arm 1) | Reach / `456 mm` (Margin) | Roll Err | Overlaps (`mm³`) |
|---|---|---|---|---|---|---|
| **`Arms_Stowed`** | — | `0.0°` | Flush inside `760 mm` bays (`Z = ±377 mm`, `X ≤ 312 mm`) | `704.3 × 606.0 × 937.4 mm` | N/A | **`0.00`** *(all checks)* |
| **`24"` + `350 mm` GEO Nozzle** | `800 mm` (`+480`) | `-15.9°` | `+30.6° / -52.1° / +21.5°` | `410.1 mm` (`+45.9 mm`) | **`0.0°`** | **`0.00`** *(incl. nozzle)* |
| **`15"` Ring (`381.0 mm`)** | `800 mm` (`+480`) | `-15.9°` | `+14.1° / -50.9° / +36.8°` | `412.0 mm` (`+44.0 mm`) | **`0.0°`** | **`0.00`** |
| **`8"` Ring (`203.2 mm`)** | `800 mm` (`+480`) | `-15.9°` | `-4.6° / -35.2° / +39.8°` | `434.9 mm` (`+21.1 mm`) | **`0.0°`** | **`0.00`** |
| **`37"` + `350 mm` GEO Nozzle** | `770 mm` (`+450`) | `-15.9°` | `+46.0° / -35.8° / -10.2°` | `434.0 mm` (`+22.0 mm`) | **`0.0°`** | **`0.00`** *(incl. nozzle)* |
| **`24"` LargeOff (`+80 Y, -50 Z`)** | `800 mm` (`+480`) | `+1.9°` / `-37.6°` | `+25.7° / -53.8° / +28.1°` *(A2 reach `411.2 mm`)* | `407.1 / 411.2 mm` (`+48.9 / +44.8 mm`) | **`0.0°`** | **`0.00`** |

*(Additional GEO-nozzle mode verified analytically: `24"` @ `500 mm` standoff → `q = (-15.9°, +24.4°, -39.1°, +14.7°)`, `reach = 429.9 / 456.0 mm (+26.1 mm)`, `+150.0 mm` nozzle clearance.)*

---

## 5. MuJoCo Physics Verification (`experiments/01_dual_arm_espa`, Checks A–E)

Simulated in `PlainMujocoBackend` (`dt = 2 ms`, `implicitfast`, elliptic friction `μ = 0.4`, `impratio = 10`, `client_p7` `300.10 kg` 144-segment 610 mm Marman ring, servicer subtree `191.10 kg` with full `ActuatorBus` thruster/wheel/lead-screw constraints) via [bench_dual_arm_capture.py](../experiments/01_dual_arm_espa/bench_dual_arm_capture.py):

| Check | Scenario & Disturbance | Touch & Active Wrist `(q₃, q₄, q₅)` | Clamp Contact (`palm / in / lip` N) | Detumble, RW Fill & Prop | Bad Contacts / Plumes | Status |
|---|---|---|---|---|---|---|
| **A. Static Parity** | `191.10 kg`, stow/ready, nominal + `5°p+5°y` 6-DOF IK, RCS 1-fail, `4× RW-12` | Max FK/IK err: **`(0.0000 mm, 0.0000 deg)`** | — | `Hx=20.29, Hy=30.76 N·m·s`; 1-fail `-Fx=1.721 N` | `0` stow/ready contacts | **PASS** |
| **B. Async Blind Touch** | Blind `2°` client tilt + `15 mm Y` offset + `3.0 N·m` hold torque (`0.3 mm` lock preload) | First `4.65 s`, Both `6.85 s`; A1 `(+2.7,-0.4,+2.5)°`, A2 `(-0.6,+0.4,-1.0)°` | A1 `223.6 / 61.6 / 316.9 N` · A2 `321.0 / 30.7 / 374.6 N` | `2.91 g` prop, `0.4%` RW fill, held `3.0 N·m` cleanly | **`0 / 0`** | **PASS** |
| **C. Roll-Spin Capture** | `1.0 °/s` target roll spin + `4× RW-12` roll-sync + detumble | `6.01 s` sync; A1 `(+12.4,0.0,+0.1)°`, A2 `(+12.5,0.0,-0.2)°` | A1 `263.3 / 37.0 / 357.6 N` · A2 `270.2 / 40.1 / 375.6 N` | **`21.4 s`** (`w_end = 0.032 °/s`), **`14.5%` RW**, `0.51 g` | **`0 / 0`** | **PASS** |
| **D. Oblique Spin Detumble** | `2.0 °/s` (`0.5, 1.4, 1.33 °/s`) + 100% wheels-only detumble | `6.00 / 6.01 s`; A1 `(+11.8,0.0,+0.1)°`, A2 `(+11.4,0.0,-0.2)°` | A1 `347.1 / 58.1 / 379.3 N` · A2 `350.7 / 59.2 / 381.5 N` | **`71.5 s`** (`w_end = 0.042 °/s`), **`69.3%` RW** (`<80%`), `1.52 g` | **`0 / 0`** | **PASS** |
| **E. 3D Tilted High-Load** | `5° pitch + 5° yaw` 3D tilt + `(15, -10) mm` offset + `5.0 N·m` hold torque | `6.00 / 6.01 s`; A1 `(+0.1,+5.1,-15.5)°`, A2 `(+23.6,-7.3,+25.9)°` | A1 `392.3 / 69.8 / 398.4 N` · A2 `303.3 / 54.9 / 337.5 N` | `0.89 g` prop, `0.1%` RW fill, held `5.0 N·m` cleanly | **`0 / 0`** | **PASS** |

---

## 6. Next Potential Milestones

1. **Orbital Full-Mission Benchmark:** Integrate the `v4.2` dual-arm 6-DOF servicer into an end-to-end orbital approach (`mjorbit` / G1–G3 stack) with closed-loop stereo / `DragonEye` pose estimation and F/T admittance control.
2. **Capture Visualization / Hardware Demo (Optional):** Design a desktop or hardware-in-the-loop capture demo around the `v4.2` Cardan-Roll wrist + Marman jaw subassembly.
