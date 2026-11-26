# Independent Reproduction & Verification Guide

This guide provides exact prerequisites and copy-pasteable commands to independently reproduce and verify every quantitative result, table, figure, and CAD/physics/thermal claim in [`docs/servicer_2_draft/servicer_2_draft.pdf`](docs/servicer_2_draft/servicer_2_draft.pdf), as well as the P7 orbital servicing baseline.

---

## 1. Software Prerequisites

| Verification Tier | Required Tools & Packages | Notes |
|---|---|---|
| **A. Analytical & Orbital Thermal** | **Python 3.10+** (`numpy`, `matplotlib`, `pillow`) | Pure Python; runs on Windows, Linux, or macOS. |
| **B. FreeCAD B-Rep CAD & 3D FEM** | **FreeCAD 1.1** (`freecadcmd`) with bundled **Gmsh** & **CalculiX (`ccx`)** | All FEM scripts run headlessly via `freecadcmd`; running inside the FreeCAD GUI additionally saves VTK contour PNGs. |
| **C. MuJoCo Physics & P7 Baseline** | **Linux x86-64** (or **Windows WSL2 Ubuntu**) + **Pixi** + NVIDIA GPU w/ CUDA & EGL | Uses [`scripts/run.sh`](scripts/run.sh). Clone with `--recurse-submodules` or export `MJORBIT_DIR=/path/to/mjorbit`. |
| **D. Draft PDF Rebuild** | **Chrome / Chromium / Microsoft Edge** (headless) + Python `pillow` | Cross-platform browser lookup in [`docs/servicer_2_draft/build_pdf.py`](docs/servicer_2_draft/build_pdf.py). |

---

## 2. Draft PDF Claim-to-Script Traceability

| Draft Reference (`servicer_2_draft.pdf`) | Authoritative Script(s) | Key Expected Output / Verification Gate |
|---|---|---|
| **Table 1 & Section 1** (`v4.2` envelope, mass, CG, HDRM, RCS/RW, kinematics) | [`cad/concepts/dual_arm_espa/verify_v4_engineering.py`](cad/concepts/dual_arm_espa/verify_v4_engineering.py) | `161.7 kg` base / `188.5 kg` Spec-MGA (`193.1 kg` flat 20%), `CG = 357.7 mm <= 508.0 mm`, `ALL v4.2 ENGINEERING VERIFICATION CHECKS PASSED.` |
| **Figures 1–5** (`v4.2` FreeCAD B-Rep solids & `0.00 mm^3` interference) | [`cad/concepts/dual_arm_espa/build_dual_arm_concept.py`](cad/concepts/dual_arm_espa/build_dual_arm_concept.py) | `704.3 X x 606.0 Y x 937.4 Z mm` stowed envelope, `0.00 mm^3` overlap across all stowed/deployed/offset scenarios. |
| **Table 3 & Figure 6** (MuJoCo Checks A–E, `2 deg/s` wheels-only detumble) | [`experiments/01_dual_arm_espa/build_scene.py`](experiments/01_dual_arm_espa/build_scene.py), [`bench_dual_arm_capture.py`](experiments/01_dual_arm_espa/bench_dual_arm_capture.py) | `191.10 kg` servicer + `300.10 kg` client; `ALL MUJOCO DUAL-ARM v4.2 VERIFICATION CHECKS (A-E) PASSED.` (`69.3%` RW fill, `0` bad contacts, `0` plumes). |
| **Section 4 & Figures 7–9** (`arm_iod_demo` 273 mm chassis, CFRP booms) | [`verify_arm_iod_demo.py`](cad/concepts/arm_iod_demo/verify_arm_iod_demo.py), [`build_arm_iod_demo.py`](cad/concepts/arm_iod_demo/build_arm_iod_demo.py) | `38.0 kg` base / `45.6 kg` MGA, `CG = 141.5 mm`, `8.2 Hz` boom mode, `+5 deg` diff-boom reach, `0.00 mm^3` overlaps. |
| **Tables 4–5 & Figure 11** (Orbital TMM across LEO, GEO & IOD) | [`cad/concepts/thermal_audit.py`](cad/concepts/thermal_audit.py) | `TOTAL HARD-LIMIT FAILURES ACROSS ALL CASES = 0`; max cold heater duty `59.7% <= 75.0%` GOLD limit; isolated IOD host leak `2.99 W <= 4.00 W`. |
| **Unit Consistency Sanity Check** (Stefan-Boltzmann FEM benchmark) | [`cad/concepts/verify_freecad_ccx_units.py`](cad/concepts/verify_freecad_ccx_units.py) | `T_fem = 392.09 K | T_analytic = 392.13 K | |dT| = 0.037 K` (`PASS`). |
| **Table 6 & Figure 12** (3D coupled `Gmsh` + `CalculiX v2.22` FEM) | [`run_fem_thermal_v4.py`](cad/concepts/dual_arm_espa/run_fem_thermal_v4.py), [`run_fem_thermal_v4_full.py`](cad/concepts/dual_arm_espa/run_fem_thermal_v4_full.py), [`run_fem_thermal_arm.py`](cad/concepts/dual_arm_espa/run_fem_thermal_arm.py), [`run_fem_thermal_iod.py`](cad/concepts/arm_iod_demo/run_fem_thermal_iod.py), [`run_fem_thermal_iod_full.py`](cad/concepts/arm_iod_demo/run_fem_thermal_iod_full.py) | Full-bus + arms LEO RPO `10.2..18.1 C` (`CTE = 0.0464 mm`), GEO Eclipse `1.1..13.3 C`, IOD Isolated `9.0..15.8 C`, Arm capture `Mini45 dT = 1.4 C`. |
| **Complete 13-Page Draft PDF** | [`docs/servicer_2_draft/build_pdf.py`](docs/servicer_2_draft/build_pdf.py) | Rebuilds `docs/servicer_2_draft/servicer_2_draft.pdf` (13 pages). |

---

## 3. Step-by-Step Verification Commands

### Step 3.1 — Pure-Python Analytical & Sizing Verification (`v4.2` & `arm_iod_demo`)

```bash
python cad/concepts/dual_arm_espa/verify_v4_engineering.py
python cad/concepts/arm_iod_demo/verify_arm_iod_demo.py
```

Expected highlights:
- `v4.2`: `Base Mass = 161.7 kg | Spec-MGA = 188.5 kg | Flat-20% MGA = 193.1 kg`, `CG = 357.7 mm <= 508.0 mm`, `16 x 1 N` 1-fail min `-Fx = 1.721 N`, `4 x RW-12` `2 deg/s` detumble `64.5%` fill, `+130.0/+150.0 mm` GEO LAE nozzle clearance, `ALL v4.2 ENGINEERING VERIFICATION CHECKS PASSED.`
- `arm_iod_demo`: `Base=38.0 kg | +20% MGA=45.6 kg | Stowed CG=141.5 mm`, `k_lat=8.9 kN/m | f1=8.2 Hz`, `ALL ARM IOD DEMO ENGINEERING CHECKS PASSED.`

### Step 3.2 — FreeCAD B-Rep CAD Build & `0.00 mm^3` Overlap Audit

> **Important FreeCAD CLI note:** Running `freecadcmd script.py` executes the file under its module name rather than `__main__`, which skips `if __name__ == "__main__":` guards. Use `runpy.run_path(..., run_name='__main__')` as shown below.

**Linux / macOS:**
```bash
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/build_dual_arm_concept.py', run_name='__main__')"
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/arm_iod_demo/build_arm_iod_demo.py', run_name='__main__')"
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/export_mujoco_assets.py', run_name='__main__')"
```

**Windows PowerShell:**
```powershell
& "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/build_dual_arm_concept.py', run_name='__main__')"
& "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" -c "import runpy; runpy.run_path('cad/concepts/arm_iod_demo/build_arm_iod_demo.py', run_name='__main__')"
& "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/export_mujoco_assets.py', run_name='__main__')"
```

Expected highlights:
- `Total Stowed Envelope (mm): 704.3 X x 606.0 Y x 937.4 Z (ESPA margins: 6.7, 4.0, 27.6)` and `Overlaps (mm^3): ... 0.00` across all configurations.
- `arm_iod_demo`: `273.0 X x 520.0 Y x 840.0 Z` stowed, `0.00 mm^3` overlaps across stowed, nominal `+480 mm`, open-jaw `+458 mm`, and tilted `+5 deg` modes.

### Step 3.3 — MuJoCo Scene Compile & Closed-Loop Capture Benchmarks (Checks A–E)

Run inside Linux / WSL2 with Pixi (`MJORBIT_DIR` defaults to `./simulator` if cloned with `--recurse-submodules`, or point to your local `mjorbit` checkout):

```bash
# Optional if not using ./simulator submodule:
export MJORBIT_DIR=/path/to/mjorbit

# Verify modular orbital backend parity, keep-out geometry, and sensor rigs
bash scripts/run.sh python src/checks/test_modular_specs.py

# Rebuild MuJoCo scene XML and run Checks A-E (~60 s)
bash scripts/run.sh python experiments/01_dual_arm_espa/build_scene.py
bash scripts/run.sh python experiments/01_dual_arm_espa/bench_dual_arm_capture.py
```

To also save the EGL capture renders (`dual_arm_mj_capture.png`, `dual_arm_mj_closeup.png`), set `DUAL_ARM_RENDER_DIR`:
```bash
DUAL_ARM_RENDER_DIR=runs/dual_arm_renders bash scripts/run.sh python experiments/01_dual_arm_espa/bench_dual_arm_capture.py
```

Expected highlights:
- `test_modular_specs.py`: `ALL MODULAR TESTS PASSED` (`10s parity: chaser=0.0003 mm, client=0.0000 mm`).
- `build_scene.py`: `servicer subtree=191.10 kg (bus=174.60) | client=300.10 kg`.
- `bench_dual_arm_capture.py`: Check A `Max 6-DOF FK/IK err = (0.0000 mm, 0.0000 deg)`, Check B `first=4.65s both=6.85s`, Check C `detumble=21.4s, 14.5% RW`, Check D `detumble=71.5s, 69.3% RW, prop=1.52 g, Bad contacts=0, Plume pulses=0`, Check E `3D tilt (5p, 5y) wr q3/4/5 up to +-25.9 deg`, `ALL MUJOCO DUAL-ARM v4.2 VERIFICATION CHECKS (A–E) PASSED.`

### Step 3.4 — Multi-Node Orbital Transient Thermal Audit (Tables 4–5, Figure 11)

```bash
python cad/concepts/thermal_audit.py
```

Expected highlights:
- Prints exact 550 km LEO view factors (`F_nadir = 0.8474`, `F_side = 0.2578`), body-mounted solar ceiling (`53.0 W` LEO avg / `94.6 W` GEO avg), power/energy balance (`32.5 W` cruise, `102.9 Wh` = `10.3%` DoD), and nodal min/max tables across all 5 scenario pairs.
- Verifies `MAX HEATER DUTY CYCLE ACROSS ALL COLD CASES: IOD_Cond_Cold_wrist = 59.7% (GOLD limit <= 75.0%)` and `TOTAL HARD-LIMIT FAILURES ACROSS ALL CASES = 0`.
- Regenerates `docs/media/thermal_audit/orbital_transient_profiles.png`.

### Step 3.5 — Coupled 3D FreeCAD `Gmsh` + `CalculiX` FEM Verification (Table 6, Figure 12)

First run the 1-node analytical unit smoke test, then run any or all of the five 3D coupled FEM audits (replace `freecadcmd` with `"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe"` on Windows):

```bash
# 1. Stefan-Boltzmann unit smoke test (should match 392.13 K within < 0.1 K)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/verify_freecad_ccx_units.py', run_name='__main__')"

# 2. Detailed 6-DOF arm chain (GEO 72-min Eclipse + Sunlit Capture)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/run_fem_thermal_arm.py', run_name='__main__')"

# 3. v4.2 bus CompSolid (Cruise + RPO Nadir Transient)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/run_fem_thermal_v4.py', run_name='__main__')"

# 4. Full v4.2 bus + BOTH attached deployed arms + optics (28 solids, LEO RPO + GEO Eclipse)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/dual_arm_espa/run_fem_thermal_v4_full.py', run_name='__main__')"

# 5. IOD chassis CompSolid (Conductive +45 C & Isolated 15-m Burst)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/arm_iod_demo/run_fem_thermal_iod.py', run_name='__main__')"

# 6. Full IOD chassis + BOTH attached deployed arms (18 solids, Isolated 15-m Burst)
freecadcmd -c "import runpy; runpy.run_path('cad/concepts/arm_iod_demo/run_fem_thermal_iod_full.py', run_name='__main__')"
```

*Note on VTK PNG exports:* In headless `freecadcmd` mode, all 5 FEM scripts execute the full `Gmsh` mesh + `CalculiX` solve and print every regional `T_min`, `T_max`, and max CTE displacement while cleanly skipping the GUI-only VTK viewport capture. Executing the exact same `runpy.run_path(...)` snippet inside the FreeCAD 1.1 GUI Python console additionally exports the 3D contour PNGs to `docs/media/thermal_audit/`.

### Step 3.6 — Draft PDF Rebuild

```bash
python docs/servicer_2_draft/build_pdf.py
```

Writes regenerated cropped panels to `docs/servicer_2_draft/figures/` and `docs/servicer_2_draft/servicer_2_draft.html` (both gitignored) and compiles the 13-page [`docs/servicer_2_draft/servicer_2_draft.pdf`](docs/servicer_2_draft/servicer_2_draft.pdf).

### Step 3.7 — P7 Orbital Servicing Baseline (Optional Full-Stack Replay)

```bash
bash scripts/run.sh python src/mission/run_e2e.py \
  --backend mjorbit --earth-bg --seeds 7720000 \
  --g3-policy weights/g3_ppo_orb.pt \
  --out runs/smoke_7720000.json

bash scripts/run.sh python src/viz/record_replay.py \
  --seed 7720000 --g3-policy weights/g3_ppo_orb.pt \
  --out runs/replay_7720000
```

---

## 4. Scope & Preliminary Thermal Caveats

As documented in Section 5.4 of [`servicer_2_draft.pdf`](docs/servicer_2_draft/servicer_2_draft.pdf), the thermal numbers verified by `thermal_audit.py` and the CalculiX FEM suite represent a **preliminary design audit**. Full Monte Carlo ray-traced geometric radiative modeling (ESATAN-TMS / Thermal Desktop) capturing deployed-arm self-shadowing across the `±Y` solar arrays and `-Z` OSR, together with TVAC correlation of the DragonEye CCHP saddle, tubular arm MLI, and Jetson Orin NX thermal transfer plate, remain required before PDR/CDR freeze.
