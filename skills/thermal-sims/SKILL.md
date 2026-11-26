---
name: thermal-sims
description: >-
  Perform preliminary aerospace orbital thermal analysis (ECSS-E-ST-31C, NASA MSFC,
  NASA GSFC GOLD Rules, Gilmore) and one-way coupled 3D FreeCAD Gmsh + CalculiX FEM
  thermal-structural verification for spacecraft, hosted payloads, and robotic appendages.
  Use when sizing radiators/heaters, checking orbital hot/cold margins, validating
  compute/optics thermal bursts, or running FreeCAD coupled thermal-CTE simulations.
---

# Aerospace Orbital Thermal & 3D FEM Workflow

Follow these **6 sequential gates** in order. **Do NOT skip a gate.** Smaller/flash models should copy the starter scripts verbatim before customizing parameters.

**Support files (relative to this skill folder):**
- Math & RK4 primitives: [thermal_helpers.py](./scripts/thermal_helpers.py)
- FreeCAD unit smoke test: [verify_freecad_ccx_units.py](./scripts/verify_freecad_ccx_units.py)
- Working 3-node starter TMM: [minimal_orbital_tmm_template.py](./examples/minimal_orbital_tmm_template.py)
- Orbital fluxes, coatings, Gilmore MLI & limits: [orbital_constants.md](./references/orbital_constants.md)
- FreeCAD 1.1 / CalculiX API & solver traps: [freecad_fem_pitfalls.md](./references/freecad_fem_pitfalls.md)

---

## Step 0 — Power & Solar Ceiling Intake (Before Writing Any Thermal Code)

1. List every operating mode (`Cold Safe`, `Nominal Cruise`, `Active Burst`, `Detumble`) and compute duration (`s` / `min`).
2. Compute solar generation FIRST using `solar_power_ceiling(area_m2)` in [thermal_helpers.py](./scripts/thermal_helpers.py).
   - **Rule:** High-power AI compute (e.g. `25 W` Jetson Orin NX or `70–130 W` Thor) cannot run continuously if orbit-average solar supply is lower than cruise + compute load. Power-gate payload compute **OFF (`0 W`)** in `Cruise` and `Safe` modes, leaving a `4–6 W` supervisor OBC active.
3. Add realistic PCDU DC-DC conversion heat on EVERY bus load and active heater: `pcdu_heat_loss_w(load_w, eff=0.89)` (~`10–12%` loss, NOT `4%`).
- **Gate 0:** Verify `cruise_load_w < solar_orbit_avg_w` and `burst_energy_wh / battery_wh <= 0.30` before Step 1.

---

## Step 1 — Physical Conductance & Gilmore Calibration

Look up exact values in [orbital_constants.md](./references/orbital_constants.md):
1. **Cross-check every lumped conductance `G` (W/K) against geometry:** Compute $G = k A / L$. Do not assume a high number like `5.5 W/K` unless a physical CCHP heat pipe (`k_eff ~ 4,500 W/m·K`) or wide Al saddle exists; a thin `10 × 14 mm` PGS strip over `400 mm` gives only `~0.35 W/K`.
2. **Gilmore MLI area-scaling rule:**
   - Large bus blanket (`> 1 m²`): `eps* = 0.03` (hot) / `0.04` (cold).
   - Small sleeve/appendage/tube blanket (`< 0.2 m²`): **`eps* = 0.07` (hot) / `0.08` (cold)** due to perimeter seams.
3. **Vacuum bolted joints:** Use realistic `h_c ~ 600–1,500 W/(m²·K)` (Cho-Therm / Grafoil), not infinite contact.
- **Gate 1:** Every conductance $G$ has an explicit physical hardware path (bracket, CCHP, isolator standoff, or TIM pad).

---

## Step 2 — Multi-Node Orbital Transient Solver (TMM)

Start from [minimal_orbital_tmm_template.py](./examples/minimal_orbital_tmm_template.py) and functions in [thermal_helpers.py](./scripts/thermal_helpers.py):
1. **MLI modeling (critical):** ALWAYS solve outer-foil radiative equilibrium first (`mli_outer_temp_k`), then pass heat across the blanket (`mli_heat_transfer_w`). **Never** put outer solar absorptivity `alpha` directly on a node that only rejects via `eps*`.
2. **Exact Earth view factors:** Call `earth_view_factors(alt_km)` for BOTH nominal (`F_side`) and worst-case attitude tilt (`F_nadir`, e.g. `0.847` vs `0.258` at `550 km` LEO).
3. **Bound Hot & Cold cases properly:**
   - *Hot Case:* EOL coatings (`alpha_EOL`), max flux (`1418 W/m²`), min/zero eclipse, max dissipation + worst-case radiator nadir/sun angle.
   - *Cold Case:* BOL coatings (`alpha_BOL`), min flux (`1322 W/m²`), max eclipse (`35.6 min` LEO / `72 min` GEO), minimum survival dissipation + conservative higher `eps*` MLI.
4. **Hysteresis survival heaters:** Set cold deadbands above minimum limits, call `check_gold_duty_cycles(duty_dict, 75.0)`, and `check_ecss_margins(...)`. Run enough orbits (`6–10` LEO, `4–6` GEO) to reach cyclic equilibrium before triggering burst loads.
- **Gate 2:** `0` hard-limit violations, max heater orbit-avg duty cycle **`<= 75.0%`**, host leak within TICD limit (if hosted), and record **pre-burst equilibrium temperatures** for Step 4.

---

## Step 3 — 1-Node FreeCAD / CalculiX Unit Smoke Test

Before building a full 3D assembly, verify FreeCAD + CalculiX units in your environment:
1. Check `get_rpc_status` first (`quiescent: true`).
2. Run [verify_freecad_ccx_units.py](./scripts/verify_freecad_ccx_units.py) via FreeCAD MCP **`execute_code`** (`include_screenshot=False`). **Never** use `execute_code_async` for document/GUI ops.
- **Gate 3:** Smoke test prints `PASS` with `|T_fem - T_analytic| < 1.0 K` (`(mm, tonne, s, N, mW, K)` confirmed).

---

## Step 4 — One-Way Coupled 3D Thermomechanical FEM (FreeCAD + CalculiX)

Read [freecad_fem_pitfalls.md](./references/freecad_fem_pitfalls.md) first, then follow this exact sequence:
1. **Watertight structure first:** Embed internal decks/saddles `2–3 mm` into the outer bus shell (`fuse`), call `.removeSplitter()`, and assert **`len(core.Solids) == 1`**. A `1 mm` gap silently disconnects decks!
2. **Multi-material assembly:** Keep touching interfaces flush (zero overlap), run `BOPTools.SplitFeatures.makeBooleanFragments` with `Mode = "CompSolid"`.
3. **Materials:** Create the primary structural material (e.g. `Mat_Al6061`) with **`m.References = []`** so FreeCAD treats it as the default fallback for all unassigned solids.
4. **One-way TMM → FEM coupling:**
   - Set `InitTemp` to the **Step 2 TMM pre-burst equilibrium temperature**.
   - Compute radiator sink via `effective_sink_temp_k(q_abs, eps)` (or `leo_osr_sink_k(vf_earth)`).
   - For finite bursts (`10–30 min`), **always use transient** (`solver.ThermoMechSteadyState = False`), NEVER steady state (`SteadyState = True` assumes infinite burst duration!).
   - For slow pseudo-steady cruise (`18,000 s`), set `TimeMaximumIncrement = 1800.0` so CalculiX finishes in ~15 steps without MCP timeout.
5. **Solve & read final result:** Call `fea.setup_working_dir()`, `fea.setup_ccx()`, `fea.update_objects()`, assert `not fea.check_prerequisites()`, write `.inp`, run `fea.ccx_run()`, then select **`[-1]`** among `Fem::FemResultObject` objects.
6. **Visualize & clean:** Render VTK contours via `Fem::FemPostPipeline`, inspect the PNG image visually to verify expected hot/cold paths, check `max_CTE_disp`, and delete any `.FCBak` files.
- **Gate 4:** Clean CalculiX exit (`0`), visually verified temperature contour, peak local T_j/hotspots within limits, `0` `.FCBak` files.

---

## Step 5 — Preliminary Thermal Audit Report

Always label first-pass TMM + 1-way FEM results as a **Preliminary Thermal Audit** structured in order:
1. **Assumptions, Specifications, Limits & Boundary Conditions** (including solar ceiling, PCDU loss, Gilmore MLI, view factors, and ECSS/GOLD targets).
2. **Orbital Transient Experiments** (table + multi-orbit plots + heater duty cycles).
3. **3D Coupled FEM Experiments** (contour plots + regional min/max + CTE shift).
4. **Operational Constraints & Pre-PDR Caveats** (explicitly note that Monte Carlo ray-traced external self-shadowing in ESATAN/Thermal Desktop and TVAC test correlation are still required before PDR/CDR).

---

## 8 Common Mistakes — Never Do These

| # | Mistake | Symptom | Fix |
|---|---|---|---|
| 1 | **Greenhouse MLI bug** | Cold eclipse `+45 °C` inside MLI | Compute outer foil `T_mli_out`, radiate across `eps*` (`mli_heat_transfer_w`) |
| 2 | **Neglecting solar ceiling / 4% PCDU loss** | Cruise exceeds array avg power | Size cruise < `solar_power_ceiling`; use `88–90%` PCDU efficiency |
| 3 | **Unverified lumped conductance** | `DragonEye` hits `180 °C` in 3D FEM | Check `k*A/L`; use real CCHP (`k_eff~4500`) + bolted saddle |
| 4 | **Steady-state solve for 15–20 min burst** | Peak electronics overheat by `50+ °C` | Set `ThermoMechSteadyState = False`, `TimePeriod = burst_s`, `T0 = T_cruise` |
| 5 | **Gap around internal structural decks** | Isolated components hit `70+ °C` | Embed `2–3 mm` before `fuse()` and assert `len(core.Solids) == 1` |
| 6 | **Missing `fea.setup_ccx()` or default material** | `FileNotFoundError` / unassigned element | Call `fea.setup_ccx()` and set structural material `.References = []` |
| 7 | **Small `TimeMaximumIncrement` on `18,000 s` run** | MCP GUI dispatch timeout (`> 90 s`) | Use `TimeMaximumIncrement = 1800.0` for pseudo-steady runs |
| 8 | **Using `Vector.multiply()` in FreeCAD loops** | Geometry explodes (`10^4 mm`) | `v.multiply(s)` mutates in place; use non-mutating `v * s` |
