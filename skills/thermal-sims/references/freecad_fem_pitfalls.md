# FreeCAD 1.1 + CalculiX `v2.22` Thermal FEM Reference & Pitfalls

## 1. Verified CalculiX `.inp` Unit System in FreeCAD 1.1

FreeCAD writes CalculiX files in **`(mm, tonne, s, N, mW, K)`**:
- `STEFAN BOLTZMANN = 5.670374419e-11` (`mW / (mm² · K⁴)`)
- `*DFLUX` `BF` body heat flux in **`mW / mm³`** (`1 W/cm³ = 1 mW/mm³`)
- `*RADIATE` `R` ambient sink in **`K`**
- `*INITIAL CONDITIONS, TYPE=TEMPERATURE` in **`K`**
- Result `res.Temperature` is in **`K`** (subtract `273.15` for `°C`); `res.DisplacementLengths` is in **`mm`**.

Always confirm units on your host with [verify_freecad_ccx_units.py](../scripts/verify_freecad_ccx_units.py).

---

## 2. Hard FreeCAD MCP Execution Rules

1. **Always use `execute_code`**, never `execute_code_async` (document/GUI/FEM objects crash or wedge Qt from background threads).
2. **Pass `include_screenshot=False`** during builds and FEM solves; inspect saved contour PNGs afterward.
3. **Timeout management:** Default MCP GUI dispatch timeout is ~`90 s`. Keep meshes moderate (`5,000–12,000` nodes, linear `1st` order elements, targeted `MeshRegion` only on slender/high-flux parts) or run 1 case per call. For `18,000 s` pseudo-steady transients, set `TimeMaximumIncrement = 1800.0` so CalculiX takes ~12–15 increments instead of 150!
4. **Never mutate FreeCAD vectors in place:** `vec.multiply(s)` modifies `vec`! Always use `vec * s`.

---

## 3. Common FEM Modeling & API Fixes

- **Disconnected `CompSolid` islands (`AlCore` split into 2+ solids):**
  - Cause: leaving `0.5–1.0 mm` CAD clearance around internal decks or mounting saddles breaks conduction completely.
  - Fix: overlap structural decks/saddles `2–3 mm` into outer shell walls before `.fuse()`, call `.removeSplitter()`, and **`assert len(core.Solids) == 1`**.
- **CalculiX error `no material was assigned to element N`:**
  - Fix: set **`mat_default.References = []`** on your primary background structural material (`Mat_Al6061` or `Mat_Ti6Al4V`). FreeCAD assigns any leftover fragmented solids to the empty-reference material.
- **`FileNotFoundError` when running CalculiX:**
  - Fix: call in exact order:
    ```python
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    assert not fea.check_prerequisites()
    fea.write_inp_file()
    ret = fea.ccx_run()
    assert ret == 0
    fea.load_results()
    ```
- **Transient result selection:**
  - Transient CalculiX produces multiple `Fem::FemResultObject` steps. Always take **`[-1]`** (`res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]`) and assert `len(res.Temperature) > 0`.
- **Steady-state solver fail on pulsed bursts or high `k` ratios:**
  - Do NOT solve `15–20 min` peak RPO/capture bursts with `ThermoMechSteadyState = True` — infinite duration overheats the bus and high-`k` heat pipes (`k = 4500`) can stall steady mechanical Newton iterations. Use `ThermoMechSteadyState = False`, `TimePeriod = burst_s`, and `InitTemp = T_cruise_K`.
- **VTK Contour Post-Processing in FreeCAD 1.1:**
  - `ObjectsFem.makePostPipeline` does not exist; use:
    ```python
    pipe = doc.addObject("Fem::FemPostPipeline", "TempPipeline")
    pipe.load(res)
    doc.recompute()
    pipe.ViewObject.Visibility = True
    pipe.ViewObject.DisplayMode = "Surface"
    if "Temperature" in pipe.ViewObject.getEnumerationsOfProperty("Field"):
        pipe.ViewObject.Field = "Temperature"
    ```
- **Repository hygiene:** Never commit `.FCBak` files. Clean them if any FreeCAD save creates backups.
