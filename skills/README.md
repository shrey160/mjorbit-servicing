# Agent Skills (`skills/`)

Portable, self-contained AI agent skills covering the FreeCAD CAD workflow and aerospace orbital + 3D CalculiX FEM thermal verification used in `mjorbit-servicing`.

## Included Skills

| Skill Folder | Domain & Purpose |
|---|---|
| [`freecad-mcp/`](freecad-mcp/SKILL.md) | Core entry point for driving FreeCAD 1.1 via FreeCAD MCP (RPC connection, `fcmcp.py` helpers, exports, hang recovery, and domain routing including `thermal-sims`). |
| [`freecad-3d-model/`](freecad-3d-model/SKILL.md) | Non-PartDesign 3D assemblies, CSG booleans, multi-part mass/CG computation, and STEP/STL simulator export. |
| [`freecad-parametric-sketch/`](freecad-parametric-sketch/SKILL.md) | Fully constrained Sketcher profiles bound to parameter spreadsheets. |
| [`freecad-parametric-body/`](freecad-parametric-body/SKILL.md) | Parametric PartDesign bodies, change-tests, and geometry-based topological-naming edge/face selection. |
| [`freecad-techdraw/`](freecad-techdraw/SKILL.md) | 2D engineering projection views, vertex/edge dimensioning, title blocks, and SVG/PDF/DXF export. |
| [`thermal-sims/`](thermal-sims/SKILL.md) | 6-gate preliminary spacecraft orbital TMM (ECSS/NASA GOLD/Gilmore) + coupled 3D FreeCAD `Gmsh` / `CalculiX` FEM workflow, reference tables, `thermal_helpers.py`, unit smoke test, and runnable RK4 TMM template. |

## Quick Usage

- **Smoke-test the bundled orbital thermal helper template** (pure Python, no external deps):
  ```bash
  python skills/thermal-sims/examples/minimal_orbital_tmm_template.py
  ```
- **Verify FreeCAD 1.1 + CalculiX thermal unit consistency (`|dT| = 0.037 K`)**:
  ```bash
  freecadcmd -c "import runpy; runpy.run_path('skills/thermal-sims/scripts/verify_freecad_ccx_units.py', run_name='__main__')"
  ```
- **Install into an AI coding agent:** Copy any subdirectory under `skills/` into your agent's global skills folder (e.g., `~/.gemini/config/skills/` or `~/.claude/skills/`) and replace `<path-to-skills>` in `SKILL.md` snippets if invoking `fcmcp.py` from outside this directory tree.
