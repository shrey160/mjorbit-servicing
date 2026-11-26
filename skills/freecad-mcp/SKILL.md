---
name: freecad-mcp
description: Use whenever you drive FreeCAD through the FreeCAD MCP server (tools named freecad / execute_code, get_view, create_object, ...) - connecting, choosing the right MCP tool, running build scripts, screenshots, checking and exporting models (FCStd, STEP, STL), and recovering when FreeCAD hangs or the RPC stops answering. Start here for any FreeCAD task; it routes to freecad-3d-model, freecad-parametric-sketch, freecad-parametric-body, freecad-techdraw, and thermal-sims, and ships the fcmcp.py helper library they all use.
---

# FreeCAD through the MCP

FreeCAD 1.1 driven by the `freecad-mcp` server (0.1.32) talking to the FreeCADMCP addon over
RPC on 127.0.0.1:9875. Read the **hard rules** first, then the task skill:

| Task | Skill |
|---|---|
| Quick solid, concept, CSG, simulation meshes, multi-part model | `freecad-3d-model` |
| Fully constrained, spreadsheet-driven sketches (the "parametric drawing") | `freecad-parametric-sketch` |
| Parametric part: parameters, then a PartDesign Body, then features | `freecad-parametric-body` |
| 2D engineering drawing (views, dimensions, title block, PDF/SVG/DXF) | `freecad-techdraw` |
| Orbital thermal & coupled 3D thermal-CTE FEM (`Gmsh` + `CalculiX`) | `thermal-sims` |

## Hard rules (breaking these hangs FreeCAD or loses work)

1. **Build with `execute_code`. Never with `execute_code_async`.** `execute_code` runs on
   FreeCAD's GUI thread, which is the only safe place for `newDocument`, `addObject`,
   `recompute`, `saveAs`, `ViewObject`, `Gui.*`, Qt, and every `fc.*` helper.
   `execute_code_async` runs in a background thread. A document or view write from there
   wedges FreeCAD's event loop: Windows logs "freecad.exe stopped interacting with Windows",
   every MCP call fails, and the user must restart FreeCAD. The fcmcp helpers raise
   `ThreadError` if called from that thread.
2. **Slow build? Raise the timeout; don't switch tools.** Use `execute_code(code, timeout=600)`
   (the cap is 1800 s), or split the build into several `execute_code` calls.
3. **One call at a time.** Wait for each result before the next call. A second script gets
   `SCRIPT_BUSY`: wait and resend it later.
4. **A timeout does not stop the code.** The task keeps running on the GUI thread, and every
   later GUI call fails fast ("stuck") until it finishes. Do not resend it. Call
   `get_rpc_status` (or `get_task_status`) until `quiescent: true`, then decide.
5. **Run script files, not long inline code.** Write `build_<part>.py` and run it with
   `fc.run(path)` (fresh namespace). Don't write your own staging, resume or log-file
   framework (`_stage.txt`, `_build_log.txt`). If something fails, fix the script and rerun it.
6. **Use the helpers** (`fcmcp.py`, below) instead of raw `AttachmentSupport`/`MapMode`/
   constraint code. They make fully constrained sketches, set the Body Tip, recompute in the
   right places, and **stop at the first broken feature** with a `FeatureError` that says why.
7. **Check after every feature, not at the end.** When a helper raises, fix that feature
   before adding more. When writing raw code, check `feat.Shape.isValid()`,
   `len(feat.Shape.Solids) == 1`, and that the volume changed.
8. **Don't close or overwrite the user's documents.** `fc.new_doc(name)` closes an open
   document of the same name *without saving*. Call `list_documents` first and pick a new name
   for your part.
9. **No blocking calls on the GUI thread.**
   - No `time.sleep` loops: use `fc.pump(s)`, which keeps Qt running.
   - No `input()` and no modal dialogs.
   - Never enter edit mode (`setEdit`, `doubleClicked()` on sketches or features). That
     opens a task panel that blocks later commands. Opening a TechDraw page is fine.
10. **Screenshots cost tokens.** Pass `include_screenshot=False` on every call except the
    one where you want to look. Call `fc.show(doc)` before that one.

## Decision: which tool

```
Need to build / edit / check / export / draw?        -> execute_code (timeout up to 1800)
Just inspect?                                        -> list_documents, get_objects, get_object
Look at the model?                                   -> fc.show(doc) inside execute_code, then a screenshot or get_view
A single primitive or boolean, no script at all?     -> create_object / edit_object
One OCCT operation that alone takes > 10 min,        -> execute_code_async: pure Part.* shape math only,
  on shapes already fetched into variables?             document writes only inside commit(fn)
Code that may CRASH OCCT (helical thread sweeps,     -> execute_code_headless (separate freecadcmd process:
  huge many-tool booleans, risky lofts)?                open/save files yourself), then reload_document
FEM?                                                 -> create_object (analysis, material, mesh) + run_fem_analysis
```
When unsure, use `execute_code`. Async is almost never needed for single parts: the
verified flange-hub build, including its change-test, finishes in seconds.

## Standard procedure (follow in order)

1. `get_rpc_status`. It must say `rpc_server: running`, `gui_dispatch.state: healthy` and
   `quiescent: true`. If not, see Recovery.
2. `list_documents`, so you know what is already open.
3. Write the script file. Start from this template:
   ```python
   """<part>: design intent, parameters, which plane each feature uses."""
   import sys, importlib, os
   sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
   import fcmcp as fc; importlib.reload(fc)

   FOLDER = os.path.dirname(os.path.abspath(__file__))
   PARAMS = [("Length", 90, "mm", "base X"), ("Thk", 10, "mm", "plate thickness")]

   def build():
       doc = fc.new_doc("my_part")          # unique name
       fc.params(doc, PARAMS)
       b = fc.body(doc)
       sk = fc.sketch(b, "BaseSketch", "XY")
       fc.rect(sk, "Length", "Length")
       fc.pad(b, sk, "Thk", "Base")         # raises FeatureError if broken
       return doc, b, [sk]

   if __name__ == "__main__":
       doc, b, sketches = build()
       for s in sketches:
           st = fc.check_sketch(s); assert st["fully"], st
       print(fc.report(b, density_g_cm3=2.70))
       print(fc.export(doc, b, FOLDER, "my_part", step=True))
   ```
4. Run it:
   ```python
   import sys, importlib
   sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
   import fcmcp as fc; importlib.reload(fc)
   fc.preflight()          # prints version, gui_thread=True, open documents
   fc.run(r"C:/path/to/build_my_part.py")
   ```
   with `include_screenshot=False` and `timeout=300`.
5. Read the printed output. On an exception, fix the script and rerun from step 4. The
   rerun replaces the document.
6. For parametric parts, run the change-test (`freecad-parametric-body`).
7. `fc.show(doc)`, then one screenshot. Check that it matches the intent, then report what
   was checked and what was not.

## Recovery

| Symptom | Do this |
|---|---|
| `SCRIPT_BUSY` | Another script is still running. Wait, call `get_rpc_status`, and retry once it is idle. |
| execute_code timed out | Don't resend. Poll `get_rpc_status` until `quiescent: true`, then read `get_task_status` for the late result. |
| `gui_dispatch.state: stuck` / `SCRIPT_UNAVAILABLE` | A GUI task is still running. Wait. If it doesn't clear in a few minutes, tell the user FreeCAD needs a restart. |
| Connection refused / every call fails / Windows "not responding" | FreeCAD is hung or closed. Ask the user to restart FreeCAD and start the RPC server (MCP Addon toolbar, "Start RPC Server"). Don't retry in a loop. |
| A hang after you used `execute_code_async` | That was the cause (rule 1). After the restart, rebuild with `execute_code`. |
| Port 9875 in use / wrong documents visible | Two FreeCAD instances are running and only one owns the port. Ask the user to close the extra one. |
| "protocol mismatch" / version error | The addon in `%APPDATA%/FreeCAD/v1-1/Mod/FreeCADMCP` must match the `uvx freecad-mcp` version. Replace the addon folder (keep a backup) and restart FreeCAD. |
| `execute_code_headless`: "freecadcmd not found" | The client config must pass `--freecadcmd "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe"` (see below). |

After a restart, documents that were open but unsaved are gone. Rerun the build script;
that's why the script, not the .FCStd, is the source of truth.

## Client configuration (all three use the same server)

- **Claude Code** (`~/.claude.json` → `mcpServers.freecad`), **OpenCode**
  (`~/.config/opencode/opencode.json` → `mcp.servers.freecad`) and **Antigravity**
  (`~/.gemini/config/mcp_config.json`) all run:
  `uvx freecad-mcp --freecadcmd "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe"`.
  freecadcmd is not on PATH, so without that flag `execute_code_headless` fails.
- **OpenCode:** set `"timeout": {"startup": 60000, "execution": 600000}`. Without the execution
  timeout the default is 12 h, so a stuck call looks like an endless hang. Don't set `pool`:
  freecad-mcp uses the legacy protocol.
- OpenCode starts one freecad-mcp proxy per open project directory, all talking to the one
  addon. If one session wedges FreeCAD, all of them break. For CAD-only use, put the entry in a
  project config instead of the global one.
- Run exactly one FreeCAD instance.

## Helper library: `scripts/fcmcp.py`

```python
import sys, importlib
sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
import fcmcp as fc; importlib.reload(fc)   # reload picks up edits to the helper
```
Values passed to helpers are numbers (mm or deg) or strings. An identifier string is a
parameter name (`"Length"` → `Params.Length`); any other string is an expression
(`"Params.SlotX - Params.SlotLen / 2"`). Strings are bound with `setExpression`, so the model
follows the spreadsheet.

- **Session:** `preflight()`; `run(path, **overrides)`. Settings: `STRICT = True` (set it
  False to only warn); `FeatureError` and `ThreadError` are the exceptions they raise.
- **Document and parameters:** `new_doc`, `params(doc, rows)`, `set_params(doc, **v)`,
  `num(doc, x)`, `expr(x)`.
- **Body and sketch:** `body`, `plane(body, "XY"|"XZ"|"YZ"|"X"|"Y"|"Z")`,
  `sketch(body, name, on="XY" | (feature, "FaceN"), offset=None)`.
- **Constrained sketch geometry:** `rect`, `rounded_rect`, `circle`, `copy_circle`,
  `hole_pattern_rect`, `slot`, `polyline`, `place_point`, `constrain`, `check_sketch`.
- **Features:** `pad`, `pocket`, `revolve`, `groove`, `hole`, `fillet`, `chamfer`,
  `polar_pattern`, `linear_pattern`, `mirrored`. Each one recomputes, sets the Tip, and
  verifies the result: a valid shape, one solid, and a volume that changed.
- **Picking by geometry:** `edges_where`, `faces_where`, `is_line_along`, `is_circle`, `at_z`,
  `planar_face_normal`, `top_face`, `pick`, and `update(doc, picks, **params)`.
- **Output:** `report(body, density)`, `export(doc, obj, folder, stem, step, stl, stl_metres)`,
  `clean_backups`, `show(doc, view)`, `pump(seconds)`.
- **TechDraw:** `drawing_page`, `proj_group`, `wait_views`, `view_of`, `vertex`, `edge`,
  `list_view_geometry`, `dimension`, `refresh_dimensions`, `export_page`.

## Mistakes agents make (and the fix)

| Mistake | Fix |
|---|---|
| Building with `execute_code_async` "because it is long" | `execute_code` with a bigger `timeout` (rule 1). |
| Retrying a timed-out call right away | It is still running. Poll `get_rpc_status` first. |
| One-letter parameter names (`H`, `T`, `W`, `S`, `C`, `F`, `N`, `M`) | Rejected as unit symbols. Use words: `Height`, `Thk`, `Width`. `fc.params` tells you which name failed. |
| `+ 10` in an expression with lengths | `+ 10 mm` (Unit mismatch). |
| Hard-coded `Edge5` / `Face6` from a listing | Pick by geometry (`fc.pick`, PICKS, `fc.update`). |
| One-sided pocket on XZ cuts air ("volume unchanged") | XZ normal is −Y. Use `reversed=True`, or `through=True, symmetric=True`. |
| A rib or gusset flush with a plate side face, or touching along an edge only | Overlap by ≥0.5 mm (embed it) and set it back ≥0.5 mm from flush faces. Otherwise the fuse can come out invalid. |
| Holes cut through a rib or the upright because the bolt spacing ignores other features | Use parameters that keep holes clear: check the gap in the sketch, and look at a screenshot. |
| `sk.Support = ...` (pre-1.0 API) | `fc.sketch(...)`. In raw code: `AttachmentSupport` and `MapMode = "FlatFace"`. |
| `pad.Midplane = True` | `SideType = "Symmetric"` (`fc.pad(..., symmetric=True)`). |
| A pattern of a pattern, or a mirror of a pattern | Pattern the original feature, or `fc.mirrored(b, None, "YZ", whole_shape=True)`. |
| Checking only at the end | Helpers check each feature. In raw code, call `report` after each feature. |
| Calling it done from the numbers alone | Also look at one screenshot, and state what wasn't verified (fits, tolerances, threads). |

## Errors you will meet

| Message | Cause and fix |
|---|---|
| `ThreadError: fcmcp.xxx() ... background thread` | You used `execute_code_async`. Rerun with `execute_code`. |
| `FeatureError: <Feat> ... is broken: ...` | The message names the problem and the usual cause. Fix that feature and rerun. |
| `Base feature's TopoShape is invalid` while setting properties | The base isn't computed yet. `doc.recompute()` before Hole, patterns and dress-ups (helpers do this). |
| Pattern computes but the body ignores it, or features are out of order | `newObject` didn't move the Tip. Set `body.Tip = feature` (helpers do this). |
| `Invalid edge link ... ?Edge5` after a parameter change | Topological naming. Re-pick by geometry with `fc.update(doc, picks, ...)`. |
| `Parameter name 'X' rejected` / `ValueError: Invalid alias` | Rename the parameter (see the mistakes table). |
| `Unit mismatch in plus operation` | Constants need units: `"Params.Thk + 10 mm"`. |
| `Only additive and subtractive features can be transformed` | Use `whole_shape=True`, or pattern the original feature. |
| `KeyError` reading a sheet | Use `sheet.getCellFromAlias("Name")`, not `getContents`. |
| Sketch `Invalid`, solve −3 | Conflicting constraints. The lists are 1-based (see freecad-parametric-sketch). |
| `SCRIPT_BUSY` / timeout / stuck | See Recovery. |

## Checks before calling something done

```python
print(fc.report(body, density_g_cm3=2.70))   # valid, solids == 1, bbox, errors == []
print([fc.check_sketch(s) for s in sketches]) # solve 0, dof 0, fully True, no conflicts
```
For parametric work, also run a change-test (`freecad-parametric-body`).

## Screenshots: known traps

- The screenshot shows the **active window**. If a TechDraw page or another document is in
  front, you get a garbled or wrong image. `fc.show(doc)` raises the document's 3D view, hides
  sketches, clears the selection and fits.
- Views with only sketches, or with transparent bodies, often capture as noise. Show solids.
  To show a sketch, plot its edges (`sk.Shape.Edges`, `e.discretize(60)`) with matplotlib
  (available inside FreeCAD) and save a PNG.
- A green or teal tint on faces after selections is display state only; the geometry is fine.

## Saving and exporting

- `fc.export(doc, body, folder, stem, step=True, stl=True, stl_metres=True)`. STEP is in mm;
  STL is in metres for MuJoCo/ROS (`MeshPart.meshFromShape`, then scale by 0.001).
- Re-saving writes `name.<timestamp>.FCBak` backups next to the file. Remove them with
  `fc.clean_backups(folder)`.
- Never leave a model only in FreeCAD's memory: save the .FCStd and keep the build script.
