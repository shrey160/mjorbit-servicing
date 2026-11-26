---
name: freecad-parametric-body
description: Use when building a parametric 3D part in FreeCAD through the MCP - parameters spreadsheet first, then a PartDesign Body, then sketch-based features (pad, pocket, revolve, groove, hole, fillet, chamfer, patterns, mirror) on the XY/XZ/YZ planes or on faces, so the model rebuilds when a parameter changes. Includes the change-test and the topological-naming fix (re-picking edges/faces by geometry).
---

# Parametric body: parameters -> Body -> features (FreeCAD 1.1 via MCP)

Uses `freecad-mcp` (tools, helpers) and `freecad-parametric-sketch` (constrained sketches).
Worked example, verified end to end: `examples/flange_hub.py` (revolve, fillet, chamfer,
counterbored Hole, polar pattern, keyway on a face, change-test).

## 1. Design intent before geometry

Write down, in the build script's docstring:
- the parameters (name, value, unit, meaning) - these become the spreadsheet;
- which plane each feature starts from and why (what the front/top/side views show);
- what must stay true when parameters change (symmetric about the origin, holes stay on
  the pitch circle, gussets touch the upright, ...).

Put the origin where the symmetry is: centre the part on the origin planes so most sketches
are symmetric and need few dimensions.

## 2. Skeleton

```python
import sys, importlib
sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
import fcmcp as fc; importlib.reload(fc)

doc = fc.new_doc("bracket")
fc.params(doc, [("Length", 90, "mm", "base X"), ("BaseT", 10, "mm", "base thickness"),
                ("BoltN", 6, "", "count")])
b = fc.body(doc)
```

Run the script with `execute_code` and `fc.run(path)`, with `include_screenshot=False` and
`timeout=300`. Never use `execute_code_async`; see the hard rules in `freecad-mcp`.

## 3. Features from sketches

| Want | Sketch on | Feature helper |
|---|---|---|
| Extrude a profile | XY/XZ/YZ, or offset plane | `fc.pad(b, sk, "BaseT")`; `symmetric=True` centres it on the plane |
| Cut | plane or face | `fc.pocket(b, sk, depth)` or `through=True` (`symmetric=True` = both ways) |
| Turned part | half-section on XZ (x = radius, y = Z) | `fc.revolve(b, sk, "V_Axis")` (axis = sketch V axis = Z) |
| Turned cut (groove, O-ring seat) | half-section | `fc.groove(b, sk, "V_Axis")` |
| Drilled / counterbored / tapped holes | circles on a plane at the entry face | `fc.hole(b, sk, "BoltD", counterbore=("CbD","CbDepth"))` (one hole per circle centre) |
| Round / bevel edges | - | `fc.fillet(b, base, edges, "R")`, `fc.chamfer(...)` with edges picked by geometry |
| Repeat | - | `fc.polar_pattern(b, [feat], "BoltN", "Z")`, `fc.linear_pattern(...)`, `fc.mirrored(..., "YZ")` |

Plane orientation (FreeCAD 1.1): XY normal +Z; **XZ normal -Y** (sketch y is +Z, a one-sided
pad goes toward -Y, use `symmetric=True` or `reversed=True`); YZ normal +X (sketch x is +Y).

Using all three planes (top, front, side views): base outline on XY padded up; upright
profile on XZ padded symmetric in Y; ribs/gussets on YZ padded symmetric in X. Touching or
overlapping solids fuse into one. Example: `models/examples/build_clevis_bracket.py` in the
mjorbit repo.

## 4. Robust references (topological naming)

FreeCAD 1.1 still loses element links when a parameter change regenerates the base shape.
In order of preference:
1. **Reference the origin**, not faces: sketch on XY/XZ/YZ with `offset="FlangeT + HubH"`
   instead of on a face; dimension from the origin, not from external edges.
2. Put rounds into the sketch (tangent arcs) when they are part of the profile.
3. For Fillet/Chamfer and face-attached sketches, pick by **geometry rules** that read the
   current parameters, and keep them in a `PICKS` dict:
   ```python
   PICKS = {
     "Fillet":    lambda doc, e: fc.is_circle(fc.num(doc, "HubD") / 2)(e) and fc.at_z(fc.num(doc, "FlangeT"))(e),
     "KeySketch": lambda doc, shape: fc.top_face(shape, "+Z"),        # face for a sketch
   }
   fil = fc.fillet(b, rev, fc.pick(doc, rev, PICKS["Fillet"]), "FilletR")
   ks  = fc.sketch(b, "KeySketch", (pat, PICKS["KeySketch"](doc, pat.Shape)))
   ```
   Then change parameters only through `fc.update(doc, PICKS, Name=value, ...)`, which
   re-picks in feature order. Never hard-code `Edge5` / `Face6` from a listing.

## 5. Change-test (required before calling it parametric)

```python
print(fc.report(b, 2.70))                                   # baseline
print(fc.update(doc, PICKS, FlangeD=100, HubD=46, BoltN="8"))   # [] = every feature valid
print(fc.report(b)["volume_mm3"])                           # must change sensibly
print(fc.update(doc, PICKS, FlangeD=80, HubD=40, BoltN="6"))    # restore defaults
```
Pass: all features valid, one solid, the volume moves in the expected direction, and
restoring defaults gives back the original volume exactly. Look at a screenshot of the changed
state too. Unitless params are set as strings (`BoltN="8"`), lengths as numbers (mm).

## 6. Rules the helpers already enforce (do the same in raw code)

- `doc.recompute()` **before** creating Hole, patterns and dress-ups: their property setters
  validate the base shape and raise `Base feature's TopoShape is invalid` otherwise.
- After `body.newObject(...)`, set `body.Tip = feature`. Pattern features do not move the Tip,
  and the next feature is then inserted before them (silently out of order).
- Set length/angle properties with `setExpression(prop, "Params.X")`, not plain values, when
  they should follow the spreadsheet (`Pad.Length`, `Hole.Diameter`, `PolarPattern.Occurrences`).
- Patterns and mirrors take additive/subtractive features as Originals, not other patterns
  (`Only additive and subtractive features can be transformed`). To mirror a patterned
  result, `fc.mirrored(b, None, "YZ", whole_shape=True)`.
- Expression constants carry units: `"Params.Length / 2 - 15 mm"`.
- Hide consumed sketches; keep one Body per part; check `fc.report(b)["errors"] == []`.

## 7. Geometry traps (seen in real builds)

Feature helpers raise `fc.FeatureError` at the first broken feature, with the cause. Read
the message, fix that feature, and rerun the whole script.

| Symptom | Cause | Fix |
|---|---|---|
| `shape not valid` after padding a rib or gusset | its outer face is flush with a plate's side face, or it touches only along an edge | embed it ≥0.5-2 mm into the parts it joins, and set it back ≥0.5 mm from flush faces (add a parameter such as `GussetInset`) |
| `2 solids` | the new pad floats free of the body (wrong offset, wrong side of XZ) | check the sketch plane offset and sign; XZ normal is −Y |
| `volume unchanged` on a pocket | it cuts air: wrong direction or wrong plane offset | `reversed=True`, or `through=True, symmetric=True` |
| Valid, but a hole breaks into a rib or wall | parameters overlap: hole spacing ignores other features | constrain the spacing from the features it must clear (e.g. hole X < half-width − rib thickness − hole radius), and look at a screenshot |
| Change-test fails only at large values | a sketch flips or profiles cross when one parameter outgrows another | test both up and down, and add derived parameters or limits in the docstring |

Plan in the docstring which parameters must stay in which order (bore < upright width, hole
spacing clear of ribs). That is the design intent the change-test checks.

## 8. Finish

`fc.show(doc)` for the screenshot, `fc.export(doc, b, folder, stem, step=True)` (add
`stl=True, stl_metres=True` for simulators), then optionally a drawing (`freecad-techdraw`).
Report: parameters, features, checks run (sketch DoF, validity, change-test), files.
