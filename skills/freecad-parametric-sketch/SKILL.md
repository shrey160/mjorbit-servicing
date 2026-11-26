---
name: freecad-parametric-sketch
description: Use when making a parametric drawing in FreeCAD through the MCP - fully constrained Sketcher sketches whose dimensions are bound to a spreadsheet of parameters. Covers geometry and constraint signatures, point and axis ids, DoF counting, binding expressions, debugging conflicting/redundant constraints, and ready-made constrained profiles (rectangle, rounded rectangle, slot, circles, bolt patterns, polylines).
---

# Parametric sketches (FreeCAD 1.1, via MCP execute_code)

Goal: every sketch **fully constrained** (DoF 0), every driving dimension **named** and
**bound to the Params spreadsheet**. Then the drawing is the design intent, and changing a cell
reshapes the part predictably. Load the helpers first (see `freecad-mcp`):

```python
import sys, importlib
sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
import fcmcp as fc; importlib.reload(fc)
```

## Workflow

1. **Parameters first**: `fc.params(doc, [(alias, value, unit, note), ...])`. Units: `"mm"`,
   `"deg"`, `""` for counts. Aliases must be identifiers, not unit symbols and not cell
   names. On 1.1, `H T C F W S N M l h t mm cm in ft rad min` (and `m s g A K V J Pa deg`) are
   rejected, so never use 1-2 letter names: write `Height`, `Thk`, `Width`, `HoleD`.
2. **Sketch on a plane**: `sk = fc.sketch(body, "BaseSketch", "XY")` (or `"XZ"`, `"YZ"`,
   a face `(feature, "Face6")`, optional `offset="Thickness"`).
3. **Geometry + constraints together** with the helpers (each one fully defines what it adds,
   relative to the origin):
   ```python
   fc.rounded_rect(sk, "Length", "Width", "CornerR")
   fc.slot(sk, "SlotLen", "SlotW", "SlotX", 0, "slotR")          # length is centre-to-centre
   g = fc.slot(sk, "SlotLen", "SlotW", "-Params.SlotX", 0, "slotL")
   fc.circle(sk, "BoreD")                                         # at the origin
   fc.hole_pattern_rect(sk, "BoltD", "SpacingX", "SpacingY")      # 4 equal holes, symmetric
   fc.rect(sk, "2 * Params.KeyDepth", "KeyW", "Params.BoreD / 2", 0, "key")
   ```
4. **Check** before building on it:
   ```python
   st = fc.check_sketch(sk)
   assert st["fully"] and not (st["conflicting"] or st["redundant"] or st["malformed"]), st
   ```
5. Hide it once a feature consumes it (feature helpers do).

Closed, non-overlapping loops only: an outer loop plus inner loops makes holes when padded
(one sketch can hold a plate outline and its slots). Open wires fail in Pad and crash
TechDraw's projector.

## Ids you need

- Geometry index = order added (`sk.addGeometry` returns it; a list returns a list).
- Point position: **1 start, 2 end, 3 centre** (circles/arcs).
- Special geometry: **-1 = H axis** (its point 1 is the **origin**), **-2 = V axis**.
- Arcs run counter-clockwise from start angle to end angle (radians).
- Sketch axes on body planes: XY -> (x=X, y=Y, normal +Z); **XZ -> (x=X, y=Z, normal -Y)**;
  YZ -> (x=Y, y=Z, normal +X).

## Constraint signatures (verified on FreeCAD 1.1)

| Constraint | Python | DoF removed |
|---|---|---|
| Coincident | `("Coincident", g1, p1, g2, p2)` | 2 |
| Point on object | `("PointOnObject", g, p, curve)` - e.g. curve -1 / -2 axes | 1 |
| Horizontal / Vertical | `("Horizontal", line)` or two points `("Horizontal", g1,p1,g2,p2)` | 1 |
| Parallel / Perpendicular | `("Parallel", l1, l2)`, `("Perpendicular", l1, l2)` | 1 |
| Tangent (edge-edge) | `("Tangent", g1, g2)` | 1 |
| Tangent at end points | `("Tangent", g1, p1, g2, p2)` - **also makes them coincident; never add Coincident too** | 3 |
| Equal | `("Equal", g1, g2)` (lengths or radii) | 1 |
| Symmetric about a point | `("Symmetric", g1,p1, g2,p2, g3,p3)` (e.g. `-1, 1` = origin) | 2 |
| Symmetric about a line | `("Symmetric", g1,p1, g2,p2, line)` (e.g. `-2` = V axis) | 2 |
| DistanceX / DistanceY | `("DistanceX", g1,p1, g2,p2, v)` - **signed**, negative values work; `("DistanceX", line, v)` | 1 |
| Distance | `("Distance", line, v)` length; `("Distance", g,p, line, v)` point-line | 1 |
| Radius / Diameter | `("Radius", g, r)`, `("Diameter", g, d)` | 1 |
| Angle | `("Angle", line, rad)` vs H axis; `("Angle", l1, l2, rad)` | 1 |
| Block | `("Block", g)` fixes a geometry in place (avoid in parametric work) | all |

Values in constraints are mm and **radians**. Bind instead of hard-coding:

```python
i = sk.addConstraint(Sketcher.Constraint("DistanceX", -1, 1, g, 3, 25.0))
sk.renameConstraint(i, "SlotInner")                       # names must be unique per sketch
sk.setExpression("Constraints.SlotInner", "Params.SlotX - Params.SlotLen / 2")
# fc.constrain(sk, Sketcher.Constraint(...), "Params.SlotX - Params.SlotLen / 2", "SlotInner") does all three
```
Constants in expressions need units when combined with lengths: `"Params.Thk + 10 mm"`
(`+ 10` raises `Unit mismatch in plus operation`); multiplying/dividing by plain numbers is
fine (`"Params.Width / 4"`, `"2 * Params.KeyDepth"`).
Angles bind cleanly when the parameter has a unit: `("Ang", 30, "deg")` -> `Params.Ang`.
A unitless parameter used for an angle needs `"Params.N * 1 deg"`.

## DoF bookkeeping (count before you constrain)

Free DoF: point 2, line 4, circle 3, arc 5. Subtract per constraint (table above). Example,
rounded rectangle: 4 lines + 4 arcs = 36; 8 end-point tangents -24; H/V -4; 3 Equal -3;
centre symmetric -2; radius, length, width -3 = **0**. If the count says 0 but
`check_sketch` says not fully constrained, a constraint is redundant (it removed nothing).

## Debugging

- `fc.check_sketch(sk)` -> `solve` (0 ok, -3 conflicting, other negatives = solver failure),
  `dof`, `fully`, `conflicting`, `redundant`, `malformed`.
- **The lists are 1-based**: `conflicting [3, 4]` = `sk.Constraints[2]` and `[3]`.
- Typical causes: Coincident added on top of an end-point Tangent; Horizontal on both
  parallel sides of a rectangle that is also Symmetric about a line; a dimension measured
  between two points that are already tied (e.g. DistanceY between points of one horizontal
  line = 0 -> wrong points, check pos ids); Equal plus explicit sizes on both.
- Watch for geometry flipping when a parameter changes a lot (solver picks the nearest
  solution). Draw the initial geometry close to the parametric values, and test with a big
  change (see `freecad-parametric-body`).

## Recipes beyond the helpers

- **Polygon profile** (e.g. half-section for a revolve): `g = fc.polyline(sk, pts)` then
  Horizontal/Vertical per segment, one point on an axis, and DistanceX/DistanceY from the origin
  (`-1, 1`) to the corners. See `freecad-parametric-body/examples/flange_hub.py`.
- **Mirrored features**: build one side, then `Symmetric` each new point/centre with its twin
  about `-2` (V axis) or `-1` (H axis) and `Equal` for sizes (`fc.copy_circle`).
- **Construction geometry**: `sk.addGeometry(Part.Point(...), True)` or a construction line;
  use it as a symmetry centre or reference (`fc.rect` off-origin does this).
- **External geometry** (reference an existing edge): `sk.addExternal("Pad", "Edge3")`; ids
  then start at -3. Prefer origin-relative dimensions: external links break on topology changes.
- Showing a sketch to the user: screenshots of bare sketches capture badly; plot
  `sk.Shape.Edges` with matplotlib and annotate the named dimensions.
