---
name: freecad-3d-model
description: Use when making a 3D model in FreeCAD through the MCP that does not need to be a fully parametric PartDesign body - quick concept parts, CSG from primitives and booleans, extrusions, revolves, lofts, multi-part assemblies with per-part mass, and exports for simulators (STEP, STL in metres, mass properties for MuJoCo/URDF). For spreadsheet-driven parts use freecad-parametric-body instead.
---

# Direct 3D modelling (Part workbench, FreeCAD 1.1 via MCP)

Choose this when speed matters more than editability: concept geometry, fixtures, visual
detail, simulation bodies. If the user will change dimensions later, use
`freecad-parametric-body`. Tools and helpers: `freecad-mcp`.

## Two routes

- **No code, one or two objects**: `create_object` with `Part::Box`, `Part::Cylinder`,
  `Part::Cone`, `Part::Sphere`, `Part::Tube`, then `Part::Cut` / `Part::Fuse` /
  `Part::Common` with `{"Base": "Box", "Tool": "Cylinder"}`; adjust with `edit_object`
  (Placement as `{"Base": {...}, "Rotation": {"Axis": {...}, "Angle": deg}}`).
- **Anything more**: a build script with the `Part` API, run through `execute_code`
  (pattern in `freecad-mcp`). Shapes are computed in Python, then stored in `Part::Feature`
  objects - one feature per physical part, so colours, masses and exports stay per part.

## Part API essentials (verified on 1.1)

```python
import FreeCAD as App, Part
V = App.Vector
box  = Part.makeBox(60, 40, 10, V(-30, -20, 0))                 # l, w, h, corner
cyl  = Part.makeCylinder(8, 30, V(0, 0, 0), V(0, 0, 1))          # r, h, base, axis
cone = Part.makeCone(10, 4, 20, V(0, 0, 0), V(0, 0, 1))          # r1, r2, h
solid = box.fuse(cyl).removeSplitter()                           # removeSplitter merges coplanar faces
solid = solid.cut(Part.makeCylinder(4, 50, V(0, 0, -5)))
edges = [e for e in solid.Edges if isinstance(e.Curve, Part.Line)
         and abs(e.tangentAt(e.FirstParameter).z) > 0.99]        # pick edges by geometry
solid = solid.makeFillet(3, edges)                               # or makeChamfer(size, edges)
prism = Part.Face(Part.makePolygon([V(0,0,0), V(20,0,0), V(20,5,0), V(0,5,0), V(0,0,0)])).extrude(V(0, 0, 4))
turned = Part.Face(Part.makePolygon([V(5,0,0), V(10,0,0), V(10,0,8), V(5,0,8), V(5,0,0)])).revolve(V(0,0,0), V(0,0,1), 360)
loft = Part.makeLoft([Part.Wire(Part.makeCircle(10, V(0,0,0))), Part.Wire(Part.makeCircle(4, V(0,0,20)))], True)
placed = solid.copy(); placed.Placement = App.Placement(V(100, 0, 0), App.Rotation(V(0, 0, 1), 90))
assert solid.isValid() and len(solid.Solids) == 1
obj = doc.addObject("Part::Feature", "Bracket"); obj.Shape = solid
if App.GuiUp: obj.ViewObject.ShapeColor = (0.62, 0.66, 0.72)
doc.recompute()
```

Units are mm and degrees. A closed polygon must repeat its first point. Faces need closed
wires; `extrude` of a wire gives a shell, of a face a solid.

## Multi-part models (assemblies, satellites, robots)

- Build each part as its own solid in the part's final position; keep a small class or list of
  `{name, shape, mass, rgba}` (see `models/clients/build_clients.py` in the mjorbit repo:
  `add_box`, `add_shape`, `_place(shape, base, axis)` helpers).
- Mass properties: `Volume`, `CenterOfMass`, `MatrixOfInertia` exist on **solids**, not on
  compounds (`'Part.Compound' object has no attribute 'CenterOfMass'`). For a compound part,
  spread its mass over `shape.Solids` by volume and sum with the parallel-axis theorem.
  Inertia from FreeCAD is per unit density in mm^5: multiply by density, convert to kg m^2.
- Visual-only details (no mass, no collision) are separate parts with mass 0, so a simulator
  export can mark them as visual geoms.

## Export for simulators and CAD

```python
import MeshPart
m = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.5, AngularDeflection=0.35)
s = App.Matrix(); s.scale(0.001, 0.001, 0.001); m.transform(s)      # mm -> m for MuJoCo/ROS
m.write("parts/bracket.stl")
shape.exportStep("bracket.step")                                     # STEP stays in mm
```
Or `fc.export(doc, obj, folder, stem, step=True, stl=True, stl_metres=True)`.
LinearDeflection is in mm: 0.1-0.5 for small parts, ~1 for metre-scale bodies (smaller =
more triangles). For MuJoCo keep collision as primitives (box/cylinder/capsule) and use meshes
for visuals; write explicit `<inertial>` from the computed mass properties.

## Heavy or fragile geometry

- Many booleans: fuse the tools together first, then do one boolean against the main body.
- Slow GUI-thread work: `execute_code` with `timeout=600` (up to 1800). This is the default
  answer.
- `execute_code_async` only for one pure-OCCT computation on shapes already in variables, such
  as `big = a.fuse(b).removeSplitter()`. Inside it, never call `newDocument`, `addObject`,
  `recompute`, `save`, `ViewObject`, `Gui` or any `fc.*` helper; doing so wedges FreeCAD
  until it is restarted. Hand the result over with `commit(lambda: setattr(obj, "Shape", big))`,
  or keep it in a variable and apply it in a later `execute_code`. Poll `get_async_status`.
- Lofts, sweeps and threads that may crash OCCT: use `execute_code_headless`, a separate
  freecadcmd process (it needs `--freecadcmd` in the client config, see `freecad-mcp`). The
  script imports FreeCAD and Part itself and writes a .FCStd or `.brep`, then call
  `reload_document`. A crash there kills only the helper process.
- Booleans of solids that only touch (coplanar faces, a shared edge) can come out invalid.
  Overlap the tools by ≥0.5 mm and check `isValid()` after each boolean.

## Check and show

`shape.isValid()`, `len(shape.Solids)`, `shape.Volume`, `shape.BoundBox`; then
`fc.show(doc)` (raises the 3D view, fits) for the screenshot. Save the .FCStd and keep the
build script; clean `.FCBak` files.
