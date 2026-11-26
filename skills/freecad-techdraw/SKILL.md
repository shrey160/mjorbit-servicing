---
name: freecad-techdraw
description: Use when making a 2D engineering drawing of a FreeCAD model through the MCP - TechDraw page and template, front/top/side/isometric projection group, dimensions attached to model vertices and edges, title block, and PDF/SVG/DXF export. Covers waiting for TechDraw's background projection, view coordinate conventions and the export/screenshot traps.
---

# TechDraw drawings (FreeCAD 1.1 via MCP)

Uses the `freecad-mcp` helpers. Verified example: `examples/flange_hub_drawing.py`
(front + top + isometric, 5 dimensions, title block, PDF/SVG), and
`models/examples/build_clevis_bracket.py` in the mjorbit repo (front/top/right/iso, 12 dims).

Run everything with `execute_code` (never `execute_code_async`). TechDraw computes projections
in its own background thread; `fc.wait_views(group)` waits for them by pumping the Qt loop on the
GUI thread. That has nothing to do with the MCP's async tool.

## Recipe

```python
import sys, importlib
sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
import fcmcp as fc; importlib.reload(fc)

page = fc.drawing_page(doc, "ISO/A4_Landscape_TD.svg", texts={
    "FC-Title": "Clevis bracket", "Subtitle": "...", "Designed_by_Name": "FreeCAD MCP",
    "Drawing_number": "MX-002", "FC-SC": "1 : 1.25", "FC-SH": "1 / 1", "Weight": "0.257"})
g = fc.proj_group(page, body, ("Front", "Top", "Right", "FrontTopRight"), scale=0.8,
                  at=(80, 100), positions={"Front": (0, 0), "Top": (0, 72),
                                           "Right": (82, 0), "FrontTopRight": (155, 62)})
fc.wait_views(g)                                   # REQUIRED before picking geometry
f = fc.view_of(g, "Front")
fc.dimension(page, f, "DistanceX", [fc.vertex(f, -45, 35), fc.vertex(f, 45, 35)], (0, -42))
fc.dimension(page, f, "Diameter", [fc.edge(f, lambda e: abs(getattr(e.Curve, "Radius", 0) - 9 * g.Scale) < 1e-3)], (38, 22))
print(fc.refresh_dimensions(doc))                  # check every value against the design
fc.export_page(page, pdf=".../part_drawing.pdf", svg=".../part_drawing.svg")
```

## Conventions (get these right or dimensions attach to the wrong points)

- **Projection**: `"Third angle"` puts Top above Front and Right to the right; `"First angle"`
  is the ISO/European layout. Say which on the sheet.
- **View 2D frame** (for `vertex()`/`edge()`): origin = centre of the part's projected bounding
  box, units = model mm x scale, **y points down**. Front view: x = model X, y = -(Z - Zmid)
  (verified); Top: x = X; Right: x = Y (verified); the sign of y in Top/Right depends on the
  view direction, so for asymmetric parts confirm with `fc.list_view_geometry(view)` before
  picking. `fc.vertex(view, x, y)` takes model mm and applies the scale; `edge()` predicates
  see scaled values (radius x scale).
- **Dimension label position** `at=(x, y)`: page mm relative to the view centre, **y up**.
  Set it for every dimension; the default (0, 0) piles all labels in the middle.
- Unsure what is where: `fc.list_view_geometry(view)` lists vertices and edges (indices are
  what `References2D` uses: `Vertex0`, `Edge12`, 0-based).
- `MeasureType = "Projected"` measures the 2D view; values come out in model mm.

## Traps (each one cost a debugging round)

| Symptom | Cause | Fix |
|---|---|---|
| `min() arg is an empty sequence`, empty `getVisibleVertexes()` | projection runs on worker threads | `fc.wait_views(g)`; it pumps the Qt loop (a bare `time.sleep` never delivers the result) |
| Dimensions 0 or value / scale (e.g. 112.5 for 90 at 0.8) | geometry picked before re-projection after a scale change | set `Scale` **before** `addProjection` (`proj_group` does), wait, then pick |
| PDF shows only a corner of the sheet | page not open in the GUI when exported | `fc.export_page` opens it (`page.ViewObject.doubleClicked()`) and pumps 2 s first |
| `Could not read the new template file` | wrong template path | templates live in `<ResourceDir>/Mod/TechDraw/Templates/ISO/` (and `ANSI/`, `DIN/`...) |
| MCP screenshot garbled after making a drawing | the page window is active | `fc.show(doc)` brings the 3D view back |
| Labels overlap the title block or each other | auto layout | `AutoDistribute=False` + explicit view positions (`positions=`), leave ~60 mm at the bottom right for the A4 TD title block |
| Projector crash `NCollection_Array1::Create` | open wires / loose edges in the source | project solids or closed faces only |

## Layout guide (A4 landscape, 297 x 210 mm, TD title block about 150 x 57 mm bottom right)

- Pick the scale so Front + Top + dimension space fit in about 185 mm height: scale 1 for parts
  up to ~80 mm, 0.8 up to ~100 mm, 0.5 beyond. Write the scale into `FC-SC` (it is not automatic).
- Front view centre around y = 90-100, Top 70-80 above it, Right 80-90 to the right, the
  isometric (`FrontTopRight`) in the free upper-right area.
- Dimension each feature once, in the view where it shows true size: diameters in the view
  that shows circles, heights in Front/Right, widths in Top.

## Export

`fc.export_page(page, pdf=..., svg=..., dxf=...)` (`TechDrawGui.exportPageAsPdf/Svg`,
`TechDraw.writeDXFPage`). Read the PDF back (the Read tool renders it) and check layout and
values before telling the user it is done. Dimensions are associative: after a model change,
`fc.refresh_dimensions(doc)` and re-export; label positions are fixed and may need nudging.
