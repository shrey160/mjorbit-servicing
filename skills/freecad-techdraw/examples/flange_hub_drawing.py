"""Worked example for the freecad-techdraw skill: an A4 drawing of the flange hub.

Needs the flange_hub document (run ../../freecad-parametric-body/examples/flange_hub.py first).
Run inside FreeCAD (MCP execute_code):
    p = r"<path-to-skills>/freecad-techdraw/examples/flange_hub_drawing.py"
    exec(open(p).read(), {"__file__": p, "__name__": "__main__", "OUT": r"/tmp/techdraw_out"})
"""
import importlib
import os
import sys
import tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "freecad-mcp", "scripts"))
import fcmcp as fc
importlib.reload(fc)
import FreeCAD as App


def make_drawing(doc, out_dir):
    for o in [o for o in doc.Objects if o.TypeId.startswith("TechDraw")][::-1]:
        doc.removeObject(o.Name)                       # idempotent re-runs
    b = doc.Body
    mass = fc.report(b, 2.70)["mass_g"] / 1000
    page = fc.drawing_page(doc, texts={"FC-Title": "Flange hub", "Subtitle": "Revolved section, 6x M5 cbore",
                                       "Designed_by_Name": "FreeCAD MCP", "Drawing_number": "MX-003",
                                       "FC-SC": "1 : 1", "FC-SH": "1 / 1", "Weight": f"{mass:.3f} (Al)"})
    g = fc.proj_group(page, b, ("Front", "Top", "FrontTopRight"), scale=1.0, at=(95, 80),
                      positions={"Front": (0, 0), "Top": (0, 80), "FrontTopRight": (135, 60)})
    fc.wait_views(g)
    f, t = fc.view_of(g, "Front"), fc.view_of(g, "Top")
    bb = b.Shape.BoundBox
    R, zc = bb.XLength / 2, (bb.ZMin + bb.ZMax) / 2          # view origin = bbox centre, y down
    bot, top = bb.ZMin - zc, bb.ZMax - zc                     # model z -> view y is -(z - zc)
    hub_r, ft = fc.num(doc, "HubD") / 2, fc.num(doc, "FlangeT")
    s = g.Scale
    fc.dimension(page, f, "DistanceX", [fc.vertex(f, -R, -bot), fc.vertex(f, R, -bot)], (0, -32))
    fc.dimension(page, f, "DistanceY", [fc.vertex(f, -R, -bot), fc.vertex(f, -hub_r, -top)], (-52, 0))
    fc.dimension(page, f, "DistanceY", [fc.vertex(f, R, -bot), fc.vertex(f, R, -(bot + ft))], (50, -12))
    fc.dimension(page, t, "Diameter", [fc.edge(t, lambda e: abs(getattr(e.Curve, "Radius", 0) - hub_r * s) < 1e-3)], (-40, 30))
    cb_r = fc.num(doc, "CbD") / 2
    fc.dimension(page, t, "Diameter", [fc.edge(t, lambda e: abs(getattr(e.Curve, "Radius", 0) - cb_r * s) < 1e-3)], (45, 32))
    dims = fc.refresh_dimensions(doc)
    pdf = os.path.join(out_dir, "flange_hub_drawing.pdf")
    fc.export_page(page, pdf=pdf, svg=os.path.join(out_dir, "flange_hub_drawing.svg"))
    return dims, pdf


if __name__ == "__main__":
    out = globals().get("OUT") or tempfile.gettempdir()
    print(make_drawing(App.getDocument("flange_hub"), out))
