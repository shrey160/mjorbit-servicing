"""Worked example for the freecad-parametric-body skill: a motor flange hub.

Exercises: Params sheet -> revolve from an XZ half-section -> fillet and chamfer picked by
geometry -> counterbored Hole on an offset XY sketch -> PolarPattern with a unitless count
-> keyway Pocket sketched on a face found by geometry -> report -> change-test.

Run inside FreeCAD (MCP execute_code):
    p = r"<path-to-skills>/freecad-parametric-body/examples/flange_hub.py"
    exec(open(p).read(), {"__file__": p, "__name__": "__main__"})
"""
import importlib
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "freecad-mcp", "scripts"))
import fcmcp as fc
importlib.reload(fc)
import Sketcher

PARAMS = [
    ("FlangeD", 80, "mm", "flange diameter"),
    ("FlangeT", 8, "mm", "flange thickness"),
    ("HubD", 40, "mm", "hub diameter"),
    ("HubH", 25, "mm", "hub height above the flange"),
    ("BoreD", 16, "mm", "shaft bore"),
    ("KeyW", 5, "mm", "keyway width"),
    ("KeyDepth", 2.5, "mm", "keyway depth into the hub"),
    ("BoltPCD", 60, "mm", "bolt pitch circle diameter"),
    ("BoltD", 5.5, "mm", "M5 clearance"),
    ("CbD", 10, "mm", "counterbore diameter"),
    ("CbDepth", 5, "mm", "counterbore depth"),
    ("BoltN", 6, "", "number of bolts (unitless)"),
    ("FilletR", 3, "mm", "hub-to-flange fillet"),
    ("ChamferS", 1, "mm", "bore chamfer"),
]


PICKS = {   # dress-up edge rules: (doc, edge) -> bool, evaluated with the current params
    "Fillet": lambda doc, e: (fc.is_circle(fc.num(doc, "HubD") / 2)(e) and fc.at_z(fc.num(doc, "FlangeT"))(e)),
    "Chamfer": lambda doc, e: (fc.is_circle(fc.num(doc, "BoreD") / 2)(e) and
                               (fc.at_z(0)(e) or fc.at_z(fc.num(doc, "FlangeT + HubH"))(e))),
    "KeySketch": lambda doc, shape: fc.top_face(shape, "+Z"),   # face-attached sketch
}


def build(name="flange_hub"):
    doc = fc.new_doc(name)
    fc.params(doc, PARAMS)
    b = fc.body(doc)

    # 1. half cross-section on XZ (sketch x = radius, sketch y = Z), revolved about sketch V axis
    sk = fc.sketch(b, "SectionSketch", "XZ")
    R0, R1, T, RH, H = 8, 40, 8, 20, 33
    g = fc.polyline(sk, [(R0, 0), (R1, 0), (R1, T), (RH, T), (RH, H), (R0, H)])
    for i in (0, 2, 4):
        sk.addConstraint(Sketcher.Constraint("Horizontal", g[i]))
    for i in (1, 3, 5):
        sk.addConstraint(Sketcher.Constraint("Vertical", g[i]))
    sk.addConstraint(Sketcher.Constraint("PointOnObject", g[0], 1, fc.H_AXIS))
    fc.constrain(sk, Sketcher.Constraint("DistanceX", -1, 1, g[0], 1, R0), "Params.BoreD / 2", "BoreR")
    fc.constrain(sk, Sketcher.Constraint("DistanceX", -1, 1, g[0], 2, R1), "Params.FlangeD / 2", "FlangeR")
    fc.constrain(sk, Sketcher.Constraint("DistanceY", g[1], 1, g[1], 2, T), "FlangeT")
    fc.constrain(sk, Sketcher.Constraint("DistanceX", -1, 1, g[3], 2, RH), "Params.HubD / 2", "HubR")
    fc.constrain(sk, Sketcher.Constraint("DistanceY", g[3], 1, g[3], 2, H - T), "HubH")
    st = fc.check_sketch(sk)                    # check every sketch before building on it
    assert st["fully"] and not (st["conflicting"] or st["redundant"]), st
    rev = fc.revolve(b, sk, "V_Axis", 360, "Hub")
    doc.recompute()

    # 2. fillet the hub/flange corner and chamfer both bore ends, picked by geometry rules
    #    (rules read the current params, so update() can re-pick after a change)
    fil = fc.fillet(b, rev, fc.pick(doc, rev, PICKS["Fillet"]), "FilletR")
    cha = fc.chamfer(b, fil, fc.pick(doc, fil, PICKS["Chamfer"]), "ChamferS")

    # 3. one counterbored bolt hole on an XY sketch lifted to the flange top, then a polar pattern
    hs = fc.sketch(b, "BoltSketch", "XY", offset="FlangeT")
    fc.circle(hs, "BoltD", "Params.BoltPCD / 2", 0, "bolt")
    hl = fc.hole(b, hs, "BoltD", name="BoltHole", counterbore=("CbD", "CbDepth"))
    hl.Reversed = False
    pat = fc.polar_pattern(b, [hl], "BoltN", "Z", 360, "BoltPattern")
    doc.recompute()

    # 4. keyway: sketch on the hub's top face (found by geometry), pocket through all
    #    (a sketch on the XY plane with offset="FlangeT + HubH" would avoid the face link entirely)
    ks = fc.sketch(b, "KeySketch", (pat, PICKS["KeySketch"](doc, pat.Shape)))
    fc.rect(ks, "2 * Params.KeyDepth", "KeyW", "Params.BoreD / 2", 0, "key")
    key = fc.pocket(b, ks, through=True, name="Keyway")
    doc.recompute()
    return doc, b, [sk, hs, ks]


if __name__ == "__main__":
    doc, b, sks = build()
    for s in sks:
        print(fc.check_sketch(s))
    print(fc.report(b, density_g_cm3=2.70))
    # change-test: every feature must stay valid; then restore the defaults
    print("changed, errors:", fc.update(doc, PICKS, FlangeD=100, BoltPCD=78, HubD=46, BoreD=20, BoltN="8"),
          fc.report(b)["volume_mm3"])
    print("restored, errors:", fc.update(doc, PICKS, **{k: (f"{v}" if u == "" else v) for k, v, u, _ in PARAMS}),
          fc.report(b)["volume_mm3"])
    fc.show(doc)
