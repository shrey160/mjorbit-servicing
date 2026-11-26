"""fcmcp: helpers for driving FreeCAD 1.1 through the FreeCAD MCP `execute_code` tool.

Load inside FreeCAD (execute_code):
    import sys, importlib
    sys.path.insert(0, r"<path-to-skills>/freecad-mcp/scripts")
    import fcmcp; importlib.reload(fcmcp)

Threading: run everything through execute_code (FreeCAD's GUI thread). Never call these helpers
from execute_code_async: document/GUI writes from that background thread wedge FreeCAD. The
document/GUI entry points raise ThreadError there, and feature helpers raise FeatureError at the
first broken feature (set fcmcp.STRICT = False to only warn).

Conventions
- Dimensions are numbers (mm, deg) or strings. A plain identifier string such as "Length" means
  the spreadsheet alias Params.Length; any other string is used as an expression as written
  ("Params.SlotX - Params.SlotLen / 2"). Strings are bound with setExpression, so editing the
  spreadsheet rebuilds the model; numbers are fixed values.
- Sketch helpers add geometry AND the constraints that fully define it relative to the sketch
  origin, so a sketch built only from them is fully constrained. Check with check_sketch().
- Sketch plane axes (FreeCAD 1.1, body origin planes):
    XY: sketch x -> +X, y -> +Y, normal +Z
    XZ: sketch x -> +X, y -> +Z, normal -Y   (a one-sided pad on XZ grows toward -Y)
    YZ: sketch x -> +Y, y -> +Z, normal +X
- Sketcher ids: point pos 1 start, 2 end, 3 centre; geometry -1 = H axis (its pos 1 = origin),
  -2 = V axis.
"""
import math
import os
import re
import time

import FreeCAD as App
import Part
import Sketcher

V = App.Vector
ORIGIN, H_AXIS, V_AXIS = -1, -1, -2
_IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
VERSION = "2026-11-17b"
STRICT = True   # feature helpers raise FeatureError on a broken feature (set False to only warn)


class FeatureError(RuntimeError):
    """A PartDesign feature computed to an error, an invalid shape, or no change at all."""


class ThreadError(RuntimeError):
    """A document/GUI helper was called off FreeCAD's GUI thread (execute_code_async)."""


# ---------------------------------------------------------------- safety rails
def _on_gui_thread():
    import threading
    return threading.current_thread() is threading.main_thread()


def _gui_thread_only(fn):
    """Refuse to touch the document or the view from a background thread.

    execute_code_async runs code in a worker thread. FreeCAD documents and the Coin3D scene
    are not thread-safe: addObject / recompute / saveAs / ViewObject / Qt from that thread can
    wedge FreeCAD's event loop (Windows then reports 'freecad.exe stopped interacting' and
    the RPC server stops answering until FreeCAD is restarted). Fail fast instead."""
    import functools

    @functools.wraps(fn)
    def wrapper(*a, **k):
        if not _on_gui_thread():
            raise ThreadError(
                f"fcmcp.{fn.__name__}() touches the FreeCAD document/GUI and was called from a "
                "background thread (execute_code_async). Run builds with execute_code (raise its "
                "timeout if needed); use execute_code_async only for pure Part/OCCT shape math and "
                "hand document writes to commit(fn).")
        return fn(*a, **k)
    wrapper._fcmcp_guarded = True
    return wrapper


def preflight():
    """One call to check the session before building. Prints and returns a dict."""
    out = dict(fcmcp=VERSION, freecad=".".join(App.Version()[:3]), gui=bool(App.GuiUp),
               gui_thread=_on_gui_thread(), documents=list(App.listDocuments()))
    if not out["gui_thread"]:
        out["problem"] = "not on the GUI thread: use execute_code, not execute_code_async"
    print("fcmcp preflight:", out)
    return out


def run(path, **overrides):
    """Run a build script file in a fresh namespace as __main__ - the recommended way to
    execute a script through execute_code:  fc.run(r"C:/.../build_part.py")
    Keyword args are passed to the script as the dict OVERRIDES. Returns the namespace."""
    ns = {"__file__": path, "__name__": "__main__", "OVERRIDES": overrides}
    with open(path, encoding="utf-8") as fh:
        code = compile(fh.read(), path, "exec")
    exec(code, ns)
    return ns


_HINTS = {
    "PartDesign::Pad": "Pad: is the sketch closed? Solids that only touch along an edge or a "
                       "point, or whose side faces are flush/coplanar with the body's, can give "
                       "invalid fuses - overlap by >=0.5 mm or set the new solid back >=0.5 mm. A pad floating free of the body gives "
                       "2 solids. One-sided pads on XZ grow toward -Y.",
    "PartDesign::Pocket": "Pocket: 'removed nothing' usually means the wrong direction (XZ normal "
                          "is -Y; try reversed=True, or through=True with symmetric=True) or a "
                          "sketch outside the material.",
    "PartDesign::Revolution": "Revolve: the profile must not cross the axis; one closed loop.",
    "PartDesign::Groove": "Groove: profile must overlap the material and not cross the axis.",
    "PartDesign::Fillet": "Fillet: radius too big for a neighbouring face, or edges lost after a "
                          "parameter change - re-pick by geometry (fc.update with PICKS).",
    "PartDesign::Chamfer": "Chamfer: size too big, or edges lost - re-pick by geometry.",
    "PartDesign::Hole": "Hole: circle centres must lie on the entry face; check depth/direction.",
}


def _done(bdy, f):
    """Recompute and verify a feature the helpers just made. A broken feature otherwise
    surfaces several steps later as a confusing error on something else."""
    bdy.Document.recompute()
    problems = []
    state = list(getattr(f, "State", []))
    if "Invalid" in state or "Error" in state:
        problems.append(f"state {state}")
    shp = f.Shape
    if shp.isNull():
        problems.append("empty shape")
    else:
        if not shp.isValid():
            problems.append("shape not valid (OCCT check failed)")
        if len(shp.Solids) != 1:
            problems.append(f"{len(shp.Solids)} solids (a Body must stay one solid)")
        base = getattr(f, "BaseFeature", None)
        transform = "Pattern" in f.TypeId or "Mirrored" in f.TypeId
        if base is not None and not base.Shape.isNull() and not transform and \
                abs(shp.Volume - base.Shape.Volume) < 1e-6:
            problems.append("volume unchanged: the feature added/removed nothing")
    if not problems:
        return f
    msg = (f"{f.Name} ({f.TypeId}) is broken: " + "; ".join(problems) + ". "
           + _HINTS.get(f.TypeId, "Check the profile sketch and the feature direction."))
    if STRICT:
        raise FeatureError(msg)
    print("WARNING:", msg)
    return f


# ---------------------------------------------------------------- document and parameters
def new_doc(name, replace=True):
    """Create a document, closing an open one with the same name first (no save)."""
    if replace and name in App.listDocuments():
        App.closeDocument(name)
    return App.newDocument(name)


def params(doc, rows, name="Params"):
    """Parameter spreadsheet. rows: [(alias, value, unit, note), ...]; unit "" for counts.

    Aliases must be identifiers that are not unit symbols and not cell names. Verified rejected
    on 1.1: H T C F W S N M l h t mm cm in ft rad min (plus m s g A K V J Pa deg). Use words:
    Length, Thk, HoleD - never 1-2 letter names."""
    sh = doc.getObject(name) or doc.addObject("Spreadsheet::Sheet", name)
    sh.set("A1", "Parameter"); sh.set("B1", "Value"); sh.set("C1", "Note")
    for r, row in enumerate(rows, start=2):
        alias, val, unit = row[0], row[1], row[2] if len(row) > 2 else "mm"
        sh.set(f"A{r}", alias)
        sh.set(f"B{r}", f"={val} {unit}".strip())
        try:
            sh.setAlias(f"B{r}", alias)
        except ValueError:
            raise ValueError(
                f"Parameter name {alias!r} rejected by the spreadsheet: it is a unit symbol "
                "(verified rejected: H T C F W S N M l h t mm cm in ft rad min; also m s g A K V J "
                "Pa deg) or a cell name (A1, AB12). Avoid 1-2 letter names; use e.g. "
                f"{alias + 'Len' if len(alias) < 3 else alias + '_'}.") from None
        if len(row) > 3:
            sh.set(f"C{r}", row[3])
    doc.recompute()
    return sh


def set_params(doc, recompute=True, sheet="Params", **values):
    """set_params(doc, Length=120, Angle="45 deg"): numbers get mm, strings are used as given."""
    sh = doc.getObject(sheet)
    for alias, v in values.items():
        cell = sh.getCellFromAlias(alias)
        if cell is None:
            raise KeyError(f"no alias {alias} in {sheet}")
        sh.set(cell, f"={v} mm" if isinstance(v, (int, float)) else f"={v}")
    if recompute:
        doc.recompute()


def expr(x):
    """Expression string for x, or None when x is a plain number."""
    if isinstance(x, str):
        return f"Params.{x}" if _IDENT.match(x) else x
    return None


def num(doc, x, sheet="Params"):
    """Current numeric value (mm or deg) of a number, alias or expression."""
    if not isinstance(x, str):
        return float(x)
    q = doc.getObject(sheet).evalExpression(expr(x))
    return float(getattr(q, "Value", q))


# ---------------------------------------------------------------- body, planes, sketches
def body(doc, name="Body"):
    return doc.getObject(name) or doc.addObject("PartDesign::Body", name)


def plane(bdy, which):
    """Body origin plane or axis: 'XY', 'XZ', 'YZ', 'X', 'Y', 'Z' (or full role names)."""
    role = {"XY": "XY_Plane", "XZ": "XZ_Plane", "YZ": "YZ_Plane",
            "X": "X_Axis", "Y": "Y_Axis", "Z": "Z_Axis"}.get(which.upper(), which)
    return next(f for f in bdy.Origin.OriginFeatures if f.Role == role)


def sketch(bdy, name, on="XY", offset=None, reverse=False):
    """Sketch attached to an origin plane ('XY'/'XZ'/'YZ') or to a face: on=(feature, 'Face6').

    offset: shift along the sketch normal (number or param/expression). reverse flips the
    attachment (MapReversed), which flips the normal."""
    sk = bdy.newObject("Sketcher::SketchObject", name)
    if isinstance(on, str):
        sk.AttachmentSupport = [(plane(bdy, on), "")]
    else:
        sk.AttachmentSupport = [tuple(on)]
    sk.MapMode = "FlatFace"
    sk.MapReversed = reverse
    if offset is not None:
        e = expr(offset)
        if e:
            sk.setExpression(".AttachmentOffset.Base.z", e)
        else:
            sk.AttachmentOffset = App.Placement(V(0, 0, offset), App.Rotation())
    return sk


def _name_for(sk, e, name):
    used = {c.Name for c in sk.Constraints if c.Name}
    base = name or (e[7:] if e.startswith("Params.") and _IDENT.match(e[7:]) else "d")
    n, k = base, 1
    while n in used:
        k += 1
        n = f"{base}{k}"
    return n


def constrain(sk, c, value=None, name=None):
    """Add a Sketcher.Constraint. If value is an alias/expression, name it and bind it."""
    i = sk.addConstraint(c)
    e = expr(value)
    if e:
        n = _name_for(sk, e, name)
        sk.renameConstraint(i, n)
        sk.setExpression(f"Constraints.{n}", e)
    elif name:
        sk.renameConstraint(i, name)
    return i


def _dim(doc, sk, kind, args, value, name=None):
    return constrain(sk, Sketcher.Constraint(kind, *args, num(doc, value)), value, name)


def place_point(sk, g, pos, x, y, name=None):
    """Fix point (g, pos) at (x, y) from the sketch origin (numbers, aliases or expressions).
    0 puts it on the axis; negative values work (DistanceX/Y are signed)."""
    doc = sk.Document
    for val, kind, axis in ((x, "DistanceX", V_AXIS), (y, "DistanceY", H_AXIS)):
        if not isinstance(val, str) and abs(val) < 1e-12:
            sk.addConstraint(Sketcher.Constraint("PointOnObject", g, pos, axis))
        else:
            _dim(doc, sk, kind, (ORIGIN, 1, g, pos), val, name and f"{name}_{kind[-1].lower()}")


def rect(sk, w, h, cx=0, cy=0, name=None):
    """Rectangle w x h centred at (cx, cy). Returns the 4 line ids (bottom, right, top, left)."""
    doc = sk.Document
    W, H, X, Y = (num(doc, v) for v in (w, h, cx, cy))
    p = [V(X - W / 2, Y - H / 2, 0), V(X + W / 2, Y - H / 2, 0), V(X + W / 2, Y + H / 2, 0), V(X - W / 2, Y + H / 2, 0)]
    g0 = sk.GeometryCount
    sk.addGeometry([Part.LineSegment(p[i], p[(i + 1) % 4]) for i in range(4)], False)
    g = list(range(g0, g0 + 4))
    sk.addConstraint([Sketcher.Constraint("Coincident", g[i], 2, g[(i + 1) % 4], 1) for i in range(4)] +
                     [Sketcher.Constraint("Horizontal", g[0]), Sketcher.Constraint("Horizontal", g[2]),
                      Sketcher.Constraint("Vertical", g[1]), Sketcher.Constraint("Vertical", g[3])])
    _dim(doc, sk, "DistanceX", (g[0], 1, g[0], 2), w, name and f"{name}_w")
    _dim(doc, sk, "DistanceY", (g[1], 1, g[1], 2), h, name and f"{name}_h")
    if all(not isinstance(v, str) and abs(v) < 1e-12 for v in (cx, cy)):
        sk.addConstraint(Sketcher.Constraint("Symmetric", g[0], 1, g[1], 2, ORIGIN, 1))
    else:   # centre point (construction) placed from the origin
        c = sk.addGeometry(Part.Point(V(X, Y, 0)), True)
        sk.addConstraint(Sketcher.Constraint("Symmetric", g[0], 1, g[1], 2, c, 1))
        place_point(sk, c, 1, cx, cy, name)
    return g


def rounded_rect(sk, length, width, radius, name=None):
    """Rounded rectangle centred on the origin: 4 lines + 4 tangent arcs. Returns geometry ids."""
    doc = sk.Document
    L, W, R = (num(doc, v) for v in (length, width, radius))
    a, b, h = L / 2, W / 2, math.pi / 2
    geo = [Part.LineSegment(V(-a + R, -b, 0), V(a - R, -b, 0)),
           Part.ArcOfCircle(Part.Circle(V(a - R, -b + R, 0), V(0, 0, 1), R), -h, 0),
           Part.LineSegment(V(a, -b + R, 0), V(a, b - R, 0)),
           Part.ArcOfCircle(Part.Circle(V(a - R, b - R, 0), V(0, 0, 1), R), 0, h),
           Part.LineSegment(V(a - R, b, 0), V(-a + R, b, 0)),
           Part.ArcOfCircle(Part.Circle(V(-a + R, b - R, 0), V(0, 0, 1), R), h, 2 * h),
           Part.LineSegment(V(-a, b - R, 0), V(-a, -b + R, 0)),
           Part.ArcOfCircle(Part.Circle(V(-a + R, -b + R, 0), V(0, 0, 1), R), 2 * h, 3 * h)]
    g0 = sk.GeometryCount
    sk.addGeometry(geo, False)
    g = [g0 + i for i in range(8)]
    # Tangent on end points already makes them coincident: never add Coincident as well
    sk.addConstraint([Sketcher.Constraint("Tangent", g[i], 2, g[(i + 1) % 8], 1) for i in range(8)] +
                     [Sketcher.Constraint("Horizontal", g[0]), Sketcher.Constraint("Horizontal", g[4]),
                      Sketcher.Constraint("Vertical", g[2]), Sketcher.Constraint("Vertical", g[6]),
                      Sketcher.Constraint("Equal", g[1], g[3]), Sketcher.Constraint("Equal", g[3], g[5]),
                      Sketcher.Constraint("Equal", g[5], g[7]),
                      Sketcher.Constraint("Symmetric", g[1], 3, g[5], 3, ORIGIN, 1)])
    _dim(doc, sk, "Radius", (g[1],), radius, name and f"{name}_r")
    _dim(doc, sk, "DistanceX", (g[6], 1, g[2], 1), length, name and f"{name}_l")
    _dim(doc, sk, "DistanceY", (g[0], 1, g[4], 1), width, name and f"{name}_w")
    return g


def circle(sk, d, x=0, y=0, name=None):
    """Circle of diameter d with its centre at (x, y). Returns the geometry id."""
    doc = sk.Document
    g = sk.addGeometry(Part.Circle(V(num(doc, x), num(doc, y), 0), V(0, 0, 1), num(doc, d) / 2), False)
    _dim(doc, sk, "Diameter", (g,), d, name)
    if all(not isinstance(v, str) and abs(v) < 1e-12 for v in (x, y)):
        sk.addConstraint(Sketcher.Constraint("Coincident", g, 3, ORIGIN, 1))
    else:
        place_point(sk, g, 3, x, y, name)
    return g


def copy_circle(sk, g, mirror="origin"):
    """Equal circle mirrored from circle g: about 'origin', 'x' (H axis) or 'y' (V axis)."""
    c = sk.Geometry[g].Center
    m = {"origin": V(-c.x, -c.y, 0), "x": V(c.x, -c.y, 0), "y": V(-c.x, c.y, 0)}[mirror]
    n = sk.addGeometry(Part.Circle(m, V(0, 0, 1), sk.Geometry[g].Radius), False)
    about = {"origin": (ORIGIN, 1), "x": (H_AXIS,), "y": (V_AXIS,)}[mirror]
    sk.addConstraint([Sketcher.Constraint("Equal", g, n), Sketcher.Constraint("Symmetric", g, 3, n, 3, *about)])
    return n


def hole_pattern_rect(sk, d, sx, sy, name="bolt"):
    """Four equal circles at (+-sx/2, +-sy/2), sx and sy = centre-to-centre spacing."""
    doc = sk.Document
    hx = f"({expr(sx)}) / 2" if isinstance(sx, str) else num(doc, sx) / 2
    hy = f"({expr(sy)}) / 2" if isinstance(sy, str) else num(doc, sy) / 2
    g = circle(sk, d, hx, hy, name)
    return [g, copy_circle(sk, g, "y"), copy_circle(sk, g, "origin"), copy_circle(sk, g, "x")]


def slot(sk, length, width, x=0, y=0, name=None):
    """Slot along sketch x: centre-to-centre length, width; its middle at (x, y)."""
    doc = sk.Document
    L, Wd, X, Y = (num(doc, v) for v in (length, width, x, y))
    r, h = Wd / 2, math.pi / 2
    geo = [Part.LineSegment(V(X - L / 2, Y - r, 0), V(X + L / 2, Y - r, 0)),
           Part.ArcOfCircle(Part.Circle(V(X + L / 2, Y, 0), V(0, 0, 1), r), -h, h),
           Part.LineSegment(V(X + L / 2, Y + r, 0), V(X - L / 2, Y + r, 0)),
           Part.ArcOfCircle(Part.Circle(V(X - L / 2, Y, 0), V(0, 0, 1), r), h, 3 * h)]
    g0 = sk.GeometryCount
    sk.addGeometry(geo, False)
    g = [g0 + i for i in range(4)]
    sk.addConstraint([Sketcher.Constraint("Tangent", g[i], 2, g[(i + 1) % 4], 1) for i in range(4)] +
                     [Sketcher.Constraint("Horizontal", g[0]), Sketcher.Constraint("Equal", g[1], g[3])])
    w2 = f"({expr(width)}) / 2" if isinstance(width, str) else Wd / 2
    _dim(doc, sk, "Radius", (g[1],), w2, name and f"{name}_r")
    _dim(doc, sk, "DistanceX", (g[3], 3, g[1], 3), length, name and f"{name}_l")
    mid = sk.addGeometry(Part.Point(V(X, Y, 0)), True)                       # construction midpoint
    sk.addConstraint(Sketcher.Constraint("Symmetric", g[3], 3, g[1], 3, mid, 1))
    place_point(sk, mid, 1, x, y, name)
    return g


def polyline(sk, pts, closed=True):
    """Lines through pts, joined end to start. Constraints for size/position are up to you."""
    n = len(pts)
    segs = [(pts[i], pts[(i + 1) % n]) for i in range(n if closed else n - 1)]
    g0 = sk.GeometryCount
    sk.addGeometry([Part.LineSegment(V(*a, 0), V(*b, 0)) for a, b in segs], False)
    g = [g0 + i for i in range(len(segs))]
    sk.addConstraint([Sketcher.Constraint("Coincident", g[i], 2, g[(i + 1) % len(g)], 1)
                      for i in range(len(g) if closed else len(g) - 1)])
    return g


def check_sketch(sk):
    """Solver state. A good sketch: solve 0, dof 0, fully True, every list empty.
    solve < 0 means failure (-3 conflicting). The constraint lists are 1-BASED:
    conflicting [3, 4] means sk.Constraints[2] and sk.Constraints[3]."""
    rc = sk.solve()
    return dict(name=sk.Name, solve=rc, dof=sk.DoF if hasattr(sk, "DoF") else None,
                fully=sk.FullyConstrained, conflicting=list(sk.ConflictingConstraints),
                redundant=list(sk.RedundantConstraints), malformed=list(sk.MalformedConstraints))


# ---------------------------------------------------------------- PartDesign features
def _new(bdy, typ, name):
    """Recompute first: Hole, patterns and dress-ups validate their base shape when their
    properties are set, and an uncomputed base raises 'Base feature's TopoShape is invalid'.
    Then make the new feature the Tip: Body.newObject does not always move the Tip (pattern
    features do not), and later features would be inserted before it, out of order."""
    bdy.Document.recompute()
    f = bdy.newObject(typ, name)
    bdy.Tip = f
    return f


def _profile(feat, sk, hide=True):
    feat.Profile = sk
    if hide:
        sk.Visibility = False


def _length(feat, prop, value):
    e = expr(value)
    if e:
        feat.setExpression(prop, e)
    else:
        setattr(feat, prop, value)


def pad(bdy, sk, length, name="Pad", symmetric=False, reversed=False):
    f = _new(bdy, "PartDesign::Pad", name)
    _profile(f, sk)
    _length(f, "Length", length)
    if symmetric:
        f.SideType = "Symmetric"
    f.Reversed = reversed
    return _done(bdy, f)


def pocket(bdy, sk, depth=None, name="Pocket", through=False, symmetric=False, reversed=False):
    """depth for a blind pocket; through=True cuts through all (both sides when symmetric)."""
    f = _new(bdy, "PartDesign::Pocket", name)
    _profile(f, sk)
    if through:
        f.Type = "ThroughAll"
    else:
        _length(f, "Length", depth)
    if symmetric:
        f.SideType = "Symmetric"
    f.Reversed = reversed
    return _done(bdy, f)


def _axis_ref(bdy, sk, axis):
    if axis in ("V_Axis", "H_Axis", "N_Axis"):
        return (sk, [axis])
    return (plane(bdy, axis), [""])            # 'X', 'Y' or 'Z' body axis


def revolve(bdy, sk, axis="V_Axis", angle=360, name="Revolution"):
    f = _new(bdy, "PartDesign::Revolution", name)
    _profile(f, sk)
    f.ReferenceAxis = _axis_ref(bdy, sk, axis)
    _length(f, "Angle", angle)
    return _done(bdy, f)


def groove(bdy, sk, axis="V_Axis", angle=360, name="Groove"):
    f = _new(bdy, "PartDesign::Groove", name)
    _profile(f, sk)
    f.ReferenceAxis = _axis_ref(bdy, sk, axis)
    _length(f, "Angle", angle)
    return _done(bdy, f)


def hole(bdy, sk, diameter, depth=None, name="Hole", counterbore=None, countersink=None):
    """Hole feature at every circle centre of sk. counterbore=(dia, depth); countersink=(dia, angle)."""
    f = _new(bdy, "PartDesign::Hole", name)
    _profile(f, sk)
    _length(f, "Diameter", diameter)
    if depth is None:
        f.DepthType = "ThroughAll"
    else:
        f.DepthType = "Dimension"
        _length(f, "Depth", depth)
    bdy.Document.recompute()                    # cut types need the computed hole
    if counterbore:
        f.HoleCutType = "Counterbore"
        _length(f, "HoleCutDiameter", counterbore[0]); _length(f, "HoleCutDepth", counterbore[1])
    if countersink:
        f.HoleCutType = "Countersink"
        _length(f, "HoleCutDiameter", countersink[0]); _length(f, "HoleCutCountersinkAngle", countersink[1])
    return _done(bdy, f)


def fillet(bdy, base, edges, radius, name="Fillet"):
    """edges: names on base.Shape, e.g. from edges_where(base.Shape, ...)."""
    f = _new(bdy, "PartDesign::Fillet", name)
    f.Base = (base, list(edges))
    _length(f, "Radius", radius)
    return _done(bdy, f)


def chamfer(bdy, base, edges, size, name="Chamfer"):
    f = _new(bdy, "PartDesign::Chamfer", name)
    f.Base = (base, list(edges))
    _length(f, "Size", size)
    return _done(bdy, f)


def polar_pattern(bdy, features, count, axis="Z", angle=360, name="PolarPattern"):
    """count may be an int or an alias/expression (unitless parameter)."""
    f = _new(bdy, "PartDesign::PolarPattern", name)
    f.Originals = list(features)
    f.Axis = (plane(bdy, axis), [""])
    _length(f, "Angle", angle)
    _length(f, "Occurrences", count)
    return _done(bdy, f)


def linear_pattern(bdy, features, count, length, direction="X", name="LinearPattern"):
    """count copies spread over length (Extent mode) along a body axis."""
    f = _new(bdy, "PartDesign::LinearPattern", name)
    f.Originals = list(features)
    f.Direction = (plane(bdy, direction), [""])
    _length(f, "Length", length)
    _length(f, "Occurrences", count)
    return _done(bdy, f)


def mirrored(bdy, features, mirror_plane="YZ", name="Mirrored", whole_shape=False):
    """Originals must be additive/subtractive features (Pad, Pocket, Hole, ...), not patterns:
    'Only additive and subtractive features can be transformed'. To mirror everything so far
    (e.g. a pattern's result) use whole_shape=True."""
    f = _new(bdy, "PartDesign::Mirrored", name)
    if whole_shape:
        f.TransformMode = "Whole shape"
    else:
        f.Originals = list(features)
    f.MirrorPlane = (plane(bdy, mirror_plane), [""])
    return _done(bdy, f)


# ---------------------------------------------------------------- picking edges and faces by geometry
def edges_where(shape, pred):
    """Names ('EdgeN') of edges for which pred(edge) is true. Never hard-code EdgeN."""
    return [f"Edge{i + 1}" for i, e in enumerate(shape.Edges) if pred(e)]


def faces_where(shape, pred):
    return [f"Face{i + 1}" for i, f in enumerate(shape.Faces) if pred(f)]


def is_line_along(axis, tol=1e-6):
    a = {"X": V(1, 0, 0), "Y": V(0, 1, 0), "Z": V(0, 0, 1)}[axis.upper()]
    return lambda e: (isinstance(e.Curve, Part.Line) and
                      abs(abs((e.Vertexes[-1].Point - e.Vertexes[0].Point).normalize().dot(a)) - 1) < tol)


def is_circle(radius=None, tol=1e-6):
    return lambda e: isinstance(e.Curve, Part.Circle) and (radius is None or abs(e.Curve.Radius - radius) < tol)


def at_z(z, tol=1e-6):
    return lambda s: abs(s.BoundBox.ZMin - z) < tol and abs(s.BoundBox.ZMax - z) < tol


def planar_face_normal(n, tol=1e-6):
    """Planar faces whose outward normal is n ('+Z', '-X', ...)."""
    v = {"+X": V(1, 0, 0), "-X": V(-1, 0, 0), "+Y": V(0, 1, 0), "-Y": V(0, -1, 0),
         "+Z": V(0, 0, 1), "-Z": V(0, 0, -1)}[n]

    def p(f):
        if not isinstance(f.Surface, Part.Plane):
            return False
        u0, u1, v0, v1 = f.ParameterRange
        return f.normalAt((u0 + u1) / 2, (v0 + v1) / 2).dot(v) > 1 - tol
    return p


def top_face(shape, normal="+Z"):
    """The outermost planar face facing `normal` (largest coordinate along it), e.g. the top."""
    names = faces_where(shape, planar_face_normal(normal))
    key = {"+X": lambda f: f.BoundBox.XMax, "-X": lambda f: -f.BoundBox.XMin, "+Y": lambda f: f.BoundBox.YMax,
           "-Y": lambda f: -f.BoundBox.YMin, "+Z": lambda f: f.BoundBox.ZMax, "-Z": lambda f: -f.BoundBox.ZMin}[normal]
    return max(names, key=lambda n: (round(key(shape.getElement(n)), 6), shape.getElement(n).Area))


def pick(doc, base, rule):
    """Edges of base.Shape selected by rule(doc, edge) -> bool (rule reads current params)."""
    return edges_where(base.Shape, lambda e: rule(doc, e))


def update(doc, picks=None, **values):
    """Change parameters and rebuild, re-picking dress-up edges by geometry.

    picks: {feature name: rule}
      Fillet/Chamfer: rule(doc, edge) -> bool, re-selects the edges of the feature's Base.
      Sketch on a face: rule(doc, shape) -> 'FaceN', re-attaches to that face of its support.
    Needed because a parameter change that regenerates the base (revolve, pad of a resized
    sketch) can break stored element links even in FreeCAD 1.1: 'Invalid edge link ... ?Edge5'
    on dress-ups, an error on face-attached sketches. Returns names of objects in error
    (empty list = all valid)."""
    set_params(doc, recompute=True, **values)
    if picks:
        order = {o.Name: i for i, o in enumerate(doc.Objects)}
        for name in sorted(picks, key=lambda n: order[n]):
            f = doc.getObject(name)
            doc.recompute()
            if f.TypeId == "Sketcher::SketchObject":
                sup = f.AttachmentSupport[0][0]
                f.AttachmentSupport = [(sup, picks[name](doc, sup.Shape))]
            else:
                base = f.Base[0]
                f.Base = (base, pick(doc, base, picks[name]))
            doc.recompute()
    return [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]


# ---------------------------------------------------------------- checks, export, view
def report(bdy, density_g_cm3=None):
    s = bdy.Shape
    bb = s.BoundBox
    out = dict(valid=s.isValid(), solids=len(s.Solids), volume_mm3=round(s.Volume, 2),
               bbox_mm=[round(v, 3) for v in (bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax)],
               tip=bdy.Tip.Name if bdy.Tip else None,
               errors=[o.Name for o in bdy.Document.Objects if "Invalid" in o.State or "Error" in o.State])
    if density_g_cm3:
        out["mass_g"] = round(s.Volume * density_g_cm3 / 1000, 2)
    return out


def export(doc, obj, folder, stem, step=True, stl=False, stl_metres=False, deflection=0.1):
    """Save the .FCStd and export STEP (mm) and/or STL (mm, or metres for simulators)."""
    os.makedirs(folder, exist_ok=True)
    out = [os.path.join(folder, stem + ".FCStd")]
    doc.saveAs(out[0])
    if step:
        out.append(os.path.join(folder, stem + ".step"))
        obj.Shape.exportStep(out[-1])
    if stl:
        import MeshPart
        m = MeshPart.meshFromShape(Shape=obj.Shape, LinearDeflection=deflection, AngularDeflection=0.35)
        if stl_metres:
            s = App.Matrix(); s.scale(0.001, 0.001, 0.001); m.transform(s)
        out.append(os.path.join(folder, stem + ".stl"))
        m.write(out[-1])
    clean_backups(folder)
    return out


def clean_backups(folder):
    """FreeCAD writes name.<timestamp>.FCBak next to re-saved files; remove them."""
    for f in os.listdir(folder):
        if f.endswith(".FCBak"):
            os.remove(os.path.join(folder, f))


def show(doc, view="Isometric", hide_sketches=True):
    """Tidy the 3D view for a screenshot: hide sketches, clear selection, fit."""
    if not App.GuiUp:
        return
    import FreeCADGui as Gui
    for o in doc.Objects:
        if hide_sketches and o.TypeId == "Sketcher::SketchObject":
            o.Visibility = False
    Gui.Selection.clearSelection()
    # The MCP screenshot grabs the ACTIVE window. If a TechDraw page or another document is in
    # front you get a garbled or wrong image, so raise this document's 3D view first.
    from PySide import QtWidgets
    mdi = Gui.getMainWindow().findChild(QtWidgets.QMdiArea)
    for w in mdi.subWindowList():
        if w.windowTitle().startswith(doc.Label + " : "):
            mdi.setActiveSubWindow(w)
            break
    Gui.setActiveDocument(doc.Name)
    v = Gui.ActiveDocument.ActiveView
    getattr(v, "view" + view)()
    v.fitAll()
    pump(0.3)


def pump(seconds):
    """Let the Qt event loop run (TechDraw and some views update through it)."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if App.GuiUp:
            from PySide import QtCore
            QtCore.QCoreApplication.processEvents()
        time.sleep(0.05)


# ---------------------------------------------------------------- TechDraw
def drawing_page(doc, template="ISO/A4_Landscape_TD.svg", name="Drawing", texts=None):
    """Page with a template from <ResourceDir>/Mod/TechDraw/Templates. texts: EditableTexts keys
    (A4_Landscape_TD: FC-Title, Subtitle, Designed_by_Name, Drawing_number, FC-SC, FC-SH, Weight)."""
    page = doc.addObject("TechDraw::DrawPage", name)
    tpl = doc.addObject("TechDraw::DrawSVGTemplate", name + "Template")
    tpl.Template = os.path.join(App.getResourceDir(), "Mod", "TechDraw", "Templates", *template.split("/"))
    page.Template = tpl
    if texts:
        t = tpl.EditableTexts
        t.update(texts)
        tpl.EditableTexts = t
    return page


def proj_group(page, source, views=("Front", "Top", "Right"), scale=1.0, at=(100, 100),
               positions=None, projection="Third angle", name="Views"):
    """Projection group. Set the scale BEFORE adding views (a later change leaves stale geometry
    that dimension picking will hit). positions: {view type: (x, y)} relative to the group, mm."""
    doc = page.Document
    g = doc.addObject("TechDraw::DrawProjGroup", name)
    page.addView(g)
    g.Source = list(source) if isinstance(source, (list, tuple)) else [source]
    g.ProjectionType = projection
    g.ScaleType = "Custom"
    g.Scale = scale
    g.AutoDistribute = positions is None
    for v in views:
        g.addProjection(v)
    g.X, g.Y = at
    if positions:
        for v in g.Views:
            if v.Type in positions:
                v.X, v.Y = positions[v.Type]
    doc.recompute()
    return g


def wait_views(group, timeout=30):
    """Block until Front/Top/Right-type views have projected at the group scale.

    TechDraw projects on worker threads; geometry appears only while the Qt loop runs, and a
    first pass can still be at an older scale. Picking vertices before this returns gives wrong
    or zero-length dimensions."""
    src = group.Source[0].Shape.BoundBox
    half = {"Front": src.XLength / 2, "Rear": src.XLength / 2, "Top": src.XLength / 2,
            "Bottom": src.XLength / 2, "Right": src.YLength / 2, "Left": src.YLength / 2}
    t0 = time.time()
    while True:
        ok = True
        for v in group.Views:
            if v.Type not in half:
                continue
            p = v.getVisibleVertexes()
            if not p or abs(max(abs(q.x) for q in p) - half[v.Type] * group.Scale) > 0.05:
                ok = False
        if ok:
            return
        if time.time() - t0 > timeout:
            raise RuntimeError("TechDraw views did not finish projecting")
        pump(0.1)


def view_of(group, vtype):
    return next(v for v in group.Views if v.Type == vtype)


def vertex(view, x, y, scale=None):
    """'VertexN' nearest to model-mm point (x, y) in the view's 2D frame (y points DOWN, origin at
    the view centre = centre of the part's projected bounding box)."""
    s = scale if scale is not None else view.getScale() if hasattr(view, "getScale") else view.Scale
    pts = view.getVisibleVertexes()
    i = min(range(len(pts)), key=lambda k: (pts[k].x - x * s) ** 2 + (pts[k].y - y * s) ** 2)
    return f"Vertex{i}"


def edge(view, pred):
    """'EdgeN' of the first visible edge with pred(edge) true (edges are 2D, in scaled units)."""
    es = view.getVisibleEdges()
    return f"Edge{next(i for i, e in enumerate(es) if pred(e))}"


def list_view_geometry(view):
    """Debug listing of a view's visible vertices and edges (scaled, y down)."""
    vs = [(i, round(p.x, 2), round(p.y, 2)) for i, p in enumerate(view.getVisibleVertexes())]
    es = [(i, type(e.Curve).__name__, [round(c, 2) for c in (e.BoundBox.XMin, e.BoundBox.YMin,
                                                               e.BoundBox.XMax, e.BoundBox.YMax)],
           round(getattr(e.Curve, "Radius", 0), 2)) for i, e in enumerate(view.getVisibleEdges())]
    return vs, es


def dimension(page, view, kind, refs, at, name="Dim"):
    """kind: DistanceX, DistanceY, Distance, Radius, Diameter, Angle. refs: ['Vertex3', 'Vertex7']
    or ['Edge4']. at: label position (x, y) relative to the view centre, page mm, y UP."""
    doc = page.Document
    d = doc.addObject("TechDraw::DrawViewDimension", name)
    d.Type = kind
    d.References2D = [(view, r) for r in refs]
    d.MeasureType = "Projected"
    page.addView(d)
    d.X, d.Y = at
    return d


def refresh_dimensions(doc):
    for o in doc.Objects:
        if o.TypeId == "TechDraw::DrawViewDimension":
            o.touch()
    doc.recompute()
    return [(o.Name, o.Type, round(o.getRawValue(), 3)) for o in doc.Objects
            if o.TypeId == "TechDraw::DrawViewDimension"]


def export_page(page, pdf=None, svg=None, dxf=None):
    """Open the page in the GUI first: exporting a page that is not open crops the PDF."""
    import TechDrawGui
    page.ViewObject.doubleClicked()
    pump(2.0)
    if pdf:
        TechDrawGui.exportPageAsPdf(page, pdf)
    if svg:
        TechDrawGui.exportPageAsSvg(page, svg)
    if dxf:
        import TechDraw
        TechDraw.writeDXFPage(page, dxf)


# ---------------------------------------------------------------- guard every document/GUI entry point
for _name in ("new_doc", "params", "set_params", "body", "sketch", "_new", "update", "export",
              "show", "pump", "run", "drawing_page", "proj_group", "wait_views", "dimension",
              "refresh_dimensions", "export_page"):
    _fn = globals()[_name]
    if not getattr(_fn, "_fcmcp_guarded", False):
        globals()[_name] = _gui_thread_only(_fn)
del _name, _fn
