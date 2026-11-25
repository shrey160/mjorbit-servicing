"""Servicer rev 2 with its arm and ring tool, as one FreeCAD model (21 Nov, user request).

Sources (all in the bus frame, mm):
  - bus: servicer_rev2_bus.FCStd (build_servicer_rev2.py), every part copied with its colour;
  - arm: arm_geoms.json (projects/p7_unprepared_client/checks/export_arm_geoms.py: the MuJoCo scene's arm links,
    housings and wrist, posed by the scene's own kinematics) for the poses ready, stow and grasp;
  - ring tool: ring_tool.FCStd (build_ring_tool.py), placed at the wrist3 frame of each pose; jaws open in ready
    and stow (inboard 20 mm off contact, hook 26 mm back along its cam path), closed in grasp.
The arm in this model is the MuJoCo collision / visual primitives (a stand-in, not a detailed arm design).
Groups arm_ready (shown), arm_stow, arm_grasp. Run in FreeCAD (MCP execute_code):
  exec(open(p).read(), {"__file__": p, "__name__": "__main__"})
Writes servicer_rev2_arm.FCStd, servicer_rev2_arm_ready.step and renders servicer_rev2_arm_*.png.
"""
import json
import os

import FreeCAD as App
import Part

V = App.Vector
HERE = os.path.dirname(os.path.abspath(__file__))
NAME = "servicer_rev2_arm"
HOOK_OPEN = V(0.0, 0.0, 25.0)                         # hook radially out (open; tool v3)
IN_OPEN = V(0.0, 0.0, -20.0)                          # inboard jaw radially in (open)


def prim(g):
    t, s = g["type"], g["size"]
    if t == "box":
        sh = Part.makeBox(2 * s[0], 2 * s[1], 2 * s[2], V(-s[0], -s[1], -s[2]))
    elif t == "cylinder":
        sh = Part.makeCylinder(s[0], 2 * s[1], V(0, 0, -s[1]), V(0, 0, 1))
    elif t == "capsule":
        sh = Part.makeCylinder(s[0], 2 * s[1], V(0, 0, -s[1]), V(0, 0, 1))
        sh = sh.fuse(Part.makeSphere(s[0], V(0, 0, s[1]))).fuse(Part.makeSphere(s[0], V(0, 0, -s[1])))
    else:
        sh = Part.makeSphere(s[0])
    w, x, y, z = g["quat"]
    sh.Placement = App.Placement(V(*g["pos"]), App.Rotation(x, y, z, w))
    return sh


def copy_doc(path, keep_open):
    """(name, shape, colour) of every solid object of a document (opened if needed, closed again)."""
    name = os.path.splitext(os.path.basename(path))[0]
    was_open = name in App.listDocuments()
    src = App.getDocument(name) if was_open else App.openDocument(path, hidden=True)
    out = []
    for o in src.Objects:
        if hasattr(o, "Shape") and o.Shape.Solids and o.TypeId == "Part::Feature":
            col = tuple(o.ViewObject.ShapeColor[:3]) if App.GuiUp and o.ViewObject else (0.7, 0.7, 0.7)
            out.append((o.Name, o.Shape.copy(), col))
    if not was_open and not keep_open:
        App.closeDocument(name)
    return out


def build():
    if NAME in App.listDocuments():
        App.closeDocument(NAME)
    doc = App.newDocument(NAME)
    arm = json.load(open(os.path.join(HERE, "arm_geoms.json")))
    tool_groups = {k: v["group"] for k, v in json.load(open(os.path.join(HERE, "ring_tool.json")))["parts"].items()}
    bus = copy_doc(os.path.join(HERE, "servicer_rev2_bus.FCStd"), False)
    tool = copy_doc(os.path.join(HERE, "ring_tool.FCStd"), False)
    gb = doc.addObject("App::DocumentObjectGroup", "bus")
    for n, sh, col in bus:
        o = doc.addObject("Part::Feature", "bus_" + n)
        o.Shape = sh
        gb.addObject(o)
        if App.GuiUp:
            o.ViewObject.ShapeColor = col
    shapes = {}
    for pose, P in arm["poses"].items():
        g = doc.addObject("App::DocumentObjectGroup", "arm_" + pose)
        shapes[pose] = []
        for k, ge in enumerate(P["geoms"]):
            o = doc.addObject("Part::Feature", f"{pose}_{ge['body']}_{k}")
            o.Shape = prim(ge)
            g.addObject(o)
            shapes[pose].append(o.Shape)
            if App.GuiUp:
                o.ViewObject.ShapeColor = tuple(ge["rgba"][:3])
        w, x, y, z = P["tool"]["quat"]
        T = App.Placement(V(*P["tool"]["pos"]), App.Rotation(x, y, z, w))
        for n, sh, col in tool:
            if not n.startswith("tool_"):
                continue
            part = n[5:]
            s = sh.copy()
            grp = tool_groups.get(part, "base")
            if pose == "ready" and grp == "in":     # open in ready; closed in grasp and stow (on the stow fixture)
                s.translate(IN_OPEN)
            if pose == "ready" and grp == "hook":
                s.translate(HOOK_OPEN)
            s.Placement = T.multiply(s.Placement)
            o = doc.addObject("Part::Feature", f"{pose}_{n}")
            o.Shape = s
            g.addObject(o)
            shapes[pose].append(s)
            if App.GuiUp:
                o.ViewObject.ShapeColor = col
        if App.GuiUp and pose != "ready":
            for o in g.Group:
                o.ViewObject.Visibility = False
            g.ViewObject.Visibility = False
    doc.recompute()
    # checks: arm / tool parts intersecting the bus (stow and ready), envelope depth from the port face
    bus_solid = Part.makeCompound([sh for n, sh, c in bus])
    for pose in ("ready", "stow"):
        ov = sum(s.common(bus_solid).Volume for s in shapes[pose])
        bb = Part.makeCompound(shapes[pose] + [sh for n, sh, c in bus]).BoundBox
        print(pose, "arm-bus overlap mm^3", round(ov, 1), " x range", round(bb.XMin, 1), round(bb.XMax, 1),
              " depth from the port face (-300)", round(bb.XMax + 300.0, 1))
    doc.saveAs(os.path.join(HERE, NAME + ".FCStd"))
    Part.export([o for o in doc.Objects if o.Name.startswith(("bus_", "ready_"))], os.path.join(HERE, NAME + "_ready.step"))
    return doc


def show_pose(doc, pose):
    for g in doc.Objects:
        if g.TypeId == "App::DocumentObjectGroup" and g.Name.startswith("arm_"):
            vis = g.Name == "arm_" + pose
            g.ViewObject.Visibility = vis
            for o in g.Group:
                o.ViewObject.Visibility = vis


def render(doc):
    """Renders per pose (23 Nov: the 3D view taken from the document itself; Gui.ActiveDocument can be another
    document's window after newDocument, which made the old render fail)."""
    import FreeCADGui as Gui
    v = Gui.getDocument(doc.Name).mdiViewsOfType("Gui::View3DInventor")[0]
    for pose, views in (("ready", ("iso", "front", "right")), ("stow", ("iso", "face")), ("grasp", ("iso",))):
        show_pose(doc, pose)
        for name in views:
            {"iso": v.viewIsometric, "front": v.viewFront, "right": v.viewRight, "face": v.viewRear}[name]()
            v.fitAll()
            Gui.updateGui()
            stem = f"{NAME}_{name}" if pose == "ready" else f"{NAME}_{pose}_{name}"
            v.saveImage(os.path.join(HERE, stem + ".png"), 1200, 900, "Current")
    show_pose(doc, "stow")
    v.viewIsometric()
    v.fitAll()


if __name__ == "__main__":
    d = build()
    if App.GuiUp:
        render(d)
