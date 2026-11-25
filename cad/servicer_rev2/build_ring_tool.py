"""Servicer rev 2 ring tool, v3 (21 Nov, user: full rework, then "review it again: the errors, how it opens, the
crooked structures"; v1 kept as build_ring_tool_v1.py). Form-closure end effector for the 610 mm adapter ring of
client_p7 (US 10,414,053 layout: palm datum on the ring top, inboard jaw on the inner wall, outboard hook under the
clamp lip that draws the ring onto the palm).

How it opens and closes: both jaws move PURELY RADIALLY (z), each on a straight carriage riding a rail on the front
face of the back housing, driven by a lead screw (motors in the housing). Inboard jaw: 20 mm inward to open. Hook:
25 mm outward to open. The axial draw comes from a 20 deg wedge face on the hook tip: closing radially, the wedge
meets the lip's outer edge and pulls the ring onto the palm (the Marman-clamp principle: radial squeeze on an
inclined face -> axial preload). v2 got the draw from an inclined slide instead (rail, screw and end plate leaning
13.5 deg, 60 mm out of the tool): gone.

Kept from v2: jaw faces concentric with the ring (inboard r 283, hook tip r 305.5, hook finger r 323), the palm as a
plate between the fingers, lead-in chamfers on the finger tips. Layout: flange -> back housing (full height, x 62-82)
-> neck (x 82-110, between the carriages) -> palm (x 110-130).

Frame (mm): x along the approach (wrist flange at x = WRIST_R, palm face at x = PALM_X = 130, the grip point), y along
the ring (tangential at the grasp centre), z radially outward, z = 0 on the wall's mid radius (ring centre at
z = -RC). Ring section: wall r 283..305 (x 130..190), lip r 305..317 (x 130..145).
Run in FreeCAD (MCP execute_code):  exec(open(p).read(), {"__file__": p, "__name__": "__main__"})
Writes ring_tool.FCStd, ring_tool.step, parts/tool_*.stl (metres), ring_tool.json, and prints the checks.
"""
import json
import math
import os

import FreeCAD as App
import Part

V = App.Vector
HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "parts")

PALM_X = 130.0
W = 80.0                 # palm width along the ring (y)
JW = 60.0                # jaw width along the ring
WRIST_R = 55.0           # flange on the wrist ball / roll output
RC = 294.0               # ring wall mid radius: the ring centre is at z = -RC
R_IN, R_WALL, R_LIP = 283.0, 305.0, 317.0
WALL_Z = (R_IN - RC, R_WALL - RC)            # -11, +11 at y = 0
LIP_Z = (R_WALL - RC, R_LIP - RC)            # +11, +23
RING_TOP, LIP_BOT, RING_BOT = PALM_X, PALM_X + 15.0, PALM_X + 60.0
SEG_Y = 160.0
JAW_T = 10.0
JAW_X0, JAW_X1 = 100.0, PALM_X + 40.0        # fingers: root behind the palm (carriage) to the tip
IN_STROKE = 20.0
HOOK_H = 12.0
HOOK_OPEN = 25.0
DRAW = 6.0
FINGER_GAP = 6.0
TIP_GAP = 0.0          # v3: the tip face lands on the wall as the wedge reaches the lip edge (0.5 left the radial clamp to the wedge)
CH = 4.0                 # lead-in chamfer
NECK_Z = (-10.0, 27.0)   # neck and palm band (z): the curved hook finger dips to z 27.6 at its edges
HOUSE_X = (62.0, 82.0)   # back housing (motors, screw drives)
HOUSE_Z = (-64.0, 86.0)
RAIL_X = (82.0, 86.0)    # rails on the housing's front face
CAR_X = (86.0, 110.0)    # carriages, behind the palm
WEDGE = 20.0             # hook tip wedge angle (deg): top face x = LIP_BOT - (r - R_LIP) tan(WEDGE)
HOOK_AX = V(0.0, 0.0, 1.0)
DENS = {"al": 2.70, "steel": 7.85, "ti": 4.43,
        "cam": 5.86}     # stereo camera + LED lamp module, effective (0.45 kg in its 24 x 40 x 80 mm box: two board cameras,
                         # driver, lamp, housing; 23 Nov, s25; the mono unit was 0.40 kg in 54 x 44 x 36 mm)
# 23 Nov (SPEC_7 s25, wrist stereo; user: "a single module, parallel to the movement plane of the grippers", on the
# housing's side face): one module on the back housing's -y face (in the launch stow the forearm is on the tool's +y
# side), two parallel cameras 50 mm apart along z (the jaws' travel), both pitched 10 deg radially in, the lamp
# between them. Replaces the inboard camera unit (z -101..-65), so the housing end sets the stow depth again
CAM_X, CAM_Y, CAM_Z = (62.0, 86.0), (-75.0, -35.0), (-60.0, 20.0)
CAM_YC, CAM_ZS = -55.0, (-45.0, 5.0)           # the two windows' centres (y, z); optical centres at x 87


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def shell(r0, r1, x0, x1, w):
    """Annulus sector about the ring axis (x, through (0, 0, -RC)), radii r0..r1, x0..x1, cut to |y| <= w / 2 on the
    +z side (the grasp side)."""
    c = V(x0, 0, -RC)
    a = Part.makeCylinder(r1, x1 - x0, c, V(1, 0, 0)).cut(Part.makeCylinder(r0, x1 - x0, c, V(1, 0, 0)))
    return a.common(box(x0, x1, -w / 2, w / 2, 0.0 - RC + 0.5 * r0, 400))


def chamfer_cut(x, z, sx, sz, w=200.0):
    """A 45 deg chamfer of the edge (along y) at (x, z): a square prism turned 45 deg, centred on the edge, removes
    a triangle with CH legs (sx, sz name the corner's direction, for the reader)."""
    h = CH / math.sqrt(2.0)
    b = Part.makeBox(2 * h, w, 2 * h, V(-h, -w / 2, -h))
    b.rotate(V(0, 0, 0), V(0, 1, 0), 45.0)
    b.translate(V(x, 0, z))
    return b


def parts(in_open=0.0, hook_open=0.0, draw=0.0):
    """Tool solids with the inboard jaw opened by in_open (radially in, -z) and the hook by hook_open (radially out,
    +z); draw is unused (v3: the draw is the wedge's, no separate motion). Returns {name: (shape, material, group)};
    group base / in / hook."""
    P = {}
    # --- base: flange, back housing, neck, palm; rails, screws and end bearings on the housing's front face
    P["flange"] = (Part.makeCylinder(45, 7, V(WRIST_R, 0, 0), V(1, 0, 0)), "al", "base")
    P["housing"] = (box(HOUSE_X[0], HOUSE_X[1], -35, 35, *HOUSE_Z), "al", "base")
    P["neck"] = (box(HOUSE_X[1], CAR_X[1], -35, 35, *NECK_Z), "al", "base")
    P["palm"] = (box(CAR_X[1], PALM_X, -W / 2, W / 2, -10.5, NECK_Z[1]), "al", "base")
    P["rail_in"] = (box(*RAIL_X, -10, 10, HOUSE_Z[0], NECK_Z[0]), "steel", "base")
    P["rail_hook"] = (box(*RAIL_X, -10, 10, NECK_Z[1], HOUSE_Z[1]), "steel", "base")
    P["brg_in"] = (box(RAIL_X[1], 104, 8, 28, HOUSE_Z[0], HOUSE_Z[0] + 4), "steel", "base")
    P["brg_hook"] = (box(RAIL_X[1], 104, -28, -8, HOUSE_Z[1] - 4, HOUSE_Z[1]), "steel", "base")
    P["screw_in"] = (Part.makeCylinder(4, NECK_Z[0] - HOUSE_Z[0] - 4, V(96, 18, HOUSE_Z[0] + 4), V(0, 0, 1)), "steel", "base")
    P["screw_hook"] = (Part.makeCylinder(4, HOUSE_Z[1] - 4 - NECK_Z[1], V(96, -18, NECK_Z[1]), V(0, 0, 1)), "steel", "base")
    # --- inboard jaw: finger shell r 273..283 (closed), carriage below its root; opening moves it in (-z)
    fi = shell(R_IN - JAW_T, R_IN, JAW_X0, JAW_X1, JW)
    fi = fi.cut(chamfer_cut(JAW_X1, WALL_Z[0], +1, +1))                  # tip edge on the wall side
    car_i = box(CAR_X[0], CAR_X[1], -JW / 2, JW / 2, -36.0, -21.5)
    for n, s in (("jaw_in", fi), ("car_in", car_i)):
        s.translate(V(0, 0, -in_open))
        P[n] = (s, "ti" if n == "jaw_in" else "al", "in")
    # --- hook: finger shell outside the lip, tip under the lip with the wedge face, carriage above the root
    r_f = R_LIP + FINGER_GAP                                               # 323
    fh = shell(r_f, r_f + JAW_T, JAW_X0, JAW_X1, JW)
    fh = fh.cut(chamfer_cut(JAW_X1, r_f - RC, +1, -1))                    # tip edge on the ring side
    ht = shell(R_WALL + TIP_GAP, r_f + 0.5, LIP_BOT - 8.0, LIP_BOT + HOOK_H, JW)
    hs = Part.makeBox(100, 400, 400, V(-100, -200, -200))                 # half-space x < 0 ...
    hs.rotate(V(0, 0, 0), V(0, 1, 0), -WEDGE)                             # ... its face turned to the wedge slope
    hs.translate(V(LIP_BOT, 0, R_LIP - RC))                               # through the lip's outer edge
    ht = ht.cut(hs)
    car_h = box(CAR_X[0], CAR_X[1], -JW / 2, JW / 2, r_f + JAW_T - RC - 0.5, r_f + JAW_T - RC + 14.0)
    for n, s in (("jaw_hook", fh), ("hook_tip", ht), ("car_hook", car_h)):
        s.translate(HOOK_AX * hook_open)
        P[n] = (s, "al" if n == "car_hook" else "ti", "hook")
    # --- wrist camera + LED lamp unit (22 Nov, P7.4): bolted to the housing's inboard end (z -64), 1 mm off it;
    # camera window (optical centre x 83, z -76) and lamp window on its front face, looking at the ring section.
    # Inboard so that it clears the bus in the launch stow (tool +z points at the bus there); group "cam"
    P["cam_unit"] = (box(*CAM_X, *CAM_Y, *CAM_Z), "cam", "cam")
    P["cam_lens"] = (Part.makeCylinder(7, 1, V(CAM_X[1], CAM_YC, CAM_ZS[0]), V(1, 0, 0)), "al", "cam")
    P["cam_lens2"] = (Part.makeCylinder(7, 1, V(CAM_X[1], CAM_YC, CAM_ZS[1]), V(1, 0, 0)), "al", "cam")
    P["cam_lamp"] = (box(CAM_X[1], CAM_X[1] + 1.0, CAM_YC - 7, CAM_YC + 7, -27, -13), "al", "cam")
    for n, (s, mat, grp) in list(P.items()):      # booleans return compounds: one solid per part
        ss = s.Solids
        P[n] = (ss[0] if len(ss) == 1 else ss[0].fuse(ss[1:]).removeSplitter().Solids[0], mat, grp)
    return P


def ring_section():
    c = V(RING_TOP, 0, -RC)
    wall = Part.makeCylinder(R_WALL, RING_BOT - RING_TOP, c, V(1, 0, 0)).cut(
        Part.makeCylinder(R_IN, RING_BOT - RING_TOP, c, V(1, 0, 0)))
    lip = Part.makeCylinder(R_LIP, LIP_BOT - RING_TOP, c, V(1, 0, 0)).cut(
        Part.makeCylinder(R_WALL - 0.01, LIP_BOT - RING_TOP, c, V(1, 0, 0)))
    return wall.fuse(lip).common(box(RING_TOP, RING_BOT, -SEG_Y / 2, SEG_Y / 2, -60, 400))


def check(P, ring, label):
    out = {}
    for n, (s, _, _) in P.items():
        out[n] = (round(s.distToShape(ring)[0], 2), round(s.common(ring).Volume, 1))
    print(label, {k: v for k, v in out.items() if k.startswith(("jaw", "hook", "palm", "car"))})
    return out


def v_touch(a, b):
    """Base and camera unit are bolted together: overlap is still an error, touching faces are not."""
    return a.common(b).Volume <= 1.0


def internal(P):
    """Overlaps between parts of different groups, and moving parts with the base (should be none)."""
    bad = {}
    ns = list(P)
    for i, a in enumerate(ns):
        for b in ns[i + 1:]:
            if P[a][2] == P[b][2] == "base" or {P[a][2], P[b][2]} == {"base", "cam"} and v_touch(P[a][0], P[b][0]):
                continue
            if P[a][2] == P[b][2] or "screw" in a + b:      # the lead screws pass through their carriage nuts
                continue
            v = P[a][0].common(P[b][0]).Volume
            if v > 1.0:
                bad[f"{a}-{b}"] = round(v, 1)
    return bad


def mass_props(P):
    M, c, shapes = 0.0, V(0, 0, 0), []
    for n, (s, mat, g) in P.items():
        m = s.Volume * DENS[mat] * 1e-6
        M += m
        c += s.CenterOfMass * m
        shapes.append((n, s, m))
    c = c * (1.0 / M)
    I = [[0.0] * 3 for _ in range(3)]
    for n, s, m in shapes:
        Ms = s.MatrixOfInertia
        rho = m / s.Volume
        r = s.CenterOfMass - c
        rv = [r.x, r.y, r.z]
        for i in range(3):
            for j in range(3):
                I[i][j] += (Ms.A[i * 4 + j] * rho + m * ((r.Length ** 2 if i == j else 0.0) - rv[i] * rv[j])) * 1e-6
    return M, [c.x / 1000, c.y / 1000, c.z / 1000], I, shapes


def build():
    if "ring_tool" in App.listDocuments():
        App.closeDocument("ring_tool")
    doc = App.newDocument("ring_tool")
    ring = ring_section()
    opened = parts(in_open=IN_STROKE, hook_open=HOOK_OPEN)
    closed = parts()
    drawn = parts(draw=DRAW)
    c_open = check(opened, ring, "OPEN  (dist mm, overlap mm^3)")
    c_closed = check(closed, ring, "CLOSED")
    c_drawn = check(drawn, ring, "DRAWN ")
    ov = {k: internal(P) for k, P in (("open", opened), ("closed", closed), ("drawn", drawn))}
    print("moving parts vs base / other jaw (mm^3):", ov)
    colours = {"base": (0.62, 0.64, 0.68), "in": (0.2, 0.25, 0.3), "hook": (0.2, 0.25, 0.3), "cam": (0.12, 0.13, 0.15)}
    for n, (s, mat, g) in drawn.items():
        o = doc.addObject("Part::Feature", "tool_" + n)
        o.Shape = s
        if App.GuiUp:
            o.ViewObject.ShapeColor = colours[g]
    r = doc.addObject("Part::Feature", "ring_section")
    r.Shape = ring
    if App.GuiUp:
        r.ViewObject.ShapeColor = (0.55, 0.56, 0.58)
        r.ViewObject.Transparency = 50
    doc.recompute()
    M, com, I, shapes = mass_props(drawn)
    bb = Part.makeCompound([s for s, _, _ in opened.values()]).BoundBox
    os.makedirs(PARTS, exist_ok=True)
    import MeshPart
    for f in os.listdir(PARTS):                       # v1 part meshes no longer used
        if f.startswith("tool_") and f[5:-4] not in drawn:
            os.remove(os.path.join(PARTS, f))
    for n, s, m in shapes:
        me = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.2, AngularDeflection=0.2)
        me.transform(App.Matrix(0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0))
        me.write(os.path.join(PARTS, f"tool_{n}.stl"))
    doc.saveAs(os.path.join(HERE, "ring_tool.FCStd"))
    Part.export([o for o in doc.Objects if o.Name.startswith("tool_")], os.path.join(HERE, "ring_tool.step"))
    info = dict(version=2, mass_kg=round(M, 3), com_m=com, inertia_kgm2=I, palm_x_m=PALM_X / 1000, width_m=W / 1000,
                jaw_width_m=JW / 1000, ring_centre_z_m=-RC / 1000,
                envelope_open_mm=[round(bb.XLength, 1), round(bb.YLength, 1), round(bb.ZLength, 1)],
                parts={n: dict(mass_kg=round(m, 4), group=drawn[n][2]) for n, s, m in shapes},
                joints=dict(jaw_in=dict(axis=[0, 0, -1], stroke_m=IN_STROKE / 1000),
                            hook=dict(axis=[HOOK_AX.x, 0, HOOK_AX.z], stroke_m=HOOK_OPEN / 1000),
                            draw=dict(axis=[-1, 0, 0], stroke_m=DRAW / 1000)),
                radii_mm=dict(jaw_in=[R_IN - JAW_T, R_IN], hook_tip=[R_WALL + TIP_GAP, R_LIP + FINGER_GAP + 0.5],
                              hook_finger=[R_LIP + FINGER_GAP, R_LIP + FINGER_GAP + JAW_T]),
                ring_section_mm=dict(wall_r=[R_IN, R_WALL], lip_r=[R_WALL, R_LIP], top_x=RING_TOP, lip_bottom_x=LIP_BOT),
                checks=dict(open=c_open, closed=c_closed, drawn=c_drawn, internal=ov))
    with open(os.path.join(HERE, "ring_tool.json"), "w") as fh:
        json.dump(info, fh, indent=1)
    print("mass", round(M, 2), "kg  com", [round(v * 1000, 1) for v in com], "mm  envelope (open)", info["envelope_open_mm"])
    return doc


if __name__ == "__main__":
    build()
