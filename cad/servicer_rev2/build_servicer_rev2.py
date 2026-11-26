"""Servicer revision 2 (P7, SPEC_7 D7, 20 Nov 2026): the bus built from parts in FreeCAD.

Decisions (reports/Servicer rev 2 research for P7.md): ESPA standard 15 in port, ~200 kg wet;
16 x 1 N green-monoprop thrusters in 8 corner clusters on the +X / -X faces (one X nozzle, canted outward 50 deg
on the client face / 30 deg aft, and one side nozzle along Y or Z each; checks/thruster_trade.py); four 12 N m s / 0.2 N m wheels in a pyramid
about X (beta 70 deg: more capacity about Y and Z, where the grasp lever puts the detumble momentum);
the bus collides with the client. The arm (16.55 kg, 6 joints) and the sensors carry over unchanged
(the ring tool is stage P7.2).

Run inside FreeCAD:
    freecadcmd cad/servicer_rev2/build_servicer_rev2.py

Writes assets next to this script: servicer_rev2_bus.FCStd, .step (mm), parts/*.stl (m, chaser frame) and
servicer_rev2_bus.json: per-part masses (datasheet class, x 1.2 mass growth allowance), the bus mass, CoM
and inertia from the parts, the nozzle table (site, force direction), the wheel table, the collision
boxes and the bounding box. make_servicer_rev2.py turns it into the MuJoCo scene.

Frame: the chaser body frame of the P3-P6 scenes: origin at the bus centre, +X toward the client
(the arm and sensor face), Z up. Millimetres and kilograms in this script.
"""
import json
import math
import os

import FreeCAD as App
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
LIB = os.path.join(os.path.dirname(HERE), "clients", "build_clients.py")
_ns = {"__file__": LIB, "__name__": "build_clients_lib"}
exec(open(LIB).read(), _ns)
Client, mass_properties, _place = _ns["Client"], _ns["mass_properties"], _ns["_place"]
V = App.Vector

NAME = "servicer_rev2_bus"
MGA = 1.2                       # mass growth allowance on every part (experimental concept, ~20%)
HX, HY, HZ = 250.0, 300.0, 450.0  # bus half sizes (unchanged from rev 1)
CH = 70.0                       # chamfer on the four X-parallel edges
SKIN = 25.0                     # honeycomb panel thickness
XCORNER = (230.0, 400.0)        # (y, z) of the thruster clusters on the +X / -X faces (inside the chamfer)
CANT = 30.0                     # outward cant of the +X-force nozzles (plume aft) [deg]
CANT_FWD = 50.0                 # outward cant of the -X-force nozzles (plume toward the client): 30 deg left the
                                # client inside the 30 deg plume keep-out at every range (phys_rev2 D, 20 Nov)
NOZ_L, NOZ_RT, NOZ_RE = 25.0, 4.0, 10.0
WHEEL_BETA = 70.0               # wheel axes from the X axis [deg]
WHEEL_C = (-90.0, 0.0, 130.0)   # wheel cluster centre (upper bay, above the deck at z = -120)
WHEEL_R, WHEEL_H, WHEEL_OFF = 95.0, 70.0, 150.0
SEP_H = 50.0                    # 15 in separation ring (Lightband class) height on the aft face: inside the ESPA depth

GOLD = (0.80, 0.62, 0.22, 1)
DARK = (0.25, 0.25, 0.27, 1)
METAL = (0.55, 0.56, 0.58, 1)
CELL = (0.10, 0.16, 0.42, 1)
WHITE = (0.88, 0.89, 0.90, 1)
INNER = (0.45, 0.47, 0.50, 1)


def chamfered_box(hx, hy, hz, c):
    b = Part.makeBox(2 * hx, 2 * hy, 2 * hz, V(-hx, -hy, -hz))
    edges = [e for e in b.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]  # X-parallel
    return b.makeChamfer(c, edges)


def unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def panel_holes():
    """Openings in the panels, shaped like the parts that pass through them (0.5-1 mm clearance), so no camera
    can look into the bus past a sensor head."""
    holes = [Part.makeBox(151, 131, 81, V(HX - 150.5, 200 - 65.5, 225 - 40.5)),          # lidar head
             Part.makeCylinder(32.5, 80, V(HX - 70, 200, 300), V(1, 0, 0))]              # camera barrel
    for sy in (1, -1):                                                                     # star-tracker heads
        cut = Part.makeCylinder(35.5, 71, V(0, 0, -70.5), V(0, 0, 1)).fuse(Part.makeCone(28.5, 42.5, 25.5, V(0, 0, 0), V(0, 0, 1)))
        cut = cut.fuse(Part.makeCylinder(43, 60, V(0, 0, 25), V(0, 0, 1)))                   # head + flared baffle + clear above
        holes.append(_place(cut, (-120, sy * 140, HZ - 8), unit((0, 0.35 * sy, 1))))
    return holes


def build():
    s = Client(NAME, "servicer rev 2 bus", "P7 servicer bus from parts")
    nozzles, wheels = [], []

    # ---- structure: chamfered honeycomb shell, internal deck, arm base plate, launch locks -----------
    outer = chamfered_box(HX, HY, HZ, CH)
    inner = chamfered_box(HX - SKIN, HY - SKIN, HZ - SKIN, CH)
    s.add_shape("structure_shell", outer.cut(inner), 42.0, GOLD)
    s.add_box("deck", (2 * HX - 2 * SKIN, 2 * HY - 2 * SKIN, 12), (0, 0, -120), 4.0, INNER)
    s.add_box("arm_base_plate", (8, 160, 160), (HX + 4, 0, -250), 1.5, METAL)
    s.add_box("launch_locks", (30, 60, 400), (HX + 15, -200, 20), 1.5, METAL)  # hold the stowed arm (concept)

    # ---- propulsion: tank + propellant, lines / valves, four pods of four 1 N nozzles --------------
    s.add_shape("prop_tank", Part.makeSphere(130, V(-90, 0, -270)).cut(Part.makeSphere(126, V(-90, 0, -270))), 3.0, METAL)
    s.add_shape("propellant", Part.makeSphere(105, V(-90, 0, -270)), 4.5, (0.6, 0.7, 0.9, 1))  # LMP-103S ~1.24 kg/L
    s.add_box("prop_lines_valves", (300, 60, 12), (-60, 0, -132), 3.0, METAL)   # under the deck, over the tank
    c, sn = math.cos(math.radians(CANT)), math.sin(math.radians(CANT))
    cf, sf = math.cos(math.radians(CANT_FWD)), math.sin(math.radians(CANT_FWD))
    # Layout: 8 corner clusters (user 20 Nov, after checks/thruster_trade.py; replaced X nozzles on the face corners
    # + 4 side pods). Each cluster sits on a bracket at a corner of the +X or -X face and carries one X-force nozzle
    # (firing out of the face, canted outward: +X face 50 deg, -X face 30 deg) and one side nozzle firing straight
    # out along Y or Z (Y where sx*sy*sz > 0, else Z), so side forces act at x = +-0.25 m and give pitch / yaw couples.
    k = 0
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                tag = f"{'f' if sx > 0 else 'a'}_{'p' if sy > 0 else 'n'}y{'p' if sz > 0 else 'n'}z"
                out = unit((0, sy, sz))
                cy, cz = sy * XCORNER[0], sz * XCORNER[1]
                s.add_box(f"cluster_{tag}", (20, 60, 60), (sx * (HX + 10), cy, cz), 0.35, DARK)
                cc, ss = (cf, sf) if sx > 0 else (c, sn)
                xdir = (-sx * cc, -ss * out[1], -ss * out[2])                      # force; plume out of the face
                side = (0, -sy, 0) if sx * sy * sz > 0 else (0, 0, -sz)              # force inward; plume outward
                side_base = (sx * (HX + 14), cy + 30 * sy * (side[1] != 0), cz + 30 * sz * (side[2] != 0))  # clear of the face
                for fdir, base in ((xdir, (sx * (HX + 20), cy, cz)), (side, side_base)):
                    fdir = unit(fdir)
                    plume = [-x for x in fdir]
                    outer_n = Part.makeCone(NOZ_RT + 5, NOZ_RE + 2, NOZ_L, V(0, 0, 0), V(0, 0, 1))
                    inner_n = Part.makeCone(NOZ_RT, NOZ_RE, NOZ_L + 0.2, V(0, 0, -0.1), V(0, 0, 1))
                    s.add_shape(f"thr_{k:02d}", _place(outer_n.cut(inner_n), base, plume), 0.38, METAL)
                    nozzles.append(dict(name=f"thr_{k:02d}", pos_m=[x / 1000 for x in base],
                                        force_dir=[round(x, 6) for x in fdir], cluster=tag))
                    k += 1

    # ---- attitude: four wheels (pyramid about X), drive electronics, star trackers, IMU -------------
    cb, sb = math.cos(math.radians(WHEEL_BETA)), math.sin(math.radians(WHEEL_BETA))
    for i, phi in enumerate((0, 90, 180, 270)):
        cp, sp = math.cos(math.radians(phi)), math.sin(math.radians(phi))
        ax = unit((cb, sb * cp, sb * sp))
        ctr = (WHEEL_C[0], WHEEL_C[1] + WHEEL_OFF * cp, WHEEL_C[2] + WHEEL_OFF * sp)
        base = tuple(ctr[j] - ax[j] * WHEEL_H / 2 for j in range(3))
        s.add_shape(f"wheel_{i}", _place(Part.makeCylinder(WHEEL_R, WHEEL_H), base, ax), 5.0, DARK)
        wheels.append(dict(name=f"wheel_{i}", pos_m=[x / 1000 for x in ctr], axis=[round(x, 6) for x in ax]))
    s.add_box("wheel_electronics", (120, 160, 50), (110, -170, 20), 1.5, INNER)
    for tag, sy in (("a", 1), ("b", -1)):  # head inside the bus, baffle 25 mm proud (NST / Auriga class)
        ax = unit((0, 0.35 * sy, 1))
        head = Part.makeCylinder(35, 70, V(0, 0, -70), V(0, 0, 1)).fuse(Part.makeCone(28, 42, 25, V(0, 0, 0), V(0, 0, 1)))
        bore = Part.makeCone(22, 36, 26, V(0, 0, 0), V(0, 0, 1))
        s.add_shape(f"star_tracker_{tag}", _place(head.cut(bore).removeSplitter(), (-120, sy * 140, HZ - 8), ax), 0.4, DARK)
    s.add_box("imu", (90, 90, 80), (120, 0, -70), 0.8, INNER)

    # ---- RPO sensors: optical bench (flash lidar + camera) at the +X face, where the scene's bus_lidar (0.26, 0.2,
    # 0.24) and bus_cam (0.26, 0.2, 0.30) look out; everything ends at x <= 0.255 so the render cameras sit in front
    s.add_box("lidar", (150, 130, 80), (HX - 75, 200, 225), 6.7, DARK)
    s.add_box("lidar_window", (3, 100, 62), (HX + 1.5, 200, 233), 0.05, (0.05, 0.05, 0.08, 1))  # fills the hood opening (z 202..264)
    cam_tube = Part.makeCylinder(32, 60, V(0, 0, 0), V(0, 0, 1)).cut(Part.makeCylinder(27, 62, V(0, 0, -1), V(0, 0, 1)))
    s.add_shape("camera_barrel", _place(cam_tube, (HX - 55, 200, 300), (1, 0, 0)), 0.4, DARK)
    s.add_box("camera", (120, 70, 70), (HX - 115, 200, 300), 0.8, DARK)
    s.add_shape("lens", Part.makeCylinder(26.5, 3, V(HX - 53.5, 200, 300), V(1, 0, 0)), 0.05, (0.05, 0.05, 0.08, 1))  # disc in the barrel (ID 54)
    hood = Part.makeBox(6, 170, 175, V(HX, 115, 175))                                       # z 175..350, 20 mm under the cluster
    hood = hood.cut(Part.makeBox(10, 100, 62, V(HX - 2, 150, 202)))                       # lidar opening (z 202..264)
    hood = hood.cut(Part.makeCylinder(32.5, 10, V(HX - 2, 200, 300), V(1, 0, 0)))            # camera opening
    s.add_shape("sensor_hood", hood, 0.6, METAL)                                           # frame on the face
    s.add_box("optical_bench", (195, 150, 20), (HX - SKIN - 97.5, 200, 175), 2.5, METAL)

    # ---- launch interface: 15 in separation ring (Lightband class) on the aft (-X, port) face ------------
    ring = Part.makeCylinder(215, SEP_H, V(0, 0, 0), V(0, 0, 1)).cut(Part.makeCylinder(180, SEP_H + 2, V(0, 0, -1), V(0, 0, 1)))
    s.add_shape("separation_ring", _place(ring, (-HX, 0, 0), (-1, 0, 0)), 1.6, METAL)

    # panel openings: the shell is cut where the sensor heads and star trackers pass through it
    shell = next(p for p in s.parts if p["name"] == "structure_shell")
    for hole in panel_holes():
        shell["shape"] = shell["shape"].cut(hole)

    # ---- power and avionics --------------------------------------------------------------------------
    for sy in (1, -1):
        s.add_box(f"cells_{'p' if sy > 0 else 'n'}y", (440, 3, 740), (0, sy * (HY + 1.5), 0), 2.5, CELL)
    s.add_box("battery", (200, 150, 120), (125, -150, -345), 9.0, INNER)       # lower bay, beside the tank
    s.add_box("obc_compute", (160, 120, 80), (60, -150, 250), 6.0, INNER)
    s.add_box("pcdu", (180, 140, 70), (130, 165, -300), 5.0, INNER)
    s.add_box("transponder", (120, 100, 50), (-160, -190, -250), 2.0, INNER)
    s.add_box("gnss_sband", (80, 80, 10), (100, 0, HZ + 5), 0.8, WHITE)

    # ---- thermal and harness -------------------------------------------------------------------------
    s.add_box("radiator", (420, 480, 4), (0, 0, -HZ - 2), 3.0, WHITE)
    for k, (sy, sz) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):      # harness: looms along the four inner walls
        size = (220, 10, 340) if sy else (250, 400, 10)            # clear of the deck, sensors and star trackers
        ctr = (-90, sy * (HY - SKIN - 6), 70) if sy else (65, 0, sz * (HZ - SKIN - 6))
        s.add_box(f"harness_{k}", size, ctr, 2.25, INNER)
    mli = outer.cut(chamfered_box(HX - 3, HY - 3, HZ - 3, CH))
    for hole in panel_holes():
        mli = mli.cut(hole)
    s.add_shape("mli", mli, 6.0, GOLD)
    return s, nozzles, wheels


def protrusion_boxes(s):
    """Axis-aligned collision boxes (m) for the parts outside the bus box: thruster clusters with their nozzles,
    arm base plate, launch locks, sensor hood, star-tracker baffles, separation ring."""
    groups = {}
    for p in s.parts:
        n = p["name"]
        key = None
        if n.startswith("cluster_"):
            key = n
        elif n.startswith("thr_"):
            c = p["shape"].BoundBox.Center
            key = min((q["name"] for q in s.parts if q["name"].startswith("cluster_")),
                      key=lambda m: (next(r for r in s.parts if r["name"] == m)["shape"].BoundBox.Center - c).Length)
        elif n in ("arm_base_plate", "launch_locks", "sensor_hood", "star_tracker_a", "star_tracker_b", "separation_ring"):
            key = n
        if key:
            groups.setdefault(key, []).append(p["shape"].BoundBox)
    out = []
    for k, bbs in groups.items():
        bb = bbs[0]
        for x in bbs[1:]:
            bb.add(x)
        out.append(dict(name=k, pos=[round(v / 1000, 4) for v in (bb.Center.x, bb.Center.y, bb.Center.z)],
                        size=[round(v / 2000, 4) for v in (bb.XLength, bb.YLength, bb.ZLength)]))
    return out


def export(s, nozzles, wheels):
    os.makedirs(os.path.join(HERE, "parts"), exist_ok=True)
    base_mass = {p["name"]: p["mass"] for p in s.parts}
    for p in s.parts:
        p["mass"] = p["mass"] * MGA
    M, com, I = mass_properties(s)
    if NAME in App.listDocuments():
        App.closeDocument(NAME)
    doc = App.newDocument(NAME)
    for p in s.parts:
        o = doc.addObject("Part::Feature", p["name"])
        o.Shape = p["shape"]
        if App.GuiUp:
            o.ViewObject.ShapeColor = tuple(p["rgba"][:3])
    doc.recompute()
    doc.saveAs(os.path.join(HERE, NAME + ".FCStd"))
    comp = Part.makeCompound([p["shape"] for p in s.parts])
    comp.exportStep(os.path.join(HERE, NAME + ".step"))
    scale = App.Matrix()
    scale.scale(0.001, 0.001, 0.001)
    for p in s.parts:
        msh = MeshPart.meshFromShape(Shape=p["shape"], LinearDeflection=0.6, AngularDeflection=0.35)
        msh.transform(scale)
        msh.write(os.path.join(HERE, "parts", p["name"] + ".stl"))
    bb = comp.BoundBox
    out = dict(
        name=NAME, frame="chaser body (bus centre, +X to the client), metres", mga=MGA,
        mass_kg=round(M, 3), com_m=[round(v, 5) for v in com], inertia_kgm2=[[round(v, 4) for v in r] for r in I],
        bbox_m=[round(v / 1000, 4) for v in (bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax)],
        collision_boxes=[dict(name="bus_col", pos=[0, 0, 0], size=[HX / 1000, HY / 1000, HZ / 1000])] + protrusion_boxes(s),
        nozzles=nozzles, thrust_n=1.0, isp_s=220.0, cant_deg=CANT, cant_fwd_deg=CANT_FWD, port_face_x_m=-(HX + SEP_H) / 1000,
        wheels=wheels, wheel_h_nms=12.0, wheel_tau_nm=0.2, wheel_beta_deg=WHEEL_BETA,
        parts=[dict(name=p["name"], mass_kg=round(p["mass"], 3), base_mass_kg=base_mass[p["name"]],
                    rgba=list(p["rgba"])) for p in s.parts])
    with open(os.path.join(HERE, NAME + ".json"), "w") as fh:
        json.dump(out, fh, indent=1)
    return out, doc


if __name__ == "__main__":
    s, nozzles, wheels = build()
    bad = [p["name"] for p in s.parts if not p["shape"].isValid() or not p["shape"].Solids]
    assert not bad, f"invalid parts: {bad}"
    out, doc = export(s, nozzles, wheels)
    print(f"bus {out['mass_kg']} kg (x{MGA} MGA), CoM {out['com_m']} m, inertia diag "
          f"{[out['inertia_kgm2'][i][i] for i in range(3)]} kg m^2")
    print("bbox m", out["bbox_m"], "parts", len(s.parts), "nozzles", len(nozzles), "wheels", len(wheels))
    if App.GuiUp:
        import FreeCADGui as Gui
        Gui.getDocument(NAME).activeView().viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
