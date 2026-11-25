"""FreeCAD concept model: Centerline Dual-Arm Recessed-Cavity Servicer (Standard 15" ESPA Fit).

Design update per user feedback:
- Maximizes depth inside the standard 15" ESPA envelope (711 X x 610 Y x 965 Z mm):
  core bus extended from 500 -> 600 mm deep (600 X x 600 Y x 900 Z mm), keeping width at 606 mm
  with solar panels and total stowed envelope at 695 x 606 x ~950 mm <= 711 x 610 x 965 mm.
- Moves both recessed vertical arm cavities inward to flank the vertical centerline (y = +-70 mm,
  20 mm structural spine between them, each bay 130 deep x 120 wide x 740 mm tall).
- Places the RPO optical bench (flash lidar + bus_cam) cleanly on the upper +Y cheek (y = +215 mm).
- Because y = +-70 mm lies well inside 15" (381 mm), 24" (609.6 mm), and 37" (939.8 mm) rings,
  both arms grab across the full diameter of small and medium satellites with shallow rim angles
  (13.3 deg at 24", 21.6 deg at 15").

Toggleable groups in FreeCAD:
  - Bus
  - ESPA_Envelope_Ref (711 x 610 x 965 mm bounding box)
  - Arms_Stowed (flush inside centerline bays, zero overhang)
  - Arms_Deployed_24in + Target_Ring_24in_610mm
  - Arms_Deployed_15in + Target_Ring_15in_381mm
"""
import glob
import math
import os

import FreeCAD as App
import Part

V = App.Vector
HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else \
    r"c:/Shrey_Projs/robotics/mjorbit/models/concepts/dual_arm_concept"
DOC_NAME = "DualArmServicerConcept"

# Bus geometry (mm, chaser body frame: origin at bus centre, +X to client, Z up)
HX, HY, HZ = 300.0, 300.0, 450.0     # 600 X x 600 Y x 900 Z mm core (ESPA-depth optimized)
CH = 50.0                             # chamfer on X-parallel edges
SEP_H = 50.0                          # 15 in separation ring depth on -X (-350..-300)
CAV_XMIN = 170.0                      # 130 mm recessed depth from +300 face
CAV_W = 120.0                         # Y width of each cavity
CAV_H = 800.0                         # Z height of each cavity (-400..+400; 40 mm ceiling clearance on deploy)
CAV_Y = 70.0                          # Centerline-flanking Y centers (+-70 mm -> 20 mm spine)

# Arm & gripper kinematics (mm)
ARM_MOUNT_X = 235.0
ARM_MOUNT_Z = 300.0
L1 = 300.0
L2 = 250.0
GRIP_LEN = 105.0

# Target plane standoff (+420 mm ahead of +300 bus face)
RING_X = 720.0
R_24IN = 304.8                        # 609.6 mm diameter (P7 client)
R_15IN = 190.5                        # 381.0 mm diameter (smallsat / ESPA ring)

# Colors (RGB 0..1)
GOLD = (0.80, 0.62, 0.22)
DARK = (0.20, 0.21, 0.24)
TITANIUM = (0.34, 0.36, 0.40)
CARBON = (0.58, 0.62, 0.68)
METAL = (0.74, 0.76, 0.80)
JAW_ORANGE = (0.88, 0.42, 0.12)
CELL_BLUE = (0.10, 0.16, 0.42)
WHITE = (0.88, 0.89, 0.90)
CAM_DARK = (0.10, 0.11, 0.14)
LENS_CYAN = (0.18, 0.56, 0.86)
RING_SILVER = (0.78, 0.80, 0.84)
CLIENT_GHOST = (0.75, 0.60, 0.25)
ESPA_CYAN = (0.20, 0.78, 0.95)


def add_part(doc, group, name, shape, color, transparency=0):
    assert shape.isValid() and len(shape.Solids) >= 1, f"Invalid shape for {name}"
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    group.addObject(obj)
    if App.GuiUp:
        obj.ViewObject.ShapeColor = color
        if transparency:
            obj.ViewObject.Transparency = transparency
    return obj


def chamfered_box(hx, hy, hz, ch):
    b = Part.makeBox(2.0 * hx, 2.0 * hy, 2.0 * hz, V(-hx, -hy, -hz))
    edges = [e for e in b.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
    return b.makeChamfer(ch, edges)


def make_cylinder_between(p0, p1, radius):
    d = p1.sub(p0)
    length = d.Length
    assert length > 1e-3, "Zero-length cylinder"
    return Part.makeCylinder(radius, length, p0, d.normalize())


def make_y_joint(center, radius, length):
    base = V(center.x, center.y - 0.5 * length, center.z)
    return Part.makeCylinder(radius, length, base, V(0, 1, 0))


def make_gripper_parts(wrist_pos, u_reach, u_rad, u_tan, cam_side=-1.0):
    """Build compact Marman-ring jaw gripper + outboard 50 mm dual stereo camera bar."""
    R = App.Rotation(u_reach, u_rad, u_tan, "XYZ")
    T = App.Placement(wrist_pos, R)

    def xform(sh):
        s = sh.copy()
        s.Placement = T.multiply(s.Placement)
        return s

    palm_base = Part.makeBox(72.0, 56.0, 60.0, V(0.0, -28.0, -30.0))
    outer_finger = Part.makeBox(38.0, 14.0, 52.0, V(67.0, 18.0, -26.0))
    housing = palm_base.fuse(outer_finger).removeSplitter()

    in_jaw = Part.makeBox(36.0, 14.0, 48.0, V(69.0, -32.0, -24.0))
    jaw_pad = Part.makeBox(22.0, 8.0, 40.0, V(78.0, 11.0, -20.0))
    jaws = in_jaw.fuse(jaw_pad).removeSplitter()

    # Cam bar on local Z side (cam_side * [32..54]) -> outboard in both stow & deployed poses
    z_min = 32.0 if cam_side > 0 else -54.0
    z_ctr = 43.0 if cam_side > 0 else -43.0
    cam_bar = Part.makeBox(26.0, 62.0, 22.0, V(18.0, -31.0, z_min))
    lens_a = Part.makeCylinder(7.0, 10.0, V(44.0, -25.0, z_ctr), V(1, 0, 0))
    lens_b = Part.makeCylinder(7.0, 10.0, V(44.0, 25.0, z_ctr), V(1, 0, 0))
    lenses = lens_a.fuse(lens_b).removeSplitter()

    return xform(housing), xform(jaws), xform(cam_bar), xform(lenses)


def solve_ik_xz(base_pos, wrist_pos, L1_len, L2_len, elbow_sign):
    """Exact 2-link planar IK in the X-Z plane."""
    dx = wrist_pos.x - base_pos.x
    dz = wrist_pos.z - base_pos.z
    dist = math.hypot(dx, dz)
    assert abs(L1_len - L2_len) < dist < (L1_len + L2_len), f"Unreachable dist={dist:.1f}"
    cos_q2 = (dist * dist - L1_len * L1_len - L2_len * L2_len) / (2.0 * L1_len * L2_len)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    q2 = elbow_sign * math.acos(cos_q2)
    phi = math.atan2(dz, dx)
    psi = math.atan2(L2_len * math.sin(q2), L1_len + L2_len * math.cos(q2))
    q1 = phi - psi
    elbow = V(
        base_pos.x + L1_len * math.cos(q1),
        base_pos.y,
        base_pos.z + L1_len * math.sin(q1),
    )
    return elbow, math.degrees(q1), math.degrees(q2), dist


def build_arm_assembly(doc, group, prefix, base_pos, elbow_pos, wrist_pos, u_reach, u_rad, u_tan, cam_side=-1.0):
    objs = []
    objs.append(add_part(doc, group, f"{prefix}_BaseJoint", make_y_joint(base_pos, 38.0, 68.0), TITANIUM))
    objs.append(add_part(doc, group, f"{prefix}_UpperLink", make_cylinder_between(base_pos, elbow_pos, 22.0), CARBON))
    objs.append(add_part(doc, group, f"{prefix}_ElbowJoint", make_y_joint(elbow_pos, 32.0, 58.0), TITANIUM))
    objs.append(add_part(doc, group, f"{prefix}_ForearmLink", make_cylinder_between(elbow_pos, wrist_pos, 18.0), CARBON))
    h, j, cb, ls = make_gripper_parts(wrist_pos, u_reach, u_rad, u_tan, cam_side=cam_side)
    objs.append(add_part(doc, group, f"{prefix}_Gripper", h, METAL))
    objs.append(add_part(doc, group, f"{prefix}_GripperJaws", j, JAW_ORANGE))
    objs.append(add_part(doc, group, f"{prefix}_StereoCamBar", cb, CAM_DARK))
    objs.append(add_part(doc, group, f"{prefix}_StereoLenses", ls, LENS_CYAN))
    return objs


def build_deployed_ring_scenario(doc, g_arms, g_tgt, shell, tag, ring_r, b1, b2):
    z_ring = math.sqrt(ring_r * ring_r - CAV_Y * CAV_Y)
    phi_deg = math.degrees(math.asin(CAV_Y / ring_r))
    w1 = V(RING_X - GRIP_LEN, CAV_Y, z_ring)
    w2 = V(RING_X - GRIP_LEN, -CAV_Y, -z_ring)

    e1, q1_1, q2_1, d1 = solve_ik_xz(b1, w1, L1, L2, elbow_sign=-1.0)
    e2, q1_2, q2_2, d2 = solve_ik_xz(b2, w2, L1, L2, elbow_sign=+1.0)

    u_r1 = V(0.0, CAV_Y, z_ring).normalize()
    u_t1 = V(1.0, 0.0, 0.0).cross(u_r1).normalize()
    u_r2 = V(0.0, -CAV_Y, -z_ring).normalize()
    u_t2 = V(1.0, 0.0, 0.0).cross(u_r2).normalize()

    a1_objs = build_arm_assembly(doc, g_arms, f"Dep{tag}_Arm1", b1, e1, w1, V(1, 0, 0), u_r1, u_t1, cam_side=-1.0)
    a2_objs = build_arm_assembly(doc, g_arms, f"Dep{tag}_Arm2", b2, e2, w2, V(1, 0, 0), u_r2, u_t2, cam_side=-1.0)

    a1_comp = Part.makeCompound([o.Shape for o in a1_objs])
    a2_comp = Part.makeCompound([o.Shape for o in a2_objs])
    ov_shell = a1_comp.common(shell).Volume + a2_comp.common(shell).Volume
    ov_inter = a1_comp.common(a2_comp).Volume

    # Target Marman ring + subtle ghost bus stub behind it
    ring_outer = Part.makeCylinder(ring_r + 10.0, 68.0, V(RING_X, 0, 0), V(1, 0, 0))
    ring_inner = Part.makeCylinder(ring_r - 18.0, 72.0, V(RING_X - 2.0, 0, 0), V(1, 0, 0))
    ring_flange = Part.makeCylinder(ring_r + 18.0, 14.0, V(RING_X, 0, 0), V(1, 0, 0)).cut(ring_inner)
    marman = ring_outer.cut(ring_inner).fuse(ring_flange).removeSplitter()
    add_part(doc, g_tgt, f"Ring_{tag}_{int(round(2*ring_r))}mm", marman, RING_SILVER, transparency=18)

    stub_w = max(500.0, 2.0 * ring_r + 200.0)
    stub = Part.makeBox(160.0, stub_w, stub_w, V(RING_X + 68.0, -0.5 * stub_w, -0.5 * stub_w))
    add_part(doc, g_tgt, f"ClientStub_{tag}", stub, CLIENT_GHOST, transparency=84)

    print(
        f"[{tag}] D={2*ring_r:.1f}mm z_ring=+-{z_ring:.1f}mm rim_angle={phi_deg:.1f}deg "
        f"reach={d1:.1f}/{L1+L2:.0f}mm (margin={L1+L2-d1:.1f}mm) "
        f"q1=+-{abs(q1_1):.1f} q2=+-{abs(q2_1):.1f} "
        f"ov_shell={ov_shell:.2f} ov_arms={ov_inter:.2f}"
    )


def build():
    if DOC_NAME in App.listDocuments():
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)

    g_bus = doc.addObject("App::DocumentObjectGroup", "Bus")
    g_espa = doc.addObject("App::DocumentObjectGroup", "ESPA_Envelope_Ref")
    g_stow = doc.addObject("App::DocumentObjectGroup", "Arms_Stowed")
    g_d24 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_24in")
    g_t24 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_24in_610mm")
    g_d15 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_15in")
    g_t15 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_15in_381mm")

    # 1. Core chamfered bus shell (600 X x 600 Y x 900 Z mm) with centerline-flanking cavities
    shell = chamfered_box(HX, HY, HZ, CH)
    cav_p = Part.makeBox(HX - CAV_XMIN + 10.0, CAV_W, CAV_H, V(CAV_XMIN, CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
    cav_n = Part.makeBox(HX - CAV_XMIN + 10.0, CAV_W, CAV_H, V(CAV_XMIN, -CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
    shell = shell.cut(cav_p.fuse(cav_n)).removeSplitter()
    add_part(doc, g_bus, "BusShell_GoldMLI", shell, GOLD)

    # Embedded dark backing strips inside each cavity (zero coplanar Z-fighting)
    for tag, cy in (("Liner_Arm1_PY", CAV_Y), ("Liner_Arm2_NY", -CAV_Y)):
        liner = Part.makeBox(4.0, CAV_W - 6.0, CAV_H - 6.0, V(CAV_XMIN - 1.0, cy - 0.5 * (CAV_W - 6.0), -0.5 * (CAV_H - 6.0)))
        add_part(doc, g_bus, tag, liner, DARK)

    # 2. Optical bench on upper +Y cheek (y = +215 mm, max X = +345 mm -> 695 mm <= 711 mm ESPA)
    bench = Part.makeBox(20.0, 136.0, 166.0, V(HX - 5.0, 215.0 - 68.0, 165.0))
    lidar_win = Part.makeBox(8.0, 100.0, 62.0, V(HX + 14.0, 215.0 - 50.0, 185.0))
    cam_barrel = Part.makeCylinder(28.0, 26.0, V(HX + 14.0, 215.0, 285.0), V(1, 0, 0))
    cam_lens = Part.makeCylinder(22.0, 5.0, V(HX + 40.0, 215.0, 285.0), V(1, 0, 0))
    add_part(doc, g_bus, "CheekSensorBench", bench, TITANIUM)
    add_part(doc, g_bus, "FlashLidarAperture", lidar_win, CAM_DARK)
    add_part(doc, g_bus, "BusCamBarrel", cam_barrel, DARK)
    add_part(doc, g_bus, "BusCamLens", cam_lens, LENS_CYAN)

    # 3. Aft 15-inch separation ring (-X port face at x = -350 mm)
    sep_ring = Part.makeCylinder(215.0, SEP_H, V(-HX - SEP_H, 0.0, 0.0), V(1, 0, 0)).cut(
        Part.makeCylinder(180.0, SEP_H + 4.0, V(-HX - SEP_H - 2.0, 0.0, 0.0), V(1, 0, 0))
    )
    add_part(doc, g_bus, "SeparationRing_15in", sep_ring, METAL)

    # 4. +-Y body solar panels (606 mm <= 610), -Z radiator, +Z star trackers, 8 corner RCS clusters
    for tag, sy in (("SolarPanel_PY", 1.0), ("SolarPanel_NY", -1.0)):
        y0 = sy * HY if sy > 0 else -(HY + 3.0)
        add_part(doc, g_bus, tag, Part.makeBox(520.0, 3.0, 740.0, V(-260.0, y0, -370.0)), CELL_BLUE)
    add_part(doc, g_bus, "Radiator_NZ", Part.makeBox(520.0, 480.0, 4.0, V(-260.0, -240.0, -HZ - 4.0)), WHITE)
    for tag, sy in (("StarTracker_PY", 1.0), ("StarTracker_NY", -1.0)):
        st = Part.makeCone(24.0, 36.0, 42.0, V(0, 0, 0), V(0, 0, 1))
        st.Placement = App.Placement(V(-120.0, sy * 150.0, HZ - 4.0), App.Rotation(V(1, 0, 0), -sy * 18.0))
        add_part(doc, g_bus, tag, st, DARK)

    clusters = []
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            for sz in (1.0, -1.0):
                cx = sx * (HX + 10.0)
                cy = sy * 245.0
                cz = sz * 400.0
                clusters.append(Part.makeBox(22.0, 50.0, 50.0, V(cx - 11.0, cy - 25.0, cz - 25.0)))
    c_comp = clusters[0]
    for c in clusters[1:]:
        c_comp = c_comp.fuse(c)
    add_part(doc, g_bus, "RCS_CornerClusters_8x", c_comp, DARK)

    # 5. Standard 15" ESPA envelope reference box (711 X x 610 Y x 965 Z mm from port face x=-350)
    espa_box = Part.makeBox(711.0, 610.0, 965.0, V(-HX - SEP_H, -305.0, -460.0))
    add_part(doc, g_espa, "ESPA_Standard_711x610x965", espa_box, ESPA_CYAN, transparency=92)

    # 6. Arms_Stowed (flush inside centerline cavities)
    b1 = V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z)
    b2 = V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z)
    stow_objs = []
    stow_objs += build_arm_assembly(
        doc, g_stow, "Stow_Arm1", b1,
        V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1),
        V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1 - L2),
        u_reach=V(0, 0, -1), u_rad=V(1, 0, 0), u_tan=V(0, -1, 0), cam_side=-1.0,
    )
    stow_objs += build_arm_assembly(
        doc, g_stow, "Stow_Arm2", b2,
        V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1),
        V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1 + L2),
        u_reach=V(0, 0, 1), u_rad=V(1, 0, 0), u_tan=V(0, 1, 0), cam_side=-1.0,
    )

    stow_comp = Part.makeCompound([o.Shape for o in stow_objs])
    ov_stow = stow_comp.common(shell).Volume
    sbb = stow_comp.BoundBox
    bus_bb = Part.makeCompound([o.Shape for o in g_bus.Group]).BoundBox
    tot_x = max(bus_bb.XMax, sbb.XMax) - bus_bb.XMin
    tot_y = max(bus_bb.YMax, sbb.YMax) - min(bus_bb.YMin, sbb.YMin)
    tot_z = max(bus_bb.ZMax, sbb.ZMax) - min(bus_bb.ZMin, sbb.ZMin)

    # 7. Deployed scenarios: 24" (609.6 mm) and 15" (381.0 mm) rings
    build_deployed_ring_scenario(doc, g_d24, g_t24, shell, "24in", R_24IN, b1, b2)
    build_deployed_ring_scenario(doc, g_d15, g_t15, shell, "15in", R_15IN, b1, b2)

    doc.recompute()
    out_fcstd = os.path.join(HERE, "dual_arm_concept.FCStd")
    doc.saveAs(out_fcstd)
    for bak in glob.glob(os.path.join(HERE, "*.FCBak")):
        try:
            os.remove(bak)
        except OSError:
            pass

    print(f"SAVED: {out_fcstd}")
    print(f"Bus bbox (mm): X=[{bus_bb.XMin:.1f}, {bus_bb.XMax:.1f}] Y=[{bus_bb.YMin:.1f}, {bus_bb.YMax:.1f}] Z=[{bus_bb.ZMin:.1f}, {bus_bb.ZMax:.1f}]")
    print(f"Stowed arms bbox (mm): X=[{sbb.XMin:.1f}, {sbb.XMax:.1f}] Y=[{sbb.YMin:.1f}, {sbb.YMax:.1f}] Z=[{sbb.ZMin:.1f}, {sbb.ZMax:.1f}]")
    print(f"Total Stowed Envelope (mm): {tot_x:.1f} X x {tot_y:.1f} Y x {tot_z:.1f} Z (ESPA margins: {711-tot_x:.1f}, {610-tot_y:.1f}, {965-tot_z:.1f})")
    print(f"Stowed overlap vs shell (mm^3): {ov_stow:.2f}")
    assert tot_x <= 711.0 and tot_y <= 610.0 and tot_z <= 965.0, "Exceeded ESPA envelope!"
    assert ov_stow < 0.1, f"Stowed overlap {ov_stow}!"
    return doc


if __name__ == "__main__":
    build()
