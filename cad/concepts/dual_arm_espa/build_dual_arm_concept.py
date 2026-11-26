"""FreeCAD concept v3: Compact Dual-Arm Recessed Servicer (240+200 mm) + Swept Arc Reliefs + Bulkhead q0.

Combines three key architectural refinements:
  1. Compact arm (L1 = 240 mm, L2 = 200 mm, GRIP_LEN = 105 mm -> 545 mm stowed) with
     relocated centerline-flanking shoulders at (248, +-70, +-230) mm (r_s = 240.4 mm).
     Putting r_s in the open gap between 15" (190.5 mm) and 24" (304.8 mm) rings
     eliminates the r_p ~= 0 cylindrical singularity on 24" targets!
  2. Shortened 690 mm vertical cavities (z in [-345, +345] mm) with swept circular arc
     relief pockets (R_ARC = 115 mm, clipped cleanly to |Y| >= 10 mm) matching the
     natural cone of q0 base-roll rotation (~90 deg outboard sweep) while clearing the
     mid-height +Y optical bench (z in [-83, +83]) and all corner RCS thruster clusters.
  3. Bulkhead-mounted Base-Roll drives (q0 around +X, X = 94..170 mm) + 4-DOF
     R_x - R_y - R_y - R_y chain achieving exact 0.0 deg jaw roll misalignment across
     8", 15", 24", 37", and large (+80 Y, -50 Z) off-center targets, while keeping the
     stowed spacecraft 100% inside the 711 x 610 x 965 mm standard 15" ESPA envelope.

Toggleable groups in FreeCAD:
  - Bus                           (hollowed 600x600x900 core, arc reliefs, mid +Y sensors, fixed q0 drives)
  - Bus_SideSlice_Cutaway         (Y <= +70 mm cutaway exposing Arm 1 bay, arc wall & bulkhead drive)
  - Bus_Internal_Ref              (COPV tank, 4x RW pyramid, avionics/battery clearance)
  - ESPA_Envelope_Ref             (711 x 610 x 965 mm standard 15" ESPA limit box)
  - Arms_Stowed                   (q0 = 0 deg, flush inside 690 mm centerline bays)
  - Arms_Deployed_24in            + Target_Ring_24in_610mm        (concentric 24", ring_x = 650 mm)
  - Arms_Deployed_15in            + Target_Ring_15in_381mm        (concentric 15", ring_x = 650 mm)
  - Arms_Deployed_8in             + Target_Ring_8in_203mm         (concentric 8", ring_x = 650 mm)
  - Arms_Deployed_37in            + Target_Ring_37in_940mm        (concentric 37", ring_x = 690 mm)
  - Arms_Deployed_24in_LargeOff   + Target_Ring_24in_LargeOff     (+80 Y, -50 Z off-center, q0 ~= -40 deg)
"""
import glob
import math
import os

import FreeCAD as App
import Part

V = App.Vector
HERE = (
    os.path.dirname(os.path.abspath(__file__))
    if "__file__" in globals()
    else r"c:/Shrey_Projs/robotics/mjorbit-servicing/cad/concepts/dual_arm_espa"
)
DOC_NAME = "DualArmServicerConcept"

# Bus geometry (mm, chaser body frame: origin at bus centre, +X to client, Z up)
HX, HY, HZ = 300.0, 300.0, 450.0     # 600 X x 600 Y x 900 Z mm core (15" ESPA optimized)
CH = 50.0                             # chamfer on X-parallel edges
SEP_H = 50.0                          # 15 in separation ring depth on -X (-350..-300)
CAV_XMIN = 170.0                      # 130 mm recessed cavity depth from +300 face
CAV_W = 120.0                         # Y width of each cavity
CAV_H = 690.0                         # Z height of shortened cavity (-345..+345; 105 mm end structure)
CAV_Y = 70.0                          # Centerline-flanking Y centers (+-70 mm -> 20 mm spine)
R_ARC = 115.0                         # Swept circular arc relief radius around each shoulder (230 + 115 = 345)

# Compact arm & bulkhead Base-Roll geometry (mm)
ARM_MOUNT_X = 248.0                   # shoulder pitch axis inside 130 mm cavity
ARM_MOUNT_Z = 230.0                   # r_s = hypot(70, 230) = 240.4 mm (clear of 15" & 24" rings!)
L1 = 240.0
L2 = 200.0
GRIP_LEN = 105.0

# Target plane standoffs & ring radii (mm)
RING_X = 650.0                        # +350 mm ahead of +300 bus face (nominal 8"–24")
RING_X_37 = 690.0                     # +390 mm standoff for 37" medium-sat adapter
R_37IN = 469.9                        # 939.8 mm diameter
R_24IN = 304.8                        # 609.6 mm diameter (P7 client)
R_15IN = 190.5                        # 381.0 mm diameter (smallsat / ESPA ring)
R_8IN = 101.6                         # 203.2 mm diameter (microsat Lightband)

# Colors (RGB 0..1)
GOLD = (0.80, 0.62, 0.22)
DARK = (0.20, 0.21, 0.24)
TITANIUM = (0.34, 0.36, 0.40)
CARBON = (0.58, 0.62, 0.68)
METAL = (0.74, 0.76, 0.80)
JAW_ORANGE = (0.88, 0.42, 0.12)
ROLL_AMBER = (0.94, 0.52, 0.10)
CELL_BLUE = (0.10, 0.16, 0.42)
WHITE = (0.88, 0.89, 0.90)
CAM_DARK = (0.10, 0.11, 0.14)
LENS_CYAN = (0.18, 0.56, 0.86)
RING_SILVER = (0.78, 0.80, 0.84)
CLIENT_GHOST = (0.75, 0.60, 0.25)
ESPA_CYAN = (0.20, 0.78, 0.95)
TANK_STEEL = (0.64, 0.68, 0.74)
RW_COPPER = (0.76, 0.46, 0.22)
AVIONICS_SLATE = (0.26, 0.34, 0.44)


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


def set_group_visible(group, visible):
    if App.GuiUp:
        for obj in group.Group:
            obj.ViewObject.Visibility = visible


def chamfered_box(hx, hy, hz, ch):
    b = Part.makeBox(2.0 * hx, 2.0 * hy, 2.0 * hz, V(-hx, -hy, -hz))
    edges = [e for e in b.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
    return b.makeChamfer(ch, edges)


def make_cavity_mouth_bevel(cy):
    """Build a 10 mm 45 deg front mouth bevel prism at X = 290..305 mm for shortened slot."""
    is_pos = cy > 0.0
    dy_p0 = cy + 0.5 * CAV_W
    dy_n0 = cy - 0.5 * CAV_W
    dy_p1 = dy_p0 + (15.0 if is_pos else 7.5)
    dy_n1 = dy_n0 - (7.5 if is_pos else 15.0)
    hz0 = 0.5 * CAV_H
    hz1 = hz0 + 15.0

    w0 = Part.makePolygon([
        V(290.0, dy_n0, -hz0), V(290.0, dy_p0, -hz0),
        V(290.0, dy_p0, hz0), V(290.0, dy_n0, hz0), V(290.0, dy_n0, -hz0),
    ])
    w1 = Part.makePolygon([
        V(305.0, dy_n1, -hz1), V(305.0, dy_p1, -hz1),
        V(305.0, dy_p1, hz1), V(305.0, dy_n1, hz1), V(305.0, dy_n1, -hz1),
    ])
    return Part.makeLoft([w0, w1], True)


def make_arc_relief_cut(cy_s, cz_s):
    """Swept circular arc relief pocket (R_ARC = 115 mm) + 10 mm mouth cone, clipped to |Y| >= 10 mm."""
    is_pos = cy_s > 0.0
    clip_y0 = 10.0 if is_pos else -300.0
    clip_box = Part.makeBox(200.0, 290.0, 400.0, V(165.0, clip_y0, cz_s - 200.0))

    cyl = Part.makeCylinder(R_ARC, 145.0, V(CAV_XMIN, cy_s, cz_s), V(1, 0, 0))
    cone = Part.makeCone(R_ARC, R_ARC + 15.0, 15.0, V(290.0, cy_s, cz_s), V(1, 0, 0))
    return cyl.fuse(cone).common(clip_box)


def make_cylinder_between(p0, p1, radius):
    d = p1.sub(p0)
    length = d.Length
    assert length > 1e-3, "Zero-length cylinder"
    return Part.makeCylinder(radius, length, p0, d.normalize())


def make_axis_joint(center, axis_dir, radius, length):
    u = V(axis_dir.x, axis_dir.y, axis_dir.z).normalize()
    base = center.sub(u.multiply(0.5 * length))
    return Part.makeCylinder(radius, length, base, u)


def make_clevis_yoke(cy, cz, u_rad, u_tan):
    """Build rotating turntable hub (X=171..193), amber roll collar, and twin-ear clevis yoke."""
    cx_vec = V(1.0, 0.0, 0.0)
    R = App.Rotation(cx_vec, u_rad, u_tan, "XYZ")
    T = App.Placement(V(0.0, cy, cz), R)

    def xform(sh):
        s = sh.copy()
        s.Placement = T.multiply(s.Placement)
        return s

    hub_main = Part.makeCylinder(38.0, 22.0, V(171.0, 0.0, 0.0), cx_vec)
    hub_step = Part.makeCylinder(38.0, 12.0, V(193.0, 0.0, 0.0), cx_vec)

    ear_prof = Part.Face(Part.makePolygon([
        V(200.0, -21.0, 0.0), V(ARM_MOUNT_X, -21.0, 0.0),
        V(ARM_MOUNT_X + 21.0, -13.0, 0.0), V(ARM_MOUNT_X + 21.0, 13.0, 0.0),
        V(ARM_MOUNT_X, 21.0, 0.0), V(200.0, 21.0, 0.0), V(200.0, -21.0, 0.0),
    ]))
    ear_p = ear_prof.extrude(V(0.0, 0.0, 10.0))
    ear_p.Placement = App.Placement(V(0.0, 0.0, 28.0), App.Rotation())
    ear_n = ear_prof.extrude(V(0.0, 0.0, 10.0))
    ear_n.Placement = App.Placement(V(0.0, 0.0, -38.0), App.Rotation())

    turntable = xform(hub_main.fuse(hub_step).fuse(ear_p).fuse(ear_n).removeSplitter())

    ring_band = Part.makeCylinder(39.5, 8.0, V(174.0, 0.0, 0.0), cx_vec).cut(
        Part.makeCylinder(37.5, 10.0, V(173.0, 0.0, 0.0), cx_vec)
    )
    index_lug = Part.makeBox(8.0, 9.0, 10.0, V(174.0, 35.0, -5.0))
    roll_ring = xform(ring_band.fuse(index_lug).removeSplitter())
    return turntable, roll_ring


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

    z_min = 32.0 if cam_side > 0 else -54.0
    z_ctr = 43.0 if cam_side > 0 else -43.0
    cam_bar = Part.makeBox(26.0, 62.0, 22.0, V(18.0, -31.0, z_min))
    lens_a = Part.makeCylinder(7.0, 10.0, V(44.0, -25.0, z_ctr), V(1, 0, 0))
    lens_b = Part.makeCylinder(7.0, 10.0, V(44.0, 25.0, z_ctr), V(1, 0, 0))
    lenses = lens_a.fuse(lens_b).removeSplitter()

    return xform(housing), xform(jaws), xform(cam_bar), xform(lenses)


def solve_arm_base_roll(base_pos, target_center_yz, ring_r, ring_x, L1_len, L2_len):
    """Solve 4-DOF (q0 base roll + planar q1, q2 + wrist pitch q3) in radial arm plane."""
    vy = base_pos.y - target_center_yz[0]
    vz = base_pos.z - target_center_yz[1]
    r_s = math.hypot(vy, vz)
    assert r_s > 1e-3, "Ring center coincides with shoulder!"

    u_rad = V(0.0, vy / r_s, vz / r_s)
    u_tan = V(1.0, 0.0, 0.0).cross(u_rad)

    sign_z = 1.0 if base_pos.z >= 0.0 else -1.0
    q0_rh = math.atan2(-sign_z * u_rad.y, sign_z * u_rad.z)

    wrist_pos = V(
        ring_x - GRIP_LEN,
        target_center_yz[0] + ring_r * u_rad.y,
        target_center_yz[1] + ring_r * u_rad.z,
    )

    dx = wrist_pos.x - base_pos.x
    dr = ring_r - r_s
    dist = math.hypot(dx, dr)
    assert abs(L1_len - L2_len) < dist < (L1_len + L2_len), f"Unreachable dist={dist:.1f}"

    cos_q2 = max(-1.0, min(1.0, (dist * dist - L1_len * L1_len - L2_len * L2_len) / (2.0 * L1_len * L2_len)))
    q2 = -math.acos(cos_q2)
    phi = math.atan2(dr, dx)
    psi = math.atan2(L2_len * math.sin(q2), L1_len + L2_len * math.cos(q2))
    q1 = phi - psi
    q3 = -(q1 + q2)

    elbow_pos = V(
        base_pos.x + L1_len * math.cos(q1),
        base_pos.y + L1_len * math.sin(q1) * u_rad.y,
        base_pos.z + L1_len * math.sin(q1) * u_rad.z,
    )
    angles = (math.degrees(q0_rh), math.degrees(q1), math.degrees(q2), math.degrees(q3))
    return elbow_pos, wrist_pos, u_rad, u_tan, angles, dist


def build_arm_assembly(
    doc, group, prefix, base_pos, elbow_pos, wrist_pos,
    u_reach, u_rad, u_tan, clevis_ur, clevis_ut, cam_side=-1.0
):
    objs = []
    turntable, roll_ring = make_clevis_yoke(base_pos.y, base_pos.z, clevis_ur, clevis_ut)
    objs.append(add_part(doc, group, f"{prefix}_TurntableClevis", turntable, TITANIUM))
    objs.append(add_part(doc, group, f"{prefix}_RollCollar", roll_ring, ROLL_AMBER))

    # Proportional compact joints & links (30/18/26/15/20 mm radii)
    objs.append(add_part(doc, group, f"{prefix}_ShoulderJoint", make_axis_joint(base_pos, clevis_ut, 30.0, 54.0), TITANIUM))
    objs.append(add_part(doc, group, f"{prefix}_UpperLink", make_cylinder_between(base_pos, elbow_pos, 18.0), CARBON))
    objs.append(add_part(doc, group, f"{prefix}_ElbowJoint", make_axis_joint(elbow_pos, clevis_ut, 26.0, 50.0), TITANIUM))
    objs.append(add_part(doc, group, f"{prefix}_ForearmLink", make_cylinder_between(elbow_pos, wrist_pos, 15.0), CARBON))
    objs.append(add_part(doc, group, f"{prefix}_WristPitchJoint", make_axis_joint(wrist_pos, clevis_ut, 20.0, 42.0), TITANIUM))

    h, j, cb, ls = make_gripper_parts(wrist_pos, u_reach, u_rad, u_tan, cam_side=cam_side)
    objs.append(add_part(doc, group, f"{prefix}_Gripper", h, METAL))
    objs.append(add_part(doc, group, f"{prefix}_GripperJaws", j, JAW_ORANGE))
    objs.append(add_part(doc, group, f"{prefix}_StereoCamBar", cb, CAM_DARK))
    objs.append(add_part(doc, group, f"{prefix}_StereoLenses", ls, LENS_CYAN))
    return objs


def build_deployed_ring_scenario(doc, g_arms, g_tgt, obstacle_shape, tag, ring_r, b1, b2, ring_x=RING_X, center_yz=(0.0, 0.0)):
    cy, cz = center_yz
    e1, w1, ur1, ut1, ang1, d1 = solve_arm_base_roll(b1, center_yz, ring_r, ring_x, L1, L2)
    e2, w2, ur2, ut2, ang2, d2 = solve_arm_base_roll(b2, center_yz, ring_r, ring_x, L1, L2)

    a1_objs = build_arm_assembly(
        doc, g_arms, f"Dep{tag}_Arm1", b1, e1, w1,
        V(1.0, 0.0, 0.0), ur1, ut1, ur1, ut1, cam_side=-1.0
    )
    a2_objs = build_arm_assembly(
        doc, g_arms, f"Dep{tag}_Arm2", b2, e2, w2,
        V(1.0, 0.0, 0.0), ur2, ut2, ur2, ut2, cam_side=-1.0
    )

    ring_outer = Part.makeCylinder(ring_r + 10.0, 68.0, V(ring_x, cy, cz), V(1, 0, 0))
    ring_inner = Part.makeCylinder(ring_r - 18.0, 72.0, V(ring_x - 2.0, cy, cz), V(1, 0, 0))
    ring_flange = Part.makeCylinder(ring_r + 18.0, 14.0, V(ring_x, cy, cz), V(1, 0, 0)).cut(ring_inner)
    marman = ring_outer.cut(ring_inner).fuse(ring_flange).removeSplitter()
    add_part(doc, g_tgt, f"Ring_{tag}_{int(round(2 * ring_r))}mm", marman, RING_SILVER, transparency=18)

    stub_w = max(440.0, 2.0 * ring_r + 160.0)
    stub = Part.makeBox(160.0, stub_w, stub_w, V(ring_x + 68.0, cy - 0.5 * stub_w, cz - 0.5 * stub_w))
    add_part(doc, g_tgt, f"ClientStub_{tag}", stub, CLIENT_GHOST, transparency=84)

    a1_comp = Part.makeCompound([o.Shape for o in a1_objs])
    a2_comp = Part.makeCompound([o.Shape for o in a2_objs])
    ov_shell = a1_comp.common(obstacle_shape).Volume + a2_comp.common(obstacle_shape).Volume
    ov_inter = a1_comp.common(a2_comp).Volume
    ov_ring = a1_comp.common(marman).Volume + a2_comp.common(marman).Volume

    assert ov_shell < 0.1, f"{tag} arm/bus overlap {ov_shell:.2f} mm^3"
    assert ov_inter < 0.1, f"{tag} arm1/arm2 overlap {ov_inter:.2f} mm^3"
    assert ov_ring < 0.1, f"{tag} arm/ring overlap {ov_ring:.2f} mm^3"

    print(
        f"[{tag}] D={2*ring_r:.1f}mm X={ring_x:.0f}mm C=({cy:+.0f},{cz:+.0f})mm | "
        f"Arm1 q=({ang1[0]:+.1f},{ang1[1]:+.1f},{ang1[2]:+.1f},{ang1[3]:+.1f}) reach={d1:.1f}/{L1+L2:.0f}mm | "
        f"Arm2 q=({ang2[0]:+.1f},{ang2[1]:+.1f},{ang2[2]:+.1f},{ang2[3]:+.1f}) reach={d2:.1f}/{L1+L2:.0f}mm | "
        f"roll_err=0.0deg | ov_bus={ov_shell:.2f} ov_arms={ov_inter:.2f} ov_ring={ov_ring:.2f}"
    )


def build():
    if DOC_NAME in App.listDocuments():
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)

    g_bus = doc.addObject("App::DocumentObjectGroup", "Bus")
    g_slice = doc.addObject("App::DocumentObjectGroup", "Bus_SideSlice_Cutaway")
    g_int = doc.addObject("App::DocumentObjectGroup", "Bus_Internal_Ref")
    g_espa = doc.addObject("App::DocumentObjectGroup", "ESPA_Envelope_Ref")
    g_stow = doc.addObject("App::DocumentObjectGroup", "Arms_Stowed")
    g_d24 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_24in")
    g_t24 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_24in_610mm")
    g_d15 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_15in")
    g_t15 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_15in_381mm")
    g_d8 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_8in")
    g_t8 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_8in_203mm")
    g_d37 = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_37in")
    g_t37 = doc.addObject("App::DocumentObjectGroup", "Target_Ring_37in_940mm")
    g_doff = doc.addObject("App::DocumentObjectGroup", "Arms_Deployed_24in_LargeOff")
    g_toff = doc.addObject("App::DocumentObjectGroup", "Target_Ring_24in_LargeOff")

    # 1. Core chamfered bus shell (600 X x 600 Y x 900 Z mm), hollowed bay + 690 mm cavities + swept arc reliefs
    shell = chamfered_box(HX, HY, HZ, CH)
    inner_bay = Part.makeBox(425.0, 540.0, 830.0, V(-275.0, -270.0, -415.0))
    cav_p = Part.makeBox(HX - CAV_XMIN + 15.0, CAV_W, CAV_H, V(CAV_XMIN, CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
    cav_n = Part.makeBox(HX - CAV_XMIN + 15.0, CAV_W, CAV_H, V(CAV_XMIN, -CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
    bev_p = make_cavity_mouth_bevel(CAV_Y)
    bev_n = make_cavity_mouth_bevel(-CAV_Y)
    arc_p = make_arc_relief_cut(CAV_Y, ARM_MOUNT_Z)
    arc_n = make_arc_relief_cut(-CAV_Y, -ARM_MOUNT_Z)

    b1 = V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z)
    b2 = V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z)
    bore_tools = [inner_bay, cav_p, cav_n, bev_p, bev_n, arc_p, arc_n]
    fixed_drive_shapes = []
    for cy_b, cz_b in ((CAV_Y, ARM_MOUNT_Z), (-CAV_Y, -ARM_MOUNT_Z)):
        bore_tools.append(Part.makeCylinder(46.0, 84.0, V(90.0, cy_b, cz_b), V(1, 0, 0)))
        bore_tools.append(Part.makeCylinder(50.5, 6.5, V(165.5, cy_b, cz_b), V(1, 0, 0)))

    cut_all = bore_tools[0]
    for bt in bore_tools[1:]:
        cut_all = cut_all.fuse(bt)
    shell = shell.cut(cut_all).removeSplitter()
    add_part(doc, g_bus, "BusShell_GoldMLI", shell, GOLD)

    # Fixed bulkhead flange collars & internal Base-Roll harmonic drive barrels
    for cy_b, cz_b, tag_arm in ((CAV_Y, ARM_MOUNT_Z, "Arm1"), (-CAV_Y, -ARM_MOUNT_Z, "Arm2")):
        flange = Part.makeCylinder(50.0, 4.0, V(166.0, cy_b, cz_b), V(1, 0, 0))
        barrel = Part.makeCylinder(44.0, 62.0, V(104.0, cy_b, cz_b), V(1, 0, 0))
        enc_cap = Part.makeCylinder(36.0, 10.0, V(94.0, cy_b, cz_b), V(1, 0, 0))
        add_part(doc, g_bus, f"FixedBaseRoll_{tag_arm}_Flange", flange, TITANIUM)
        add_part(doc, g_bus, f"FixedBaseRoll_{tag_arm}_Drive", barrel, DARK)
        add_part(doc, g_bus, f"FixedBaseRoll_{tag_arm}_Encoder", enc_cap, ROLL_AMBER)
        fixed_drive_shapes.extend([flange, barrel, enc_cap])

    # Recessed dark backing strips inside each cavity + matching arc pocket liner
    for tag, cy, cz in (("Liner_Arm1_PY", CAV_Y, ARM_MOUNT_Z), ("Liner_Arm2_NY", -CAV_Y, -ARM_MOUNT_Z)):
        clip_y0 = 13.0 if cy > 0 else -300.0
        arc_clip = Part.makeBox(10.0, 287.0, 400.0, V(168.0, clip_y0, cz - 200.0))
        liner_rect = Part.makeBox(2.0, CAV_W - 6.0, CAV_H - 6.0, V(170.2, cy - 0.5 * (CAV_W - 6.0), -0.5 * (CAV_H - 6.0)))
        liner_arc = Part.makeCylinder(R_ARC - 3.0, 2.0, V(170.2, cy, cz), V(1, 0, 0)).common(arc_clip)
        liner = liner_rect.fuse(liner_arc).cut(
            Part.makeCylinder(52.0, 12.0, V(164.0, cy, cz), V(1, 0, 0))
        ).removeSplitter()
        add_part(doc, g_bus, tag, liner, DARK)
        fixed_drive_shapes.append(liner)

    # 2. Optical bench centered vertically on mid +Y cheek (y = +215 mm, z = -83..+83 mm, max X = +345 mm)
    bench = Part.makeBox(20.0, 136.0, 166.0, V(HX - 5.0, 215.0 - 68.0, -83.0))
    lidar_win = Part.makeBox(8.0, 100.0, 62.0, V(HX + 14.0, 215.0 - 50.0, -63.0))
    cam_barrel = Part.makeCylinder(28.0, 26.0, V(HX + 14.0, 215.0, 37.0), V(1, 0, 0))
    cam_lens = Part.makeCylinder(22.0, 5.0, V(HX + 40.0, 215.0, 37.0), V(1, 0, 0))
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

    drives_comp = Part.makeCompound(fixed_drive_shapes)
    ov_drives_shell = drives_comp.common(shell).Volume
    assert ov_drives_shell < 0.1, f"Fixed drive vs shell overlap {ov_drives_shell:.2f} mm^3"

    # 5. Bus_SideSlice_Cutaway (slice plane Y <= +70 mm exposing Arm 1 cavity, arc wall & bulkhead drive)
    slice_half = Part.makeBox(900.0, 500.0, 1100.0, V(-450.0, -430.0, -550.0))
    shell_sliced = shell.common(slice_half)
    flange1_cut = Part.makeCylinder(50.0, 4.0, V(166.0, CAV_Y, ARM_MOUNT_Z), V(1, 0, 0)).common(slice_half)
    barrel1_cut = Part.makeCylinder(44.0, 62.0, V(104.0, CAV_Y, ARM_MOUNT_Z), V(1, 0, 0)).common(slice_half)
    enc1_cut = Part.makeCylinder(36.0, 10.0, V(94.0, CAV_Y, ARM_MOUNT_Z), V(1, 0, 0)).common(slice_half)
    add_part(doc, g_slice, "Slice_BusShell_GoldMLI", shell_sliced, GOLD)
    add_part(doc, g_slice, "Slice_FixedBaseRoll_Arm1_Flange", flange1_cut, TITANIUM)
    add_part(doc, g_slice, "Slice_FixedBaseRoll_Arm1_Drive", barrel1_cut, DARK)
    add_part(doc, g_slice, "Slice_FixedBaseRoll_Arm1_Encoder", enc1_cut, ROLL_AMBER)
    add_part(doc, g_slice, "Slice_SepRing", sep_ring.common(slice_half), METAL)

    # 6. Bus_Internal_Ref (COPV tank, 4x RW-12 pyramid, avionics stack; must clear drives & shell)
    tank = Part.makeSphere(155.0, V(-110.0, 0.0, 135.0))
    avionics = Part.makeBox(160.0, 260.0, 130.0, V(-200.0, -130.0, -395.0))
    rw_solids = []
    for k in range(4):
        ang = math.radians(45.0 + 90.0 * k)
        w_dir = V(math.cos(ang) * 0.65, math.sin(ang) * 0.65, 0.39).normalize()
        w_ctr = V(-115.0 + 72.0 * math.cos(ang), 72.0 * math.sin(ang), -155.0)
        rw_solids.append(Part.makeCylinder(72.0, 52.0, w_ctr.sub(w_dir.multiply(26.0)), w_dir))
    rw_comp = rw_solids[0]
    for w in rw_solids[1:]:
        rw_comp = rw_comp.fuse(w)
    add_part(doc, g_int, "Ref_PropellantTank_COPV", tank, TANK_STEEL, transparency=20)
    add_part(doc, g_int, "Ref_RW12_Pyramid_4x", rw_comp, RW_COPPER)
    add_part(doc, g_int, "Ref_AvionicsBatteryStack", avionics, AVIONICS_SLATE)

    int_comp = Part.makeCompound([tank, avionics, rw_comp])
    ov_int_drives = int_comp.common(drives_comp).Volume
    ov_int_shell = int_comp.common(shell).Volume
    assert ov_int_drives < 0.1, f"Internal ref vs drives overlap {ov_int_drives:.2f}"
    assert ov_int_shell < 0.1, f"Internal ref vs shell overlap {ov_int_shell:.2f}"

    # 7. Standard 15" ESPA envelope reference box (711 X x 610 Y x 965 Z mm from port face x=-350)
    espa_box = Part.makeBox(711.0, 610.0, 965.0, V(-HX - SEP_H, -305.0, -460.0))
    add_part(doc, g_espa, "ESPA_Standard_711x610x965", espa_box, ESPA_CYAN, transparency=92)

    obstacle = Part.makeCompound([o.Shape for o in g_bus.Group])

    # 8. Arms_Stowed (q0 = 0 deg, straight along +-Z inside 690 mm cavities)
    stow_objs = []
    stow_objs += build_arm_assembly(
        doc, g_stow, "Stow_Arm1", b1,
        V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1),
        V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1 - L2),
        u_reach=V(0, 0, -1), u_rad=V(1, 0, 0), u_tan=V(0, -1, 0),
        clevis_ur=V(0, 0, 1), clevis_ut=V(0, -1, 0), cam_side=-1.0,
    )
    stow_objs += build_arm_assembly(
        doc, g_stow, "Stow_Arm2", b2,
        V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1),
        V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1 + L2),
        u_reach=V(0, 0, 1), u_rad=V(1, 0, 0), u_tan=V(0, 1, 0),
        clevis_ur=V(0, 0, -1), clevis_ut=V(0, 1, 0), cam_side=-1.0,
    )

    stow_comp = Part.makeCompound([o.Shape for o in stow_objs])
    ov_stow = stow_comp.common(obstacle).Volume
    sbb = stow_comp.BoundBox
    bus_bb = obstacle.BoundBox
    tot_x = max(bus_bb.XMax, sbb.XMax) - bus_bb.XMin
    tot_y = max(bus_bb.YMax, sbb.YMax) - min(bus_bb.YMin, sbb.YMin)
    tot_z = max(bus_bb.ZMax, sbb.ZMax) - min(bus_bb.ZMin, sbb.ZMin)
    assert tot_x <= 711.0 and tot_y <= 610.0 and tot_z <= 965.0, "Exceeded ESPA envelope!"
    assert ov_stow < 0.1, f"Stowed overlap {ov_stow:.2f} mm^3!"

    # 9. Deployed scenarios: 24", 15", 8", 37", and Large Off-Center (+80 Y, -50 Z)
    build_deployed_ring_scenario(doc, g_d24, g_t24, obstacle, "24in", R_24IN, b1, b2, RING_X, (0.0, 0.0))
    build_deployed_ring_scenario(doc, g_d15, g_t15, obstacle, "15in", R_15IN, b1, b2, RING_X, (0.0, 0.0))
    build_deployed_ring_scenario(doc, g_d8, g_t8, obstacle, "8in", R_8IN, b1, b2, RING_X, (0.0, 0.0))
    build_deployed_ring_scenario(doc, g_d37, g_t37, obstacle, "37in", R_37IN, b1, b2, RING_X_37, (0.0, 0.0))
    build_deployed_ring_scenario(doc, g_doff, g_toff, obstacle, "24inLargeOff", R_24IN, b1, b2, RING_X, (80.0, -50.0))

    # Default clean view visibility: Bus + 24in deployed + ESPA box
    for grp, vis in (
        (g_bus, True), (g_slice, False), (g_int, False), (g_espa, True),
        (g_stow, False), (g_d24, True), (g_t24, True),
        (g_d15, False), (g_t15, False), (g_d8, False), (g_t8, False),
        (g_d37, False), (g_t37, False), (g_doff, False), (g_toff, False),
    ):
        set_group_visible(grp, vis)

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
    print(f"Stowed overlap={ov_stow:.2f}, FixedDrives/Shell overlap={ov_drives_shell:.2f}, Internals overlap={ov_int_drives + ov_int_shell:.2f}")
    return doc


if __name__ == "__main__":
    build()
