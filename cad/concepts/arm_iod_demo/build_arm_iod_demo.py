"""FreeCAD concept: Ultra-Shallow Dual-Arm Hosted-Payload IOD Demo.

Demonstrates the v4.2 6-DOF Cardan-Roll dual-arm capture hardware on a stripped-down,
ultra-shallow hosted-payload deck (no propulsion, tanks, RCS, RW pyramid, or bus LiDAR/IRIS):
  - Core Envelope (stowed): ~273 X x 520 Y x 840 Z mm (55% thinner than 704 mm ESPA bus)
  - 0..10 mm   : Host mounting baseplate w/ perimeter bolt lugs
  - 10..90 mm  : Rear deployer & avionics bay:
                  * 2x independent boom drive motors + shafts (Upper Z=+155, Lower Z=-155 mm)
                  * 4x Ø70 mm CFRP boom spool drums at (Y=+-210, Z=+-155 mm)
                  * Central avionics / OBC / 14x motor driver stack
                  * Aft q0 Base-Roll fixed drives at (Y=+-70, Z=+-245 mm)
  - 90..110 mm : 20 mm structural bulkhead (q0 bores + 4x boom pass-throughs)
  - 110..245 mm: 135 mm recessed v4.2 arm cavities (arms flush inside X <= 244 mm),
                 4x rigid boom guide chutes (hiding 100-120 mm transition zone),
                 annular 15" nesting groove (X=222..245), tiny centerline witness cam
  - 245..273 mm: Stowed 15" (Ø381 mm) dummy target ring plate on 4x perimeter HDRM standoffs
  - Deployed   : 4x Ø20 mm rollable CFRP slit-tube booms deploy ring to +480 mm standoff
                 (independent upper/lower motors allow +-5 deg pitch tilt testing!)

Toggleable FreeCAD groups:
  - Chassis_Shell
  - RearBay_Internals
  - Chassis_Cutaway
  - Stowed_Config
  - Deployed_Nominal_15in
  - Deployed_OpenApproach_15in
  - Deployed_Tilted_5deg_Pitch
"""
import glob
import math
import os
import sys

import FreeCAD as App
import Part

V = App.Vector
HERE = (
    os.path.dirname(os.path.abspath(__file__))
    if "__file__" in globals()
    else os.getcwd()
)
ROOT_CONCEPTS = os.path.dirname(HERE)
ESPA_DIR = os.path.join(ROOT_CONCEPTS, "dual_arm_espa")
if ESPA_DIR not in sys.path:
    sys.path.insert(0, ESPA_DIR)

import build_dual_arm_concept as v4  # noqa: E402

DOC_NAME = "ArmIODDemo"

# Shallow Hosted-Payload Frame: X = 0 at rear host interface -> +X toward target plate
CHASSIS_LEN = 245.0        # X = 0..245 mm chassis block
REAR_PLATE_THK = 10.0      # X = 0..10 mm host mounting deck
BULKHEAD_X0 = 90.0         # X = 90..110 mm 20 mm structural bulkhead
CAV_XMIN = 110.0           # X = 110..245 mm (135 mm recessed cavity depth, identical to v4.2)
ARM_MOUNT_X = 190.0        # shifted -75 mm from v4.2's 265 mm

HY = 260.0                 # 520 mm Y width (-260..+260)
HZ = 420.0                 # 840 mm Z height (-420..+420)
CH = 25.0                  # chamfer on X-parallel edges

CAV_W = v4.CAV_W           # 120.0 mm
CAV_H = v4.CAV_H           # 760.0 mm (-380..+380)
CAV_Y = v4.CAV_Y           # +-70.0 mm
R_ARC = v4.R_ARC           # 115.0 mm
ARM_MOUNT_Z = v4.ARM_MOUNT_Z  # +-245.0 mm
L1 = v4.L1                 # 248.0 mm
L2 = v4.L2                 # 208.0 mm
PALM_X = v4.PALM_X         # 128.0 mm

# Deployable 15" (381 mm) Dummy Target Ring Plate & 4x Rollable CFRP Booms
R_15IN = v4.R_15IN         # 190.5 mm
PLATE_W = 480.0            # Y width
PLATE_H = 440.0            # Z height
PLATE_THK = 8.0            # honeycomb backing plate thickness
RING_H_TOTAL = 42.0        # 14 mm lip + 28 mm skirt (clears 38 mm gripper fingers)
STANDOFF_X = 480.0         # +480 mm nominal standoff from chassis front X=245 -> X_ring = 725 mm
RING_X_DEPLOYED = CHASSIS_LEN + STANDOFF_X  # 725.0 mm

BOOM_Y = 210.0             # +-210 mm
BOOM_Z = 155.0             # +-155 mm guide-chute & socket height
BOOM_DRUM_Z = 123.0        # +-123.0 mm shaft axis = BOOM_Z - DRUM_R_HUB (tangent tape peel -> Z=+-155!)
BOOM_R = 10.0              # Ø20 mm deployed rollable CFRP slit tube (0.24 mm wall)
DRUM_X = 50.0              # centered inside X=10..90 mm rear bay
DRUM_R_HUB = 32.0          # Ø64 mm hub (t/D = 0.34% <= 0.4%)
DRUM_R_FLANGE = 35.0       # Ø70 mm outer flange (X in [15, 85], +5 mm clearance each side)
DRUM_LEN = 64.0            # Y width of drum assembly

# Stowed Dummy Plate & Ring placement (20 mm standoff off X=245 front face -> plate X=265..273)
STOW_PLATE_X0 = 265.0      # plate outer X_max = 273.0 mm
STOW_RING_X = STOW_PLATE_X0 - RING_H_TOTAL  # 223.0 mm (nests 22 mm into annular groove X=221.5..246)

# Local Palette additions
BOOM_CFRP = (0.14, 0.15, 0.18)
PLATE_AL = (0.76, 0.78, 0.82)
GOLD = v4.GOLD
DARK = v4.DARK
TITANIUM = v4.TITANIUM
METAL = v4.METAL
ROLL_AMBER = v4.ROLL_AMBER
AVIONICS_SLATE = v4.AVIONICS_SLATE
HDRM_RED = v4.HDRM_RED
LENS_CYAN = v4.LENS_CYAN
CAM_DARK = v4.CAM_DARK
RING_SILVER = v4.RING_SILVER
FLEX_TEAL = v4.FLEX_TEAL

# Sync v4 helpers to shallow X coordinates (restored at end of build)
_ORIG_CAV_XMIN = v4.CAV_XMIN
_ORIG_ARM_MOUNT_X = v4.ARM_MOUNT_X


def _sync_v4_coords(cav_xmin, arm_mount_x):
    v4.CAV_XMIN = cav_xmin
    v4.ARM_MOUNT_X = arm_mount_x


def make_shallow_cavity_bevel(cy):
    """Cavity mouth bevel keeping 14 mm center spine clean for witness cam and outer cheeks clean for booms."""
    is_pos = cy > 0.0
    dy_p0 = cy + 0.5 * CAV_W
    dy_n0 = cy - 0.5 * CAV_W
    dy_p1 = dy_p0 + (12.0 if is_pos else 3.5)
    dy_n1 = dy_n0 - (3.5 if is_pos else 12.0)
    hz0 = 0.5 * CAV_H
    hz1 = hz0 + 12.0
    x0, x1 = CHASSIS_LEN - 10.0, CHASSIS_LEN + 4.0
    w0 = Part.makePolygon([
        V(x0, dy_n0, -hz0), V(x0, dy_p0, -hz0),
        V(x0, dy_p0, hz0), V(x0, dy_n0, hz0), V(x0, dy_n0, -hz0),
    ])
    w1 = Part.makePolygon([
        V(x1, dy_n1, -hz1), V(x1, dy_p1, -hz1),
        V(x1, dy_p1, hz1), V(x1, dy_n1, hz1), V(x1, dy_n1, -hz1),
    ])
    return Part.makeLoft([w0, w1], True)


def make_shallow_arc_relief(cy_s, cz_s):
    """Circular arc relief around shoulder inside X = CAV_XMIN..CHASSIS_LEN, keeping |Y| >= 10."""
    is_pos = cy_s > 0.0
    clip_y0 = 10.0 if is_pos else -HY
    clip_box = Part.makeBox(170.0, HY - 10.0, 360.0, V(CAV_XMIN - 5.0, clip_y0, cz_s - 180.0))
    cyl = Part.makeCylinder(R_ARC, CHASSIS_LEN - CAV_XMIN + 15.0, V(CAV_XMIN, cy_s, cz_s), V(1, 0, 0))
    cone = Part.makeCone(R_ARC, R_ARC + 12.0, 14.0, V(CHASSIS_LEN - 10.0, cy_s, cz_s), V(1, 0, 0))
    return cyl.fuse(cone).common(clip_box)


def make_target_plate_and_ring(ring_center, plate_rot=None):
    """Build 15 in Marman target ring + 480x440x8 mm plate + 4x spherical tip sockets.

    Local frame before placement:
      ring_lip_front at x_loc = 0 .. 14 mm (r in [172.5, 208.5])
      ring_skirt     at x_loc = 14 .. 42 mm (r in [172.5, 200.5])
      plate          at x_loc = 42 .. 50 mm (Y in [-240,+240], Z in [-220,+220])
      4x boom cups   at x_loc = 36 .. 42 mm, (Y, Z) = (+-210, +-155)
    """
    if plate_rot is None:
        plate_rot = App.Rotation()
    T = App.Placement(ring_center, plate_rot)

    def xp(sh):
        s = sh.copy()
        s.Placement = T.multiply(s.Placement)
        return s

    ring_lip = Part.makeCylinder(R_15IN + 18.0, 14.0, V(0.0, 0.0, 0.0), V(1, 0, 0)).cut(
        Part.makeCylinder(R_15IN - 18.0, 16.0, V(-1.0, 0.0, 0.0), V(1, 0, 0))
    )
    ring_skirt = Part.makeCylinder(R_15IN + 10.0, RING_H_TOTAL - 14.0, V(14.0, 0.0, 0.0), V(1, 0, 0)).cut(
        Part.makeCylinder(R_15IN - 18.0, RING_H_TOTAL - 10.0, V(12.0, 0.0, 0.0), V(1, 0, 0))
    )
    marman = ring_lip.fuse(ring_skirt).removeSplitter()

    plate_box = Part.makeBox(
        PLATE_THK, PLATE_W, PLATE_H, V(RING_H_TOTAL, -0.5 * PLATE_W, -0.5 * PLATE_H)
    )
    # Four corner HDRM/socket lugs + boom tip receivers on -X side of plate
    sockets = []
    for sy in (1.0, -1.0):
        for sz in (1.0, -1.0):
            sockets.append(
                Part.makeCylinder(15.0, 8.0, V(RING_H_TOTAL - 8.0, sy * BOOM_Y, sz * BOOM_Z), V(1, 0, 0))
            )
    plate_full = plate_box
    for sc in sockets:
        plate_full = plate_full.fuse(sc)
    plate_full = plate_full.removeSplitter()

    # Optical crosshair fiducial frame around inner aperture
    fid = Part.makeCylinder(72.0, 2.0, V(RING_H_TOTAL - 2.0, 0.0, 0.0), V(1, 0, 0)).cut(
        Part.makeCylinder(58.0, 6.0, V(RING_H_TOTAL - 4.0, 0.0, 0.0), V(1, 0, 0))
    )
    return xp(marman), xp(plate_full), xp(fid)


def make_boom_tube(p_start, p_end, radius=BOOM_R, wall=1.2):
    """Create thin-walled slit-tube boom cylinder between two 3D points."""
    vec = p_end.sub(p_start)
    length = vec.Length
    d = vec.normalize()
    outer = Part.makeCylinder(radius, length, p_start, d)
    inner = Part.makeCylinder(radius - wall, length + 2.0, p_start.sub(d), d)
    # Longitudinal STEM slit facing radially outward in Z for visual authenticity
    up = V(0.0, 1.0, 0.0) if abs(d.y) < 0.8 else V(0.0, 0.0, 1.0)
    side = d.cross(up).normalize()
    slit = Part.makeBox(2.0, radius + 4.0, length + 4.0, V(-1.0, 0.0, -2.0))
    slit_rot = App.Rotation(side, d.cross(side).normalize(), d, "XYZ")
    slit.Placement = App.Placement(p_start, slit_rot)
    return outer.cut(inner).cut(slit).removeSplitter()


def solve_arm_tilted_ring(base_pos, ring_center, ring_rot, ring_r, palm_x=PALM_X):
    """Exact 6-DOF inverse kinematics onto a 3D-tilted 15 in Marman ring.

    Chooses the contact point on the tilted ring where the palm radial vector matches the
    arm's planar shoulder-to-wrist roll orientation (so q0 aligns the elbow plane with wrist).
    Computes exact Cardan-Roll wrist axes (u_reach, u_rad, u_tan) and elbow/wrist positions.
    """
    n_reach = ring_rot.multVec(V(1.0, 0.0, 0.0)).normalize()
    e1 = ring_rot.multVec(V(0.0, 1.0, 0.0)).normalize()
    e2 = ring_rot.multVec(V(0.0, 0.0, 1.0)).normalize()

    # Initial planar direction from ring center toward shoulder base (in ring plane)
    to_b = base_pos.sub(ring_center)
    proj_y = to_b.dot(e1)
    proj_z = to_b.dot(e2)
    theta = math.atan2(proj_z, proj_y)

    # Refine theta 4 iterations so wrist_pos lies strictly in the shoulder's q0 roll plane around +X
    for _ in range(4):
        u_rad = (e1 * math.cos(theta) + e2 * math.sin(theta)).normalize()
        p_ring = ring_center + u_rad * ring_r
        wrist_pos = p_ring - n_reach * palm_x
        # Shoulder->wrist YZ projection vs u_rad YZ projection align when cross_x == 0
        wy = wrist_pos.y - base_pos.y
        wz = wrist_pos.z - base_pos.z
        # We want the arm plane passing through base_pos & wrist_pos to stay near shoulder's natural meridian
        ang_w = math.atan2(wz - (-base_pos.z) * 0.0, wy)
        ang_sh = math.atan2(base_pos.z - ring_center.z, base_pos.y - ring_center.y)
        # Solve angle so YZ of p_ring aligns with YZ meridian through base_pos & ring_center
        u_mer_y = base_pos.y - ring_center.y
        u_mer_z = base_pos.z - ring_center.z
        r_mer = math.hypot(u_mer_y, u_mer_z)
        uy_tgt, uz_tgt = u_mer_y / r_mer, u_mer_z / r_mer
        # Target YZ radial direction from ring_center is (uy_tgt, uz_tgt) after removing palm_x * n_reach YZ shift
        res_y = ring_r * uy_tgt - (wrist_pos.y - p_ring.y)
        res_z = ring_r * uz_tgt - (wrist_pos.z - p_ring.z)
        # Keep ring contact on exact meridian from ring_center to shoulder YZ (gives zero tangential skew)
        theta_num = math.atan2(uz_tgt * e1.y - uy_tgt * e1.z, uy_tgt * e2.z - uz_tgt * e2.y)
        break

    # Direct exact meridian intersection: find u_rad in span(e1, e2) whose YZ aligns with shoulder meridian
    u_mer = V(0.0, base_pos.y - ring_center.y, base_pos.z - ring_center.z).normalize()
    # u_rad = a*e1 + b*e2 with (a*e1.y + b*e2.y, a*e1.z + b*e2.z) proportional to (u_mer.y, u_mer.z)
    # Cross product in YZ = 0 -> (e1 x u_mer)_x * a + (e2 x u_mer)_x * b = 0
    c1 = e1.y * u_mer.z - e1.z * u_mer.y
    c2 = e2.y * u_mer.z - e2.z * u_mer.y
    a = -c2
    b = c1
    if (a * e1 + b * e2).dot(u_mer) < 0.0:
        a, b = -a, -b
    u_rad = (e1 * a + e2 * b).normalize()
    u_tan = n_reach.cross(u_rad).normalize()

    p_ring = ring_center + u_rad * ring_r
    wrist_pos = p_ring - n_reach * palm_x

    # Arm planar kinematics in plane containing base_pos, wrist_pos, and +X direction
    arm_x = V(1.0, 0.0, 0.0)
    w_vec = wrist_pos.sub(base_pos)
    dx = w_vec.x
    v_yz = V(0.0, w_vec.y, w_vec.z)
    clevis_ur = v_yz.normalize()
    # Notice shoulder radially points AWAY from ring center while wrist lies inward of base_pos:
    # dr = (wrist - base) along outward u_rad_plane
    if clevis_ur.dot(u_mer) < 0.0:
        clevis_ur = clevis_ur * -1.0
    clevis_ut = arm_x.cross(clevis_ur).normalize()
    dr = v_yz.dot(clevis_ur)

    dist = math.hypot(dx, dr)
    assert abs(L1 - L2) < dist < (L1 + L2), f"Tilted ring unreachable dist={dist:.1f}"

    cos_q2 = max(-1.0, min(1.0, (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)))
    q2 = -math.acos(cos_q2)
    phi = math.atan2(dr, dx)
    psi = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q1 = phi - psi

    elbow_pos = base_pos + arm_x * (L1 * math.cos(q1)) + clevis_ur * (L1 * math.sin(q1))
    sign_z = 1.0 if base_pos.z >= 0.0 else -1.0
    q0_deg = math.degrees(math.atan2(-sign_z * clevis_ur.y, sign_z * clevis_ur.z))
    q3_deg = math.degrees(-(q1 + q2))  # nominal pitch component
    return elbow_pos, wrist_pos, n_reach, u_rad, u_tan, clevis_ur, clevis_ut, (q0_deg, math.degrees(q1), math.degrees(q2), q3_deg), dist


def make_clevis_yoke_shallow(cy, cz, u_rad, u_tan):
    """Build rotating turntable hub (X=CAV_XMIN+1..+33), amber q0 collar, and straddle clevis."""
    cx_vec = V(1.0, 0.0, 0.0)
    R = App.Rotation(cx_vec, u_rad, u_tan, "XYZ")
    T = App.Placement(V(0.0, cy, cz), R)

    def xform(sh):
        s = sh.copy()
        s.Placement = T.multiply(s.Placement)
        return s

    x_ear0 = CAV_XMIN + 30.0  # 140.0 mm
    hub_main = Part.makeCylinder(38.0, 22.0, V(CAV_XMIN + 1.0, 0.0, 0.0), cx_vec)
    hub_step = Part.makeCylinder(38.0, 10.0, V(CAV_XMIN + 23.0, 0.0, 0.0), cx_vec)

    ear_p_strut = Part.makeBox(ARM_MOUNT_X - x_ear0, 42.0, 8.5, V(x_ear0, -21.0, 23.5))
    ear_p_ring = Part.makeCylinder(41.5, 8.5, V(ARM_MOUNT_X, 0.0, 23.5), V(0.0, 0.0, 1.0))
    ear_p_bore = Part.makeCylinder(37.2, 12.0, V(ARM_MOUNT_X, 0.0, 22.0), V(0.0, 0.0, 1.0))
    ear_p = ear_p_strut.fuse(ear_p_ring).cut(ear_p_bore).removeSplitter()

    ear_n_strut = Part.makeBox(ARM_MOUNT_X - x_ear0, 42.0, 8.5, V(x_ear0, -21.0, -32.0))
    ear_n_boss = Part.makeCylinder(26.0, 8.5, V(ARM_MOUNT_X, 0.0, -32.0), V(0.0, 0.0, 1.0))
    ear_n_cap = Part.makeCylinder(18.0, 2.5, V(ARM_MOUNT_X, 0.0, -34.5), V(0.0, 0.0, 1.0))
    ear_n = ear_n_strut.fuse(ear_n_boss).fuse(ear_n_cap).removeSplitter()

    turntable = xform(hub_main.fuse(hub_step).fuse(ear_p).fuse(ear_n).removeSplitter())

    ring_band = Part.makeCylinder(39.5, 8.0, V(CAV_XMIN + 4.0, 0.0, 0.0), cx_vec).cut(
        Part.makeCylinder(37.5, 10.0, V(CAV_XMIN + 3.0, 0.0, 0.0), cx_vec)
    )
    index_lug = Part.makeBox(8.0, 9.0, 10.0, V(CAV_XMIN + 4.0, 35.0, -5.0))
    roll_ring = xform(ring_band.fuse(index_lug).removeSplitter())
    return turntable, roll_ring


def build_arm_iod(
    doc, group, prefix, base_pos, elbow_pos, wrist_pos,
    u_reach, u_rad, u_tan, clevis_ur, clevis_ut, cam_side=-1.0, in_open=0.0, hook_open=0.0
):
    """Build v4.2 arm using shallow CAV_XMIN/ARM_MOUNT_X coordinates and true clevis/Cardan pitch alignment."""
    objs = []
    turntable, roll_ring = make_clevis_yoke_shallow(base_pos.y, base_pos.z, clevis_ur, clevis_ut)
    objs.append(v4.add_part(doc, group, f"{prefix}_TurntableClevis", turntable, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_RollCollar", roll_ring, ROLL_AMBER))

    upper_dir = elbow_pos.sub(base_pos).normalize()
    forearm_dir = wrist_pos.sub(elbow_pos).normalize()

    sh_st, sh_col, sh_out = v4.make_shoulder_joint(base_pos, upper_dir, clevis_ur, clevis_ut)
    ov_tt = turntable.common(sh_st).Volume + turntable.common(sh_col).Volume + turntable.common(sh_out).Volume
    assert ov_tt < 0.1, f"{prefix} clevis vs shoulder overlap {ov_tt:.2f} mm^3"
    objs.append(v4.add_part(doc, group, f"{prefix}_ShoulderStator", sh_st, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_ShoulderRollCollar", sh_col, ROLL_AMBER))
    objs.append(v4.add_part(doc, group, f"{prefix}_ShoulderOutput", sh_out, METAL))

    p_up0 = base_pos + upper_dir * 54.2
    p_up1 = elbow_pos - upper_dir * 52.2
    objs.append(v4.add_part(doc, group, f"{prefix}_UpperLink", v4.make_cylinder_between(p_up0, p_up1, 18.0), v4.CARBON))

    el_clv, el_st, el_col, el_out = v4.make_elbow_joint(elbow_pos, upper_dir, forearm_dir, clevis_ut)
    objs.append(v4.add_part(doc, group, f"{prefix}_ElbowClevis", el_clv, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_ElbowStator", el_st, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_ElbowRollCollar", el_col, ROLL_AMBER))
    objs.append(v4.add_part(doc, group, f"{prefix}_ElbowOutput", el_out, METAL))

    p_fore0 = elbow_pos + forearm_dir * 48.2
    p_fore_end = wrist_pos - forearm_dir * 34.2
    objs.append(v4.add_part(doc, group, f"{prefix}_ForearmLink", v4.make_cylinder_between(p_fore0, p_fore_end, 15.0), v4.CARBON))

    # Align q3 stator drum along true pitch axis clevis_ut so Cardan-Roll wrist handles 3D yaw/pitch tilts
    R_w = App.Rotation(u_reach, u_rad, u_tan, "XYZ")
    T_w = App.Placement(wrist_pos, R_w)
    R_f = App.Rotation(forearm_dir, clevis_ur, clevis_ut, "XYZ")
    T_f = App.Placement(wrist_pos, R_f)

    def xw(sh):
        s = sh.copy()
        s.Placement = T_w.multiply(s.Placement)
        return s

    def xf(sh):
        s = sh.copy()
        s.Placement = T_f.multiply(s.Placement)
        return s

    clevis_base = Part.makeBox(5.4, 28.0, 56.0, V(-34.0, -14.0, -24.0))
    ear_arm = Part.makeBox(30.0, 28.0, 5.8, V(-30.0, -14.0, -24.0))
    ear_cap = Part.makeCylinder(14.0, 5.8, V(0.0, 0.0, -24.0), V(0.0, 0.0, 1.0))
    fclv = xf(clevis_base.fuse(ear_arm).fuse(ear_cap).removeSplitter())

    # True spherical Cardan-Roll frame T_yoke: Z = clevis_ut (q3 axis), Y = e_q4 (orthogonal to q3 & q5), X = Y x Z
    q3_ut = clevis_ut
    e_q4 = q3_ut.cross(u_reach).normalize()
    yoke_ux = e_q4.cross(q3_ut).normalize()
    T_yoke = App.Placement(wrist_pos, App.Rotation(yoke_ux, e_q4, q3_ut, "XYZ"))

    def xy(sh):
        s = sh.copy()
        s.Placement = T_yoke.multiply(s.Placement)
        return s

    q3d = xy(Part.makeCylinder(28.5, 38.0, V(0.0, 0.0, 18.2), V(0.0, 0.0, 1.0)))
    q3_pin = xy(Part.makeCylinder(10.0, 35.0, V(0.0, 0.0, -17.5), V(0.0, 0.0, 1.0)))
    spider = xy(Part.makeBox(26.0, 39.5, 34.0, V(-13.0, -29.5, -17.0)))
    cyoke = q3_pin.fuse(spider).removeSplitter()

    # q4 yaw barrel around e_q4 (y in [-68, -30]) + bridge rotated around e_q4 into u_reach (q5 roll axis)
    q4_cyl = xy(Part.makeCylinder(28.5, 38.0, V(0.0, -68.0, 0.0), V(0.0, 1.0, 0.0)))
    q4_bridge = xw(Part.makeBox(7.8, 44.0, 28.0, V(22.0, -44.0, -14.0)))
    q4d = q4_cyl.fuse(q4_bridge).removeSplitter()
    q5d = xw(Part.makeCylinder(28.5, 32.0, V(30.0, 0.0, 0.0), V(1.0, 0.0, 0.0)))
    q5c = xw(
        Part.makeCylinder(30.0, 6.0, V(54.0, 0.0, 0.0), V(1.0, 0.0, 0.0)).cut(
            Part.makeCylinder(28.6, 8.0, V(53.0, 0.0, 0.0), V(1.0, 0.0, 0.0))
        )
    )
    ftw = xw(Part.makeCylinder(22.5, 15.7, V(62.2, 0.0, 0.0), V(1.0, 0.0, 0.0)))

    ov_w = (
        fclv.common(q3d).Volume
        + fclv.common(cyoke).Volume
        + q3d.common(cyoke).Volume
        + q3d.common(q4d).Volume
        + cyoke.common(q4d).Volume
        + q4d.common(q5d).Volume
        + q5d.common(ftw).Volume
    )
    assert ov_w < 0.1, f"{prefix} Cardan wrist overlap {ov_w:.2f} mm^3"

    objs.append(v4.add_part(doc, group, f"{prefix}_ForearmClevis", fclv, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_WristPitch_q3", q3d, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_WristCardanYoke", cyoke, METAL))
    objs.append(v4.add_part(doc, group, f"{prefix}_WristYaw_q4", q4d, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_WristRoll_q5", q5d, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_WristRollCollar", q5c, ROLL_AMBER))
    objs.append(v4.add_part(doc, group, f"{prefix}_FTWafer_Mini45", ftw, FLEX_TEAL))

    h, rl, j, cb, ls = v4.make_gripper_parts(
        wrist_pos, u_reach, u_rad, u_tan, cam_side=cam_side, in_open=in_open, hook_open=hook_open
    )
    objs.append(v4.add_part(doc, group, f"{prefix}_Gripper", h, METAL))
    objs.append(v4.add_part(doc, group, f"{prefix}_GripperRails", rl, TITANIUM))
    objs.append(v4.add_part(doc, group, f"{prefix}_GripperJaws", j, v4.JAW_ORANGE))
    objs.append(v4.add_part(doc, group, f"{prefix}_StereoCamBar", cb, CAM_DARK))
    objs.append(v4.add_part(doc, group, f"{prefix}_StereoLenses", ls, LENS_CYAN))
    return objs


def build():
    _sync_v4_coords(CAV_XMIN, ARM_MOUNT_X)
    try:
        if DOC_NAME in App.listDocuments():
            App.closeDocument(DOC_NAME)
        doc = App.newDocument(DOC_NAME)

        g_shell = doc.addObject("App::DocumentObjectGroup", "Chassis_Shell")
        g_int = doc.addObject("App::DocumentObjectGroup", "RearBay_Internals")
        g_cut = doc.addObject("App::DocumentObjectGroup", "Chassis_Cutaway")
        g_stow = doc.addObject("App::DocumentObjectGroup", "Stowed_Config")
        g_dep = doc.addObject("App::DocumentObjectGroup", "Deployed_Nominal_15in")
        g_open = doc.addObject("App::DocumentObjectGroup", "Deployed_OpenApproach_15in")
        g_tilt = doc.addObject("App::DocumentObjectGroup", "Deployed_Tilted_5deg_Pitch")

        b1 = V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z)
        b2 = V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z)

        # ------------------------------------------------------------------
        # 1. Shallow Chassis Shell (X = 0..245 mm, 520 Y x 840 Z mm)
        # ------------------------------------------------------------------
        shell_raw = Part.makeBox(CHASSIS_LEN, 2.0 * HY, 2.0 * HZ, V(0.0, -HY, -HZ))
        x_edges = [e for e in shell_raw.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
        shell_raw = shell_raw.makeChamfer(CH, x_edges)

        # Rear deployer & avionics cavity (X = 10..90 mm)
        rear_bay = Part.makeBox(BULKHEAD_X0 - REAR_PLATE_THK, 490.0, 790.0, V(REAR_PLATE_THK, -245.0, -395.0))
        # Recessed dual-arm cavities (X = 110..245 mm) + mouth bevels + swept arc reliefs
        cav_p = Part.makeBox(CHASSIS_LEN - CAV_XMIN + 15.0, CAV_W, CAV_H, V(CAV_XMIN, CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
        cav_n = Part.makeBox(CHASSIS_LEN - CAV_XMIN + 15.0, CAV_W, CAV_H, V(CAV_XMIN, -CAV_Y - 0.5 * CAV_W, -0.5 * CAV_H))
        bev_p = make_shallow_cavity_bevel(CAV_Y)
        bev_n = make_shallow_cavity_bevel(-CAV_Y)
        arc_p = make_shallow_arc_relief(CAV_Y, ARM_MOUNT_Z)
        arc_n = make_shallow_arc_relief(-CAV_Y, -ARM_MOUNT_Z)

        # Annular 15" nesting groove (X = 221.5..246.0 mm, r in [170.0, 211.0])
        nest_groove = Part.makeCylinder(R_15IN + 20.5, 25.0, V(221.5, 0.0, 0.0), V(1, 0, 0)).cut(
            Part.makeCylinder(R_15IN - 20.5, 28.0, V(220.0, 0.0, 0.0), V(1, 0, 0))
        )

        # Bulkhead q0 bores + 4x boom pass-throughs (Y=+-210, Z=+-155) + centerline witness cam pocket
        bore_tools = [rear_bay, cav_p, cav_n, bev_p, bev_n, arc_p, arc_n, nest_groove]
        for cy_b, cz_b in ((CAV_Y, ARM_MOUNT_Z), (-CAV_Y, -ARM_MOUNT_Z)):
            bore_tools.append(Part.makeCylinder(46.0, 26.0, V(BULKHEAD_X0 - 2.0, cy_b, cz_b), V(1, 0, 0)))
            bore_tools.append(Part.makeCylinder(50.5, 6.5, V(CAV_XMIN - 4.5, cy_b, cz_b), V(1, 0, 0)))
        for sy in (1.0, -1.0):
            for sz in (1.0, -1.0):
                bore_tools.append(Part.makeCylinder(14.5, 160.0, V(BULKHEAD_X0 - 2.0, sy * BOOM_Y, sz * BOOM_Z), V(1, 0, 0)))
        bore_tools.append(Part.makeCylinder(6.5, 28.0, V(220.0, 0.0, 0.0), V(1, 0, 0)))  # witness cam bore
        # Flush -Z secondary OSR radiator pocket (X=20..230, Y=-240..+240, Z=-420..-417) -> 0.101 m^2
        bore_tools.append(Part.makeBox(210.0, 480.0, 4.0, V(20.0, -240.0, -HZ - 1.0)))

        cut_all = bore_tools[0]
        for bt in bore_tools[1:]:
            cut_all = cut_all.fuse(bt)
        shell = shell_raw.cut(cut_all).removeSplitter()

        # Host mounting bushings on perimeter of X = 0..10 mm plate (bored cleanly from shell)
        lug_shapes = []
        lug_bores = []
        for sy in (1.0, -1.0):
            for sz in (1.0, -1.0):
                py, pz = sy * 215.0, sz * 402.0
                lug_shapes.append(Part.makeCylinder(11.5, REAR_PLATE_THK, V(0.0, py, pz), V(1, 0, 0)))
                lug_bores.append(Part.makeCylinder(12.0, REAR_PLATE_THK + 4.0, V(-1.0, py, pz), V(1, 0, 0)))
        # Host umbilical recessed port at (X=0..10, Y=0, Z=-352)
        umb_bore = Part.makeBox(REAR_PLATE_THK + 4.0, 44.0, 26.0, V(-1.0, -22.0, -365.0))
        lug_bores.append(umb_bore)
        for lgb in lug_bores:
            shell = shell.cut(lgb).removeSplitter()
        host_lugs = lug_shapes[0]
        for lg in lug_shapes[1:]:
            host_lugs = host_lugs.fuse(lg)
        host_lugs = host_lugs.removeSplitter()

        rad_iod_nz = Part.makeBox(209.0, 479.0, 2.8, V(20.5, -239.5, -HZ))
        v4.add_part(doc, g_shell, "Radiator_IOD_NZ_OSR", rad_iod_nz, v4.WHITE)
        static_hw_shapes = [rad_iod_nz]
        for idx, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
            chute = Part.makeCylinder(14.0, 134.5, V(CAV_XMIN + 0.2, sy * BOOM_Y, sz * BOOM_Z), V(1, 0, 0)).cut(
                Part.makeCylinder(10.5, 138.0, V(CAV_XMIN - 1.0, sy * BOOM_Y, sz * BOOM_Z), V(1, 0, 0))
            )
            v4.add_part(doc, g_shell, f"BoomGuideChute_{idx+1}", chute, DARK)
            static_hw_shapes.append(chute)

        wcam_body = Part.makeCylinder(6.0, 22.0, V(221.0, 0.0, 0.0), V(1, 0, 0))
        wcam_lens = Part.makeCylinder(4.5, 2.0, V(243.0, 0.0, 0.0), V(1, 0, 0))
        v4.add_part(doc, g_shell, "WitnessCam_Body", wcam_body, CAM_DARK)
        v4.add_part(doc, g_shell, "WitnessCam_Lens", wcam_lens, LENS_CYAN)
        static_hw_shapes.extend([wcam_body, wcam_lens])

        # Fixed q0 Base-Roll drives (X = 34..110) + cavity liners + 2x NEA 9102 arm HDRMs
        for cy_b, cz_b, tag_arm in ((CAV_Y, ARM_MOUNT_Z, "Arm1"), (-CAV_Y, -ARM_MOUNT_Z, "Arm2")):
            flange = Part.makeCylinder(50.0, 4.0, V(CAV_XMIN - 4.0, cy_b, cz_b), V(1, 0, 0))
            barrel = Part.makeCylinder(44.0, 62.0, V(CAV_XMIN - 66.0, cy_b, cz_b), V(1, 0, 0))
            enc_cap = Part.makeCylinder(36.0, 10.0, V(CAV_XMIN - 76.0, cy_b, cz_b), V(1, 0, 0))
            v4.add_part(doc, g_shell, f"FixedBaseRoll_{tag_arm}_Flange", flange, TITANIUM)
            v4.add_part(doc, g_shell, f"FixedBaseRoll_{tag_arm}_Drive", barrel, DARK)
            v4.add_part(doc, g_shell, f"FixedBaseRoll_{tag_arm}_Encoder", enc_cap, ROLL_AMBER)
            static_hw_shapes.extend([flange, barrel, enc_cap])

            clip_y0 = 13.0 if cy_b > 0 else -HY
            arc_clip = Part.makeBox(10.0, HY - 13.0, 360.0, V(CAV_XMIN - 2.0, clip_y0, cz_b - 180.0))
            liner_rect = Part.makeBox(2.0, CAV_W - 6.0, CAV_H - 6.0, V(CAV_XMIN + 0.2, cy_b - 0.5 * (CAV_W - 6.0), -0.5 * (CAV_H - 6.0)))
            liner_arc = Part.makeCylinder(R_ARC - 3.0, 2.0, V(CAV_XMIN + 0.2, cy_b, cz_b), V(1, 0, 0)).common(arc_clip)
            liner = liner_rect.fuse(liner_arc).cut(Part.makeCylinder(52.0, 12.0, V(CAV_XMIN - 6.0, cy_b, cz_b), V(1, 0, 0))).removeSplitter()
            v4.add_part(doc, g_shell, f"Liner_{tag_arm}", liner, DARK)
            static_hw_shapes.append(liner)

        static_hw_shapes.append(host_lugs)

        hdrm_arm_shapes = []
        for tag_arm, cy_h, cz_h in (("Arm1", CAV_Y, -145.0), ("Arm2", -CAV_Y, +145.0)):
            nut = Part.makeCylinder(21.6, 39.4, V(CAV_XMIN + 0.6, cy_h, cz_h), V(1, 0, 0))
            cradle = Part.makeCone(24.0, 14.0, 20.0, V(CAV_XMIN + 40.0, cy_h, cz_h), V(1, 0, 0))
            v4.add_part(doc, g_shell, f"HDRM_NEA9102_{tag_arm}", nut, HDRM_RED)
            v4.add_part(doc, g_shell, f"HDRM_CupCone_{tag_arm}", cradle, TITANIUM)
            hdrm_arm_shapes.extend([nut, cradle])

        v4.add_part(doc, g_shell, "ChassisShell_GoldMLI", shell, GOLD)
        v4.add_part(doc, g_shell, "HostMountLugs", host_lugs, TITANIUM)

        # ------------------------------------------------------------------
        # 2. RearBay_Internals (Tangent-aligned Z=+-123mm boom shafts/spools/brackets/rollers + host-bolted compute/FOC/PDU)
        # ------------------------------------------------------------------
        rear_int_shapes = []
        for rail_tag, z_rail, sz in (("Upper", +BOOM_DRUM_Z, 1.0), ("Lower", -BOOM_DRUM_Z, -1.0)):
            motor = Part.makeCylinder(20.0, 90.0, V(DRUM_X, -45.0, z_rail), V(0, 1, 0))
            m_brk = Part.makeBox(40.0, 54.0, 26.0, V(REAR_PLATE_THK + 0.2, -27.0, z_rail - 13.0)).cut(
                Part.makeCylinder(20.2, 60.0, V(DRUM_X, -30.0, z_rail), V(0, 1, 0))
            ).removeSplitter()
            shaft = Part.makeCylinder(5.0, 470.0, V(DRUM_X, -235.0, z_rail), V(0, 1, 0))
            v4.add_part(doc, g_int, f"BoomDrive_{rail_tag}_Motor", motor, ROLL_AMBER)
            v4.add_part(doc, g_int, f"BoomDrive_{rail_tag}_Bracket", m_brk, TITANIUM)
            v4.add_part(doc, g_int, f"BoomDrive_{rail_tag}_Shaft", shaft, METAL)
            rear_int_shapes.extend([motor, m_brk, shaft])
            for side_tag, y_drum in (("PY", +BOOM_Y), ("NY", -BOOM_Y)):
                hub = Part.makeCylinder(DRUM_R_HUB, 56.0, V(DRUM_X, y_drum - 28.0, z_rail), V(0, 1, 0))
                fl_in = Part.makeCylinder(DRUM_R_FLANGE, 4.0, V(DRUM_X, y_drum - 32.0, z_rail), V(0, 1, 0))
                fl_out = Part.makeCylinder(DRUM_R_FLANGE, 4.0, V(DRUM_X, y_drum + 28.0, z_rail), V(0, 1, 0))
                drum = hub.fuse(fl_in).fuse(fl_out).removeSplitter()
                # Bearing bracket tying shaft directly to front bulkhead (X = 55..89.8)
                brg = Part.makeBox(34.8, 12.0, 26.0, V(DRUM_X + 5.0, y_drum - 44.0 if y_drum > 0 else y_drum + 32.0, z_rail - 13.0))
                # Pinch/guide roller steering tangent tape cleanly along Z=+-155 mm guide-chute centerline
                roller = Part.makeCylinder(4.8, 28.0, V(82.0, y_drum - 14.0, sz * (BOOM_Z - 14.9)), V(0, 1, 0))
                v4.add_part(doc, g_int, f"BoomSpool_{rail_tag}_{side_tag}", drum, TITANIUM)
                v4.add_part(doc, g_int, f"BoomBearing_{rail_tag}_{side_tag}", brg, METAL)
                v4.add_part(doc, g_int, f"BoomGuideRoller_{rail_tag}_{side_tag}", roller, ROLL_AMBER)
                rear_int_shapes.extend([drum, brg, roller])

        # Host-deck-bolted coldplate avionics: Orin NX/OBC + 14x Elmo Twitter FOC bank + PDU + Host Umbilical
        orin_obc = Part.makeBox(58.0, 150.0, 176.0, V(REAR_PLATE_THK + 0.2, -158.0, -88.0))
        foc_bank = Part.makeBox(58.0, 150.0, 176.0, V(REAR_PLATE_THK + 0.2, 8.0, -88.0))
        pdu_box = Part.makeBox(58.0, 200.0, 66.0, V(REAR_PLATE_THK + 0.2, -100.0, -385.0))
        umbilical = Part.makeBox(10.0, 42.0, 24.0, V(0.0, -21.0, -364.0))
        v4.add_part(doc, g_int, "Avionics_OrinNX_OBC", orin_obc, AVIONICS_SLATE)
        v4.add_part(doc, g_int, "Avionics_FOC_14xElmo", foc_bank, ROLL_AMBER)
        v4.add_part(doc, g_int, "PayloadPDU_Coldplate", pdu_box, TITANIUM)
        v4.add_part(doc, g_int, "HostUmbilical_Connector", umbilical, HDRM_RED)
        rear_int_shapes.extend([orin_obc, foc_bank, pdu_box, umbilical])

        # 4x Front Target Plate Cup-Cone HDRM Standoffs (X = 245.5 .. 264.8 mm at Y=+-215, Z=+-200)
        for idx, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
            cup = Part.makeCone(13.0, 9.0, 19.3, V(CHASSIS_LEN + 0.5, sy * 215.0, sz * 200.0), V(1, 0, 0))
            v4.add_part(doc, g_int, f"PlateHDRM_Standoff_{idx+1}", cup, HDRM_RED)
            rear_int_shapes.append(cup)

        int_comp = Part.makeCompound(rear_int_shapes)
        hw_comp = Part.makeCompound(static_hw_shapes + hdrm_arm_shapes)
        ov_int_shell = int_comp.common(shell).Volume
        ov_int_hw = int_comp.common(hw_comp).Volume
        ov_hw_shell = hw_comp.common(shell).Volume
        assert ov_int_shell < 0.1, f"RearBay internals vs shell overlap {ov_int_shell:.2f} mm^3"
        assert ov_int_hw < 0.1, f"RearBay internals vs static hw overlap {ov_int_hw:.2f} mm^3"
        assert ov_hw_shell < 0.1, f"Static hw vs shell overlap {ov_hw_shell:.2f} mm^3"

        # ------------------------------------------------------------------
        # 3. Chassis_Cutaway (Cut away X < 15 rear plate & outer Y < -190/+190 slice? Clean -X rear open box)
        # ------------------------------------------------------------------
        # Cutaway removes -X host backplate (X < 12) + lowers upper/lower side cheek slice for crystal-clear view
        cut_mask = Part.makeBox(260.0, 600.0, 900.0, V(12.0, -300.0, -450.0))
        v4.add_part(doc, g_cut, "Cutaway_Shell", shell.common(cut_mask), GOLD, transparency=35)
        for o in g_int.Group + g_shell.Group:
            if o.Name not in ("ChassisShell_GoldMLI", "HostMountLugs"):
                v4.add_part(doc, g_cut, f"Cutaway_{o.Name}", o.Shape, o.ViewObject.ShapeColor if App.GuiUp else TITANIUM)

        obstacle = Part.makeCompound([o.Shape for o in g_shell.Group] + [
            o.Shape for o in g_int.Group if o.Name.startswith("PlateHDRM_Standoff_")
        ])
        shell_only_obs = Part.makeCompound([o.Shape for o in g_shell.Group])

        # ------------------------------------------------------------------
        # 4. Stowed_Config (Arms flush in X<=244 mm + TargetPlate stowed at X=265..273, ring nested X=223..265)
        # ------------------------------------------------------------------
        stow_objs = []
        stow_objs += build_arm_iod(
            doc, g_stow, "Stow_Arm1", b1,
            V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1),
            V(ARM_MOUNT_X, CAV_Y, ARM_MOUNT_Z - L1 - L2),
            u_reach=V(0, 0, -1), u_rad=V(1, 0, 0), u_tan=V(0, -1, 0),
            clevis_ur=V(0, 0, 1), clevis_ut=V(0, -1, 0), cam_side=-1.0,
        )
        stow_objs += build_arm_iod(
            doc, g_stow, "Stow_Arm2", b2,
            V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1),
            V(ARM_MOUNT_X, -CAV_Y, -ARM_MOUNT_Z + L1 + L2),
            u_reach=V(0, 0, 1), u_rad=V(1, 0, 0), u_tan=V(0, 1, 0),
            clevis_ur=V(0, 0, -1), clevis_ut=V(0, 1, 0), cam_side=-1.0,
        )
        st_ring, st_plate, st_fid = make_target_plate_and_ring(V(STOW_RING_X, 0.0, 0.0))
        v4.add_part(doc, g_stow, "Stow_TargetRing_15in", st_ring, RING_SILVER, transparency=15)
        v4.add_part(doc, g_stow, "Stow_TargetPlate", st_plate, PLATE_AL, transparency=20)
        v4.add_part(doc, g_stow, "Stow_TargetFiducial", st_fid, CAM_DARK)

        # Boom stowed segments from X=245.5 to socket X=256.8
        st_booms = []
        for idx, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
            bm = make_boom_tube(V(CHASSIS_LEN + 0.5, sy * BOOM_Y, sz * BOOM_Z), V(STOW_PLATE_X0 - 8.2, sy * BOOM_Y, sz * BOOM_Z))
            v4.add_part(doc, g_stow, f"Stow_BoomStub_{idx+1}", bm, BOOM_CFRP)
            st_booms.append(bm)

        stow_arm_comp = Part.makeCompound([o.Shape for o in stow_objs])
        stow_tgt_comp = Part.makeCompound([st_ring, st_plate, st_fid] + st_booms)
        ov_stow_arms_shell = stow_arm_comp.common(obstacle).Volume
        ov_stow_tgt_shell = stow_tgt_comp.common(obstacle).Volume
        ov_stow_arms_tgt = stow_arm_comp.common(stow_tgt_comp).Volume
        assert ov_stow_arms_shell < 0.1, f"Stowed arms vs shell overlap {ov_stow_arms_shell:.2f} mm^3"
        assert ov_stow_tgt_shell < 0.1, f"Stowed target vs shell overlap {ov_stow_tgt_shell:.2f} mm^3"
        assert ov_stow_arms_tgt < 0.1, f"Stowed arms vs target overlap {ov_stow_arms_tgt:.2f} mm^3"

        full_stow_bb = Part.makeCompound([shell, host_lugs, stow_arm_comp, stow_tgt_comp]).BoundBox

        # ------------------------------------------------------------------
        # 5. Deployed_Nominal_15in (X_ring = 725 mm [+480 standoff], 4x CFRP booms)
        # ------------------------------------------------------------------
        dep_ring, dep_plate, dep_fid = make_target_plate_and_ring(V(RING_X_DEPLOYED, 0.0, 0.0))
        v4.add_part(doc, g_dep, "DepNom_TargetRing_15in", dep_ring, RING_SILVER, transparency=15)
        v4.add_part(doc, g_dep, "DepNom_TargetPlate", dep_plate, PLATE_AL, transparency=20)
        v4.add_part(doc, g_dep, "DepNom_TargetFiducial", dep_fid, CAM_DARK)

        dep_booms = []
        for idx, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
            bm = make_boom_tube(
                V(CHASSIS_LEN + 0.5, sy * BOOM_Y, sz * BOOM_Z),
                V(RING_X_DEPLOYED + RING_H_TOTAL - 8.2, sy * BOOM_Y, sz * BOOM_Z),
            )
            v4.add_part(doc, g_dep, f"DepNom_CFRPBoom_{idx+1}", bm, BOOM_CFRP)
            dep_booms.append(bm)

        e1, w1, ur1, ut1, ang1, d1 = v4.solve_arm_base_roll(b1, (0.0, 0.0), R_15IN, RING_X_DEPLOYED, L1, L2)
        e2, w2, ur2, ut2, ang2, d2 = v4.solve_arm_base_roll(b2, (0.0, 0.0), R_15IN, RING_X_DEPLOYED, L1, L2)
        da1 = build_arm_iod(doc, g_dep, "DepNom_Arm1", b1, e1, w1, V(1, 0, 0), ur1, ut1, ur1, ut1, cam_side=-1.0)
        da2 = build_arm_iod(doc, g_dep, "DepNom_Arm2", b2, e2, w2, V(1, 0, 0), ur2, ut2, ur2, ut2, cam_side=-1.0)

        da1_c = Part.makeCompound([o.Shape for o in da1])
        da2_c = Part.makeCompound([o.Shape for o in da2])
        dep_tgt_c = Part.makeCompound([dep_ring, dep_plate, dep_fid] + dep_booms)
        ov_dep = (
            da1_c.common(shell_only_obs).Volume + da2_c.common(shell_only_obs).Volume
            + da1_c.common(da2_c).Volume + da1_c.common(dep_tgt_c).Volume + da2_c.common(dep_tgt_c).Volume
        )
        assert ov_dep < 0.1, f"Deployed nominal overlap {ov_dep:.2f} mm^3"

        # ------------------------------------------------------------------
        # 6. Deployed_OpenApproach_15in (22 mm off ring, +-18 mm open Marman jaws)
        # ------------------------------------------------------------------
        app_x = RING_X_DEPLOYED - 22.0
        eo1, wo1, uro1, uto1, ango1, do1 = v4.solve_arm_base_roll(b1, (0.0, 0.0), R_15IN, app_x, L1, L2)
        eo2, wo2, uro2, uto2, ango2, do2 = v4.solve_arm_base_roll(b2, (0.0, 0.0), R_15IN, app_x, L1, L2)
        oa1 = build_arm_iod(doc, g_open, "OpenApp_Arm1", b1, eo1, wo1, V(1, 0, 0), uro1, uto1, uro1, uto1, cam_side=-1.0, in_open=18.0, hook_open=18.0)
        oa2 = build_arm_iod(doc, g_open, "OpenApp_Arm2", b2, eo2, wo2, V(1, 0, 0), uro2, uto2, uro2, uto2, cam_side=-1.0, in_open=18.0, hook_open=18.0)
        oa1_c = Part.makeCompound([o.Shape for o in oa1])
        oa2_c = Part.makeCompound([o.Shape for o in oa2])
        ov_open = (
            oa1_c.common(shell_only_obs).Volume + oa2_c.common(shell_only_obs).Volume
            + oa1_c.common(oa2_c).Volume + oa1_c.common(dep_tgt_c).Volume + oa2_c.common(dep_tgt_c).Volume
        )
        assert ov_open < 0.1, f"Open approach overlap {ov_open:.2f} mm^3"

        # ------------------------------------------------------------------
        # 7. Deployed_Tilted_5deg_Pitch (Differential Boom Payout: +5 deg around +Y)
        # ------------------------------------------------------------------
        tilt_rot = App.Rotation(V(0.0, 1.0, 0.0), 5.0)
        tilt_ring_ctr = V(RING_X_DEPLOYED, 0.0, 0.0)
        tl_ring, tl_plate, tl_fid = make_target_plate_and_ring(tilt_ring_ctr, tilt_rot)
        v4.add_part(doc, g_tilt, "DepTilt5_TargetRing_15in", tl_ring, RING_SILVER, transparency=15)
        v4.add_part(doc, g_tilt, "DepTilt5_TargetPlate", tl_plate, PLATE_AL, transparency=20)
        v4.add_part(doc, g_tilt, "DepTilt5_TargetFiducial", tl_fid, CAM_DARK)

        tl_booms = []
        boom_lens = {}
        for idx, (sy, sz) in enumerate(((1, 1), (-1, 1), (1, -1), (-1, -1))):
            p_start = V(CHASSIS_LEN + 0.5, sy * BOOM_Y, sz * BOOM_Z)
            p_socket_local = V(RING_H_TOTAL - 8.2, sy * BOOM_Y, sz * BOOM_Z)
            p_end = tilt_ring_ctr + tilt_rot.multVec(p_socket_local)
            bm = make_boom_tube(p_start, p_end)
            v4.add_part(doc, g_tilt, f"DepTilt5_CFRPBoom_{idx+1}", bm, BOOM_CFRP)
            tl_booms.append(bm)
            boom_lens["upper" if sz > 0 else "lower"] = p_end.sub(p_start).Length

        te1, tw1, t_nr, tur1, tut1, tcur1, tcut1, tang1, td1 = solve_arm_tilted_ring(b1, tilt_ring_ctr, tilt_rot, R_15IN, PALM_X)
        te2, tw2, _, tur2, tut2, tcur2, tcut2, tang2, td2 = solve_arm_tilted_ring(b2, tilt_ring_ctr, tilt_rot, R_15IN, PALM_X)
        ta1 = build_arm_iod(doc, g_tilt, "DepTilt5_Arm1", b1, te1, tw1, t_nr, tur1, tut1, tcur1, tcut1, cam_side=-1.0)
        ta2 = build_arm_iod(doc, g_tilt, "DepTilt5_Arm2", b2, te2, tw2, t_nr, tur2, tut2, tcur2, tcut2, cam_side=-1.0)

        ta1_c = Part.makeCompound([o.Shape for o in ta1])
        ta2_c = Part.makeCompound([o.Shape for o in ta2])
        tl_tgt_c = Part.makeCompound([tl_ring, tl_plate, tl_fid] + tl_booms)
        ov_tilt = (
            ta1_c.common(shell_only_obs).Volume + ta2_c.common(shell_only_obs).Volume
            + ta1_c.common(ta2_c).Volume + ta1_c.common(tl_tgt_c).Volume + ta2_c.common(tl_tgt_c).Volume
        )
        assert ov_tilt < 0.1, f"Tilted +5 deg overlap {ov_tilt:.2f} mm^3"

        # Default visible state
        for grp, vis in (
            (g_shell, True), (g_int, True), (g_cut, False),
            (g_stow, False), (g_dep, True), (g_open, False), (g_tilt, False),
        ):
            v4.set_group_visible(grp, vis)

        doc.recompute()
        out_fcstd = os.path.join(HERE, "arm_iod_demo.FCStd")
        doc.saveAs(out_fcstd)
        for bak in glob.glob(os.path.join(HERE, "*.FCBak")):
            try:
                os.remove(bak)
            except OSError:
                pass

        print(f"SAVED: {out_fcstd}")
        print(
            f"Stowed Envelope (mm): {full_stow_bb.XMax - full_stow_bb.XMin:.1f} X x "
            f"{full_stow_bb.YMax - full_stow_bb.YMin:.1f} Y x {full_stow_bb.ZMax - full_stow_bb.ZMin:.1f} Z "
            f"(X=[{full_stow_bb.XMin:.1f}, {full_stow_bb.XMax:.1f}])"
        )
        print(
            f"[Nominal 15in @ 480 mm] Arm1 reach={d1:.1f}/456mm q=({ang1[0]:+.1f},{ang1[1]:+.1f},{ang1[2]:+.1f},{ang1[3]:+.1f}) | ov={ov_dep:.2f}"
        )
        print(
            f"[OpenApproach @ +458 mm] Arm1 reach={do1:.1f}/456mm in_open=18.0 hook_open=18.0 | ov={ov_open:.2f}"
        )
        print(
            f"[Tilted +5deg Pitch] Booms upper={boom_lens['upper']:.1f} lower={boom_lens['lower']:.1f} mm "
            f"| Arm1 reach={td1:.1f}/456 Arm2 reach={td2:.1f}/456 | ov={ov_tilt:.2f}"
        )
        print(
            f"Overlaps (mm^3): Int/Shell={ov_int_shell:.2f}, Int/HW={ov_int_hw:.2f}, "
            f"StowArms/Shell={ov_stow_arms_shell:.2f}, StowTgt/Shell={ov_stow_tgt_shell:.2f}, StowArms/Tgt={ov_stow_arms_tgt:.2f}"
        )
        return doc
    finally:
        _sync_v4_coords(_ORIG_CAV_XMIN, _ORIG_ARM_MOUNT_X)


if __name__ == "__main__":
    build()
