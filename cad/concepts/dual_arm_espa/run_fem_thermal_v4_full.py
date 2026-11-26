"""FreeCAD FEM (Gmsh + CalculiX ccx.exe) Full Spacecraft + Attached Deployed 6-DOF Arms 3D Thermal Audit (v4.2).

Combines the v4.2 15" ESPA bus CompSolid with BOTH deployed 6-DOF Cardan-Roll arms + ATI Mini45 wrists
physically bonded onto the X = 185.0 mm front bulkhead at (Y = +70, Z = +245) and (Y = -70, Z = -245) mm.

Solves:
  1. v4_full_rpo_hot  (1200 s): LEO 550 km Worst-Case Nadir-Tilt RPO Capture Burst (bus 199 W + 2x arms 28 W)
  2. v4_full_geo_cold (4320 s): GEO 72-min Cold Equinox Eclipse (heaters active on battery/tank & both arms)
"""
import math
import os

import BOPTools.SplitFeatures
import FreeCAD as App
try:
    import FreeCADGui as Gui
except Exception:
    Gui = None
import ObjectsFem
import Part
from femmesh import gmshtools
from femtools import ccxtools

V = App.Vector

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
REPO_FIG_DIR = os.path.join(REPO_ROOT, "docs", "media", "thermal_audit")
ART_FIG_DIR = os.environ.get("THERMAL_ART_DIR", "")
OUT_FIG_DIRS = [d for d in (REPO_FIG_DIR, ART_FIG_DIR) if d]


def make_attached_arm_solids(prefix: str, x_bulkhead: float, y_s: float, z_s: float, sign_z: float):
    """Build 7 sliver-free material solids for a deployed 6-DOF arm flush-mounted at x_bulkhead."""
    x_sh = x_bulkhead + 80.0
    p_base = V(x_bulkhead, y_s, z_s)
    p_sh = V(x_sh, y_s, z_s)

    base_cyl = Part.makeCylinder(40.0, 80.0, p_base, V(1, 0, 0))
    sh_sph = Part.makeSphere(37.0, p_sh)
    base_sh = base_cyl.fuse(sh_sph).removeSplitter()

    el_ang = math.radians(20.0)
    p_el = V(x_sh + 248.0 * math.cos(el_ang), y_s, z_s + sign_z * 248.0 * math.sin(el_ang))
    el_sph = Part.makeSphere(33.0, p_el)

    wr_ang = math.radians(15.0)
    p_wr = V(p_el.x + 208.0 * math.cos(wr_ang), y_s, p_el.z - sign_z * 208.0 * math.sin(wr_ang))
    wr_sph = Part.makeSphere(29.0, p_wr)
    wr_cyl = Part.makeCylinder(28.5, 46.0, p_wr, V(1, 0, 0))
    wrist_ti = wr_sph.fuse(wr_cyl).removeSplitter()

    # Coaxial Mini45 wafer + Al-7075 Gripper Palm w/ jaw fingers
    p_ft = V(p_wr.x + 46.0, y_s, p_wr.z)
    mini45_ti = Part.makeCylinder(22.5, 16.0, p_ft, V(1, 0, 0))

    x_palm = p_wr.x + 62.0
    palm_box = Part.makeBox(70.0, 80.0, 52.0, V(x_palm, y_s - 40.0, p_wr.z - 26.0))
    jaw_p = Part.makeBox(36.0, 20.0, 14.0, V(x_palm + 70.0, y_s - 10.0, p_wr.z + 12.0))
    jaw_n = Part.makeBox(36.0, 20.0, 14.0, V(x_palm + 70.0, y_s - 10.0, p_wr.z - 26.0))
    palm_al = palm_box.fuse(jaw_p).fuse(jaw_n).removeSplitter()

    # CFRP upper & forearm tubes trimmed cleanly at joint spheres
    d_up = (p_el - p_sh).normalize()
    up_cfrp = Part.makeCylinder(18.0, p_sh.distanceToPoint(p_el), p_sh, d_up).cut(sh_sph).cut(el_sph).removeSplitter()

    d_fo = (p_wr - p_el).normalize()
    fo_cfrp = Part.makeCylinder(15.0, p_el.distanceToPoint(p_wr), p_el, d_fo).cut(el_sph).cut(wr_sph).removeSplitter()

    solids = [
        (f"{prefix}_Ti_BaseSh", base_sh),
        (f"{prefix}_CFRP_Up", up_cfrp),
        (f"{prefix}_Ti_Elbow", el_sph),
        (f"{prefix}_CFRP_Fore", fo_cfrp),
        (f"{prefix}_Ti_Wrist", wrist_ti),
        (f"{prefix}_Mini45", mini45_ti),
        (f"{prefix}_Palm", palm_al),
    ]
    return solids, x_palm


def run_v4_full_fem(
    doc_name: str,
    tag: str,
    transient_s: float,
    init_temp_k: float,
    rad_ambient_k: float,
    mli_bus_eps: float,
    mli_bus_k: float,
    mli_arm_eps: float,
    mli_arm_k: float,
    palm_sink_k: float,
    q_av: float,
    q_foc: float,
    q_de: float,
    q_rw: float,
    q_batt: float,
    q_tank: float,
    q_arm_sh: float,
    q_arm_el: float,
    q_arm_wr: float,
    q_arm_palm: float,
    time_max_inc: float = 180.0,
):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # 1. Unified Al Bus Structure (640x600x910 chamfered shell w/ decks, -Z coldplate & OSR, DragonEye saddle)
    outer = Part.makeBox(640.0, 600.0, 910.0, V(-320.0, -300.0, -455.0))
    outer = outer.makeChamfer(50.0, [e for e in outer.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0])
    inner = Part.makeBox(470.0, 570.0, 880.0, V(-305.0, -285.0, -440.0))
    inner = inner.makeChamfer(55.0, [e for e in inner.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0])

    front_cavs = Part.makeBox(140.0, 280.0, 760.0, V(185.0, -140.0, -380.0))
    opt_pkt = Part.makeBox(150.0, 134.0, 207.0, V(185.0, 154.0, -119.0))
    strap_slot = Part.makeBox(26.0, 69.0, 24.0, V(162.0, 210.0, -40.0))
    bus_shell = outer.cut(inner).cut(front_cavs).cut(opt_pkt).cut(strap_slot).removeSplitter()

    de_mount = Part.makeBox(13.0, 119.0, 112.0, V(184.0, 162.0, -36.0)).cut(strap_slot)
    iris_mount = Part.makeBox(49.0, 72.0, 69.0, V(184.0, 185.0, -112.0))
    cp_nz = Part.makeBox(456.0, 450.0, 8.0, V(-298.0, -225.0, -442.0))
    rad_nz = Part.makeBox(520.0, 375.0, 6.0, V(-260.0, -187.5, -459.0))
    strap_notch = Part.makeBox(22.0, 26.0, 14.0, V(144.0, 208.0, -320.0))
    lower_deck = Part.makeBox(476.0, 576.0, 8.0, V(-308.0, -288.0, -318.0)).common(outer).cut(strap_notch).removeSplitter()
    upper_deck = Part.makeBox(476.0, 576.0, 8.0, V(-308.0, -288.0, 166.0)).common(outer)
    rw_spine = Part.makeBox(56.0, 80.0, 478.0, V(-160.0, -40.0, -311.0))
    rib_p = Part.makeBox(56.0, 18.0, 118.0, V(-160.0, 205.0, -435.0))
    rib_n = Part.makeBox(56.0, 18.0, 118.0, V(-160.0, -223.0, -435.0))

    # Recessed +Z Star Trackers tilted +-20 deg outward at (-130, +-175, +455) without piercing Z=440 inner bay
    st_solids = []
    for sy in (1.0, -1.0):
        st_body = Part.makeCylinder(28.0, 12.0, V(0, 0, -9.0), V(0, 0, 1))
        st_baff = Part.makeCone(24.0, 30.0, 18.0, V(0, 0, 3.0), V(0, 0, 1))
        st = st_body.fuse(st_baff).removeSplitter()
        st.Placement = App.Placement(V(-130.0, sy * 175.0, 455.0), App.Rotation(V(1, 0, 0), -sy * 20.0))
        st_solids.append(st)

    al_core = (
        bus_shell.fuse(de_mount).fuse(iris_mount).fuse(cp_nz).fuse(rad_nz).fuse(lower_deck).fuse(upper_deck)
        .fuse(rw_spine).fuse(rib_p).fuse(rib_n).cut(st_solids[0]).cut(st_solids[1]).removeSplitter()
    )
    assert len(al_core.Solids) == 1, f"AlCore disconnected: {len(al_core.Solids)} solids"

    pgs_solid = (
        Part.makeBox(44.2, 50.0, 12.0, V(152.8, 212.0, -34.0))
        .fuse(Part.makeBox(18.0, 20.0, 400.0, V(146.0, 212.0, -434.0)))
        .fuse(Part.makeBox(20.0, 32.0, 8.0, V(145.0, 198.0, -434.0)))
        .removeSplitter()
    )
    # Full DragonEye flash LiDAR (122x119x112 body + protruding 13mm front aperture hood/window -> X=333)
    de_solid = (
        Part.makeBox(122.0, 119.0, 112.0, V(197.0, 162.0, -36.0))
        .fuse(Part.makeBox(12.0, 121.0, 114.0, V(319.0, 161.0, -37.0)))
        .fuse(Part.makeBox(4.0, 96.0, 88.0, V(330.0, 173.5, -24.0)))
        .removeSplitter()
    )
    # 3D PLUS IRIS camera in lower +Y optical pocket (body 86x72x69 + cylindrical lens barrel -> X=332.5)
    iris_solid = (
        Part.makeBox(86.0, 72.0, 69.0, V(232.0, 185.0, -112.0))
        .fuse(Part.makeCylinder(26.0, 14.5, V(318.0, 221.0, -77.5), V(1, 0, 0)))
        .removeSplitter()
    )
    av_solid = Part.makeBox(360.0, 400.0, 72.0, V(-280.0, -200.0, -434.0))
    foc_solid = Part.makeBox(65.0, 180.0, 52.0, V(90.0, -90.0, -434.0))
    rw_block = Part.makeBox(140.0, 280.0, 180.0, V(-104.0, -140.0, -162.0))
    batt_g10 = Part.makeBox(29.0, 220.0, 240.0, V(136.0, -110.0, -120.0))
    batt_solid = Part.makeBox(84.0, 252.0, 280.0, V(52.0, -126.0, -140.0))
    tank_block = Part.makeCylinder(110.0, 180.0, V(-165.0, 0.0, 174.0), V(0, 0, 1))

    # 2. Attach BOTH deployed 6-DOF arms onto the X = 185.0 mm front bulkhead!
    arm1_specs, x_palm1 = make_attached_arm_solids("Arm1", 185.0, 70.0, 245.0, +1.0)
    arm2_specs, x_palm2 = make_attached_arm_solids("Arm2", 185.0, -70.0, -245.0, -1.0)
    assert abs(x_palm1 - x_palm2) < 1e-6

    specs = [
        ("AlCore", al_core),
        ("PGSStrap", pgs_solid),
        ("DragonEye", de_solid),
        ("IRIS_Cam", iris_solid),
        ("StarTracker_PY", st_solids[0]),
        ("StarTracker_NY", st_solids[1]),
        ("AvionicsNZ", av_solid),
        ("FOCBank", foc_solid),
        ("RW12Block", rw_block),
        ("BattG10", batt_g10),
        ("BatteryPack", batt_solid),
        ("PropTank", tank_block),
    ] + arm1_specs + arm2_specs

    part_objs = []
    for name, shp in specs:
        assert shp.isValid() and len(shp.Solids) >= 1, f"Invalid shape {name}"
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        part_objs.append(o)

    frag = BOPTools.SplitFeatures.makeBooleanFragments(name="FullThermalAssembly")
    frag.Objects = part_objs
    frag.Mode = "CompSolid"
    doc.recompute()
    if App.GuiUp:
        for po in part_objs:
            if getattr(po, "ViewObject", None):
                po.ViewObject.Visibility = False

    solids = frag.Shape.Solids
    print(f"[{tag}] Combined CompSolid created: {len(solids)} solids (valid={frag.Shape.isValid()})")

    solid_names = {}
    used = set()
    for name, orig in specs[1:]:
        matches = []
        for idx, s in enumerate(solids):
            if s.Volume > 1e-3:
                ov = s.common(orig).Volume
                if ov > 0.40 * min(s.Volume, orig.Volume):
                    matches.append(f"Solid{idx + 1}")
        solid_names[name] = [m for m in matches if m not in used]
        used.update(solid_names[name])
    solid_names["AlCore"] = [f"Solid{i + 1}" for i in range(len(solids)) if f"Solid{i + 1}" not in used]

    analysis = ObjectsFem.makeAnalysis(doc, "FullThermalAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    solver.ThermoMechSteadyState = False
    solver.TimePeriod = float(transient_s)
    solver.TimeInitialIncrement = min(120.0, time_max_inc)
    solver.TimeMaximumIncrement = float(time_max_inc)
    analysis.addObject(solver)

    def add_mat(mat_name, refs, k_wk, rho, cp, cte, egpa, is_default=False):
        if not refs and not is_default:
            return
        m = ObjectsFem.makeMaterialSolid(doc, mat_name)
        m.Material = {
            "Name": mat_name,
            "Density": f"{rho} kg/m^3",
            "ThermalConductivity": f"{k_wk} W/(m*K)",
            "SpecificHeat": f"{cp} J/(kg*K)",
            "ThermalExpansionCoefficient": f"{cte} 1/K",
            "YoungsModulus": f"{egpa} GPa",
            "PoissonRatio": "0.33",
        }
        m.References = [] if is_default else [(frag, r) for r in refs]
        analysis.addObject(m)

    add_mat("Mat_Al6061", [], 167.0, 2700.0, 896.0, 2.3e-5, 69.0, is_default=True)
    add_mat("Mat_CCHP_HeatPipe", solid_names["PGSStrap"], 4500.0, 3500.0, 600.0, 1.6e-5, 40.0)
    optics_refs = solid_names["DragonEye"] + solid_names["IRIS_Cam"] + solid_names["StarTracker_PY"] + solid_names["StarTracker_NY"]
    add_mat("Mat_Optics", optics_refs, 80.0, 2500.0, 800.0, 2.1e-5, 70.0)
    add_mat("Mat_Avionics", solid_names["AvionicsNZ"] + solid_names["FOCBank"] + solid_names["RW12Block"], 60.0, 2200.0, 800.0, 2.0e-5, 70.0)
    add_mat("Mat_G10_Iso", solid_names["BattG10"], 0.35, 1800.0, 1100.0, 1.2e-5, 18.0)
    add_mat("Mat_Battery", solid_names["BatteryPack"], 25.0, 2100.0, 950.0, 2.0e-5, 30.0)
    add_mat("Mat_Ti_Tank", solid_names["PropTank"], 7.0, 1400.0, 2100.0, 8.6e-6, 114.0)

    arm_ti_refs = (
        solid_names["Arm1_Ti_BaseSh"] + solid_names["Arm1_Ti_Elbow"] + solid_names["Arm1_Ti_Wrist"] + solid_names["Arm1_Mini45"]
        + solid_names["Arm2_Ti_BaseSh"] + solid_names["Arm2_Ti_Elbow"] + solid_names["Arm2_Ti_Wrist"] + solid_names["Arm2_Mini45"]
    )
    arm_cfrp_refs = solid_names["Arm1_CFRP_Up"] + solid_names["Arm1_CFRP_Fore"] + solid_names["Arm2_CFRP_Up"] + solid_names["Arm2_CFRP_Fore"]
    arm_palm_refs = solid_names["Arm1_Palm"] + solid_names["Arm2_Palm"]
    add_mat("Mat_Arm_Ti", arm_ti_refs, 7.0, 4430.0, 526.0, 8.6e-6, 114.0)
    add_mat("Mat_Arm_CFRP", arm_cfrp_refs, 25.0, 1550.0, 900.0, 1.5e-6, 90.0)
    add_mat("Mat_Arm_Al7075", arm_palm_refs, 130.0, 2810.0, 960.0, 2.3e-5, 72.0)

    # Gmsh Mesh w/ targeted arm + optical suite refinement
    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = frag
    mesh.CharacteristicLengthMax = 48.0
    mesh.CharacteristicLengthMin = 8.0
    mesh.ElementOrder = "1st"
    mreg = ObjectsFem.makeMeshRegion(doc, mesh, element_length=18.0, name="ArmOpticsMeshRegion")
    mreg.References = [(frag, r) for r in (arm_ti_refs + arm_cfrp_refs + arm_palm_refs + solid_names["IRIS_Cam"] + solid_names["StarTracker_PY"] + solid_names["StarTracker_NY"])]
    doc.recompute()
    err = gmshtools.GmshTools(mesh).create_mesh()
    assert not err, f"Full Gmsh error: {err}"
    analysis.addObject(mesh)
    print(f"[{tag}] Combined mesh nodes={mesh.FemMesh.NodeCount}, vols={mesh.FemMesh.VolumeCount}")

    fix_faces, rad_faces, bus_mli_faces, arm_mli_faces, palm_faces, opt_faces = [], [], [], [], [], []
    for fi, f in enumerate(frag.Shape.Faces):
        c = f.CenterOfMass
        bb = f.BoundBox
        if abs(bb.XMin + 320.0) < 0.5 and abs(bb.XMax + 320.0) < 0.5 and f.Area > 1000.0:
            fix_faces.append(f"Face{fi + 1}")
        elif abs(bb.ZMin + 459.0) < 0.5 and abs(bb.ZMax + 459.0) < 0.5 and f.Area > 5000.0:
            rad_faces.append(f"Face{fi + 1}")
        elif c.x > 186.0 and 15.0 < abs(c.y) < 120.0 and abs(c.z) < 375.0:
            if bb.XMin >= x_palm1 - 0.5 and bb.XMax > x_palm1 + 0.5:
                palm_faces.append(f"Face{fi + 1}")
            elif f.Area > 40.0 and bb.XMin < x_palm1:
                arm_mli_faces.append(f"Face{fi + 1}")
        elif (c.x > 318.0 and c.y > 160.0 and -115.0 < c.z < 85.0) or (c.z > 455.5 and c.x < -90.0):
            opt_faces.append(f"Face{fi + 1}")
        else:
            on_outer = (
                abs(c.x - 320.0) < 1.0 or abs(c.x + 320.0) < 1.0 or
                abs(c.y - 300.0) < 1.0 or abs(c.y + 300.0) < 1.0 or
                abs(c.z - 455.0) < 1.0 or abs(c.z + 455.0) < 1.0
            )
            if on_outer and abs(c.z + 459.0) > 1.0 and f.Area > 500.0:
                bus_mli_faces.append(f"Face{fi + 1}")

    print(f"[{tag}] BCs: fix={len(fix_faces)} rad={len(rad_faces)} bus_mli={len(bus_mli_faces)} arm_mli={len(arm_mli_faces)} palm={len(palm_faces)} opt={len(opt_faces)}")
    assert fix_faces and rad_faces and bus_mli_faces and arm_mli_faces and palm_faces and opt_faces

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_AftPort")
    cfix.References = [(frag, f) for f in fix_faces]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = init_temp_k
    analysis.addObject(t_init)

    opt_sink_k = 265.0 if q_de > 20.0 else 2.7
    for cname, eps, amb, refs in (
        ("Rad_NZ_OSR", 0.80, rad_ambient_k, rad_faces),
        ("Rad_Bus_MLI", mli_bus_eps, mli_bus_k, bus_mli_faces),
        ("Rad_Arm_MLI", mli_arm_eps, mli_arm_k, arm_mli_faces),
        ("Rad_Arm_Palms", 0.45, palm_sink_k, palm_faces),
        ("Rad_Optics_Ext", 0.70, opt_sink_k, opt_faces),
    ):
        hf = ObjectsFem.makeConstraintHeatflux(doc, cname)
        hf.ConstraintType = "Radiation"
        hf.Emissivity = float(eps)
        hf.AmbientTemp = float(amb)
        hf.References = [(frag, f) for f in refs]
        analysis.addObject(hf)

    def add_power(pname, srefs, watts):
        if not srefs or watts <= 0.0:
            return
        bh = ObjectsFem.makeConstraintBodyHeatSource(doc, pname)
        bh.Mode = "Total Power"
        bh.TotalPower = f"{watts:.2f} W"
        bh.References = [(frag, r) for r in srefs]
        analysis.addObject(bh)

    add_power("Q_Avionics", solid_names["AvionicsNZ"], q_av)
    add_power("Q_FOC", solid_names["FOCBank"], q_foc)
    add_power("Q_DragonEye", solid_names["DragonEye"], q_de)
    add_power("Q_IRIS", solid_names["IRIS_Cam"], 2.0 if q_de > 20.0 else 0.5)
    add_power("Q_ST_PY", solid_names["StarTracker_PY"], 1.2)
    add_power("Q_ST_NY", solid_names["StarTracker_NY"], 1.2)
    add_power("Q_RW12", solid_names["RW12Block"], q_rw)
    add_power("Q_Battery", solid_names["BatteryPack"], q_batt)
    add_power("Q_PropTank", solid_names["PropTank"], q_tank)

    for arm_tag in ("Arm1", "Arm2"):
        add_power(f"Q_{arm_tag}_Sh", solid_names[f"{arm_tag}_Ti_BaseSh"], q_arm_sh)
        add_power(f"Q_{arm_tag}_El", solid_names[f"{arm_tag}_Ti_Elbow"], q_arm_el)
        add_power(f"Q_{arm_tag}_Wr", solid_names[f"{arm_tag}_Ti_Wrist"], q_arm_wr)
        add_power(f"Q_{arm_tag}_Pm", solid_names[f"{arm_tag}_Palm"], q_arm_palm)

    doc.recompute()
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    err_pre = fea.check_prerequisites()
    assert not err_pre, f"Full CCX prerequisites failed: {err_pre}"
    fea.write_inp_file()
    ret = fea.ccx_run()
    if ret != 0:
        print(fea.ccx_stdout)
        raise RuntimeError(f"Full ccx exited with code {ret}")
    fea.load_results()
    doc.recompute()

    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]
    assert len(res.Temperature) > 0, "Empty full FEM temperature result!"
    temps_c = [t - 273.15 for t in res.Temperature]
    disps_mm = res.DisplacementLengths
    node_map = mesh.FemMesh.Nodes
    print(f"[{tag}] FULL SPACECRAFT+ARMS FEM SUCCESS ({transient_s:.0f}s): T_min={min(temps_c):.1f} C, T_max={max(temps_c):.1f} C, max_CTE={max(disps_mm):.4f} mm")
    for rname in ("AlCore", "DragonEye", "IRIS_Cam", "StarTracker_PY", "AvionicsNZ", "Arm1_Ti_BaseSh", "Arm1_Ti_Elbow", "Arm1_Mini45", "Arm1_Palm", "Arm2_Ti_BaseSh", "Arm2_Mini45"):
        srefs = solid_names.get(rname, [])
        if not srefs:
            continue
        bb = Part.makeCompound([solids[int(r.replace("Solid", "")) - 1] for r in srefs]).BoundBox
        rt = [tc for nid, tc in zip(res.NodeNumbers, temps_c)
              if bb.XMin - 2 <= node_map[nid].x <= bb.XMax + 2 and bb.YMin - 2 <= node_map[nid].y <= bb.YMax + 2 and bb.ZMin - 2 <= node_map[nid].z <= bb.ZMax + 2]
        if rt:
            print(f"  Region {rname:15s}: T_min={min(rt):5.1f} C, T_max={max(rt):5.1f} C")

    if App.GuiUp and Gui is not None:
        pipe = doc.addObject("Fem::FemPostPipeline", "TempPipeline")
        pipe.load(res)
        doc.recompute()
        for o in doc.Objects:
            if hasattr(o, "ViewObject") and o.ViewObject:
                o.ViewObject.Visibility = False
        pipe.ViewObject.Visibility = True
        pipe.ViewObject.DisplayMode = "Surface"
        fields = pipe.ViewObject.getEnumerationsOfProperty("Field")
        if "Temperature" in fields:
            pipe.ViewObject.Field = "Temperature"

        gdoc = Gui.getDocument(doc.Name)
        gdoc.ActiveView.setAnimationEnabled(False)
        cam = gdoc.ActiveView.getCameraNode()
        # Frame +X front arms, +Y DragonEye pocket, and -Z OSR radiator cleanly
        z_cam = V(0.45, 0.78, -0.43).normalize()
        x_cam = V(0, 0, 1).cross(z_cam).normalize()
        y_cam = z_cam.cross(x_cam).normalize()
        cam.orientation.setValue(*App.Rotation(x_cam, y_cam, z_cam).Q)
        gdoc.ActiveView.fitAll()
        Gui.updateGui()

        for out_d in OUT_FIG_DIRS:
            os.makedirs(out_d, exist_ok=True)
            gdoc.ActiveView.saveImage(os.path.join(out_d, f"fem_{tag}.png"), 1400, 950, "White")

    App.closeDocument(doc_name)
    return min(temps_c), max(temps_c), max(disps_mm)


if __name__ == "__main__":
    # 1. LEO 550 km 20-min Worst Nadir-Tilt RPO Capture Burst w/ BOTH arms deployed & active
    run_v4_full_fem(
        "FEM_v4_Full_RPO", "v4_full_rpo_hot", transient_s=1200.0,
        init_temp_k=283.35, rad_ambient_k=267.9, mli_bus_eps=0.03, mli_bus_k=262.0,
        mli_arm_eps=0.07, mli_arm_k=270.0, palm_sink_k=298.0,
        q_av=72.0, q_foc=10.0, q_de=65.0, q_rw=42.0, q_batt=10.0, q_tank=0.0,
        q_arm_sh=3.5, q_arm_el=3.5, q_arm_wr=4.0, q_arm_palm=3.0, time_max_inc=150.0,
    )
    # 2. GEO 72-min Cold Equinox Eclipse w/ BOTH arms deployed + survival heaters
    run_v4_full_fem(
        "FEM_v4_Full_Cold", "v4_full_geo_cold", transient_s=4320.0,
        init_temp_k=276.15, rad_ambient_k=2.7, mli_bus_eps=0.04, mli_bus_k=2.7,
        mli_arm_eps=0.08, mli_arm_k=2.7, palm_sink_k=2.7,
        q_av=13.5, q_foc=0.0, q_de=2.0, q_rw=7.2, q_batt=12.0, q_tank=10.0,
        q_arm_sh=2.5, q_arm_el=2.5, q_arm_wr=6.5, q_arm_palm=0.0, time_max_inc=360.0,
    )
