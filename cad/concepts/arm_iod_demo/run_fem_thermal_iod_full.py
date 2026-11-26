"""FreeCAD FEM (Gmsh + CalculiX ccx.exe) Full Arm IOD Demo + Attached Deployed 6-DOF Arms 3D Thermal Audit.

Combines the ultra-shallow 245 x 520 x 840 mm IOD chassis + flush 0.1008 m^2 -Z OSR pocket with
BOTH deployed 6-DOF Cardan-Roll arms + ATI Mini45 wrists bonded onto the X = 110.0 mm bulkhead
and runs the 15-min (900 s) Isolated Host Active Capture Burst (fem_iod_full_isolated_hot.png).
"""
import os
import sys

import BOPTools.SplitFeatures
import FreeCAD as App
import FreeCADGui as Gui
import ObjectsFem
import Part
from femmesh import gmshtools
from femtools import ccxtools

V = App.Vector
ESPA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dual_arm_espa")
if ESPA_DIR not in sys.path:
    sys.path.insert(0, ESPA_DIR)

from run_fem_thermal_v4_full import make_attached_arm_solids  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
REPO_FIG_DIR = os.path.join(REPO_ROOT, "docs", "media", "thermal_audit")
ART_FIG_DIR = os.environ.get("THERMAL_ART_DIR", "")
OUT_FIG_DIRS = [d for d in (REPO_FIG_DIR, ART_FIG_DIR) if d]


def run_iod_full_fem():
    doc_name = "FEM_IOD_Full_Iso"
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # 1. IOD Al Chassis (245 x 520 x 840 mm w/ rear bay X=10..90, 20mm bulkhead X=90..110, flush -Z OSR)
    outer = Part.makeBox(245.0, 520.0, 840.0, V(0.0, -260.0, -420.0))
    outer = outer.makeChamfer(25.0, [e for e in outer.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0])
    rear_bay = Part.makeBox(80.0, 490.0, 790.0, V(10.0, -245.0, -395.0))
    cav_p = Part.makeBox(136.0, 120.0, 760.0, V(110.0, 10.0, -380.0))
    cav_n = Part.makeBox(136.0, 120.0, 760.0, V(110.0, -130.0, -380.0))
    chassis = outer.cut(rear_bay).cut(cav_p).cut(cav_n).removeSplitter()
    rad_osr = Part.makeBox(210.0, 480.0, 4.0, V(20.0, -240.0, -420.0))
    al_chassis = chassis.fuse(rad_osr).removeSplitter()
    assert len(al_chassis.Solids) == 1

    orin_obc = Part.makeBox(58.0, 150.0, 176.0, V(10.0, -158.0, -88.0))
    foc_bank = Part.makeBox(58.0, 150.0, 176.0, V(10.0, 8.0, -88.0))
    pdu_box = Part.makeBox(58.0, 200.0, 66.0, V(10.0, -100.0, -385.0))

    # 2. Both deployed 6-DOF arms mounted flush at X = 110.0 mm bulkhead
    arm1_specs, x_palm1 = make_attached_arm_solids("Arm1", 110.0, 70.0, 245.0, +1.0)
    arm2_specs, _ = make_attached_arm_solids("Arm2", 110.0, -70.0, -245.0, -1.0)

    specs = [("ChassisAl", al_chassis), ("OrinOBC", orin_obc), ("FOCBank", foc_bank), ("PDU", pdu_box)] + arm1_specs + arm2_specs
    objs = []
    for name, shp in specs:
        assert shp.isValid() and len(shp.Solids) >= 1, f"Invalid IOD full shape {name}"
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        objs.append(o)

    frag = BOPTools.SplitFeatures.makeBooleanFragments(name="IODFullThermalAssembly")
    frag.Objects = objs
    frag.Mode = "CompSolid"
    doc.recompute()
    for o in objs:
        o.ViewObject.Visibility = False

    solids = frag.Shape.Solids
    solid_names = {}
    used = set()
    for name, orig in specs[1:]:
        matches = []
        for idx, s in enumerate(solids):
            ov = s.common(orig).Volume
            if ov > 0.40 * min(s.Volume, orig.Volume):
                matches.append(f"Solid{idx + 1}")
        solid_names[name] = [m for m in matches if m not in used]
        used.update(solid_names[name])
    solid_names["ChassisAl"] = [f"Solid{i + 1}" for i in range(len(solids)) if f"Solid{i + 1}" not in used]
    print(f"[iod_full] CompSolid created: {len(solids)} solids (valid={frag.Shape.isValid()})")

    analysis = ObjectsFem.makeAnalysis(doc, "IODFullThermalAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    solver.ThermoMechSteadyState = False
    solver.TimePeriod = 900.0
    solver.TimeInitialIncrement = 90.0
    solver.TimeMaximumIncrement = 150.0
    analysis.addObject(solver)

    def add_mat(mname, refs, k_wk, rho, cp, cte, egpa, is_default=False):
        if not refs and not is_default:
            return
        m = ObjectsFem.makeMaterialSolid(doc, mname)
        m.Material = {
            "Name": mname,
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
    add_mat("Mat_Electronics", solid_names["OrinOBC"] + solid_names["FOCBank"] + solid_names["PDU"], 60.0, 2200.0, 800.0, 2.0e-5, 70.0)
    arm_ti_refs = (
        solid_names["Arm1_Ti_BaseSh"] + solid_names["Arm1_Ti_Elbow"] + solid_names["Arm1_Ti_Wrist"] + solid_names["Arm1_Mini45"]
        + solid_names["Arm2_Ti_BaseSh"] + solid_names["Arm2_Ti_Elbow"] + solid_names["Arm2_Ti_Wrist"] + solid_names["Arm2_Mini45"]
    )
    arm_cfrp_refs = solid_names["Arm1_CFRP_Up"] + solid_names["Arm1_CFRP_Fore"] + solid_names["Arm2_CFRP_Up"] + solid_names["Arm2_CFRP_Fore"]
    arm_palm_refs = solid_names["Arm1_Palm"] + solid_names["Arm2_Palm"]
    add_mat("Mat_Arm_Ti", arm_ti_refs, 7.0, 4430.0, 526.0, 8.6e-6, 114.0)
    add_mat("Mat_Arm_CFRP", arm_cfrp_refs, 25.0, 1550.0, 900.0, 1.5e-6, 90.0)
    add_mat("Mat_Arm_Al7075", arm_palm_refs, 130.0, 2810.0, 960.0, 2.3e-5, 72.0)

    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = frag
    mesh.CharacteristicLengthMax = 42.0
    mesh.CharacteristicLengthMin = 8.0
    mesh.ElementOrder = "1st"
    mreg = ObjectsFem.makeMeshRegion(doc, mesh, element_length=18.0, name="ArmMeshRegion")
    mreg.References = [(frag, r) for r in (arm_ti_refs + arm_cfrp_refs + arm_palm_refs)]
    doc.recompute()
    err = gmshtools.GmshTools(mesh).create_mesh()
    assert not err, f"IOD Full Gmsh error: {err}"
    analysis.addObject(mesh)
    print(f"[iod_full] Mesh nodes={mesh.FemMesh.NodeCount}, vols={mesh.FemMesh.VolumeCount}")

    back_faces, rad_faces, mli_faces, arm_mli_faces, palm_faces = [], [], [], [], []
    for fi, f in enumerate(frag.Shape.Faces):
        bb = f.BoundBox
        c = f.CenterOfMass
        if abs(bb.XMin) < 0.5 and abs(bb.XMax) < 0.5 and f.Area > 5000.0:
            back_faces.append(f"Face{fi + 1}")
        elif abs(bb.ZMin + 420.0) < 0.5 and abs(bb.ZMax + 420.0) < 0.5 and 18.0 <= c.x <= 232.0 and f.Area > 5000.0:
            rad_faces.append(f"Face{fi + 1}")
        elif c.x > 111.0 and 15.0 < abs(c.y) < 120.0 and abs(c.z) < 375.0:
            if bb.XMin >= x_palm1 - 0.5 and bb.XMax > x_palm1 + 0.5:
                palm_faces.append(f"Face{fi + 1}")
            elif f.Area > 40.0 and bb.XMin < x_palm1:
                arm_mli_faces.append(f"Face{fi + 1}")
        elif (abs(c.y) > 258.5 or abs(c.z) > 418.5 or abs(c.x - 245.0) < 0.5) and f.Area > 1000.0:
            mli_faces.append(f"Face{fi + 1}")

    assert back_faces and rad_faces and mli_faces and arm_mli_faces and palm_faces

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_HostDeck")
    cfix.References = [(frag, f) for f in back_faces]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = 282.65  # +9.5 C pre-burst idle equilibrium
    analysis.addObject(t_init)

    for cname, eps, amb, refs in (
        ("Rad_IOD_NZ_OSR", 0.80, 201.3, rad_faces),
        ("Rad_IOD_MLI", 0.035, 262.0, mli_faces),
        ("Rad_IOD_Arm_MLI", 0.07, 270.0, arm_mli_faces),
        ("Rad_IOD_Palms", 0.45, 298.0, palm_faces),
    ):
        hf = ObjectsFem.makeConstraintHeatflux(doc, cname)
        hf.ConstraintType = "Radiation"
        hf.Emissivity = float(eps)
        hf.AmbientTemp = float(amb)
        hf.References = [(frag, f) for f in refs]
        analysis.addObject(hf)

    def add_power(pname, srefs, watts):
        bh = ObjectsFem.makeConstraintBodyHeatSource(doc, pname)
        bh.Mode = "Total Power"
        bh.TotalPower = f"{watts:.2f} W"
        bh.References = [(frag, r) for r in srefs]
        analysis.addObject(bh)

    add_power("Q_OrinOBC", solid_names["OrinOBC"], 30.0)
    add_power("Q_FOC", solid_names["FOCBank"], 16.0)
    add_power("Q_PDU", solid_names["PDU"], 8.5)
    for arm_tag in ("Arm1", "Arm2"):
        add_power(f"Q_{arm_tag}_Sh", solid_names[f"{arm_tag}_Ti_BaseSh"], 3.5)
        add_power(f"Q_{arm_tag}_El", solid_names[f"{arm_tag}_Ti_Elbow"], 3.5)
        add_power(f"Q_{arm_tag}_Wr", solid_names[f"{arm_tag}_Ti_Wrist"], 4.0)
        add_power(f"Q_{arm_tag}_Pm", solid_names[f"{arm_tag}_Palm"], 3.0)

    doc.recompute()
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    err_pre = fea.check_prerequisites()
    assert not err_pre, f"IOD Full CCX failed: {err_pre}"
    fea.write_inp_file()
    ret = fea.ccx_run()
    if ret != 0:
        print(fea.ccx_stdout)
        raise RuntimeError(f"IOD Full ccx exited with code {ret}")
    fea.load_results()
    doc.recompute()

    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]
    assert len(res.Temperature) > 0
    temps_c = [t - 273.15 for t in res.Temperature]
    disps_mm = res.DisplacementLengths
    node_map = mesh.FemMesh.Nodes
    print(f"[iod_full_isolated_hot] IOD+ARMS FEM SUCCESS (900s): T_min={min(temps_c):.1f} C, T_max={max(temps_c):.1f} C, max_CTE={max(disps_mm):.4f} mm")
    for rname in ("ChassisAl", "OrinOBC", "FOCBank", "PDU", "Arm1_Ti_BaseSh", "Arm1_Mini45", "Arm1_Palm"):
        bb = Part.makeCompound([solids[int(r.replace("Solid", "")) - 1] for r in solid_names[rname]]).BoundBox
        rt = [tc for nid, tc in zip(res.NodeNumbers, temps_c)
              if bb.XMin - 2 <= node_map[nid].x <= bb.XMax + 2 and bb.YMin - 2 <= node_map[nid].y <= bb.YMax + 2 and bb.ZMin - 2 <= node_map[nid].z <= bb.ZMax + 2]
        if rt:
            print(f"  Region {rname:15s}: T_min={min(rt):5.1f} C, T_max={max(rt):5.1f} C")

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
    z_cam = V(-0.45, -0.78, -0.43).normalize()
    x_cam = V(0, 0, 1).cross(z_cam).normalize()
    y_cam = z_cam.cross(x_cam).normalize()
    cam.orientation.setValue(*App.Rotation(x_cam, y_cam, z_cam).Q)
    gdoc.ActiveView.fitAll()
    Gui.updateGui()

    for out_d in OUT_FIG_DIRS:
        os.makedirs(out_d, exist_ok=True)
        gdoc.ActiveView.saveImage(os.path.join(out_d, "fem_iod_full_isolated_hot.png"), 1400, 950, "White")

    App.closeDocument(doc_name)


if __name__ == "__main__":
    run_iod_full_fem()
