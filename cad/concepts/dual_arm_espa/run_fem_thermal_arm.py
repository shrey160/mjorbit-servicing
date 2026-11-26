"""FreeCAD FEM (Gmsh + CalculiX ccx.exe) 3D Transient Thermal & CTE Audit for Deployed 6-DOF Arm + ATI Mini45 Wrist.

Models the shared v4.2 / arm_iod_demo 6-DOF arm chain (248 mm Upper + 208 mm Forearm + 128 mm Cardan-Roll Wrist/Gripper)
as a multi-material BooleanFragments CompSolid:
  - BaseMount_q0 + Shoulder_q1 + Elbow_q2 + CardanWrist_q345 + FTWafer_Mini45 (Ti-6Al-4V, k=7.0 W/m*K)
  - UpperLink_CFRP + ForearmLink_CFRP (Composite tubes, k=25.0 W/m*K, CTE=1.5e-6 /K, E=90 GPa)
  - Gripper_Palm (Al-7075, k=130.0 W/m*K, bare metallic jaws/palm rad eps=0.45)

Runs two transient coupled thermomechanical scenarios:
  1. GEO Cold 72-min Eclipse (4320 s): +10 C init, -8 C bulkhead, 2.7 K space sink, w/ joint & wrist survival heaters
  2. LEO Sunlit 20-min Capture Burst (1200 s): +13 C init, +17 C bulkhead, TBM2G Cu losses + stereo/LED, CTE tip shift
"""
import os

import BOPTools.SplitFeatures
import FreeCAD as App
import FreeCADGui as Gui
import ObjectsFem
import Part
from femmesh import gmshtools
from femtools import ccxtools

V = App.Vector

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
REPO_FIG_DIR = os.path.join(REPO_ROOT, "docs", "media", "thermal_audit")
ART_FIG_DIR = os.environ.get("THERMAL_ART_DIR", "")
OUT_FIG_DIRS = [d for d in (REPO_FIG_DIR, ART_FIG_DIR) if d]


def run_arm_fem_case(
    doc_name: str,
    tag: str,
    transient_s: float,
    init_temp_k: float,
    bulkhead_temp_k: float,
    mli_sink_k: float,
    palm_sink_k: float,
    q_sh_w: float,
    q_el_w: float,
    q_wrist_w: float,
    q_palm_w: float,
    mli_eps: float = 0.075,
):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # Flush coaxial multi-material arm chain along +X (mm)
    base_q0 = Part.makeCylinder(45.0, 30.0, V(0.0, 0.0, 0.0), V(1, 0, 0))
    sh_q1 = Part.makeCylinder(37.0, 56.0, V(30.0, 0.0, 0.0), V(1, 0, 0))
    up_cfrp = Part.makeCylinder(18.0, 248.0, V(86.0, 0.0, 0.0), V(1, 0, 0))
    el_q2 = Part.makeCylinder(32.5, 50.0, V(334.0, 0.0, 0.0), V(1, 0, 0))
    fore_cfrp = Part.makeCylinder(15.0, 208.0, V(384.0, 0.0, 0.0), V(1, 0, 0))
    wrist_q345 = Part.makeCylinder(28.5, 62.0, V(592.0, 0.0, 0.0), V(1, 0, 0))
    mini45_ft = Part.makeCylinder(22.5, 15.7, V(654.0, 0.0, 0.0), V(1, 0, 0))
    palm_al = Part.makeBox(70.0, 80.0, 52.0, V(669.7, -40.0, -26.0))

    specs = [
        ("BaseMount_q0", base_q0),
        ("Shoulder_q1", sh_q1),
        ("UpperLink_CFRP", up_cfrp),
        ("Elbow_q2", el_q2),
        ("Forearm_CFRP", fore_cfrp),
        ("CardanWrist", wrist_q345),
        ("FTWafer_Mini45", mini45_ft),
        ("Gripper_Palm", palm_al),
    ]
    objs = []
    for name, shp in specs:
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        objs.append(o)

    frag = BOPTools.SplitFeatures.makeBooleanFragments(name="ArmThermalAssembly")
    frag.Objects = objs
    frag.Mode = "CompSolid"
    doc.recompute()
    for o in objs:
        o.ViewObject.Visibility = False

    solids = frag.Shape.Solids
    assert len(solids) == len(specs), f"Expected {len(specs)} arm solids, got {len(solids)}"

    solid_names = {}
    used = set()
    for name, orig in specs:
        for idx, s in enumerate(solids):
            sname = f"Solid{idx + 1}"
            if sname in used:
                continue
            if s.common(orig).Volume > 0.50 * orig.Volume:
                solid_names[name] = [sname]
                used.add(sname)
                break

    analysis = ObjectsFem.makeAnalysis(doc, "ArmThermalAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    solver.ThermoMechSteadyState = False
    solver.TimePeriod = float(transient_s)
    solver.TimeInitialIncrement = 120.0
    solver.TimeMaximumIncrement = 240.0
    analysis.addObject(solver)

    def add_mat(mname, refs, k_wk, rho, cp, cte, egpa, is_default=False):
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

    add_mat("Mat_Ti6Al4V", [], 7.0, 4430.0, 526.0, 8.6e-6, 114.0, is_default=True)
    add_mat("Mat_CFRP", solid_names["UpperLink_CFRP"] + solid_names["Forearm_CFRP"], 25.0, 1550.0, 900.0, 1.5e-6, 90.0)
    add_mat("Mat_Al7075", solid_names["Gripper_Palm"], 130.0, 2810.0, 960.0, 2.3e-5, 72.0)

    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = frag
    mesh.CharacteristicLengthMax = 14.0
    mesh.CharacteristicLengthMin = 4.0
    mesh.ElementOrder = "1st"
    doc.recompute()
    err = gmshtools.GmshTools(mesh).create_mesh()
    assert not err, f"Arm Gmsh error: {err}"
    analysis.addObject(mesh)
    print(f"[{tag}] Arm mesh nodes={mesh.FemMesh.NodeCount}, vols={mesh.FemMesh.VolumeCount}")

    base_faces, mli_faces, palm_faces = [], [], []
    for fi, f in enumerate(frag.Shape.Faces):
        bb = f.BoundBox
        c = f.CenterOfMass
        if abs(bb.XMin) < 0.2 and abs(bb.XMax) < 0.2:
            base_faces.append(f"Face{fi + 1}")
        elif abs(bb.XMax - bb.XMin) > 0.5 or (c.x > 739.0):
            if c.x >= 669.8:
                palm_faces.append(f"Face{fi + 1}")
            elif c.x >= 29.8:
                mli_faces.append(f"Face{fi + 1}")

    assert base_faces and mli_faces and palm_faces, f"Missing arm faces {len(base_faces)}/{len(mli_faces)}/{len(palm_faces)}"

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_BaseMount")
    cfix.References = [(frag, f) for f in base_faces]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = init_temp_k
    analysis.addObject(t_init)

    ct_base = ObjectsFem.makeConstraintTemperature(doc, "Temp_Bulkhead")
    ct_base.ConstraintType = "Temperature"
    ct_base.Temperature = bulkhead_temp_k
    ct_base.References = [(frag, f) for f in base_faces]
    analysis.addObject(ct_base)

    hf_mli = ObjectsFem.makeConstraintHeatflux(doc, "Rad_Arm_MLI")
    hf_mli.ConstraintType = "Radiation"
    hf_mli.Emissivity = float(mli_eps)
    hf_mli.AmbientTemp = mli_sink_k
    hf_mli.References = [(frag, f) for f in mli_faces]
    analysis.addObject(hf_mli)

    hf_palm = ObjectsFem.makeConstraintHeatflux(doc, "Rad_Gripper_Bare")
    hf_palm.ConstraintType = "Radiation"
    hf_palm.Emissivity = 0.45
    hf_palm.AmbientTemp = palm_sink_k
    hf_palm.References = [(frag, f) for f in palm_faces]
    analysis.addObject(hf_palm)

    for pname, rkey, pw in (
        ("Q_Shoulder", "Shoulder_q1", q_sh_w),
        ("Q_Elbow", "Elbow_q2", q_el_w),
        ("Q_Wrist_Mini45", "CardanWrist", q_wrist_w),
        ("Q_Gripper_Stereo", "Gripper_Palm", q_palm_w),
    ):
        if pw > 0.0:
            bh = ObjectsFem.makeConstraintBodyHeatSource(doc, pname)
            bh.Mode = "Total Power"
            bh.TotalPower = f"{pw:.2f} W"
            bh.References = [(frag, r) for r in solid_names[rkey]]
            analysis.addObject(bh)

    doc.recompute()
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    err_pre = fea.check_prerequisites()
    assert not err_pre, f"Arm CCX prerequisites failed: {err_pre}"
    fea.write_inp_file()
    ret = fea.ccx_run()
    if ret != 0:
        print(fea.ccx_stdout)
        raise RuntimeError(f"Arm ccx exited with code {ret}")
    fea.load_results()
    doc.recompute()

    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]
    assert len(res.Temperature) > 0, "Empty arm temperature result!"
    temps_c = [t - 273.15 for t in res.Temperature]
    disps_mm = res.DisplacementLengths
    node_map = mesh.FemMesh.Nodes
    print(f"[{tag}] ARM FEM SOLVE ({transient_s:.0f}s): T_min={min(temps_c):.1f} C, T_max={max(temps_c):.1f} C, max_CTE_disp={max(disps_mm):.4f} mm")
    for rname, srefs in solid_names.items():
        bb = Part.makeCompound([solids[int(r.replace("Solid", "")) - 1] for r in srefs]).BoundBox
        rt = [tc for nid, tc in zip(res.NodeNumbers, temps_c)
              if bb.XMin - 1 <= node_map[nid].x <= bb.XMax + 1 and bb.YMin - 1 <= node_map[nid].y <= bb.YMax + 1 and bb.ZMin - 1 <= node_map[nid].z <= bb.ZMax + 1]
        if rt:
            print(f"  Arm Region {rname:16s}: T_min={min(rt):5.1f} C, T_max={max(rt):5.1f} C (dT={max(rt)-min(rt):.1f} C)")

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
    z_cam = V(0.35, -0.78, 0.52).normalize()
    x_cam = V(0, 0, 1).cross(z_cam).normalize()
    y_cam = z_cam.cross(x_cam).normalize()
    cam.orientation.setValue(*App.Rotation(x_cam, y_cam, z_cam).Q)
    gdoc.ActiveView.fitAll()
    Gui.updateGui()

    for out_d in OUT_FIG_DIRS:
        os.makedirs(out_d, exist_ok=True)
        gdoc.ActiveView.saveImage(os.path.join(out_d, f"fem_arm_{tag}.png"), 1300, 900, "White")

    App.closeDocument(doc_name)
    return min(temps_c), max(temps_c), max(disps_mm)


if __name__ == "__main__":
    # 1. GEO 72-min Cold Eclipse w/ Gilmore small-blanket arm MLI (eps*=0.08) & bulkhead=-7.9 C from TMM
    run_arm_fem_case(
        "FEM_Arm_Cold", "cold_eclipse", transient_s=4320.0,
        init_temp_k=282.15, bulkhead_temp_k=265.25, mli_sink_k=2.7, palm_sink_k=2.7,
        q_sh_w=2.5, q_el_w=2.5, q_wrist_w=7.5, q_palm_w=0.0, mli_eps=0.08,
    )
    # 2. 20-min Sun-Soaked Active Capture Burst w/ Gilmore eps*=0.07 & pre-burst bulkhead=+10.2 C from TMM
    run_arm_fem_case(
        "FEM_Arm_Hot", "hot_capture", transient_s=1200.0,
        init_temp_k=283.35, bulkhead_temp_k=286.15, mli_sink_k=270.0, palm_sink_k=298.0,
        q_sh_w=3.5, q_el_w=3.5, q_wrist_w=4.0, q_palm_w=3.0, mli_eps=0.07,
    )
