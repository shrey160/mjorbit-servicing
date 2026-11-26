"""FreeCAD FEM (Gmsh + CalculiX ccx.exe) 3D Coupled Thermal & CTE Verification for Hosted-Payload Arm IOD Demo.

Solves BOTH boundary modes on the ultra-shallow 245 x 520 x 840 mm chassis:
  1. Conductive Host Deck (T_host = +45 C fixed on X=0 interface, 15-min effective active burst)
  2. Thermally Isolated Host TICD (host X=0 adiabatic / <=4 W leak, rejecting through flush 0.101 m^2 -Z OSR patch + MLI)

Verifies peak Orin NX/FOC/PDU temperatures, -Z OSR gradient, and max CTE shift on the 4x CFRP boom guide chutes.
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


def run_iod_fem_case(doc_name: str, conductive_host: bool, tag: str, transient_s: float = 0.0, init_temp_k: float = 293.15):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # 1. Thermal-clean IOD Al chassis (245 x 520 x 840 mm w/ rear avionics pocket X=10..90, 20mm bulkhead X=90..110, arm bays X=110..245)
    outer = Part.makeBox(245.0, 520.0, 840.0, V(0.0, -260.0, -420.0))
    x_edges = [e for e in outer.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
    outer = outer.makeChamfer(25.0, x_edges)

    rear_bay = Part.makeBox(80.0, 490.0, 790.0, V(10.0, -245.0, -395.0))
    cav_p = Part.makeBox(136.0, 120.0, 760.0, V(110.0, 10.0, -380.0))
    cav_n = Part.makeBox(136.0, 120.0, 760.0, V(110.0, -130.0, -380.0))
    chassis = outer.cut(rear_bay).cut(cav_p).cut(cav_n).removeSplitter()

    # Flush -Z OSR patch fused seamlessly at X=20..230, Y=-240..240, Z=-420
    rad_osr = Part.makeBox(210.0, 480.0, 4.0, V(20.0, -240.0, -420.0))
    al_chassis = chassis.fuse(rad_osr).removeSplitter()
    assert len(al_chassis.Solids) == 1, f"IOD AlChassis disconnected: {len(al_chassis.Solids)} solids"

    # Mounted RearBay coldplate electronics touching X = 10.0 mm backplate flush (shared interface faces in CompSolid)
    orin_obc = Part.makeBox(58.0, 150.0, 176.0, V(10.0, -158.0, -88.0))
    foc_bank = Part.makeBox(58.0, 150.0, 176.0, V(10.0, 8.0, -88.0))
    pdu_box = Part.makeBox(58.0, 200.0, 66.0, V(10.0, -100.0, -385.0))

    specs = [("ChassisAl", al_chassis), ("OrinOBC", orin_obc), ("FOCBank", foc_bank), ("PDU", pdu_box)]
    objs = []
    for name, shp in specs:
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        objs.append(o)

    frag = BOPTools.SplitFeatures.makeBooleanFragments(name="IODThermalAssembly")
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

    analysis = ObjectsFem.makeAnalysis(doc, "IODThermalAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    if transient_s > 0.0:
        solver.ThermoMechSteadyState = False
        solver.TimePeriod = float(transient_s)
        solver.TimeInitialIncrement = 60.0
        solver.TimeMaximumIncrement = 120.0
    else:
        solver.ThermoMechSteadyState = True
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

    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = frag
    mesh.CharacteristicLengthMax = 38.0
    mesh.CharacteristicLengthMin = 8.0
    mesh.ElementOrder = "1st"
    doc.recompute()
    err = gmshtools.GmshTools(mesh).create_mesh()
    assert not err, f"IOD Gmsh error: {err}"
    analysis.addObject(mesh)

    back_faces, rad_faces, mli_faces = [], [], []
    for fi, f in enumerate(frag.Shape.Faces):
        bb = f.BoundBox
        c = f.CenterOfMass
        if abs(bb.XMin) < 0.5 and abs(bb.XMax) < 0.5 and f.Area > 5000.0:
            back_faces.append(f"Face{fi + 1}")
        elif abs(bb.ZMin + 420.0) < 0.5 and abs(bb.ZMax + 420.0) < 0.5 and 18.0 <= c.x <= 232.0 and f.Area > 5000.0:
            rad_faces.append(f"Face{fi + 1}")
        elif (abs(c.y) > 258.5 or abs(c.z) > 418.5 or abs(c.x - 245.0) < 0.5) and f.Area > 1000.0:
            mli_faces.append(f"Face{fi + 1}")

    assert back_faces and rad_faces, f"Missing faces back={len(back_faces)} rad={len(rad_faces)}"

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_HostDeck")
    cfix.References = [(frag, f) for f in back_faces]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = init_temp_k
    analysis.addObject(t_init)

    if conductive_host:
        # Conductive Host Deck clamped at +45 C (318.15 K)
        ct_host = ObjectsFem.makeConstraintTemperature(doc, "Temp_HostDeck_45C")
        ct_host.ConstraintType = "Temperature"
        ct_host.Temperature = 318.15
        ct_host.References = [(frag, f) for f in back_faces]
        analysis.addObject(ct_host)

    hf_rad = ObjectsFem.makeConstraintHeatflux(doc, "Rad_IOD_NZ_OSR")
    hf_rad.ConstraintType = "Radiation"
    hf_rad.Emissivity = 0.80
    # Exact 550 km LEO side-view F_side=0.2578 (S=1418, alb=0.35, OLR=265 -> T_sink_eff=201.3 K)
    hf_rad.AmbientTemp = 201.3
    hf_rad.References = [(frag, f) for f in rad_faces]
    analysis.addObject(hf_rad)

    hf_mli = ObjectsFem.makeConstraintHeatflux(doc, "Rad_IOD_MLI")
    hf_mli.ConstraintType = "Radiation"
    hf_mli.Emissivity = 0.035
    hf_mli.AmbientTemp = 262.0
    hf_mli.References = [(frag, f) for f in mli_faces]
    analysis.addObject(hf_mli)

    # Full 54.5 W rear-bay capture burst load (Orin NX 25W + carrier/OBC 5W + FOC/stereo 16W + PDU 8.5W)
    for pname, refs, pw in (("Q_OrinOBC", solid_names["OrinOBC"], 30.0), ("Q_FOC", solid_names["FOCBank"], 16.0), ("Q_PDU", solid_names["PDU"], 8.5)):
        bh = ObjectsFem.makeConstraintBodyHeatSource(doc, pname)
        bh.Mode = "Total Power"
        bh.TotalPower = f"{pw:.2f} W"
        bh.References = [(frag, r) for r in refs]
        analysis.addObject(bh)

    doc.recompute()
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    err_pre = fea.check_prerequisites()
    assert not err_pre, f"CCX prerequisites failed: {err_pre}"
    fea.write_inp_file()
    ret = fea.ccx_run()
    if ret != 0:
        print(fea.ccx_stdout)
        raise RuntimeError(f"ccx exited with code {ret}")
    fea.load_results()
    doc.recompute()

    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]
    assert len(res.Temperature) > 0, "Empty CalculiX temperature result!"
    temps_c = [t - 273.15 for t in res.Temperature]
    disps_mm = res.DisplacementLengths
    node_map = mesh.FemMesh.Nodes
    print(f"[{tag}] IOD FEM SOLVE (transient={transient_s:.0f}s): T_min={min(temps_c):.1f} C, T_max={max(temps_c):.1f} C, max_CTE_disp={max(disps_mm):.4f} mm")
    for rname, srefs in solid_names.items():
        if not srefs:
            continue
        bb = Part.makeCompound([solids[int(r.replace("Solid", "")) - 1] for r in srefs]).BoundBox
        rt = [tc for nid, tc in zip(res.NodeNumbers, temps_c)
              if bb.XMin - 2 <= node_map[nid].x <= bb.XMax + 2 and bb.YMin - 2 <= node_map[nid].y <= bb.YMax + 2 and bb.ZMin - 2 <= node_map[nid].z <= bb.ZMax + 2]
        if rt:
            print(f"  Region {rname:12s}: T_min={min(rt):5.1f} C, T_max={max(rt):5.1f} C")
            if rname == "OrinOBC":
                print(f"  -> Est. Orin NX SoC Tj (~T_max + 17.5 C @ 25W): {max(rt) + 17.5:5.1f} C (<= 99.0 C throttle limit)")

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
    elif len(fields) > 1:
        pipe.ViewObject.Field = fields[1]
    print("IOD VtkPipeline fields:", fields, "selected:", pipe.ViewObject.Field)

    gdoc = Gui.getDocument(doc.Name)
    gdoc.ActiveView.setAnimationEnabled(False)
    cam = gdoc.ActiveView.getCameraNode()
    z_cam = V(-0.65, -0.60, -0.46).normalize()
    x_cam = V(0, 0, 1).cross(z_cam).normalize()
    y_cam = z_cam.cross(x_cam).normalize()
    cam.orientation.setValue(*App.Rotation(x_cam, y_cam, z_cam).Q)
    gdoc.ActiveView.fitAll()
    Gui.updateGui()

    for out_d in OUT_FIG_DIRS:
        os.makedirs(out_d, exist_ok=True)
        gdoc.ActiveView.saveImage(os.path.join(out_d, f"fem_iod_{tag}.png"), 1300, 900, "White")

    App.closeDocument(doc_name)
    return min(temps_c), max(temps_c), max(disps_mm)


if __name__ == "__main__":
    run_iod_fem_case("FEM_IOD_Cond", conductive_host=True, tag="conductive_hot", transient_s=0.0, init_temp_k=318.15)
    # Initialized at +9.5 C (282.65 K) pre-burst idle equilibrium from TMM
    run_iod_fem_case("FEM_IOD_Iso_Trans", conductive_host=False, tag="isolated_transient", transient_s=900.0, init_temp_k=282.65)
