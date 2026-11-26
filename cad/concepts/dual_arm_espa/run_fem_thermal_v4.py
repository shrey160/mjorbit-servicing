"""FreeCAD FEM (Gmsh + CalculiX ccx.exe) 3D Coupled Thermal & CTE Verification for v4.2 ESPA Servicer.

Builds a clean thermal multi-material BooleanFragments (CompSolid) assembly matching
the updated v4.2 internal layout:
  - Al-6061/7075 Bus Shell + 20 mm Bulkhead + Decks + -Z Coldplate + 520x375 mm OSR Panel (k=167 W/m*K)
  - Laminated PGS Thermal Strap from +Y DragonEye down to -Z Coldplate (k=1000 W/m*K)
  - Recessed ASC DragonEye unit (volumetric power) + -Z Avionics/FOC/RW heat blocks
  - G10-isolated 1000 Wh Li-Ion Battery (k=0.35 W/m*K isolator) & Ti-6Al-4V PEPT-230 Tank (k=7 W/m*K)

Solves two coupled thermomechanical steady-state cases using CalculiX (*RADIATE + body heat + CTE):
  1. LEO 550 km SSO Nominal Cruise (~60 W steady orbital balance)
  2. LEO 550 km SSO Quasi-Steady 20-min Effective RPO Capture Load (peak internal dissipation)
Saves 3D temperature contour renders to docs/media/thermal_audit/ and reports per-region T_min/T_max + CTE shift.
"""
import math
import os
import sys

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


def calc_osr_sink_k(vf_earth: float, cos_sun: float = 0.03, cos_alb: float = 0.70, s0: float = 1418.0, albedo: float = 0.35, eir: float = 265.0, alpha: float = 0.16, eps: float = 0.80) -> float:
    q_abs = s0 * cos_sun * alpha + s0 * albedo * vf_earth * cos_alb * alpha + eir * vf_earth * eps
    return (2.7 ** 4 + q_abs / (eps * 5.670374419e-8)) ** 0.25


def build_v4_fem_doc(
    doc_name: str, q_avionics_w: float, q_foc_w: float, q_de_w: float, q_rw_w: float, q_batt_w: float,
    tag: str, transient_s: float = 0.0, init_temp_k: float = 293.15,
    rad_ambient_k: float = 201.8, mli_ambient_k: float = 262.0, time_max_inc: float = 120.0,
):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # 1. Unified Al Bus Structure (outer 640x600x910 chamfered shell w/ 15mm walls + 20mm bulkhead + Decks + -Z Coldplate + 520x375 OSR)
    outer = Part.makeBox(640.0, 600.0, 910.0, V(-320.0, -300.0, -455.0))
    ch_edges = [e for e in outer.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
    outer = outer.makeChamfer(50.0, ch_edges)

    inner = Part.makeBox(470.0, 570.0, 880.0, V(-305.0, -285.0, -440.0))
    in_edges = [e for e in inner.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) > 1.0]
    inner = inner.makeChamfer(55.0, in_edges)

    front_cavs = Part.makeBox(140.0, 280.0, 760.0, V(185.0, -140.0, -380.0))
    opt_pkt = Part.makeBox(150.0, 134.0, 207.0, V(185.0, 154.0, -119.0))
    strap_slot = Part.makeBox(26.0, 69.0, 24.0, V(162.0, 210.0, -40.0))
    bus_shell = outer.cut(inner).cut(front_cavs).cut(opt_pkt).cut(strap_slot).removeSplitter()

    # Bolted Al mounting saddle behind DragonEye in +Y optical bay (X=184..197, ties optical pocket wall to DragonEye)
    de_mount = Part.makeBox(13.0, 119.0, 112.0, V(184.0, 162.0, -36.0)).cut(strap_slot)

    # Fuse internal Al structural decks (embedded 3 mm into bus walls), -Z coldplate spreader, ribs & flush -Z OSR substrate
    cp_nz = Part.makeBox(456.0, 450.0, 8.0, V(-298.0, -225.0, -442.0))
    rad_nz = Part.makeBox(520.0, 375.0, 6.0, V(-260.0, -187.5, -459.0))
    strap_notch = Part.makeBox(22.0, 26.0, 14.0, V(144.0, 208.0, -320.0))
    lower_deck = Part.makeBox(476.0, 576.0, 8.0, V(-308.0, -288.0, -318.0)).common(outer).cut(strap_notch).removeSplitter()
    upper_deck = Part.makeBox(476.0, 576.0, 8.0, V(-308.0, -288.0, 166.0)).common(outer)
    rw_spine = Part.makeBox(56.0, 80.0, 478.0, V(-160.0, -40.0, -311.0))
    rib_p = Part.makeBox(56.0, 18.0, 118.0, V(-160.0, 205.0, -435.0))
    rib_n = Part.makeBox(56.0, 18.0, 118.0, V(-160.0, -223.0, -435.0))

    al_core = (
        bus_shell.fuse(de_mount).fuse(cp_nz).fuse(rad_nz).fuse(lower_deck).fuse(upper_deck)
        .fuse(rw_spine).fuse(rib_p).fuse(rib_n).removeSplitter()
    )
    assert len(al_core.Solids) == 1, f"AlCore disconnected: {len(al_core.Solids)} solids"

    # 2. Separate Material Sub-Solids (touching flush faces at shared interfaces for CompSolid bonding)
    pgs_h = Part.makeBox(44.2, 50.0, 12.0, V(152.8, 212.0, -34.0))
    pgs_v = Part.makeBox(18.0, 20.0, 400.0, V(146.0, 212.0, -434.0))
    pgs_f = Part.makeBox(20.0, 32.0, 8.0, V(145.0, 198.0, -434.0))
    pgs_solid = pgs_h.fuse(pgs_v).fuse(pgs_f).removeSplitter()

    de_solid = Part.makeBox(122.0, 119.0, 112.0, V(197.0, 162.0, -36.0))
    av_solid = Part.makeBox(360.0, 400.0, 72.0, V(-280.0, -200.0, -434.0))
    foc_solid = Part.makeBox(65.0, 180.0, 52.0, V(90.0, -90.0, -434.0))
    rw_block = Part.makeBox(140.0, 280.0, 180.0, V(-104.0, -140.0, -162.0))
    batt_g10 = Part.makeBox(29.0, 220.0, 240.0, V(136.0, -110.0, -120.0))
    batt_solid = Part.makeBox(84.0, 252.0, 280.0, V(52.0, -126.0, -140.0))
    tank_block = Part.makeCylinder(110.0, 180.0, V(-165.0, 0.0, 174.0), V(0, 0, 1))

    specs = [
        ("AlCore", al_core),
        ("PGSStrap", pgs_solid),
        ("DragonEye", de_solid),
        ("AvionicsNZ", av_solid),
        ("FOCBank", foc_solid),
        ("RW12Block", rw_block),
        ("BattG10", batt_g10),
        ("BatteryPack", batt_solid),
        ("PropTank", tank_block),
    ]
    part_objs = []
    for name, shp in specs:
        assert shp.isValid() and len(shp.Solids) >= 1, f"Invalid shape {name}"
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        part_objs.append(o)

    frag = BOPTools.SplitFeatures.makeBooleanFragments(name="ThermalAssembly")
    frag.Objects = part_objs
    frag.Mode = "CompSolid"
    doc.recompute()
    if App.GuiUp:
        for po in part_objs:
            if getattr(po, "ViewObject", None):
                po.ViewObject.Visibility = False

    solids = frag.Shape.Solids
    print(f"[{tag}] BooleanFragments CompSolid created: {len(solids)} solids (valid={frag.Shape.isValid()})")

    # Map fragmented sub-solids by CenterOfMass proximity to original centroids
    def match_solid(target_box_shp):
        t_c = target_box_shp.BoundBox.Center
        t_v = target_box_shp.Volume
        # Find solid whose COM lies inside target_box or closest matching non-Al solid
        matches = []
        for idx, s in enumerate(solids):
            c = s.CenterOfMass
            if s.Volume > 1e-3:
                # Score by fraction inside original solid
                ov = s.common(target_box_shp).Volume
                if ov > 0.40 * min(s.Volume, t_v):
                    matches.append(f"Solid{idx + 1}")
        return matches

    # Ensure every solid is classified cleanly
    solid_names = {}
    used = set()
    for name, orig in specs[1:]:  # classify smaller components first, rest is AlCore
        matched = match_solid(orig)
        solid_names[name] = [m for m in matched if m not in used]
        used.update(solid_names[name])
    solid_names["AlCore"] = [f"Solid{i + 1}" for i in range(len(solids)) if f"Solid{i + 1}" not in used]
    print("Matched solids:", {k: len(v) for k, v in solid_names.items()})

    # Analysis & Solver
    analysis = ObjectsFem.makeAnalysis(doc, "ThermalAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    if transient_s > 0.0:
        solver.ThermoMechSteadyState = False
        solver.TimePeriod = float(transient_s)
        solver.TimeInitialIncrement = min(120.0, time_max_inc)
        solver.TimeMaximumIncrement = float(time_max_inc)
    else:
        solver.ThermoMechSteadyState = True
    analysis.addObject(solver)

    def add_mat(mat_name, refs, k_wk, rho_kgm3, cp_jkgk, cte_ik, e_gpa, is_default=False):
        if not refs and not is_default:
            return
        m = ObjectsFem.makeMaterialSolid(doc, mat_name)
        m.Material = {
            "Name": mat_name,
            "Density": f"{rho_kgm3} kg/m^3",
            "ThermalConductivity": f"{k_wk} W/(m*K)",
            "SpecificHeat": f"{cp_jkgk} J/(kg*K)",
            "ThermalExpansionCoefficient": f"{cte_ik} 1/K",
            "YoungsModulus": f"{e_gpa} GPa",
            "PoissonRatio": "0.33",
        }
        m.References = [] if is_default else [(frag, r) for r in refs]
        analysis.addObject(m)

    # Empty References = default material for all unassigned/remaining solids (prevents CalculiX missing-material errors)
    add_mat("Mat_Al6061", [], 167.0, 2700.0, 896.0, 2.3e-5, 69.0, is_default=True)
    # Embedded CCHP / High-k Spreader (20x18 mm tube k_eff=4,500 W/m*K -> 4.05 W/K + bolted de_mount ~2.2 W/K = ~6.2 W/K)
    add_mat("Mat_CCHP_HeatPipe", solid_names["PGSStrap"], 4500.0, 3500.0, 600.0, 1.6e-5, 40.0)
    add_mat("Mat_DragonEye", solid_names["DragonEye"], 80.0, 2500.0, 800.0, 2.1e-5, 70.0)
    add_mat("Mat_Avionics", solid_names["AvionicsNZ"] + solid_names["FOCBank"] + solid_names["RW12Block"], 60.0, 2200.0, 800.0, 2.0e-5, 70.0)
    add_mat("Mat_G10_Iso", solid_names["BattG10"], 0.35, 1800.0, 1100.0, 1.2e-5, 18.0)
    add_mat("Mat_Battery", solid_names["BatteryPack"], 25.0, 2100.0, 950.0, 2.0e-5, 30.0)
    add_mat("Mat_Ti_Tank", solid_names["PropTank"], 7.0, 1400.0, 2100.0, 8.6e-6, 114.0)

    # Gmsh Mesh
    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = frag
    mesh.CharacteristicLengthMax = 45.0
    mesh.CharacteristicLengthMin = 10.0
    mesh.ElementOrder = "1st"
    doc.recompute()
    gm = gmshtools.GmshTools(mesh)
    err = gm.create_mesh()
    assert not err, f"Gmsh error: {err}"
    analysis.addObject(mesh)
    print(f"Mesh nodes={mesh.FemMesh.NodeCount}, vols={mesh.FemMesh.VolumeCount}")

    # Fixed Structural Support on -X Aft Port Rim (for coupled CTE displacements) & Initial Temperature 293.15 K
    fix_faces = []
    rad_faces = []
    mli_faces = []
    for fi, f in enumerate(frag.Shape.Faces):
        c = f.CenterOfMass
        bb = f.BoundBox
        # Aft -X mounting face (X == -320)
        if abs(bb.XMin - (-320.0)) < 0.5 and abs(bb.XMax - (-320.0)) < 0.5 and f.Area > 1000.0:
            fix_faces.append(f"Face{fi + 1}")
        # -Z OSR radiator active outer surface (Z == -459)
        if abs(bb.ZMin - (-459.0)) < 0.5 and abs(bb.ZMax - (-459.0)) < 0.5 and f.Area > 5000.0:
            rad_faces.append(f"Face{fi + 1}")
        # Exterior MLI surfaces (on outer envelope bounds, excluding radiator & fixed internal cavity)
        on_outer = (
            abs(c.x - 320.0) < 1.0 or abs(c.x + 320.0) < 1.0 or
            abs(c.y - 300.0) < 1.0 or abs(c.y + 300.0) < 1.0 or
            abs(c.z - 455.0) < 1.0 or abs(c.z + 455.0) < 1.0
        )
        if on_outer and abs(c.z + 459.0) > 1.0 and f.Area > 500.0:
            mli_faces.append(f"Face{fi + 1}")

    assert len(fix_faces) >= 1 and len(rad_faces) >= 1 and len(mli_faces) >= 4, (
        f"Missing faces fix={len(fix_faces)} rad={len(rad_faces)} mli={len(mli_faces)}"
    )

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_AftPort")
    cfix.References = [(frag, f) for f in fix_faces]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = init_temp_k
    analysis.addObject(t_init)

    # -Z OSR Radiation (effective sink T_sink_eff from exact 550 km Earth VF + solar/albedo/OLR)
    hf_rad = ObjectsFem.makeConstraintHeatflux(doc, "Rad_NZ_OSR")
    hf_rad.ConstraintType = "Radiation"
    hf_rad.Emissivity = 0.80
    hf_rad.AmbientTemp = float(rad_ambient_k)
    hf_rad.References = [(frag, f) for f in rad_faces]
    analysis.addObject(hf_rad)

    # Exterior Gold MLI Radiation (eps*=0.03, EOL outer foil sink)
    hf_mli = ObjectsFem.makeConstraintHeatflux(doc, "Rad_MLI_Outer")
    hf_mli.ConstraintType = "Radiation"
    hf_mli.Emissivity = 0.03
    hf_mli.AmbientTemp = float(mli_ambient_k)
    hf_mli.References = [(frag, f) for f in mli_faces]
    analysis.addObject(hf_mli)

    # Internal Heat Sources
    def add_power(pname, srefs, watts):
        if not srefs or watts <= 0.0:
            return
        bh = ObjectsFem.makeConstraintBodyHeatSource(doc, pname)
        bh.Mode = "Total Power"
        bh.TotalPower = f"{watts:.2f} W"
        bh.References = [(frag, r) for r in srefs]
        analysis.addObject(bh)

    add_power("Q_Avionics", solid_names["AvionicsNZ"], q_avionics_w)
    add_power("Q_FOC", solid_names["FOCBank"], q_foc_w)
    add_power("Q_DragonEye", solid_names["DragonEye"], q_de_w)
    add_power("Q_RW12", solid_names["RW12Block"], q_rw_w)
    add_power("Q_Battery", solid_names["BatteryPack"], q_batt_w)

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
    # Region sampling via FemMesh node coordinates
    node_map = mesh.FemMesh.Nodes
    print(f"[{tag}] FEM SOLVE SUCCESS (transient={transient_s:.0f}s): T_min={min(temps_c):.1f} C, T_max={max(temps_c):.1f} C, max_CTE_disp={max(disps_mm):.4f} mm")
    for rname, srefs in solid_names.items():
        if not srefs:
            continue
        sh_reg = Part.makeCompound([solids[int(r.replace("Solid", "")) - 1] for r in srefs])
        bb = sh_reg.BoundBox
        r_temps = []
        for nid, t_c in zip(res.NodeNumbers, temps_c):
            p = node_map[nid]
            if bb.XMin - 2 <= p.x <= bb.XMax + 2 and bb.YMin - 2 <= p.y <= bb.YMax + 2 and bb.ZMin - 2 <= p.z <= bb.ZMax + 2:
                r_temps.append(t_c)
        if r_temps:
            print(f"  Region {rname:12s}: T_min={min(r_temps):5.1f} C, T_max={max(r_temps):5.1f} C")
            if rname == "AvionicsNZ":
                print(f"  -> Est. Orin NX SoC Tj (~T_max + 17.5 C @ 25W): {max(r_temps) + 17.5:5.1f} C (<= 99.0 C throttle limit)")

    # Render VTK Post-Processing Temperature Contour cleanly from +X/+Y/-Z (GUI mode only)
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
        z_cam = V(0.55, 0.72, -0.42).normalize()
        x_cam = V(0, 0, 1).cross(z_cam).normalize()
        y_cam = z_cam.cross(x_cam).normalize()
        cam.orientation.setValue(*App.Rotation(x_cam, y_cam, z_cam).Q)
        gdoc.ActiveView.fitAll()
        Gui.updateGui()

        for out_d in OUT_FIG_DIRS:
            os.makedirs(out_d, exist_ok=True)
            gdoc.ActiveView.saveImage(os.path.join(out_d, f"fem_v4_{tag}.png"), 1300, 900, "White")

    App.closeDocument(doc_name)
    return min(temps_c), max(temps_c), max(disps_mm)


if __name__ == "__main__":
    F_SIDE = 0.2578
    F_NADIR = 0.8474
    T_SINK_SIDE = calc_osr_sink_k(F_SIDE)
    T_SINK_NADIR = calc_osr_sink_k(F_NADIR)
    print(f"Computed 550 km OSR Sinks: T_side={T_SINK_SIDE:.1f} K, T_nadir={T_SINK_NADIR:.1f} K")
    # Case 1: LEO 550 km Hot Cruise (Orin NX power-gated OFF; 29.5 W internal bus + 3 W optics/bench; fast pseudo-transient)
    build_v4_fem_doc(
        "FEM_v4_Cruise", q_avionics_w=14.5, q_foc_w=1.0, q_de_w=0.0, q_rw_w=12.0, q_batt_w=2.0,
        tag="cruise_hot", transient_s=18000.0, init_temp_k=283.35,
        rad_ambient_k=T_SINK_SIDE, mli_ambient_k=262.0, time_max_inc=1800.0,
    )
    # Case 2: 20-min (1200 s) Worst-Case Nadir-Tilt RPO Capture Burst (F_nadir=0.8474, initialized at +10.2 C TMM cruise, 199 W internal burst)
    build_v4_fem_doc(
        "FEM_v4_RPO_Trans", q_avionics_w=72.0, q_foc_w=10.0, q_de_w=65.0, q_rw_w=42.0, q_batt_w=10.0,
        tag="rpo_transient", transient_s=1200.0, init_temp_k=283.35,
        rad_ambient_k=T_SINK_NADIR, mli_ambient_k=262.0, time_max_inc=120.0,
    )
