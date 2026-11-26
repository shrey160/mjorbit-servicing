"""FreeCAD 1.1 + CalculiX ccx unit smoke test (1-node radiating Al plate).

Run inside FreeCAD via MCP `execute_code(..., include_screenshot=False)`.
Verifies SI-mm / tonne / s / N / mW / K unit consistency against the exact
analytic Stefan-Boltzmann steady solution before building full spacecraft FEMs.
"""
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import FreeCAD as App
import ObjectsFem
import Part
from femmesh import gmshtools
from femtools import ccxtools

from thermal_helpers import hand_calc_radiating_plate_k


def run_smoke_test() -> None:
    doc_name = "ThermalUnitSmokeTest"
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    # 100 x 100 x 10 mm Al-6061 plate; top 100x100 mm face (0.01 m^2) radiates at eps=0.80 to T_sink=200 K
    box = doc.addObject("Part::Feature", "RadiatorPlate")
    box.Shape = Part.makeBox(100.0, 100.0, 10.0)
    doc.recompute()

    analysis = ObjectsFem.makeAnalysis(doc, "SmokeAnalysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc)
    solver.AnalysisType = "thermomech"
    solver.ThermoMechType = "coupled"
    solver.ThermoMechSteadyState = True
    analysis.addObject(solver)

    mat = ObjectsFem.makeMaterialSolid(doc, "Mat_Al6061")
    mat.Material = {
        "Name": "Al6061",
        "Density": "2700 kg/m^3",
        "ThermalConductivity": "167 W/(m*K)",
        "SpecificHeat": "896 J/(kg*K)",
        "ThermalExpansionCoefficient": "2.3e-5 1/K",
        "YoungsModulus": "69 GPa",
        "PoissonRatio": "0.33",
    }
    mat.References = []  # Empty References = default fallback material
    analysis.addObject(mat)

    mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh.Shape = box
    mesh.CharacteristicLengthMax = 25.0
    mesh.ElementOrder = "1st"
    doc.recompute()
    err = gmshtools.GmshTools(mesh).create_mesh()
    assert not err, f"Gmsh failed: {err}"
    analysis.addObject(mesh)

    bottom_faces = [f"Face{i + 1}" for i, f in enumerate(box.Shape.Faces) if abs(f.CenterOfMass.z) < 0.1]
    top_faces = [f"Face{i + 1}" for i, f in enumerate(box.Shape.Faces) if abs(f.CenterOfMass.z - 10.0) < 0.1]
    assert len(bottom_faces) == 1 and len(top_faces) == 1

    cfix = ObjectsFem.makeConstraintFixed(doc, "Fix_Bottom")
    cfix.References = [(box, bottom_faces[0])]
    analysis.addObject(cfix)

    t_init = ObjectsFem.makeConstraintInitialTemperature(doc, "InitTemp")
    t_init.initialTemperature = 350.0
    analysis.addObject(t_init)

    hf = ObjectsFem.makeConstraintHeatflux(doc, "Rad_Top")
    hf.ConstraintType = "Radiation"
    hf.Emissivity = 0.80
    hf.AmbientTemp = 200.0
    hf.References = [(box, top_faces[0])]
    analysis.addObject(hf)

    bh = ObjectsFem.makeConstraintBodyHeatSource(doc, "Power_10W")
    bh.Mode = "Total Power"
    bh.TotalPower = "10.0 W"
    bh.References = [(box, "Solid1")]
    analysis.addObject(bh)

    doc.recompute()
    fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
    fea.setup_working_dir()
    fea.setup_ccx()
    fea.update_objects()
    err_pre = fea.check_prerequisites()
    assert not err_pre, f"Prerequisites failed: {err_pre}"
    fea.write_inp_file()
    ret = fea.ccx_run()
    assert ret == 0, f"CalculiX exited with {ret}"
    fea.load_results()
    doc.recompute()

    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")][-1]
    t_fem_k = sum(res.Temperature) / len(res.Temperature)
    t_analytic_k = hand_calc_radiating_plate_k(power_w=10.0, area_m2=0.01, eps=0.80, t_sink_k=200.0)
    diff_k = abs(t_fem_k - t_analytic_k)
    print(f"FREECAD CCX UNIT SMOKE TEST: T_fem={t_fem_k:.2f} K | T_analytic={t_analytic_k:.2f} K | |dT|={diff_k:.3f} K")
    assert diff_k < 1.0, f"Unit mismatch detected: |dT| = {diff_k:.3f} K"
    App.closeDocument(doc_name)
    print("PASS: FreeCAD 1.1 + CalculiX thermal units verified.")


if __name__ == "__main__":
    run_smoke_test()
