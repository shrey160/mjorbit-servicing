"""Export v4.1 Dual-Arm ESPA FreeCAD Bus meshes (in metres) and dual_arm_espa_bus.json for MuJoCo."""
import glob
import json
import math
import os

import FreeCAD as App
import MeshPart
import Part

CAD_DIR = r"c:/Shrey_Projs/robotics/mjorbit-servicing/cad/concepts/dual_arm_espa"
OUT_DIR = r"c:/Shrey_Projs/robotics/mjorbit-servicing/experiments/01_dual_arm_espa/assets"
PARTS_DIR = os.path.join(OUT_DIR, "parts")


def export_stl(shape, path_stl, lin_defl=0.5, ang_defl=0.35):
    me = MeshPart.meshFromShape(Shape=shape, LinearDeflection=lin_defl, AngularDeflection=ang_defl)
    me.transform(App.Matrix(0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0))
    me.write(path_stl)
    return os.path.getsize(path_stl)


def build_bus_json():
    hx_f, hx_a = 0.319, -0.324
    y_lat, z_lat = 0.245, 0.410
    cf, ca = math.radians(55.0), math.radians(30.0)
    nozzles = []

    fwd_specs = [
        ("f_pypz", +1, +1, (0.0, -1.0, 0.0)),
        ("f_pynz", +1, -1, (0.0, 0.0, +1.0)),
        ("f_nypz", -1, +1, (0.0, 0.0, -1.0)),
        ("f_nynz", -1, -1, (0.0, +1.0, 0.0)),
    ]
    for idx, (cname, sy, sz, sdir) in enumerate(fwd_specs):
        p_cant = [hx_f, sy * y_lat, sz * z_lat]
        d_cant = [-math.cos(cf), -sy * math.sin(cf) / math.sqrt(2.0), -sz * math.sin(cf) / math.sqrt(2.0)]
        nozzles.append({"name": f"thr_{2*idx:02d}", "pos_m": p_cant, "force_dir": [round(x, 6) for x in d_cant], "cluster": cname})
        p_side = [
            hx_f - 0.005,
            sy * (y_lat + (0.03 if sdir[1] != 0 else 0.0)),
            sz * (z_lat + (0.03 if sdir[2] != 0 else 0.0)),
        ]
        nozzles.append({"name": f"thr_{2*idx+1:02d}", "pos_m": p_side, "force_dir": list(sdir), "cluster": cname})

    aft_specs = [
        ("a_pypz", +1, +1, (0.0, 0.0, -1.0)),
        ("a_pynz", +1, -1, (0.0, -1.0, 0.0)),
        ("a_nypz", -1, +1, (0.0, +1.0, 0.0)),
        ("a_nynz", -1, -1, (0.0, 0.0, +1.0)),
    ]
    for idx, (cname, sy, sz, sdir) in enumerate(aft_specs):
        p_cant = [hx_a, sy * y_lat, sz * z_lat]
        d_cant = [+math.cos(ca), -sy * math.sin(ca) / math.sqrt(2.0), -sz * math.sin(ca) / math.sqrt(2.0)]
        nozzles.append({"name": f"thr_{8+2*idx:02d}", "pos_m": p_cant, "force_dir": [round(x, 6) for x in d_cant], "cluster": cname})
        p_side = [
            hx_a + 0.006,
            sy * (y_lat + (0.03 if sdir[1] != 0 else 0.0)),
            sz * (z_lat + (0.03 if sdir[2] != 0 else 0.0)),
        ]
        nozzles.append({"name": f"thr_{8+2*idx+1:02d}", "pos_m": p_side, "force_dir": list(sdir), "cluster": cname})

    beta = math.radians(65.0)
    wheels = []
    for k in range(4):
        ang = math.radians(45.0 + 90.0 * k)
        w_dir = [math.cos(beta), math.sin(beta) * math.cos(ang), math.sin(beta) * math.sin(ang)]
        w_ctr = [-0.095, 0.115 * math.cos(ang), -0.155 + 0.115 * math.sin(ang)]
        wheels.append({
            "name": f"wheel_{k}",
            "pos_m": [round(x, 6) for x in w_ctr],
            "axis": [round(x, 6) for x in w_dir],
        })

    # Bus collision boxes (metres): main core rear/mid (up to X=0.185) + 20 mm central spine + +Y/+Z cheeks
    # Note: arm cavities open into X in [0.185, 0.320] at Y in [-0.130, -0.010] and [+0.010, +0.130], Z in [-0.380, +0.380].
    # Modeling the bus collision as non-overlapping boxes preserves real cavity clearance with bus_col!
    collision_boxes = [
        {"name": "bus_col", "pos": [-0.0675, 0.0, 0.0], "size": [0.2525, 0.30, 0.455]},       # X in [-0.320, +0.185]
        {"name": "bus_sep_ring", "pos": [-0.3411, 0.0, 0.0], "size": [0.0211, 0.205, 0.205]},  # MLB15.000
        {"name": "bus_spine", "pos": [0.2525, 0.0, 0.0], "size": [0.0675, 0.010, 0.380]},      # 20mm central spine
        {"name": "bus_cheek_py", "pos": [0.2525, 0.215, 0.0], "size": [0.0675, 0.085, 0.380]}, # +Y cheek outside cavity
        {"name": "bus_cheek_ny", "pos": [0.2525, -0.215, 0.0], "size": [0.0675, 0.085, 0.380]},# -Y cheek outside cavity
        {"name": "bus_cap_pz", "pos": [0.2525, 0.0, 0.4175], "size": [0.0675, 0.30, 0.0375]},  # +Z top cap
        {"name": "bus_cap_nz", "pos": [0.2525, 0.0, -0.4175], "size": [0.0675, 0.30, 0.0375]}, # -Z bot cap
        {"name": "opt_bezel", "pos": [0.326, 0.221, -0.035], "size": [0.006, 0.063, 0.085]},   # proud +Y bezel
    ]

    return {
        "name": "dual_arm_espa_v4_2",
        "frame": "chaser body (bus core centre, +X to client, Z up), metres",
        "mga": 1.17,
        "mass_kg": 174.6,  # + 16.5 kg dual 6-DOF arms = 191.1 kg total wet mass (Spec-MGA)
        "com_m": [-0.025, 0.0, 0.0],
        "inertia_kgm2": [[22.5, 0.0, 0.0], [0.0, 21.8, 0.0], [0.0, 0.0, 12.6]],
        "bbox_m": [-0.3622, -0.303, -0.459, 0.3421, 0.303, 0.4784],
        "port_face_x_m": -0.3622,
        "collision_boxes": collision_boxes,
        "nozzles": nozzles,
        "thrust_n": 1.0,
        "isp_s": 220.0,
        "cant_deg": 30.0,
        "cant_fwd_deg": 55.0,
        "wheels": wheels,
        "wheel_h_nms": 12.0,
        "wheel_tau_nm": 0.2,
        "wheel_beta_deg": 65.0,
        "parts": [
            {"name": "bus_shell_gold", "rgba": [0.80, 0.62, 0.22, 1.0]},
            {"name": "bus_dark_trim", "rgba": [0.20, 0.21, 0.24, 1.0]},
            {"name": "bus_metal_trim", "rgba": [0.68, 0.70, 0.75, 1.0]},
            {"name": "bus_solar", "rgba": [0.10, 0.16, 0.42, 1.0]},
            {"name": "bus_radiator", "rgba": [0.88, 0.89, 0.90, 1.0]},
            {"name": "bus_hdrm_red", "rgba": [0.78, 0.22, 0.18, 1.0]},
            {"name": "bus_lens_cyan", "rgba": [0.18, 0.56, 0.86, 1.0]},
        ],
    }


def main():
    os.makedirs(PARTS_DIR, exist_ok=True)
    doc_name = "DualArmServicerConcept"
    if doc_name in App.listDocuments():
        doc = App.getDocument(doc_name)
    else:
        doc = App.openDocument(os.path.join(CAD_DIR, "dual_arm_concept.FCStd"))

    groups = {
        "bus_shell_gold": ["BusShell_GoldMLI"],
        "bus_dark_trim": [
            "FixedBaseRoll_Arm1_Drive", "FixedBaseRoll_Arm2_Drive",
            "Liner_Arm1_PY", "Liner_Arm2_NY",
            "Recessed_DragonEye_Bezel", "Recessed_DragonEye_Window",
            "Recessed_IRIS_CamBody", "Recessed_IRIS_Baffle",
            "StarTracker_PY", "StarTracker_NY", "RCS_CornerClusters_8x_55deg",
        ],
        "bus_metal_trim": [
            "FixedBaseRoll_Arm1_Flange", "FixedBaseRoll_Arm2_Flange",
            "HDRM_CupCone_Arm1", "HDRM_CupCone_Arm2",
            "Recessed_DragonEye_Body", "SeparationRing_MLB15",
        ],
        "bus_solar": ["SolarPanel_PY", "SolarPanel_NY"],
        "bus_radiator": ["Radiator_NZ"],
        "bus_hdrm_red": [
            "FixedBaseRoll_Arm1_Encoder", "FixedBaseRoll_Arm2_Encoder",
            "HDRM_NEA9102_Arm1", "HDRM_NEA9102_Arm2",
        ],
        "bus_lens_cyan": ["Recessed_IRIS_Lens"],
    }

    sizes = {}
    for stl_name, obj_names in groups.items():
        shapes = [doc.getObject(nm).Shape for nm in obj_names]
        comp = Part.makeCompound(shapes)
        out_path = os.path.join(PARTS_DIR, f"{stl_name}.stl")
        sizes[stl_name] = export_stl(comp, out_path)

    bus_json_path = os.path.join(OUT_DIR, "dual_arm_espa_bus.json")
    with open(bus_json_path, "w", encoding="utf-8") as fh:
        json.dump(build_bus_json(), fh, indent=1)

    for bak in glob.glob(os.path.join(CAD_DIR, "*.FCBak")):
        try:
            os.remove(bak)
        except OSError:
            pass

    tot_kb = sum(sizes.values()) / 1024.0
    print(f"Exported {len(sizes)} STLs ({tot_kb:.1f} KB total) + {bus_json_path}")
    for k, v in sizes.items():
        print(f"  {k}.stl: {v/1024:.1f} KB")


if __name__ == "__main__":
    main()
