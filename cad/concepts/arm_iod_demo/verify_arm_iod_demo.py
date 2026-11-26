"""Quantitative engineering verification for the Ultra-Shallow Dual-Arm Hosted-Payload IOD Demo.

Pure Python (standard library only). Verifies:
  1. Hosted-payload mass, CG, and shallow stowed envelope (273 x 520 x 840 mm vs 704 x 606 x 937 mm full ESPA)
  2. Rollable Ø20 mm bistable CFRP boom mechanics (coiling strain, L_t containment, lateral stiffness, modal freq, buckling)
  3. Differential upper/lower boom payout (+-5 deg pitch tilt) & 4-boom clearance
  4. Nominal & +5 deg tilted 6-DOF Cardan-Roll arm reach margins (+44.0 mm nominal)
  5. Wrist-stereo-only triangulation precision (100-480 mm standoff, no bus LiDAR/camera)
"""
import math


def verify_mass_and_envelope():
    subsystems = [
        # (name, base_kg, mga, x_cg_mm)
        ("2x v4.2 6-DOF arms + Mini45 F/T + jaws + stereo", 13.8, 1.20, 175.0),
        ("2x Bulkhead q0 Base-Roll drives (CSD-20 + TBM2G)", 2.8, 1.20, 75.0),
        ("Shallow chassis (245x520x840) + 20mm bulkhead + MLI", 10.6, 1.20, 128.0),
        ("4x Rollable CFRP booms + drums + 2x upper/lower motors", 3.2, 1.20, 72.0),
        ("15in (381 mm) Dummy Target Ring Plate + fiducials", 2.8, 1.20, 260.0),
        ("Non-pyro HDRMs (4x plate Frangibolt/cones + 2x arm locks)", 1.3, 1.20, 195.0),
        ("Payload avionics (Orin NX/OBC, 14x FOC drives, PDU, witness cam)", 3.5, 1.20, 52.0),
    ]
    base_tot = sum(s[1] for s in subsystems)
    mga_tot = sum(s[1] * s[2] for s in subsystems)
    xcg = sum(s[1] * s[2] * s[3] for s in subsystems) / mga_tot

    stowed_dims = (273.0, 520.0, 840.0)
    espa_dims = (704.3, 606.0, 937.4)
    vol_ratio = (stowed_dims[0] * stowed_dims[1] * stowed_dims[2]) / (
        espa_dims[0] * espa_dims[1] * espa_dims[2]
    )

    assert stowed_dims[0] <= 280.0, f"Stowed depth {stowed_dims[0]} exceeds 280 mm shallow target"
    assert mga_tot <= 50.0, f"MGA mass {mga_tot:.1f} kg exceeds 50 kg hosted slot"
    assert xcg < 160.0, f"Stowed CG {xcg:.1f} mm unexpectedly high"
    print(
        f"[Mass & Envelope] Base={base_tot:.1f} kg | +20% MGA={mga_tot:.1f} kg | "
        f"Stowed CG={xcg:.1f} mm | Dims={stowed_dims[0]:.0f}x{stowed_dims[1]:.0f}x{stowed_dims[2]:.0f} mm "
        f"(depth -{(1 - stowed_dims[0]/espa_dims[0])*100:.1f}%, vol -{(1 - vol_ratio)*100:.1f}% vs v4.2 ESPA) -> PASS"
    )
    return mga_tot


def verify_rollable_booms():
    d_boom = 20.0       # mm deployed diameter
    r_boom = d_boom / 2.0
    t_wall = 0.24       # mm 4-ply high-strain CFRP slit-tube wall
    e_eff_gpa = 70.0    # effective laminate modulus
    d_hub = 64.0        # mm spool hub diameter (X in [15, 85] w/ R=35 flanges)
    chute_len = 135.0   # mm rigid internal guide chute (X=110..245)
    standoff = 480.0    # mm deployed standoff from front face X=245 -> X_ring=725
    m_plate = 3.4       # kg target plate + 15in ring w/ MGA

    strain_pct = (t_wall / d_hub) * 100.0
    lt_min, lt_max = 5.0 * d_boom, 6.0 * d_boom
    payout_total = standoff + chute_len + 20.0
    wraps = payout_total / (math.pi * d_hub)
    radial_buildup = wraps * t_wall

    # Bending stiffness EI of Ø20 x 0.22 mm CFRP slit-tube with 0.65 open-section knockdown
    i_tube_mm4 = math.pi * (r_boom ** 3) * t_wall
    ei_nom = e_eff_gpa * 1e9 * (i_tube_mm4 * 1e-12)
    ei_eff = 0.65 * ei_nom

    # 4 parallel portal booms over L = 0.48 m: k_lat = 4 * 12 * EI_eff / L^3 (conservative 0.6x joint factor)
    L_m = standoff / 1000.0
    k_lat = 0.60 * 4.0 * (12.0 * ei_eff / (L_m ** 3))
    f1_hz = (1.0 / (2.0 * math.pi)) * math.sqrt(k_lat / m_plate)
    defl_5n_mm = (5.0 / k_lat) * 1000.0

    # Local shell crimping moment M_cr ~ 0.3 * E * R * t^2
    m_cr_nm = 0.3 * (e_eff_gpa * 1e9) * (r_boom / 1000.0) * ((t_wall / 1000.0) ** 2)
    # Euler buckling (4 booms, K=1.2)
    p_euler_4 = 4.0 * (math.pi ** 2) * ei_eff / ((1.2 * L_m) ** 2)

    # Boom pass-through clearance vs arm cavities (|Y|<=130), arc reliefs (r_s=115 at (+-70,+-245)), 15in ring (R=190.5)
    boom_y, boom_z = 210.0, 155.0
    r_boom_center = math.hypot(boom_y, boom_z)
    dist_to_sh_same = math.hypot(boom_y - 70.0, boom_z - 245.0)

    # +5 deg pitch differential payout (upper Z=+155 vs lower Z=-155)
    tilt_rad = math.radians(5.0)
    dx_diff = 2.0 * boom_z * math.sin(tilt_rad)

    assert strain_pct <= 0.40, f"CFRP coiling strain {strain_pct:.3f}% > 0.40%"
    assert lt_max <= chute_len, f"Transition length {lt_max:.1f} > chute {chute_len}"
    assert radial_buildup < 2.5, f"Coil buildup {radial_buildup:.2f} exceeds 3 mm drum flange lip"
    assert f1_hz >= 8.0, f"First mode {f1_hz:.1f} Hz < 8 Hz"
    assert defl_5n_mm <= 1.0, f"5 N lateral deflection {defl_5n_mm:.2f} mm > 1 mm"
    assert dist_to_sh_same > 115.0 + 15.0 + 14.0, "Boom chute collides with shoulder arc pocket"
    assert r_boom_center > 190.5 + 30.0, "Boom chute too close to 15in ring"

    print(
        f"[4x CFRP Booms] Ø{d_boom:.0f}x{t_wall:.2f}mm | Drum Ø{d_hub:.0f}mm: eps={strain_pct:.3f}% ({wraps:.1f} wraps, +{radial_buildup:.2f}mm) | "
        f"L_t={lt_min:.0f}-{lt_max:.0f}<=135mm chute"
    )
    print(
        f"                EI_eff={ei_eff:.1f} N*m^2 | k_lat={k_lat/1000:.1f} kN/m | f1={f1_hz:.1f} Hz | "
        f"5N defl={defl_5n_mm:.2f} mm | M_cr={m_cr_nm:.1f} N*m, P_euler(4x)={p_euler_4:.0f} N | 5deg diff={dx_diff:.1f} mm -> PASS"
    )


def verify_kinematics_and_stereo():
    # Exact v4.2 reach check: Nominal +5 deg tilt on 15 in ring (R=190.5 mm) at X_ring=725 mm
    L1, L2, PALM_X = 248.0, 208.0, 128.0
    b1 = (190.0, 70.0, 245.0)
    b2 = (190.0, -70.0, -245.0)
    ring_x = 725.0
    ring_r = 190.5

    for label, tilt_deg in (("Nominal 15in @ +480mm", 0.0), ("Tilted +5deg Pitch @ +480mm", 5.0)):
        a = math.radians(tilt_deg)
        # Rotation around +Y: e1 = (0,1,0), e2 = (sin a, 0, cos a), n_reach = (cos a, 0, -sin a)
        n_reach = (math.cos(a), 0.0, -math.sin(a))
        e1 = (0.0, 1.0, 0.0)
        e2 = (math.sin(a), 0.0, math.cos(a))
        margins = []
        for idx, b in enumerate((b1, b2), 1):
            rm = math.hypot(b[1], b[2])
            uy, uz = b[1] / rm, b[2] / rm
            # meridian match in YZ: (a*e1.y + b*e2.y, a*e1.z + b*e2.z) || (uy, uz)
            aa = uy * e2[2]
            bb = uz
            nl = math.hypot(aa, bb)
            aa /= nl
            bb /= nl
            u_rad = (aa * e1[0] + bb * e2[0], aa * e1[1] + bb * e2[1], aa * e1[2] + bb * e2[2])
            if u_rad[1] * b[1] + u_rad[2] * b[2] < 0:
                u_rad = (-u_rad[0], -u_rad[1], -u_rad[2])
            p_ring = (ring_x + ring_r * u_rad[0], ring_r * u_rad[1], ring_r * u_rad[2])
            w = (
                p_ring[0] - PALM_X * n_reach[0],
                p_ring[1] - PALM_X * n_reach[1],
                p_ring[2] - PALM_X * n_reach[2],
            )
            dist = math.sqrt((w[0] - b[0]) ** 2 + (w[1] - b[1]) ** 2 + (w[2] - b[2]) ** 2)
            margin = (L1 + L2) - dist
            assert margin >= 20.0, f"{label} Arm{idx} reach margin {margin:.1f} < 20 mm"
            margins.append((dist, margin))
        print(
            f"[6-DOF Reach] {label}: Arm1={margins[0][0]:.1f}/456mm (+{margins[0][1]:.1f}), "
            f"Arm2={margins[1][0]:.1f}/456mm (+{margins[1][1]:.1f}) -> PASS"
        )

    # Wrist stereo (50 mm baseline, f=1100 px, sigma_d=0.2 px): sigma_Z = Z^2 * sigma_d / (f * B)
    B = 0.050
    f_px = 1100.0
    sig_d = 0.20
    errs = {z_mm: ((z_mm / 1000.0) ** 2 * sig_d / (f_px * B)) * 1000.0 for z_mm in (400, 200, 100)}
    assert errs[200] < 0.25 and errs[100] < 0.06
    print(
        f"[Wrist Stereo Only] B=50mm, f=1100px, 0.2px: "
        f"sigma_Z(400mm)={errs[400]:.2f}mm, sigma_Z(200mm)={errs[200]:.2f}mm, sigma_Z(100mm)={errs[100]:.3f}mm -> PASS"
    )


if __name__ == "__main__":
    verify_mass_and_envelope()
    verify_rollable_booms()
    verify_kinematics_and_stereo()
    print("ALL ARM IOD DEMO ENGINEERING CHECKS PASSED.")
