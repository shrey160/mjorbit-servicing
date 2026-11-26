"""Minimal 3-node LEO 550 km SSO orbital transient thermal template.

Demonstrates Steps 0-2 of thermal-sims:
  - Solar ceiling & 11% PCDU loss check
  - Exact 550 km Earth view factors (F_side and F_nadir tilt)
  - Outer-foil MLI equilibrium + internal eps* transfer (no greenhouse trap)
  - Hysteresis survival heater + GOLD <=75% duty check & ECSS >=10 C margin check
"""
import math
from pathlib import Path
import sys

SKILL_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SKILL_ROOT / "scripts"))

from thermal_helpers import (  # noqa: E402
    SIGMA,
    T_SPACE,
    check_ecss_margins,
    check_gold_duty_cycles,
    earth_view_factors,
    mli_heat_transfer_w,
    mli_outer_temp_k,
    pcdu_heat_loss_w,
    rk4_step,
    solar_power_ceiling,
)


def run_case(hot_nadir_rpo: bool, rad_area_m2: float = 0.18, n_orbits: int = 8) -> dict:
    _, f_nadir, f_side = earth_view_factors(550.0)
    orbit_s = 5736.0
    eclipse_s = 0.0 if hot_nadir_rpo else 2136.0
    s0 = 1418.0 if hot_nadir_rpo else 1322.0
    albedo = 0.35 if hot_nadir_rpo else 0.18
    eir = 265.0 if hot_nadir_rpo else 206.0
    osr_alpha = 0.16 if hot_nadir_rpo else 0.07
    osr_eps = 0.80
    eps_star_bus = 0.03 if hot_nadir_rpo else 0.04
    alpha_mli_out = 0.30 if hot_nadir_rpo else 0.22
    a_mli = 1.60

    # Nodes: 0 = Bus_MLI, 1 = Radiator_OSR, 2 = Avionics_Coldplate
    C = [24000.0, 2000.0, 4500.0]
    G01, G12, G02 = 7.5, 16.0, 2.5
    htr_on = False
    htr_cap = 35.0
    burst_t0 = (n_orbits - 1) * orbit_s + 0.15 * orbit_s if hot_nadir_rpo else 1e18

    def loads(y, t):
        nonlocal htr_on
        in_burst = hot_nadir_rpo and (burst_t0 <= t < burst_t0 + 1200.0)
        if y[2] - 273.15 < -11.0:
            htr_on = True
        elif y[2] - 273.15 > -8.0:
            htr_on = False
        hp = htr_cap if htr_on else 0.0

        if in_burst:
            elec = 120.0
            q_av = elec + pcdu_heat_loss_w(elec)
            return [20.0, 0.0, q_av], hp
        elif hot_nadir_rpo:
            # Cruise: high-power GPU OFF (26 W delivered + 3.2 W PCDU)
            return [4.0, 0.0, 22.0 + pcdu_heat_loss_w(26.0)], hp
        else:
            elec_safe = 18.0 + hp
            return [2.0, 0.0, 16.0 + pcdu_heat_loss_w(elec_safe) + hp], hp

    def rhs(y, t):
        t_mod = t % orbit_s
        in_ecl = eclipse_s > 0.0 and t_mod >= (orbit_s - eclipse_s)
        sun = 0.0 if in_ecl else 1.0
        cos_alb = max(0.0, math.cos(2.0 * math.pi * t_mod / orbit_s)) if sun > 0 else 0.0

        vf_rad = f_nadir if (hot_nadir_rpo and burst_t0 <= t < burst_t0 + 1200.0) else f_side
        q_alb_rad = s0 * albedo * vf_rad * cos_alb
        q_ir_rad = eir * vf_rad
        q_rad_ext = (sun * s0 * 0.03 * osr_alpha + q_alb_rad * osr_alpha + q_ir_rad * osr_eps) * rad_area_m2

        t_mli = mli_outer_temp_k(
            sun * s0 * 0.25 * alpha_mli_out,
            s0 * albedo * f_side * cos_alb * alpha_mli_out,
            eir * f_side * 0.65,
        )
        q_int, _ = loads(y, t)
        d0 = q_int[0] + mli_heat_transfer_w(a_mli, eps_star_bus, t_mli, y[0]) + G01 * (y[1] - y[0]) + G02 * (y[2] - y[0])
        d1 = q_int[1] + q_rad_ext - rad_area_m2 * osr_eps * SIGMA * (y[1] ** 4 - T_SPACE ** 4) + G01 * (y[0] - y[1]) + G12 * (y[2] - y[1])
        d2 = q_int[2] + G02 * (y[0] - y[2]) + G12 * (y[1] - y[2])
        return [d0 / C[0], d1 / C[1], d2 / C[2]]

    y = [285.0, 285.0, 285.0]
    dt, t = 10.0, 0.0
    t_end = n_orbits * orbit_s
    t_rec = (n_orbits - 1) * orbit_s
    hist, htr_j, rec_s = [], 0.0, 0.0
    while t < t_end - 1e-9:
        step = min(dt, t_end - t)
        if t >= t_rec:
            _, hp = loads(y, t)
            htr_j += hp * step
            rec_s += step
            hist.append([yi - 273.15 for yi in y])
        y = rk4_step(y, t, step, rhs)
        t += step
    hist.append([yi - 273.15 for yi in y])
    return {
        "mins_c": [min(r[i] for r in hist) for i in range(3)],
        "maxs_c": [max(r[i] for r in hist) for i in range(3)],
        "duty_pct": 100.0 * (htr_j / max(1.0, rec_s)) / htr_cap,
    }


def main() -> None:
    p_peak, p_avg = solar_power_ceiling(0.40)
    print(f"Step 0 Solar Ceiling (0.40 m^2 GaAs): Peak={p_peak:.1f} W | LEO Avg={p_avg:.1f} W")
    hot = run_case(hot_nadir_rpo=True)
    cold = run_case(hot_nadir_rpo=False)
    fails, rows = check_ecss_margins(
        ["Bus_MLI", "Radiator_OSR", "Avionics_Coldplate"],
        [-30.0, -50.0, -20.0], [60.0, 60.0, 60.0],
        cold["mins_c"], hot["maxs_c"],
    )
    ok_duty, duty_msg = check_gold_duty_cycles({"Avionics_Htr": cold["duty_pct"]})
    for r in rows:
        print(f"  {r['name']:18s}: ColdMin={r['cold_min']:5.1f} C | HotMax={r['hot_max']:5.1f} C | {r['status']}")
    print(f"GOLD Duty Check: {duty_msg} | Pass={ok_duty} | Fails={fails}")
    assert fails == 0 and ok_duty
    print("PASS: Minimal orbital transient TMM template verified.")


if __name__ == "__main__":
    main()
