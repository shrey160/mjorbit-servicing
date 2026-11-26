"""Pure-Python orbital thermal engineering primitives (ECSS / NASA GOLD / Gilmore).

Zero external dependencies. Safe for direct import in any TMM or FEM setup script.
"""
import math
from typing import Dict, List, Sequence, Tuple

SIGMA = 5.670374419e-8  # Stefan-Boltzmann constant, W / (m^2 * K^4)
T_SPACE = 2.7           # Deep space sink temperature, K
R_EARTH_KM = 6371.0     # Mean volumetric Earth radius, km


def earth_view_factors(alt_km: float) -> Tuple[float, float, float]:
    """Return (angular_radius_rad, F_nadir, F_side) for a flat plate at circular altitude alt_km.

    F_nadir: plate facing Earth nadir (normal along -r).
    F_side : plate perpendicular to nadir (e.g. zenith/anti-sun side panel).
    """
    rho = math.asin(R_EARTH_KM / (R_EARTH_KM + float(alt_km)))
    f_nadir = math.sin(rho) ** 2
    f_side = (rho - 0.5 * math.sin(2.0 * rho)) / math.pi
    return rho, f_nadir, f_side


def solar_power_ceiling(
    area_m2: float,
    cell_eff: float = 0.29,
    packing: float = 0.85,
    eol_deg: float = 0.88,
    sun_frac: float = 0.60,
    cos_avg: float = 0.85,
    batt_rt: float = 0.88,
    solar_const: float = 1361.0,
) -> Tuple[float, float]:
    """Return (peak_sun_normal_w, usable_orbit_avg_w) for body/wing solar array area."""
    p_peak = solar_const * area_m2 * cell_eff * packing * eol_deg
    p_avg = p_peak * sun_frac * cos_avg * batt_rt
    return p_peak, p_avg


def pcdu_heat_loss_w(delivered_load_w: float, efficiency: float = 0.89) -> float:
    """Return PCDU DC-DC conversion heat loss (W) for a given delivered electrical load."""
    assert 0.5 < efficiency < 1.0, f"Invalid efficiency {efficiency}"
    return delivered_load_w * (1.0 / efficiency - 1.0)


def mli_outer_temp_k(
    q_solar_abs_wm2: float,
    q_alb_abs_wm2: float,
    q_ir_abs_wm2: float,
    eps_out: float = 0.65,
) -> float:
    """Solve radiative equilibrium temperature (K) of outer MLI foil exposed to orbital fluxes."""
    q_tot = max(0.0, q_solar_abs_wm2 + q_alb_abs_wm2 + q_ir_abs_wm2)
    return (T_SPACE ** 4 + q_tot / (eps_out * SIGMA)) ** 0.25


def mli_heat_transfer_w(
    area_m2: float,
    eps_star: float,
    t_mli_out_k: float,
    t_node_k: float,
) -> float:
    """Net heat flow (W) from outer MLI foil into insulated node via effective emittance eps*."""
    return area_m2 * eps_star * SIGMA * (t_mli_out_k ** 4 - t_node_k ** 4)


def effective_sink_temp_k(q_abs_wm2: float, eps: float) -> float:
    """Convert absorbed orbital flux (W/m^2) into effective radiative sink temperature T_sink_eff (K)
    for use with CalculiX *RADIATE: q_net = eps * sigma * (T^4 - T_sink_eff^4).
    """
    return (T_SPACE ** 4 + max(0.0, q_abs_wm2) / (eps * SIGMA)) ** 0.25


def leo_osr_sink_k(
    vf_earth: float,
    cos_sun: float = 0.03,
    cos_alb: float = 0.70,
    solar_flux: float = 1418.0,
    albedo: float = 0.35,
    earth_ir: float = 265.0,
    alpha_osr: float = 0.16,
    eps_osr: float = 0.80,
) -> float:
    """Compute effective radiative sink T_sink_eff (K) for an OSR radiator in LEO."""
    q_abs = (
        solar_flux * cos_sun * alpha_osr
        + solar_flux * albedo * vf_earth * cos_alb * alpha_osr
        + earth_ir * vf_earth * eps_osr
    )
    return effective_sink_temp_k(q_abs, eps_osr)


def hand_calc_radiating_plate_k(power_w: float, area_m2: float, eps: float, t_sink_k: float) -> float:
    """Analytic steady-state temperature (K) of an isolated grey radiator rejecting power_w."""
    return (t_sink_k ** 4 + power_w / (area_m2 * eps * SIGMA)) ** 0.25


def rk4_step(y: Sequence[float], t: float, dt: float, rhs) -> List[float]:
    """Single 4th-order Runge-Kutta integration step."""
    k1 = rhs(y, t)
    k2 = rhs([yi + 0.5 * dt * ki for yi, ki in zip(y, k1)], t + 0.5 * dt)
    k3 = rhs([yi + 0.5 * dt * ki for yi, ki in zip(y, k2)], t + 0.5 * dt)
    k4 = rhs([yi + dt * ki for yi, ki in zip(y, k3)], t + dt)
    return [
        yi + (dt / 6.0) * (k1i + 2.0 * k2i + 2.0 * k3i + k4i)
        for yi, k1i, k2i, k3i, k4i in zip(y, k1, k2, k3, k4)
    ]


def check_gold_duty_cycles(duty_pct_by_zone: Dict[str, float], cap_pct: float = 75.0) -> Tuple[bool, str]:
    """Verify NASA GSFC GOLD Rules cold heater duty cycle limit (default <= 75%)."""
    if not duty_pct_by_zone:
        return True, "No active heaters"
    worst_zone, worst_val = max(duty_pct_by_zone.items(), key=lambda kv: kv[1])
    return worst_val <= cap_pct, f"{worst_zone}={worst_val:.1f}% (cap {cap_pct:.1f}%)"


def check_ecss_margins(
    names: Sequence[str],
    op_mins_c: Sequence[float],
    op_maxs_c: Sequence[float],
    cold_mins_c: Sequence[float],
    hot_maxs_c: Sequence[float],
    ecss_margin_c: float = 10.0,
) -> Tuple[int, List[Dict]]:
    """Evaluate hard operational limit violations and ECSS uncertainty margin status."""
    fails = 0
    rows = []
    for name, lmin, lmax, cmin, hmax in zip(names, op_mins_c, op_maxs_c, cold_mins_c, hot_maxs_c):
        mc = cmin - lmin
        mh = lmax - hmax
        if mc < 0.0 or mh < 0.0:
            status = "FAIL"
            fails += 1
        elif mc < ecss_margin_c or mh < ecss_margin_c:
            status = f"PASS(<{ecss_margin_c:.0f}C)"
        else:
            status = f"PASS(>={ecss_margin_c:.0f}C)"
        rows.append({"name": name, "cold_min": cmin, "hot_max": hmax, "cold_marg": mc, "hot_marg": mh, "status": status})
    return fails, rows
