"""Multi-node transient orbital & hosted-payload thermal solver for v4.2 ESPA Servicer and Arm IOD Demo.

Recalibrated to aerospace thermal modeling methodology (ECSS-E-ST-31C / ECSS-E-HB-31-03A,
NASA MSFC TM-2001-211221 correlated 97.6 deg SSO albedo/OLR, NASA GSFC GOLD Rules <=75% heater
duty cycle cap, and Gilmore's Spacecraft Thermal Control Handbook for exact Earth view factors,
small-blanket MLI area-scaling degradation, and vacuum bolted joint contact conductance).

Compute baseline: NVIDIA Jetson Orin NX 16GB (25 W active mode, power-gated OFF in cruise/safe)
paired with a dedicated rad-hard supervisor OBC (4-6 W always on).

Models BOTH:
  1. Full Dual-Arm 15" ESPA Servicer (v4.2) — LEO 550 km SSO (Hot Dawn-Dusk side-viewing,
     Hot Worst-Case RPO Nadir-Tilt F_nadir=0.8473, and Cold Noon-Midnight 35.6-min eclipse) first,
     then GEO (Hot Solstice w/ 23.5 deg -Z solar grazing & Cold Equinox 72-min eclipse).
  2. Ultra-Shallow Hosted-Payload IOD Demo (arm_iod_demo) — LEO 550 km under BOTH Conductive Host Deck
     (G_host = 12 W/K, T_host in [-20, +45] degC) and Thermally Isolated Host (G_host = 0.10 W/K, <=4 W
     NASA TICD cap, with flush 0.1008 m^2 -Z OSR radiator patch).
"""
import math
import os
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

SIGMA = 5.670374419e-8  # W / (m^2 K^4)
T_SPACE = 2.7           # K

# Exact geometric 550 km LEO Earth view factors (Gilmore Ch. 2)
R_EARTH_KM = 6371.0
ALT_LEO_KM = 550.0
RHO_LEO = math.asin(R_EARTH_KM / (R_EARTH_KM + ALT_LEO_KM))  # 67.00 deg
F_EARTH_NADIR_LEO = math.sin(RHO_LEO) ** 2                   # 0.8473
F_EARTH_SIDE_LEO = (RHO_LEO - 0.5 * math.sin(2.0 * RHO_LEO)) / math.pi  # 0.2584

# Solar power ceiling (0.40 m^2 body-mounted GaAs per +-Y side, 29% eff, 0.85 pack, 0.88 EOL)
SOLAR_PEAK_ONE_SIDE_W = 1361.0 * 0.40 * 0.29 * 0.85 * 0.88   # 118.6 W peak sun-normal
SOLAR_LEO_ORBIT_AVG_W = SOLAR_PEAK_ONE_SIDE_W * 0.60 * 0.85 * 0.88  # ~53.2 W usable avg
SOLAR_GEO_ORBIT_AVG_W = SOLAR_PEAK_ONE_SIDE_W * 0.91 * 0.88         # ~95.0 W usable avg


def rk4_step(y: List[float], t: float, dt: float, rhs: Callable[[List[float], float], List[float]]) -> List[float]:
    k1 = rhs(y, t)
    k2 = rhs([yi + 0.5 * dt * ki for yi, ki in zip(y, k1)], t + 0.5 * dt)
    k3 = rhs([yi + 0.5 * dt * ki for yi, ki in zip(y, k2)], t + 0.5 * dt)
    k4 = rhs([yi + dt * ki for yi, ki in zip(y, k3)], t + dt)
    return [yi + (dt / 6.0) * (k1i + 2.0 * k2i + 2.0 * k3i + k4i) for yi, k1i, k2i, k3i, k4i in zip(y, k1, k2, k3, k4)]


@dataclass
class NodeLimit:
    name: str
    op_min_c: float
    op_max_c: float
    surv_min_c: float
    surv_max_c: float


V4_LIMITS: List[NodeLimit] = [
    NodeLimit("Bus_Struct_MLI",   -30.0, 60.0, -40.0, 70.0),
    NodeLimit("Rad_NZ_OSR",       -50.0, 60.0, -60.0, 70.0),
    NodeLimit("Avionics_Coldpl",  -20.0, 60.0, -30.0, 70.0),
    NodeLimit("FOC_14xElmo",      -40.0, 70.0, -40.0, 85.0),
    NodeLimit("RW12_Pyramid",     -20.0, 60.0, -40.0, 70.0),
    NodeLimit("Battery_LiIon",      0.0, 40.0, -20.0, 60.0),
    NodeLimit("PropTank_PEPT230",  10.0, 50.0,  10.0, 55.0),  # ADN saturation at -7C; keep >= +10C
    NodeLimit("Optics_DragonEye", -20.0, 40.0, -30.0, 55.0),
    NodeLimit("Optics_IRIS_ST",   -40.0, 50.0, -40.0, 60.0),
    NodeLimit("SolarPanels_PYNY",-100.0, 85.0,-110.0, 110.0),  # Triple-junction GaAs non-op eclipse qual -100C
    NodeLimit("RCS_CornerPods",   -10.0, 65.0, -20.0, 85.0),
    NodeLimit("Arm_Wrist_Mini45",  -5.0, 60.0, -20.0, 70.0),  # tightest external exposed node
]

IOD_LIMITS: List[NodeLimit] = [
    NodeLimit("Chassis_Al",         -30.0, 60.0, -40.0, 70.0),
    NodeLimit("Avionics_Orin_FOC",  -25.0, 70.0, -40.0, 85.0),
    NodeLimit("PDU_RearBay",        -30.0, 60.0, -40.0, 70.0),
    NodeLimit("Rad_IOD_NZ_OSR",     -50.0, 60.0, -60.0, 70.0),
    NodeLimit("Arm_Wrist_Mini45",    -5.0, 60.0, -20.0, 70.0),
    NodeLimit("DummyPlate_Booms",   -60.0, 85.0, -80.0, 100.0),
]


def solve_v4_case(
    case_name: str,
    orbit_period_s: float,
    eclipse_s: float,
    solar_flux: float,
    albedo: float,
    earth_ir: float,
    osr_area: float,
    osr_alpha: float,
    osr_eps: float,
    pgs_cond: float,
    hot_burst: bool,
    n_orbits: int = 8,
    geo_solstice_rad_sun: bool = False,
    rad_nadir_during_burst: bool = False,
) -> Dict:
    """Simulate v4.2 12-node spacecraft over multiple orbits + optional 20-min RPO/capture & 5-min detumble."""
    N = 12
    C = [
        32000.0,  # 0: Bus_Struct_MLI (Al honeycomb shell + decks + bulkhead ~36 kg)
        2400.0,   # 1: Rad_NZ_OSR (OSR + 6mm Al spreader)
        5500.0,   # 2: Avionics_Coldpl (PCDU + rad-hard OBC + OrinNX + SBand)
        1400.0,   # 3: FOC_14xElmo
        17500.0,  # 4: RW12_Pyramid (4x RW-12 + truss)
        9000.0,   # 5: Battery_LiIon (9.0 kg pack)
        13500.0,  # 6: PropTank_PEPT230 (1.3kg Ti + 4.5kg LMP-103S)
        2900.0,   # 7: Optics_DragonEye (3.6 kg)
        1800.0,   # 8: Optics_IRIS_ST + STIM300
        3600.0,   # 9: SolarPanels_PY_NY (2x 0.40 m^2)
        2400.0,   # 10: RCS_CornerPods (8x pods)
        1200.0,   # 11: Arm_Wrist_Mini45 (exposed arm wrist/gripper assembly)
    ]

    G = [[0.0] * N for _ in range(N)]
    def link(i: int, j: int, val: float) -> None:
        G[i][j] += val
        G[j][i] += val

    link(0, 1, 7.5)        # Bus floor -> -Z OSR spreader
    link(1, 2, 18.0)       # Avionics bolted flush to -Z coldplate w/ vacuum Grafoil/Cho-Therm contact conductance
    link(0, 2, 3.0)
    link(1, 3, 14.0)       # FOC bank bolted to -Z coldplate
    link(0, 3, 2.0)
    link(0, 4, 8.5)        # RW-12 truss -> Lower/Upper decks
    link(1, 4, 4.5)        # Straps/web path to -Z spreader
    link(0, 5, 0.25)       # Battery on G10 thermal isolators
    link(0, 6, 0.30)       # PropTank on Ti-G10 saddle standoffs
    link(0, 7, 1.2)        # DragonEye bezel mounting to +Y pocket
    link(1, 7, pgs_cond)   # Dedicated PGS/CCHP strap: DragonEye -> -Z coldplate
    link(0, 8, 2.8)        # IRIS + ST + IMU optical bench -> structure
    link(0, 9, 0.35)       # Body-mounted solar panels isolated via Ti/G10 standoffs + back MLI
    link(0, 10, 0.35)      # RCS corner pods on Ti/G10 thermal standoffs
    link(0, 11, 0.14)      # Exposed arm q0->q5 conductive resistance through Ti clevis/CFRP links

    # Gilmore MLI effective emittance bounding (large bus vs small tubular wrist sleeve)
    A_mli = 2.05
    eps_mli_star_bus = 0.03 if hot_burst else 0.04
    alpha_mli_out = 0.30 if hot_burst else 0.22
    eps_mli_out = 0.65
    A_sp = 0.80
    alpha_sp_eff = 0.66
    eps_sp = 0.85
    eps_opt = 0.70
    A_rcs = 0.032
    eps_rcs = 0.65
    A_wrist_mli = 0.095
    eps_mli_star_wrist = 0.07 if hot_burst else 0.08  # Gilmore small-blanket area-scaling penalty
    A_wrist_bare = 0.008
    eps_wrist_bare = 0.45
    alpha_wrist_bare = 0.25

    R_cav = [(i, 0, 0.015 * SIGMA) for i in (2, 3, 4, 5, 6, 7, 8)]
    burst_start = (n_orbits - 1) * orbit_period_s + 0.15 * orbit_period_s if hot_burst else 1e18

    def env_flux_on_faces(t: float) -> Tuple[bool, Dict[int, float], float, float]:
        t_mod = t % orbit_period_s
        in_eclipse = (eclipse_s > 0.0) and (t_mod >= (orbit_period_s - eclipse_s))
        theta = 2.0 * math.pi * (t_mod / orbit_period_s)
        sun_flag = 0.0 if in_eclipse else 1.0

        vf_side = 0.0 if earth_ir < 1.0 else F_EARTH_SIDE_LEO
        in_maneuver = hot_burst and (t >= burst_start) and (t < burst_start + 1500.0)
        vf_rad_earth = F_EARTH_NADIR_LEO if (rad_nadir_during_burst and in_maneuver and earth_ir >= 1.0) else vf_side

        cos_sun_earth = max(0.0, math.cos(theta)) if sun_flag > 0 else 0.0
        q_alb_side = solar_flux * albedo * vf_side * cos_sun_earth
        q_ir_side = earth_ir * vf_side
        q_alb_rad = solar_flux * albedo * vf_rad_earth * cos_sun_earth
        q_ir_rad = earth_ir * vf_rad_earth

        q_abs_mli_bus = (
            sun_flag * solar_flux * (0.48 / A_mli) * alpha_mli_out
            + q_alb_side * 1.45 * alpha_mli_out
            + q_ir_side * 1.15 * eps_mli_out
        )
        T_mli_bus = (T_SPACE ** 4 + q_abs_mli_bus / (eps_mli_out * SIGMA)) ** 0.25

        q_abs_mli_wrist = (
            sun_flag * solar_flux * 0.24 * alpha_mli_out
            + q_alb_side * 1.35 * alpha_mli_out
            + q_ir_side * 1.15 * eps_mli_out
        )
        T_mli_wrist = (T_SPACE ** 4 + q_abs_mli_wrist / (eps_mli_out * SIGMA)) ** 0.25

        cos_rad_sun = math.sin(math.radians(23.5)) if (geo_solstice_rad_sun and hot_burst) else (0.03 if not geo_solstice_rad_sun else 0.0)
        q_ext = {i: 0.0 for i in range(N)}
        # 1: -Z OSR radiator using exact view factor F_side or F_nadir
        q_ext[1] = (sun_flag * solar_flux * cos_rad_sun * osr_alpha + q_alb_rad * osr_alpha + q_ir_rad * osr_eps) * osr_area
        # 7: DragonEye aperture (0.011 m^2)
        q_ext[7] = (sun_flag * solar_flux * 0.15 * 0.35 + q_alb_side * 1.2 * 0.35 + q_ir_side * 1.2 * eps_opt) * 0.011
        # 8: IRIS + Star trackers (0.015 m^2)
        q_ext[8] = (sun_flag * solar_flux * 0.20 * 0.35 + q_alb_side * 1.2 * 0.35 + q_ir_side * 1.2 * eps_opt) * 0.015
        # 9: +-Y Solar panels (~0.35 m^2 illuminated)
        q_ext[9] = sun_flag * solar_flux * 0.35 * alpha_sp_eff + q_alb_side * 1.3 * 0.40 * 0.78 + q_ir_side * 1.2 * 0.40 * eps_sp
        # 10: RCS pods
        q_ext[10] = (sun_flag * solar_flux * 0.30 * 0.40 + q_alb_side * 1.2 * 0.40 + q_ir_side * 1.2 * eps_rcs) * A_rcs
        # 11: Bare wrist jaws/FT bezel
        q_ext[11] = (sun_flag * solar_flux * 0.30 * alpha_wrist_bare + q_alb_side * 1.2 * alpha_wrist_bare + q_ir_side * 1.2 * eps_wrist_bare) * A_wrist_bare
        return in_eclipse, q_ext, T_mli_bus, T_mli_wrist

    htr_on = {"batt": False, "tank": False, "wrist": False, "opt": False, "rcs": False}
    htr_cap = {"batt": 25.0, "tank": 22.0, "wrist": 10.0, "opt": 20.0, "rcs": 16.0}

    def internal_and_heaters(y: List[float], t: float) -> Tuple[List[float], Dict[str, float], float]:
        q = [0.0] * N
        in_burst = hot_burst and (t >= burst_start) and (t < burst_start + 1200.0)
        in_detumble = hot_burst and (t >= burst_start + 1200.0) and (t < burst_start + 1500.0)

        # Thermostatted survival heaters (hysteresis loops in C)
        Tb, Tt, Tw, To, Tr = y[5] - 273.15, y[6] - 273.15, y[11] - 273.15, y[7] - 273.15, y[10] - 273.15
        if Tb < 10.0: htr_on["batt"] = True
        elif Tb > 14.0: htr_on["batt"] = False
        if Tt < 20.0: htr_on["tank"] = True
        elif Tt > 21.5: htr_on["tank"] = False
        if Tw < 6.0: htr_on["wrist"] = True
        elif Tw > 10.0: htr_on["wrist"] = False
        if To < -8.0: htr_on["opt"] = True
        elif To > -4.0: htr_on["opt"] = False
        if Tr < 1.0: htr_on["rcs"] = True
        elif Tr > 5.0: htr_on["rcs"] = False

        hp = {k: (htr_cap[k] if htr_on[k] else 0.0) for k in htr_cap}
        htr_sum = sum(hp.values())

        if in_burst:
            # 20-min RPO & Capture Burst (Orin NX 25W + rad-hard OBC 6W + S-Band 12W + PCDU 29W @ ~11%)
            q[2] = 72.0
            q[3] = 10.0   # 14x Elmo FOC
            q[4] = 42.0   # 4x RW-12 active attitude hold
            q[5] = 10.0   # Battery discharge I^2R
            q[7] = 65.0   # ASC DragonEye active
            q[8] = 4.5    # IRIS + ST + STIM300
            q[10] = 48.0  # 4x 1N HPGP catalyst beds (4x12 W staggered)
            q[0] = 8.0    # Shoulder/elbow slew joint heat into bulkhead/mounts
            q[11] = 3.5   # Wrist TBM2G Cu + stereo/LED
        elif in_detumble:
            # 5-min Wheels-Only Detumble
            q[2] = 58.0   # OBC(6) + Orin NX(20) + S-Band(12) + PCDU(20)
            q[3] = 2.0
            q[4] = 105.0  # 4x RW-12 high-torque momentum absorption
            q[5] = 12.0
            q[7] = 2.0
            q[8] = 3.0
            q[11] = 0.5
        elif hot_burst:
            # Nominal Cruise / Recharge (Orin NX power-gated OFF! 32.5 W total -> positive solar margin)
            q[2] = 14.5   # OBC(5) + S-Band RX/beacon(5) + OrinNX(0 OFF) + PCDU(4.5 @ ~12%)
            q[3] = 1.0
            q[4] = 12.0   # 4x 3.0 W steady wheel bias
            q[5] = 2.0
            q[8] = 3.0    # 2x ST + STIM300
        else:
            # Cold Safe / Survival (Orin NX OFF, dynamic 12% PCDU loss on heater bus)
            q[2] = 4.0 + 3.0 + 3.5 + 0.12 * htr_sum  # OBC(4) + S-Band RX(3) + PCDU base(3.5) + 12% of heaters
            q[4] = 7.2    # 4x 1.8 W low idle
            q[5] = 1.2
            q[8] = 2.5

        base_load_w = sum(q)
        q[5] += hp["batt"]
        q[6] += hp["tank"]
        q[11] += hp["wrist"]
        q[7] += hp["opt"]
        q[10] += hp["rcs"]
        return q, hp, base_load_w

    def rhs(y: List[float], t: float) -> List[float]:
        _, q_ext, T_mli_bus, T_mli_wrist = env_flux_on_faces(t)
        q_int, _, _ = internal_and_heaters(y, t)
        dT = [0.0] * N
        rad_ae = [
            0.0, osr_area * osr_eps, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.011 * eps_opt, 0.015 * eps_opt, A_sp * eps_sp, A_rcs * eps_rcs, A_wrist_bare * eps_wrist_bare,
        ]
        for i in range(N):
            net = q_int[i] + q_ext[i] - rad_ae[i] * SIGMA * (y[i] ** 4 - T_SPACE ** 4)
            for j in range(N):
                if G[i][j] > 0.0:
                    net += G[i][j] * (y[j] - y[i])
            dT[i] = net
        dT[0] += A_mli * eps_mli_star_bus * SIGMA * (T_mli_bus ** 4 - y[0] ** 4)
        dT[11] += A_wrist_mli * eps_mli_star_wrist * SIGMA * (T_mli_wrist ** 4 - y[11] ** 4)
        for i, j, rij in R_cav:
            ex = rij * (y[j] ** 4 - y[i] ** 4)
            dT[i] += ex
            dT[j] -= ex
        return [dT[i] / C[i] for i in range(N)]

    y = [290.0] * N
    dt = 10.0 if orbit_period_s < 10000.0 else 20.0
    total_s = n_orbits * orbit_period_s
    t = 0.0
    record_start = (n_orbits - 1) * orbit_period_s
    t_hist, y_hist = [], []
    htr_energy_j = {k: 0.0 for k in htr_cap}
    htr_peak_w = 0.0
    base_energy_j = 0.0
    last_orbit_secs = 0.0
    cruise_temps_c: List[float] = []

    while t < total_s - 1e-9:
        step = min(dt, total_s - t)
        if t >= record_start:
            _, hp, base_w = internal_and_heaters(y, t)
            for k, pw in hp.items():
                htr_energy_j[k] += pw * step
            p_tot = sum(hp.values())
            htr_peak_w = max(htr_peak_w, p_tot)
            base_energy_j += base_w * step
            last_orbit_secs += step
            t_hist.append((t - record_start) / 60.0)
            y_hist.append([yi - 273.15 for yi in y])
            if hot_burst and t < burst_start:
                cruise_temps_c.append(y[0] - 273.15)
        y = rk4_step(y, t, step, rhs)
        t += step
    t_hist.append((t - record_start) / 60.0)
    y_hist.append([yi - 273.15 for yi in y])

    mins = [min(row[i] for row in y_hist) for i in range(N)]
    maxs = [max(row[i] for row in y_hist) for i in range(N)]
    htr_avg_by_zone = {k: htr_energy_j[k] / max(1.0, last_orbit_secs) for k in htr_cap}
    htr_duty = {k: 100.0 * htr_avg_by_zone[k] / htr_cap[k] for k in htr_cap}
    return {
        "case": case_name,
        "mins_c": mins,
        "maxs_c": maxs,
        "htr_avg_w": sum(htr_avg_by_zone.values()),
        "htr_peak_w": htr_peak_w,
        "htr_duty": htr_duty,
        "base_avg_w": base_energy_j / max(1.0, last_orbit_secs),
        "total_avg_w": (base_energy_j + sum(htr_energy_j.values())) / max(1.0, last_orbit_secs),
        "pre_burst_bus_c": cruise_temps_c[-1] if cruise_temps_c else mins[0],
        "t_min": t_hist,
        "y_hist": y_hist,
    }


def solve_iod_case(
    case_name: str,
    host_temp_c: float,
    g_host_wk: float,
    hot_burst: bool,
    n_orbits: int = 8,
) -> Dict:
    """Simulate arm_iod_demo 6-node + fixed host-deck thermal network in 550 km LEO."""
    N = 6
    C = [
        11500.0,  # 0: Chassis_Al
        1800.0,   # 1: Avionics_Orin_FOC
        900.0,    # 2: PDU_RearBay
        500.0,    # 3: Rad_IOD_NZ_OSR (0.1008 m^2 flush pocket)
        1200.0,   # 4: Arm_Wrist_Mini45
        2800.0,   # 5: DummyPlate_Booms
    ]
    G = [[0.0] * N for _ in range(N)]
    def link(i: int, j: int, val: float) -> None:
        G[i][j] += val
        G[j][i] += val

    link(0, 1, 5.5)   # Bolted Orin/FOC flanges w/ vacuum TIM contact conductance
    link(0, 2, 4.2)   # PDU flanges to chassis
    link(0, 3, 5.5)   # -Z chassis pocket to flush OSR radiator
    link(0, 4, 0.14)  # Arm base roll -> distal Mini45 wrist
    link(0, 5, 0.20)  # Stowed/deployed CFRP booms conduction to plate

    T_host = host_temp_c + 273.15
    G_host_elec = 0.45 * g_host_wk
    G_host_chas = 0.55 * g_host_wk

    orbit_s = 5736.0
    eclipse_s = 0.0 if hot_burst else 2136.0
    S0 = 1418.0 if hot_burst else 1322.0
    albedo = 0.35 if hot_burst else 0.18
    eir = 265.0 if hot_burst else 206.0
    osr_area = 0.1008
    osr_alpha = 0.16 if hot_burst else 0.07
    osr_eps = 0.80
    eps_mli_chas = 0.035 if hot_burst else 0.045
    eps_mli_wrist = 0.07 if hot_burst else 0.08
    alpha_mli_out = 0.30 if hot_burst else 0.22

    htr_on = {"wrist": False, "elec": False}
    htr_cap = {"wrist": 10.0, "elec": 20.0}
    burst_start = (n_orbits - 1) * orbit_s + 0.15 * orbit_s if hot_burst else 1e18

    def iod_loads(y: List[float], t: float) -> Tuple[List[float], Dict[str, float]]:
        q = [0.0] * N
        in_burst = hot_burst and (t >= burst_start) and (t < burst_start + 900.0)
        if in_burst:
            # 15-min Active IOD Capture Burst (Orin NX 25W + carrier/OBC 5W + FOC/stereo 16W + PDU 8.5W + base 11W + wrist 3.5W)
            q[1] = 46.0
            q[2] = 8.5
            q[0] = 11.0
            q[4] = 3.5
        elif hot_burst:
            # Hot Orbit Idle (Orin NX OFF; supervisor MCU + RX + PDU)
            q[1] = 6.0
            q[2] = 2.0
        else:
            # Cold Survival
            q[1] = 4.5
            q[2] = 1.5

        Tw, Te = y[4] - 273.15, y[1] - 273.15
        if Tw < 6.0: htr_on["wrist"] = True
        elif Tw > 10.0: htr_on["wrist"] = False
        # Electronics survival band protects -25C op limit against -20C conductive or isolated cold host
        if Te < -21.0: htr_on["elec"] = True
        elif Te > -18.5: htr_on["elec"] = False

        hp = {k: (htr_cap[k] if htr_on[k] else 0.0) for k in htr_cap}
        q[4] += hp["wrist"]
        q[1] += hp["elec"]
        return q, hp

    def rhs(y: List[float], t: float) -> List[float]:
        t_mod = t % orbit_s
        in_ecl = (eclipse_s > 0.0) and (t_mod >= orbit_s - eclipse_s)
        sun = 0.0 if in_ecl else 1.0
        cos_alb = max(0.0, math.cos(2.0 * math.pi * t_mod / orbit_s)) if sun > 0 else 0.0
        q_alb = S0 * albedo * F_EARTH_SIDE_LEO * cos_alb
        q_ir = eir * F_EARTH_SIDE_LEO

        T_mli_chas = (T_SPACE ** 4 + (sun * S0 * 0.28 * alpha_mli_out + q_alb * 1.40 * alpha_mli_out + q_ir * 1.15 * 0.65) / (0.65 * SIGMA)) ** 0.25
        T_mli_wrist = (T_SPACE ** 4 + (sun * S0 * 0.24 * alpha_mli_out + q_alb * 1.35 * alpha_mli_out + q_ir * 1.15 * 0.65) / (0.65 * SIGMA)) ** 0.25

        q_int, _ = iod_loads(y, t)
        q_ext = [
            0.0,
            0.0,
            0.0,
            (sun * S0 * 0.03 * osr_alpha + q_alb * osr_alpha + q_ir * osr_eps) * osr_area,
            (sun * S0 * 0.30 * 0.25 + q_alb * 1.2 * 0.25 + q_ir * 1.2 * 0.45) * 0.008,
            sun * S0 * 0.14 * 0.20 + q_alb * 0.03 + q_ir * 0.035,
        ]
        rad_ae = [0.0, 0.0, 0.0, osr_area * osr_eps, 0.008 * 0.45, 0.42 * 0.12]

        dT = [0.0] * N
        for i in range(N):
            net = q_int[i] + q_ext[i] - rad_ae[i] * SIGMA * (y[i] ** 4 - T_SPACE ** 4)
            for j in range(N):
                if G[i][j] > 0.0:
                    net += G[i][j] * (y[j] - y[i])
            dT[i] = net
        dT[0] += 0.90 * eps_mli_chas * SIGMA * (T_mli_chas ** 4 - y[0] ** 4)
        dT[4] += 0.095 * eps_mli_wrist * SIGMA * (T_mli_wrist ** 4 - y[4] ** 4)
        dT[0] += G_host_chas * (T_host - y[0])
        dT[1] += G_host_elec * (T_host - y[1])
        return [dT[i] / C[i] for i in range(N)]

    y = [T_host] * N
    dt = 10.0
    total_s = n_orbits * orbit_s
    record_start = (n_orbits - 1) * orbit_s
    t = 0.0
    t_hist, y_hist = [], []
    htr_j = {k: 0.0 for k in htr_cap}
    htr_peak, rec_s = 0.0, 0.0
    idle_temps_c: List[float] = []

    while t < total_s - 1e-9:
        step = min(dt, total_s - t)
        if t >= record_start:
            _, hp = iod_loads(y, t)
            for k, pw in hp.items():
                htr_j[k] += pw * step
            p_tot = sum(hp.values())
            htr_peak = max(htr_peak, p_tot)
            rec_s += step
            t_hist.append((t - record_start) / 60.0)
            y_hist.append([yi - 273.15 for yi in y])
            if hot_burst and t < burst_start:
                idle_temps_c.append(y[0] - 273.15)
        y = rk4_step(y, t, step, rhs)
        t += step
    t_hist.append((t - record_start) / 60.0)
    y_hist.append([yi - 273.15 for yi in y])

    max_host_leak_w = max(
        abs(G_host_chas * (T_host - (row[0] + 273.15)) + G_host_elec * (T_host - (row[1] + 273.15)))
        for row in y_hist
    )
    htr_duty = {k: 100.0 * (htr_j[k] / max(1.0, rec_s)) / htr_cap[k] for k in htr_cap}
    return {
        "case": case_name,
        "mins_c": [min(row[i] for row in y_hist) for i in range(N)],
        "maxs_c": [max(row[i] for row in y_hist) for i in range(N)],
        "htr_avg_w": sum(htr_j.values()) / max(1.0, rec_s),
        "htr_peak_w": htr_peak,
        "htr_duty": htr_duty,
        "max_host_leak_w": max_host_leak_w,
        "pre_burst_chas_c": idle_temps_c[-1] if idle_temps_c else 0.0,
        "t_min": t_hist,
        "y_hist": y_hist,
    }


def print_table(title: str, limits: List[NodeLimit], hot_res: Dict, cold_res: Dict) -> int:
    print(f"\n=== {title} ===")
    print(
        f"Heater Power: Hot(avg={hot_res['htr_avg_w']:.1f}W, peak={hot_res['htr_peak_w']:.1f}W) | "
        f"Cold(avg={cold_res['htr_avg_w']:.1f}W, peak={cold_res['htr_peak_w']:.1f}W)"
    )
    if "htr_duty" in cold_res:
        duty_str = ", ".join(f"{k}={v:.1f}%" for k, v in cold_res["htr_duty"].items())
        print(f"Cold Heater Orbit-Avg Duty Cycles (GOLD cap <=75%): {duty_str}")
    print(f"{'Node':18s} | {'Limits(Op)':>12s} | {'Cold Min':>8s} | {'Hot Max':>8s} | {'Cold Marg':>9s} | {'Hot Marg':>8s} | Status")
    print("-" * 88)
    fails = 0
    for idx, lim in enumerate(limits):
        tmin = cold_res["mins_c"][idx]
        tmax = hot_res["maxs_c"][idx]
        m_cold = tmin - lim.op_min_c
        m_hot = lim.op_max_c - tmax
        if m_cold < 0.0 or m_hot < 0.0:
            st = "FAIL"
            fails += 1
        elif m_cold < 10.0 or m_hot < 10.0:
            st = "PASS(<10C)"
        else:
            st = "PASS(>=10C)"
        print(
            f"{lim.name:18s} | {lim.op_min_c:4.0f}..{lim.op_max_c:4.0f} C | "
            f"{tmin:6.1f} C | {tmax:6.1f} C | {m_cold:+7.1f} C | {m_hot:+6.1f} C | {st}"
        )
    return fails


def make_plots(v4_leo_nadir, v4_leo_cold, v4_geo_cold, iod_iso_hot) -> None:
    """Render clean 2x2 orbital transient chart grid directly via PIL."""
    from PIL import Image, ImageDraw, ImageFont

    out_dirs = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "docs", "media", "thermal_audit"),
    ]
    art_dir = os.environ.get("THERMAL_ART_DIR", "")
    if art_dir:
        out_dirs.append(art_dir)
    for d in out_dirs:
        os.makedirs(d, exist_ok=True)

    W, H = 1600, 1020
    im = Image.new("RGB", (W, H), (255, 255, 255))
    draw = ImageDraw.Draw(im)
    try:
        f_title = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 20)
        f_sub = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 15)
        f_lbl = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 13)
    except Exception:
        f_title = f_sub = f_lbl = ImageFont.load_default()

    draw.text(
        (34, 16),
        "Recalibrated Orbital & Hosted-Payload Thermal Audit (Orin NX 25W, Exact View Factors & Gilmore MLI)",
        fill=(20, 28, 40), font=f_title,
    )
    palette = [
        (31, 119, 180), (44, 160, 44), (214, 39, 40), (148, 103, 189),
        (255, 127, 14), (23, 190, 207), (140, 86, 75), (227, 119, 194),
    ]

    panels = [
        (0, 0, v4_leo_nadir, "v4.2 ESPA: LEO 550 km Hot (Nadir-Tilt RPO Burst F=0.8473 -> Detumble)",
         [(0, "Bus MLI"), (1, "-Z OSR Rad"), (2, "Avionics"), (5, "Battery"), (6, "PEPT-230"), (7, "DragonEye"), (11, "Wrist Mini45")]),
        (1, 0, v4_leo_cold, "v4.2 ESPA: LEO 550 km Cold Eclipse (35.6m Shadow, eps*_wrist=0.08)",
         [(0, "Bus MLI"), (1, "-Z OSR Rad"), (2, "Avionics"), (5, "Battery (+10C)"), (6, "PEPT-230 (+20C)"), (7, "DragonEye"), (11, "Wrist (+6C)")]),
        (0, 1, v4_geo_cold, "v4.2 ESPA: GEO Cold Equinox (72m Shadow, 9W Wrist Heater <=75%)",
         [(0, "Bus MLI"), (1, "-Z OSR Rad"), (2, "Avionics"), (5, "Battery (+10C)"), (6, "PEPT-230 (+20C)"), (7, "DragonEye"), (11, "Wrist (+6C)")]),
        (1, 1, iod_iso_hot, "Arm IOD Demo: LEO Hot Isolated Host (15m Orin NX Burst + 0.101m2 OSR)",
         [(0, "Chassis Al"), (1, "OrinNX+FOC"), (2, "PDU"), (3, "-Z OSR"), (4, "Wrist Mini45"), (5, "Target Plate")]),
    ]

    pw, ph = 740, 440
    for col, row_idx, res, ttl, curves in panels:
        ox = 42 + col * 780
        oy = 58 + row_idx * 480
        x0, y0, x1, y1 = ox + 52, oy + 36, ox + pw - 16, oy + ph - 48
        draw.rectangle([ox, oy, ox + pw, oy + ph], outline=(215, 222, 232), width=1)
        draw.text((ox + 16, oy + 8), ttl, fill=(25, 35, 50), font=f_sub)

        t_vals = res["t_min"]
        tmax = max(1e-6, t_vals[-1])
        all_y = [r[i] for r in res["y_hist"] for i, _ in curves]
        ymin = math.floor(min(all_y) / 10.0) * 10.0 - 5.0
        ymax = math.ceil(max(all_y) / 10.0) * 10.0 + 10.0

        for step_y in range(int(math.ceil(ymin / 10) * 10), int(ymax) + 1, 10):
            py = y1 - (step_y - ymin) / (ymax - ymin) * (y1 - y0)
            if y0 <= py <= y1:
                draw.line([(x0, py), (x1, py)], fill=(235, 239, 244), width=1)
                draw.text((x0 - 42, py - 7), f"{step_y:3d}C", fill=(90, 98, 110), font=f_lbl)
        n_xt = 5
        for k in range(n_xt + 1):
            tv = tmax * k / n_xt
            px = x0 + (tv / tmax) * (x1 - x0)
            draw.line([(px, y0), (px, y1)], fill=(240, 243, 247), width=1)
            draw.text((px - 12, y1 + 6), f"{tv:.0f}m", fill=(90, 98, 110), font=f_lbl)

        draw.rectangle([x0, y0, x1, y1], outline=(140, 150, 165), width=1)
        for ci, (idx, _) in enumerate(curves):
            pts = [
                (x0 + (tv / tmax) * (x1 - x0), y1 - (r[idx] - ymin) / (ymax - ymin) * (y1 - y0))
                for tv, r in zip(t_vals, res["y_hist"])
            ]
            draw.line(pts, fill=palette[ci % len(palette)], width=3)

        lx, ly = x0, y1 + 24
        for ci, (_, lbl) in enumerate(curves):
            c = palette[ci % len(palette)]
            draw.line([(lx, ly + 7), (lx + 16, ly + 7)], fill=c, width=3)
            draw.text((lx + 20, ly), lbl, fill=(35, 42, 54), font=f_lbl)
            lx += 96 if len(lbl) < 11 else 112

    for d in out_dirs:
        p = os.path.join(d, "orbital_transient_profiles.png")
        im.save(p, format="PNG")
        print(f"SAVED PLOT: {p}")


def main() -> None:
    print("=" * 88)
    print("RECALIBRATED MULTI-NODE ORBITAL & HOSTED-PAYLOAD THERMAL AUDIT (ORIN NX 25W BASELINE)")
    print(f"550 km LEO Exact Earth View Factors: F_nadir = {F_EARTH_NADIR_LEO:.4f}, F_side = {F_EARTH_SIDE_LEO:.4f}")
    print(f"Body-Mounted GaAs Solar Ceiling (0.40 m^2): Peak = {SOLAR_PEAK_ONE_SIDE_W:.1f} W | LEO Avg = {SOLAR_LEO_ORBIT_AVG_W:.1f} W | GEO Avg = {SOLAR_GEO_ORBIT_AVG_W:.1f} W")
    print("=" * 88)

    # Radiator trade + Nadir-Tilt RPO sensitivity
    for rad_area, label in ((0.2592, "0.259 m2 (540x480 mm) -Z OSR"), (0.1950, "0.195 m2 (520x375 mm) -Z OSR")):
        r_side = solve_v4_case(label + " SIDE", 5736.0, 0.0, 1418.0, 0.35, 265.0, rad_area, 0.16, 0.80, 5.5, True, n_orbits=8, rad_nadir_during_burst=False)
        r_nadir = solve_v4_case(label + " NADIR", 5736.0, 0.0, 1418.0, 0.35, 265.0, rad_area, 0.16, 0.80, 5.5, True, n_orbits=8, rad_nadir_during_burst=True)
        r_cold = solve_v4_case(label + " GEO COLD", 86400.0, 4320.0, 1322.0, 0.0, 0.0, rad_area, 0.07, 0.80, 5.5, False, n_orbits=4)
        print(f"\n>>> Radiator Trade: {label}, PGS Strap G = 5.5 W/K <<<")
        print(f"  LEO Hot Side-View  (F=0.2584): Rad={r_side['maxs_c'][1]:.1f}, Avionics={r_side['maxs_c'][2]:.1f} (<60), RW12={r_side['maxs_c'][4]:.1f}, DragonEye={r_side['maxs_c'][7]:.1f} (<40)")
        print(f"  LEO Hot Nadir-Tilt (F=0.8473): Rad={r_nadir['maxs_c'][1]:.1f}, Avionics={r_nadir['maxs_c'][2]:.1f} (<60), RW12={r_nadir['maxs_c'][4]:.1f}, DragonEye={r_nadir['maxs_c'][7]:.1f} (<40)")
        print(f"  GEO Cold Min (degC): Rad={r_cold['mins_c'][1]:.1f}, Avionics={r_cold['mins_c'][2]:.1f} (>-20), Wrist={r_cold['mins_c'][11]:.1f} (>-5), HtrAvg={r_cold['htr_avg_w']:.1f}W")

    RAD_AREA = 0.1950
    PGS_COND = 5.5
    v4_leo_hot_side = solve_v4_case("LEO_550_Hot_Side", 5736.0, 0.0, 1418.0, 0.35, 265.0, RAD_AREA, 0.16, 0.80, PGS_COND, True, n_orbits=8, rad_nadir_during_burst=False)
    v4_leo_hot_nadir = solve_v4_case("LEO_550_Hot_NadirRPO", 5736.0, 0.0, 1418.0, 0.35, 265.0, RAD_AREA, 0.16, 0.80, PGS_COND, True, n_orbits=8, rad_nadir_during_burst=True)
    v4_leo_cold = solve_v4_case("LEO_550_Cold", 5736.0, 2136.0, 1322.0, 0.18, 206.0, RAD_AREA, 0.07, 0.80, PGS_COND, False, n_orbits=10)

    f_leo_s = print_table("v4.2 ESPA SERVICER — LEO 550 km SSO (HOT SIDE-VIEW vs COLD)", V4_LIMITS, v4_leo_hot_side, v4_leo_cold)
    f_leo_n = print_table("v4.2 ESPA SERVICER — LEO 550 km SSO (WORST NADIR-TILT RPO F=0.8473 vs COLD)", V4_LIMITS, v4_leo_hot_nadir, v4_leo_cold)

    v4_geo_hot = solve_v4_case("GEO_Solstice_Hot", 14400.0, 0.0, 1418.0, 0.0, 0.0, RAD_AREA, 0.16, 0.80, PGS_COND, True, n_orbits=6, geo_solstice_rad_sun=True)
    v4_geo_cold = solve_v4_case("GEO_Equinox_Cold", 86400.0, 4320.0, 1322.0, 0.0, 0.0, RAD_AREA, 0.07, 0.80, PGS_COND, False, n_orbits=4)
    f_geo = print_table("v4.2 ESPA SERVICER — GEO (HOT SOLSTICE vs 72-MIN EQUINOX ECLIPSE)", V4_LIMITS, v4_geo_hot, v4_geo_cold)

    # Power & Energy Balance Summary (v4.2)
    cruise_w = 32.5  # OBC(5)+SBand(5)+PCDU(4.5)+FOC(1)+RW(12)+Batt(2)+Opt(3)
    burst_wh = 263.0 * (20.0 / 60.0) + 182.5 * (5.0 / 60.0)
    dod_pct = 100.0 * burst_wh / 1000.0
    recharge_margin_w = SOLAR_LEO_ORBIT_AVG_W - cruise_w
    recharge_orbits = (burst_wh / 0.90) / (recharge_margin_w * (5736.0 / 3600.0))
    tj_v4_nadir = v4_leo_hot_nadir["maxs_c"][2] + 0.7 * 25.0  # R_theta_JC+TIM ~ 0.7 C/W @ 25 W Orin NX
    print("\n=== v4.2 POWER, ENERGY & ORIN NX JUNCTION VERIFICATION ===")
    print(f"  LEO Cruise Load (Orin NX OFF): {cruise_w:.1f} W vs Solar Avg {SOLAR_LEO_ORBIT_AVG_W:.1f} W -> Margin = +{recharge_margin_w:.1f} W (+{100*recharge_margin_w/cruise_w:.0f}%)")
    print(f"  20m RPO (263.0 W) + 5m Detumble (182.5 W) = {burst_wh:.1f} Wh ({dod_pct:.1f}% of 1000 Wh Battery DoD)")
    print(f"  LEO Recharge Recovery Time: {recharge_orbits:.1f} orbits ({recharge_orbits*95.6/60.0:.1f} hr)")
    print(f"  LEO Cold Survival Load (Total Avg w/ Heaters): {v4_leo_cold['total_avg_w']:.1f} W")
    print(f"  GEO Cold Survival Load (Total Avg w/ Heaters): {v4_geo_cold['total_avg_w']:.1f} W (vs {SOLAR_GEO_ORBIT_AVG_W:.1f} W GEO Solar Avg)")
    print(f"  Orin NX Junction Est. (Nadir RPO Peak): T_coldpl={v4_leo_hot_nadir['maxs_c'][2]:.1f} C -> T_j ~ {tj_v4_nadir:.1f} C (<= 99.0 C throttle!)")
    print(f"  1-Way TMM->FEM Init Boundary: v4 LEO Hot Pre-Burst Bus = {v4_leo_hot_nadir['pre_burst_bus_c']:.1f} C | GEO Cold Bulkhead = {v4_geo_cold['mins_c'][0]:.1f} C")

    # Arm IOD Demo
    iod_cond_hot = solve_iod_case("IOD_Cond_Hot", host_temp_c=45.0, g_host_wk=12.0, hot_burst=True)
    iod_cond_cold = solve_iod_case("IOD_Cond_Cold", host_temp_c=-20.0, g_host_wk=12.0, hot_burst=False)
    f_iod_c = print_table("ARM IOD DEMO — CONDUCTIVE HOST DECK (+45C / -20C, G=12 W/K)", IOD_LIMITS, iod_cond_hot, iod_cond_cold)

    iod_iso_hot = solve_iod_case("IOD_Iso_Hot", host_temp_c=40.0, g_host_wk=0.10, hot_burst=True)
    iod_iso_cold = solve_iod_case("IOD_Iso_Cold", host_temp_c=-20.0, g_host_wk=0.10, hot_burst=False)
    f_iod_i = print_table("ARM IOD DEMO — ISOLATED HOST TICD (G=0.10 W/K + 0.101m2 OSR)", IOD_LIMITS, iod_iso_hot, iod_iso_cold)
    print(f"  Isolated Host Peak Conductive Leak = {max(iod_iso_hot['max_host_leak_w'], iod_iso_cold['max_host_leak_w']):.2f} W (<= 4.00 W NASA TICD!)")
    print(f"  IOD Conductive Orin NX T_j ~ {iod_cond_hot['maxs_c'][1] + 0.7*25.0:.1f} C (<= 99.0 C!) | Iso Pre-Burst Idle = {iod_iso_hot['pre_burst_chas_c']:.1f} C")

    # NASA GSFC GOLD Rules heater duty cycle <= 75% check
    all_duties = {
        **{f"v4_LEO_Cold_{k}": v for k, v in v4_leo_cold["htr_duty"].items()},
        **{f"v4_GEO_Cold_{k}": v for k, v in v4_geo_cold["htr_duty"].items()},
        **{f"IOD_Cond_Cold_{k}": v for k, v in iod_cond_cold["htr_duty"].items()},
        **{f"IOD_Iso_Cold_{k}": v for k, v in iod_iso_cold["htr_duty"].items()},
    }
    max_duty_name, max_duty_val = max(all_duties.items(), key=lambda kv: kv[1])
    print(f"\nMAX HEATER DUTY CYCLE ACROSS ALL COLD CASES: {max_duty_name} = {max_duty_val:.1f}% (GOLD limit <= 75.0%)")

    make_plots(v4_leo_hot_nadir, v4_leo_cold, v4_geo_cold, iod_iso_hot)
    total_fails = f_leo_s + f_leo_n + f_geo + f_iod_c + f_iod_i
    print(f"\nTOTAL HARD-LIMIT FAILURES ACROSS ALL CASES = {total_fails}")
    assert total_fails == 0, f"{total_fails} hard temperature limit violations!"
    assert max_duty_val <= 75.0, f"Heater duty cycle exceeded 75% GOLD cap: {max_duty_name}={max_duty_val:.1f}%"


if __name__ == "__main__":
    main()
