# Aerospace Orbital Thermal Constants & Reference Limits

## 1. Orbital Environment Bounding (NASA MSFC TM-2001-211221 / ECSS-E-ST-31C)

| Regime | Case | Solar `S` (`W/m²`) | Albedo | Earth OLR (`W/m²`) | Eclipse | Notes |
|---|---|---:|---:|---:|---|---|
| **LEO 550 km SSO (97.6°)** | Hot (Dawn-Dusk) | `1418` | `0.35` | `265` | `0 min` | Period `5736 s`; EOL coatings |
| **LEO 550 km SSO (97.6°)** | Cold (Noon-Midnight) | `1322` | `0.18` | `206` | `35.6 min` (`2136 s`) | BOL coatings; min internal loads |
| **GEO** | Hot Solstice | `1418` | `0.00` | `0` | `0 min` | `23.5°` solar grazing on `±Z` radiators |
| **GEO** | Cold Equinox | `1322` | `0.00` | `0` | `72 min` (`4320 s`) | Maximum eclipse soak |

Exact `550 km` LEO Earth view factors (`R_E = 6371 km`):
- `F_nadir = 0.8474` (`sin² ρ`, `ρ = 67.00°`)
- `F_side = 0.2578` (`(ρ - 0.5 sin 2ρ) / π`)

---

## 2. Thermo-Optical Coatings & Gilmore Small-Blanket MLI

| Surface Finish | `alpha_BOL` | `alpha_EOL` | `eps_IR` | Use Case |
|---|---:|---:|---:|---|
| **OSR (Optical Solar Reflector)** | `0.07` | `0.16` | `0.80` | Primary anti-sun/anti-nadir radiators |
| **Z-93 White Thermal Paint** | `0.14` | `0.24–0.30` | `0.88–0.91` | Secondary radiators (higher UV degradation) |
| **Gold Kapton MLI (Outer Foil)** | `0.22` | `0.30` | `0.65` | Exterior insulated bus & appendage sleeves |
| **Bare Anodized / Machined Al** | `0.20–0.25` | `0.30` | `0.12–0.45` | Exposed end-effectors (low-eps bare jaws ~`0.45`) |

Gilmore MLI effective emittance (`eps*`) area-scaling:
- Large uninterrupted blanket (`> 1 m²`): `eps* = 0.02–0.03` (use `0.03` hot / `0.04` cold).
- Medium chassis blanket (`0.5–1.0 m²`): `eps* = 0.035` hot / `0.045` cold.
- Small sleeve / tubular appendage / propellant line (`< 0.2 m²`): **`eps* = 0.07` hot / `0.08` cold** (seam & edge conductance penalty).

---

## 3. Vacuum Contact Conductance (`h_c`) & Interface Conductivities

| Interface / Material | Typical Value | Notes |
|---|---|---|
| Bare dry bolted Al-Al in vacuum | `150–400 W/(m²·K)` | Avoid high-power dissipation across dry joints |
| **Cho-Therm** elastomeric pad | `600–1,200 W/(m²·K)` | Electrically isolating; use on coldplate boxes |
| **Grafoil** compressed graphite | `1,500–3,000 W/(m²·K)` | High in-plane & through conductance (conductive) |
| **PGS** (Pyrolytic Graphite Sheet) | `k = 1,000–1,500 W/(m·K)` (in-plane) | Very thin (`0.1–1 mm`); check cross-section `A/L`! |
| **CCHP** (Al/Ammonia Heat Pipe) | `k_eff = 3,500–6,000 W/(m·K)` | Use for `> 30 W` over `> 200 mm` runs |
| **G10 / Ti-6Al-4V** thermal isolator | `k = 0.35 / 7.0 W/(m·K)` | Isolate Li-ion battery & prop tank (`0.25–0.35 W/K`) |

---

## 4. Typical Spacecraft Component Thermal Limits & Rules

| Subsystem | Operating (`°C`) | Survival (`°C`) | Rule / Margin Policy |
|---|---|---|---|
| **Li-Ion Battery** | `0 .. +40` | `-20 .. +60` | Hold `10–14 °C` w/ G10 isolators |
| **ADN Green Propellant (`LMP-103S`)** | `+10 .. +50` | `+10 .. +55` | Strictly `>= +10 °C` (salt saturation at `-7 °C`) |
| **F/T Sensor (`ATI Mini45`)** | `-5 .. +60` | `-20 .. +70` | Minimize `ΔT` during capture (`< 2–3 °C`) |
| **Flash LiDAR (`DragonEye` class)** | `-20 .. +40` | `-30 .. +55` | Check 3D hotspot + CCHP/saddle path |
| **Jetson Orin NX / Thor** | `-25 .. +99 T_j` (`T_TP <= 75–80`) | `-40 .. +85` | Add `~0.7 °C/W` junction-to-coldplate rise |
| **Triple-Junction GaAs Solar** | `-100 .. +85` | `-110 .. +110` | Isolate from bus with G10/Ti standoffs |

- **ECSS / NASA Uncertainty Margin:** Target **`±10 °C` (`ECSS`) / `±11 °C` (`MIL-STD-1540`)** margin prior to TVAC correlation.
- **NASA GSFC GOLD Rules Heater Authority:** Orbit-average survival heater duty cycle **`<= 75%`** in worst-case cold eclipse (`>= 25%` reserve).
