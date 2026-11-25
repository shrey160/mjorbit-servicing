"""The client spin the servicer can hold station on (SPEC_7 s26.4): thrust and wheel limits, from the bus data.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/spin_limit.py [--f-axis 1.5] [--eta 0.85]

Rerun it when the thrusters, the bus mass / inertia, the wheels or the hold distances change: it reads them from the
scene and the bus file (servicer/actuation.py), so its numbers follow the design.

Physics (station-keeping on a client spinning at w, rigid, torque-free; the bus holds a point fixed to the client):
  translation: the station point p_d moves on a circle about the spin axis through the client COM, radius
               r_perp = |rr - (rr . w^) w^| (rr = station point - client COM). Holding it needs the centripetal
               acceleration  a_c = w^2 r_perp  (ring_mission.station's a_ff = w x (w x rr)); moving along the station
               line at V_APP adds the Coriolis term  a_cor = 2 w x v  (<= 2 w V_APP); a nutating client adds
               w_dot x rr (small at these rates; the torque-free w_dot = I^-1 (-w x I w)).
               Available: a_max = F_axis / m per axis (F_MAX, the law's clip; the 16 x 1 N canted nozzles give more
               on some axes). Holding needs a_c + a_cor + a_ctrl <= eta a_max, eta < 1 for the control's own share
               (gusts of the PD on the estimate error, plume constraints). Worst case r_perp = |rr| (spin axis
               across the station line): w_max = sqrt(eta a_max / r_perp).
  attitude:    the bus turns with the client at w: the wheels store H = I_bus w (the 4 wheels give ~h_max along
               any axis) and see the gyroscopic torque w x (I w) and w x H; the wheel limit is
               w_max,att = h_cap / I_max (largest principal inertia, worst axis). Thruster unloading dumps the excess.
Units: SI; rates printed in deg/s.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
import _paths  # noqa: E402,F401


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--f-axis", type=float, default=None, help="force per axis, N (default: ring_mission.F_MAX)")
    ap.add_argument("--eta", type=float, default=0.85, help="share of the authority for the spin (rest: control)")
    a = ap.parse_args()
    sys.argv = sys.argv[:1]
    import ring_mission as RM
    import ring_fit
    M = RM.Mission()
    M.place_episode(7600000)
    m, I = M.bus_mass_inertia()
    F = a.f_axis or RM.F_MAX
    a_max = F / m
    com = M.com_true
    I_bus = np.linalg.eigvalsh(I)
    h_cap = M.wheels.h_max                    # per wheel; the pyramid gives about this along any body axis
    print(f"bus mass {m:.1f} kg, force per axis {F:g} N -> a_max {a_max:.5f} m/s^2; eta {a.eta}")
    print(f"bus principal inertia {np.round(I_bus, 1)} kg m^2, wheel h_max {h_cap:g} N m s, "
          f"V_APP {RM.V_APP} m/s")
    print(f"{'hold':>10s} {'s (m)':>6s} {'|rr| (m)':>8s} {'w_max thrust':>13s} {'w_max +Coriolis':>16s}")
    for name, s in (("hold 3.2", RM.S_HOLD3), ("hold 1.8", RM.S_HOLD18), ("station", RM.S_STATION)):
        rr = ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + s) - com
        r = float(np.linalg.norm(rr))
        w1 = math.sqrt(a.eta * a_max / r)
        # with the approach's Coriolis: w^2 r + 2 w V = eta a_max  ->  w = -V/r + sqrt((V/r)^2 + eta a_max / r)
        V = RM.V_APP
        w2 = -V / r + math.sqrt((V / r) ** 2 + a.eta * a_max / r)
        print(f"{name:>10s} {s:6.2f} {r:8.3f} {math.degrees(w1):10.2f} d/s {math.degrees(w2):13.2f} d/s")
    print(f"attitude (wheels): w_max = h_max / I_max = {math.degrees(h_cap / I_bus.max()):.2f} deg/s "
          f"(unloading by thrusters beyond)")
    for w_d in (1.0, 2.0, 2.5, 3.0, 4.0):
        w = math.radians(w_d)
        rr = ring_fit.C_B + ring_fit.N_B * (ring_fit.H_TOP + RM.S_HOLD3) - com
        print(f"  {w_d:3.1f} deg/s: a_c at the 3.2 m hold {w * w * np.linalg.norm(rr):.5f} m/s^2 "
              f"({100 * w * w * np.linalg.norm(rr) / a_max:.0f} % of a_max), wheel H {I_bus.max() * w:.2f} N m s")
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
