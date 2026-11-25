"""Servicer rev 2 actuation (SPEC_7 D7): 16 on/off 1 N nozzles with allocation, and 4 wheels in a pyramid.

Thrusters (`Rcs`): the nozzle table of models/servicer_rev_2/servicer_rev2_bus.json (position, force direction
in the chaser frame). `B(ref)` is the 6 x 16 map from nozzle thrusts to the body wrench (force, torque about
`ref`). `allocate(w, ref)` returns duty cycles in [0, 1] (bounded least squares, small effort weight) that
best make the body wrench `w`; `enabled` masks nozzles off (failure, plume keep-out). `max_pure(axis, ref)`
is the largest pure wrench along one axis (linear program). `Pulses` turns duty cycles into on/off pulses
with a minimum on-time (minimum impulse bit, impulse accumulation) and per-nozzle thrust scale and
misalignment errors drawn from a seed; it also counts propellant (Isp).

Wheels (`Wheels`): 4 wheels with spin axes A (3 x 4, chaser frame), momentum limit h_max and torque limit
tau_max each. `command(tau_body)` gives wheel torques for a wanted body torque (minimum norm, clipped to the
torque limit, no torque that pushes a saturated wheel further), updates the wheel momenta and returns the
torque on the body: -A tau_w (reaction) - omega x (A h) (gyroscopic). The wheels are not MuJoCo bodies (qpos
layout unchanged); apply the returned body torque as an external torque on the chaser.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import linprog, lsq_linear

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import ASSETS_DIR  # noqa: E402

BUS_JSON = ASSETS_DIR / "servicer_rev2" / "servicer_rev2_bus.json"
G0 = 9.80665


def load_bus(path=BUS_JSON):
    return json.loads(Path(path).read_text())


class Rcs:
    def __init__(self, bus=None):
        bus = bus or load_bus()
        self.names = [n["name"] for n in bus["nozzles"]]
        self.P = np.array([n["pos_m"] for n in bus["nozzles"]], float)       # (16, 3)
        self.D = np.array([n["force_dir"] for n in bus["nozzles"]], float)   # (16, 3) force directions
        self.F = float(bus["thrust_n"])
        self.isp = float(bus["isp_s"])
        self.n = len(self.names)

    def B(self, ref=(0.0, 0.0, 0.0)):
        r = self.P - np.asarray(ref, float)
        return self.F * np.vstack([self.D.T, np.cross(r, self.D).T])         # (6, 16), per unit duty

    def allocate(self, w, ref=(0.0, 0.0, 0.0), enabled=None, effort=1e-3):
        """Duty cycles u in [0, 1] minimising |B u - w|^2 + effort |u|^2 (bounded least squares)."""
        B = self.B(ref)
        ub = np.ones(self.n) if enabled is None else np.where(enabled, 1.0, 0.0)
        A = np.vstack([B, np.sqrt(effort) * np.eye(self.n)])
        b = np.concatenate([np.asarray(w, float), np.zeros(self.n)])
        return lsq_linear(A, b, bounds=(np.zeros(self.n), ub + 1e-12), method="bvls").x.clip(0, 1)

    def max_pure(self, axis, ref=(0.0, 0.0, 0.0), sign=1.0, enabled=None):
        """Largest t with B u = t e_axis (e: unit wrench direction), 0 <= u <= 1."""
        B = self.B(ref)
        e = np.zeros(6)
        e[axis] = sign
        ub = np.ones(self.n) if enabled is None else np.where(enabled, 1.0, 0.0)
        res = linprog(-np.r_[np.zeros(self.n), 1.0], A_eq=np.c_[B, -e], b_eq=np.zeros(6),
                      bounds=[(0, u) for u in ub] + [(0, None)], method="highs")
        return float(res.x[-1]) if res.success else 0.0

    def mdot(self, u):
        """Propellant flow [kg/s] for duty cycles u."""
        return float(np.sum(u)) * self.F / (self.isp * G0)


class Pulses:
    """On/off firing with a minimum on-time (pulse-frequency modulation by impulse accumulation): each nozzle
    accumulates its commanded duty x dt; when the accumulated on-time reaches half of dt_min it fires one pulse of
    dt_min (back-to-back pulses = continuous firing), so the mean thrust follows the command with the minimum
    impulse bit as granularity. dt_min 50 ms: ~50 mN s at 1 N, a conservative HPGP-class minimum impulse bit
    (unverified). Per-nozzle thrust scale 1 + N(0, scale_sd) and misalignment N(0, mis_deg) about a random axis
    perpendicular to the nominal direction, from `seed` (the misaligned directions go into the scene's sites)."""

    def __init__(self, rcs: Rcs, dt_min=0.05, scale_sd=0.05 / 2, mis_deg=1.0, seed=0):
        self.rcs, self.dt_min = rcs, dt_min
        rng = np.random.default_rng([seed, 7701])
        self.scale = 1.0 + rng.normal(0, scale_sd, rcs.n)
        D = rcs.D.copy()
        for i in range(rcs.n):
            k = np.cross(D[i], rng.normal(size=3))
            k /= np.linalg.norm(k)
            a = np.radians(rng.normal(0, mis_deg))
            D[i] = D[i] * np.cos(a) + np.cross(k, D[i]) * np.sin(a)  # rotate about k (k _|_ D)
        self.D_true = D
        self.acc = np.zeros(rcs.n)      # accumulated commanded on-time not yet fired [s]
        self.left = np.zeros(rcs.n)     # remaining on-time of the current pulse [s]
        self.prop_kg = 0.0
        self.pulses = 0

    def step(self, u, dt):
        """Per-nozzle thrust [N] (true magnitudes) for the next dt, given duty cycles u in [0, 1]."""
        self.acc += np.clip(u, 0, 1) * dt
        start = (self.left <= 1e-12) & (self.acc >= 0.5 * self.dt_min)
        self.left[start] = self.dt_min
        self.pulses += int(start.sum())
        on = self.left > 1e-12
        self.left[on] -= dt
        self.acc[on] -= dt
        f = np.where(on, self.rcs.F * self.scale, 0.0)
        self.prop_kg += f.sum() * dt / (self.rcs.isp * G0)
        return f

    def wrench(self, f, ref=(0.0, 0.0, 0.0)):
        r = self.rcs.P - np.asarray(ref, float)
        F = f[:, None] * self.D_true
        return F.sum(0), np.cross(r, F).sum(0)


class Wheels:
    def __init__(self, bus=None):
        bus = bus or load_bus()
        self.A = np.array([w["axis"] for w in bus["wheels"]], float).T   # (3, 4)
        self.h_max = float(bus["wheel_h_nms"])
        self.tau_max = float(bus["wheel_tau_nm"])
        self.h = np.zeros(self.A.shape[1])
        self.Apinv = np.linalg.pinv(self.A)

    def command(self, tau_body, omega_body, dt, failed=()):
        """Wheel torques for the wanted body torque; returns the body torque actually applied (chaser frame)."""
        tw = -self.Apinv @ np.asarray(tau_body, float)           # body torque = -A tw
        tw = np.clip(tw, -self.tau_max, self.tau_max)
        sat = np.abs(self.h) >= self.h_max - 1e-9
        tw[sat & (np.sign(tw) == np.sign(self.h))] = 0.0
        for i in failed:
            tw[i] = 0.0
        self.h = np.clip(self.h + tw * dt, -self.h_max, self.h_max)
        H = self.A @ self.h
        return -self.A @ tw - np.cross(omega_body, H), tw

    def H(self):
        return self.A @ self.h

    def fill(self):
        return float(np.max(np.abs(self.h)) / self.h_max)

    def envelope(self, direction, failed=()):
        """Largest momentum |H| storable along a body direction (each |h_i| <= h_max, H parallel)."""
        e = np.asarray(direction, float) / np.linalg.norm(direction)
        n = self.A.shape[1]
        ub = [(0.0, 0.0) if i in failed else (-self.h_max, self.h_max) for i in range(n)]
        res = linprog(-np.r_[np.zeros(n), 1.0], A_eq=np.c_[self.A, -e], b_eq=np.zeros(3),
                      bounds=ub + [(0, None)], method="highs")
        return float(res.x[-1]) if res.success else 0.0

    def torque_envelope(self, direction, failed=()):
        """Largest body torque along a direction from the wheels (each |tau_i| <= tau_max)."""
        return self.envelope(direction, failed) * self.tau_max / self.h_max
