"""Unified rev-2 RCS + reaction-wheel + arm/jaw actuation stepper over SimBackend."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable, Dict, Optional, Set, Tuple

import mujoco
import numpy as np

from specs.client_spec import ClientSpec
from specs.servicer_spec import ServicerSpec

from .backend import SimBackend

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _paths  # noqa: E402,F401
from actuation import Pulses, Rcs, Wheels  # noqa: E402


def build_bus_ids(km: mujoco.MjModel, servicer: ServicerSpec, client: ClientSpec, rcs: Rcs) -> Dict[str, object]:
    """Resolve MuJoCo body, geom, joint, equality, site, and actuator IDs from specs."""
    bid_c = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, servicer.body_name)
    bid_t = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, client.body_name)
    cq = int(km.jnt_qposadr[km.body_jntadr[bid_c]])
    tq = int(km.jnt_qposadr[km.body_jntadr[bid_t]])
    cv = int(km.jnt_dofadr[km.body_jntadr[bid_c]])
    tv = int(km.jnt_dofadr[km.body_jntadr[bid_t]])

    j_ids = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_JOINT, j) for j in servicer.arm.joint_names]
    arm_q = np.array([km.jnt_qposadr[j] for j in j_ids], dtype=int)
    arm_v = np.array([km.jnt_dofadr[j] for j in j_ids], dtype=int)
    act_arm = np.array([mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in servicer.arm.act_names], dtype=int)

    gq = np.array([km.jnt_qposadr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in servicer.tool.grip_joints], dtype=int)
    act_grip = np.array([mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in servicer.tool.grip_acts], dtype=int)
    eq_locks = [mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, e) for e in servicer.tool.lock_eqs]
    ee_site = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, servicer.tool.ee_site)

    act_name = lambda a: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
    noz = np.array([next(a for a in range(km.nu) if act_name(a) == "test_rcs_" + n[4:]) for n in rcs.names], dtype=int)
    rwm = np.array([next(a for a in range(km.nu) if act_name(a) == f"test_rwm_{ax}") for ax in "xyz"], dtype=int)

    root_c = int(km.body_rootid[bid_c])
    root_t = int(km.body_rootid[bid_t])
    srv_b = {b for b in range(km.nbody) if km.body_rootid[b] == root_c}
    cli_b = {b for b in range(km.nbody) if km.body_rootid[b] == root_t}
    tool_b = {mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, n) for n in servicer.tool.tool_bodies}
    cam_g = {mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_GEOM, g) for g in servicer.tool.cam_geoms}
    ring_g = {
        g for g in range(km.ngeom) if (mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("ring_")
    }
    arm_lim = km.actuator_ctrlrange[act_arm, 1].copy()

    return dict(
        bid_c=bid_c,
        bid_t=bid_t,
        cq=cq,
        tq=tq,
        cv=cv,
        tv=tv,
        arm_q=arm_q,
        arm_v=arm_v,
        act_arm=act_arm,
        arm_lim=arm_lim,
        gq=gq,
        act_grip=act_grip,
        eq_locks=eq_locks,
        ee_site=ee_site,
        noz=noz,
        rwm=rwm,
        srv_b=srv_b,
        cli_b=cli_b,
        tool_b=tool_b,
        cam_g=cam_g,
        ring_g=ring_g,
    )


class ActuatorBus:
    """Single source of truth for stepping rev-2 RCS, reaction wheels, unloading, plume & collision audit."""

    def __init__(self, backend: SimBackend, servicer: ServicerSpec, client: ClientSpec, seed: int = 0):
        self.backend = backend
        self.servicer = servicer
        self.client = client
        self.busd = servicer.load_bus_dict()
        self.rcs = Rcs(self.busd)
        self.ids = build_bus_ids(backend.km, servicer, client, self.rcs)
        self.pulses = Pulses(self.rcs, seed=seed)
        self.wheels = Wheels(self.busd)

        self.unloading = False
        self.u = np.zeros(self.rcs.n)
        self.tau_thr = np.zeros(3)
        self.plume_pulses = 0
        self.bad_contacts: Set[str] = set()
        self.phase = "init"
        self.com_true = backend.km.body_ipos[self.ids["bid_t"]].copy()

        # Touch sensors
        km = backend.km
        self.touch_adr = {
            k: int(km.sensor_adr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SENSOR, f"touch_{k}")])
            for k in servicer.tool.touch_names
        }
        self._cos_plume = float(np.cos(np.radians(servicer.bus_ctrl.plume_half_cone_deg)))
        self._plume_dist = float(servicer.bus_ctrl.plume_dist_m)
        self._M_buf = np.zeros((km.nv, km.nv))

    def reset_episode(self, seed: int) -> None:
        self.pulses = Pulses(self.rcs, seed=seed)
        self.wheels = Wheels(self.busd)
        self.unloading = False
        self.u = np.zeros(self.rcs.n)
        self.tau_thr = np.zeros(3)
        self.plume_pulses = 0
        self.bad_contacts = set()
        self.phase = "init"

    def bus_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        kd = self.backend.kd
        bid_c = self.ids["bid_c"]
        cv = self.ids["cv"]
        return (
            kd.xpos[bid_c].copy(),
            kd.xmat[bid_c].reshape(3, 3).copy(),
            kd.qvel[cv : cv + 3].copy(),
            kd.qvel[cv + 3 : cv + 6].copy(),
            kd.subtree_com[bid_c].copy(),
        )

    def bus_mass_inertia(self) -> Tuple[float, np.ndarray]:
        km, kd = self.backend.km, self.backend.kd
        mujoco.mj_fullM(km, self._M_buf, kd.qM)
        cv = self.ids["cv"]
        return float(km.body_subtreemass[self.ids["bid_c"]]), self._M_buf[cv + 3 : cv + 6, cv + 3 : cv + 6].copy()

    def touches(self) -> Dict[str, float]:
        kd = self.backend.kd
        return {k: float(kd.sensordata[adr]) for k, adr in self.touch_adr.items()}

    def record_contacts(self) -> None:
        km, kd = self.backend.km, self.backend.kd
        srv_b, cli_b = self.ids["srv_b"], self.ids["cli_b"]
        tool_b, ring_g, cam_g = self.ids["tool_b"], self.ids["ring_g"], self.ids["cam_g"]
        for i in range(kd.ncon):
            c = kd.contact[i]
            b1, b2 = int(km.geom_bodyid[c.geom1]), int(km.geom_bodyid[c.geom2])
            if not ((b1 in srv_b and b2 in cli_b) or (b2 in srv_b and b1 in cli_b)):
                continue
            tool = (b1 in tool_b and c.geom2 in ring_g and c.geom1 not in cam_g) or (
                b2 in tool_b and c.geom1 in ring_g and c.geom2 not in cam_g
            )
            if not tool:
                g1 = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or f"#{c.geom1}"
                g2 = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or f"#{c.geom2}"
                self.bad_contacts.add(f"{g1} - {g2} ({self.phase})")

    def step(
        self,
        F_w: np.ndarray,
        T_b: np.ndarray,
        q_arm_cmd: np.ndarray,
        g_cmd: np.ndarray,
        n_steps: int,
        *,
        thr_on: bool = True,
        on_step: Optional[Callable[[float], None]] = None,
    ) -> None:
        """Advance n_steps physics steps, allocating RCS + RW + unloading + arm PD + orbital wrenches."""
        km, kd = self.backend.km, self.backend.kd
        dt = float(km.opt.timestep)
        bc = self.servicer.bus_ctrl
        arm_kp = self.servicer.arm.kp_arr
        arm_kd = self.servicer.arm.kdv_arr
        ids = self.ids
        bid_c, bid_t = ids["bid_c"], ids["bid_t"]

        for _ in range(n_steps):
            if on_step is not None:
                on_step(float(kd.time))
            p, R, _, wb, com = self.bus_state()
            if thr_on:
                fill = self.wheels.fill()
                self.unloading = fill > bc.unload_on or (self.unloading and fill > bc.unload_off)
                self.tau_thr = (
                    np.clip(-bc.unload_k * self.wheels.H(), -bc.unload_max, bc.unload_max)
                    if self.unloading
                    else np.zeros(3)
                )
                w_cmd = np.r_[R.T @ F_w, self.tau_thr]
                ref = R.T @ (com - p)
                self.u = self.rcs.allocate(w_cmd, ref) if np.any(np.abs(w_cmd) > 1e-9) else np.zeros(self.rcs.n)
            else:
                self.u = np.zeros(self.rcs.n)
                self.tau_thr = np.zeros(3)

            f = self.pulses.step(self.u if thr_on else np.zeros(self.rcs.n), dt)
            kd.ctrl[ids["noz"]] = f
            dF = f[:, None] * (self.pulses.D_true - self.rcs.D)
            dT = np.cross(self.rcs.P, dF).sum(0)
            tau, _ = self.wheels.command(T_b - self.tau_thr if thr_on else T_b, wb, dt)
            kd.ctrl[ids["rwm"]] = tau + dT

            if thr_on and f.any():
                cc_t = kd.xpos[bid_t] + kd.xmat[bid_t].reshape(3, 3) @ self.com_true
                for idx_n in np.flatnonzero(f):
                    pz = p + R @ self.rcs.P[idx_n]
                    d_c = cc_t - pz
                    dn = float(np.linalg.norm(d_c))
                    if dn < self._plume_dist and float(-(R @ self.rcs.D[idx_n]) @ d_c) / max(dn, 1e-9) > self._cos_plume:
                        self.plume_pulses += 1

            qa, qda = kd.qpos[ids["arm_q"]], kd.qvel[ids["arm_v"]]
            kd.ctrl[ids["act_arm"]] = np.clip(arm_kp * (q_arm_cmd - qa) - arm_kd * qda, -ids["arm_lim"], ids["arm_lim"])
            kd.ctrl[ids["act_grip"]] = g_cmd

            self.backend.step_physics(chaser_bid=bid_c, chaser_force_w=R @ dF.sum(0))
            self.record_contacts()
