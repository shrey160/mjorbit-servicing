"""Shared mission context, mutable episode state, and SimBackend adapter over ring_mission.Mission."""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import find_weight  # noqa: E402

from gnc.nav import FarNavTracker  # noqa: E402
from gnc.sensors import SensorSuite, extract_client_boxes, randomize_bench_mount  # noqa: E402
from sim.actuator_bus import ActuatorBus  # noqa: E402
from sim.backend import MjorbitBackend, PlainMujocoBackend, SimBackend  # noqa: E402
from specs.client_spec import ClientSpec  # noqa: E402
from specs.mission_config import MissionConfig  # noqa: E402
from specs.servicer_spec import ServicerSpec  # noqa: E402

import cams  # noqa: E402
import lidar_nav  # noqa: E402
import ring_bench as RB  # noqa: E402
import ring_mission as RM  # noqa: E402


class BackendMission(RM.Mission):
    """Wraps ring_mission.Mission cleanly so verified close-range/grasp runs under SimBackend."""

    def __init__(self, backend: Optional[SimBackend] = None):
        super().__init__()
        self._ext_backend: Optional[SimBackend] = backend

    def bind_backend(self, backend: SimBackend) -> None:
        self._ext_backend = backend
        backend.km = self.km
        backend.kd = self.kd
        backend.base_cam = self.base
        if isinstance(backend, MjorbitBackend):
            self.km.opt.integrator = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
            if backend.render_km is self.km:
                backend.render_km = self.km
                backend.render_kd = self.kd
        else:
            backend.render_km = self.km
            backend.render_kd = self.kd

    def _before_step(self):
        super()._before_step()
        if self._ext_backend is not None:
            self._ext_backend.advance_orbit_and_apply(self.kd, chaser_bid=self.bid_c)


@dataclass
class MissionState:
    tracker: Optional[FarNavTracker] = None
    r8_des: Optional[np.ndarray] = None
    roll_ref: Optional[np.ndarray] = None
    ring_seen8: bool = False
    q_lock_arm: Optional[np.ndarray] = None
    q_lock_jaw: Optional[np.ndarray] = None
    n_est: Optional[np.ndarray] = None


class MissionContext:
    """Owns specs, SimBackend, ActuatorBus, SensorSuite, and verified Mission harness."""

    def __init__(self, client: ClientSpec, servicer: ServicerSpec, config: MissionConfig):
        self.client = client
        self.servicer = servicer
        self.config = config

        # Configure stereo/camera modes matching MissionConfig (no hidden defaults)
        cams.LIDAR_MAX = float(servicer.sensors.lidar_max_m)
        RM.USE_CAM[0] = config.cam_mode if config.cam_mode else False
        RM.STEREO["act"] = "in+out"
        RM.STEREO["tilt"] = config.stereo_tilt
        RM.STEREO["frame"] = "camera"
        RM.STEREO["mot_m"] = 0.0
        RM.LAT[0] = config.lat_s
        RM.LAT_J[0] = config.lat_j_s
        RM.LIDAR_DT = config.lidar_dt

        # Instantiate RM.Mission first, then bind selected SimBackend onto its MjModel/MjData
        self.M = BackendMission()
        if config.backend == "mjorbit":
            self.backend: SimBackend = MjorbitBackend(
                servicer.scene_xml_path,
                orbit_alt_km=config.orbit_alt_km,
                orbit_inc_deg=config.orbit_inc_deg,
                earth_bg=config.earth_bg,
                tool_friction=servicer.tool.friction,
                sun_phase_deg=config.sun_phase_deg,
            )
            # Re-bind BackendMission's km/kd cleanly: use BackendMission's renderers, adopt mjorbit integrator
            self.M.bind_backend(self.backend)
        else:
            self.backend = PlainMujocoBackend(servicer.scene_xml_path, tool_friction=servicer.tool.friction)
            self.M.bind_backend(self.backend)

        lidar_nav.set_model(extract_client_boxes(self.M.km, self.M.bid_t))
        self.bus = ActuatorBus(self.backend, servicer, client, seed=0)
        self.sensors = SensorSuite(self.backend, self.M.rig, servicer.sensors, self.M.bid_c)

        km = self.M.km
        self.ev_u = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, servicer.tool.lock_eqs[0])
        self.ev_l = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, servicer.tool.lock_eqs[1])
        self.gsite = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, servicer.tool.ee_site)
        self.q_stow = self.M.poses["stow"].copy()
        self.q_ready = self.M.poses["ready"].copy()
        self.q_app = self.M.q_app.copy()
        self.bore_b = self.sensors.bore_b.copy()

        # Optional separate wrist stereo renderers when earth_bg=True
        if config.backend == "mjorbit" and config.earth_bg:
            rkm = self.backend.render_km
            ws = servicer.sensors.wrist_res
            self._bg_wr = mujoco.Renderer(rkm, ws, ws)
            self._bg_wcam1 = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_CAMERA, servicer.sensors.wrist_cams[0])
            self._bg_wcam2 = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_CAMERA, servicer.sensors.wrist_cams[1])
            self._bg_lamp = mujoco.mj_name2id(rkm, mujoco.mjtObj.mjOBJ_LIGHT, servicer.sensors.wrist_lamp)
        else:
            self._bg_wr = None

        self.bus_net = None
        if config.earth_bg and find_weight("bus_net_orb.pt").exists():
            import torch

            torch.set_num_threads(1)
            from nets.bus_net import BusNet

            self.bus_net = BusNet.load()

        if config.cam_mode == "stereo" and not RM.STEREO["nets"]:
            import torch

            torch.set_num_threads(1)
            from stereo_net import StereoNet

            suf = "_orb" if config.earth_bg else ""
            for v in ("in", "in+out"):
                RM.STEREO["nets"][v] = StereoNet.load(v, suffix=suf)

        self.row: Dict[str, Any] = {}
        self.rng: Optional[np.random.Generator] = None
        self.t_wall0: float = 0.0

    def reset_episode(self, seed: int) -> MissionState:
        self.t_wall0 = time.time()
        M = self.M
        km, kd = M.km, M.kd
        rng = np.random.default_rng([seed, 771])
        self.rng = rng

        # Client attitude & spin
        Rc = Rotation.random(random_state=int(rng.integers(2**31))).as_matrix()
        spin = float(rng.uniform(self.config.spin_range[0], self.config.spin_range[1]))
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        w_c = np.radians(spin) * (Rc @ ax)
        n_true0 = Rc @ self.client.ring.n_arr

        # Backend reset (sets lights/orbit; on Plain chooses sunlit cone within 60 deg of +n_true0)
        sun_from = self.backend.reset(rng=rng, sun_cone_axis=n_true0, sun_cone_deg=60.0)
        M._vff = np.zeros(6)
        M.bad, M.phase, M.Eb = set(), "far_hold", None
        M.arm_guard(False)
        M.peak = None
        kd.eq_active[self.ev_u] = 0
        kd.eq_active[self.ev_l] = 0

        com_w = Rc @ M.com_true
        kd.qpos[M.tq : M.tq + 7] = [0.0, 0.0, 0.0, *RB._q_wxyz(Rc)]
        kd.qvel[M.tv : M.tv + 3] = -np.cross(w_c, com_w)
        kd.qvel[M.tv + 3 : M.tv + 6] = Rc.T @ w_c

        # Approach line u within 30 deg of sun_from
        u = RB._rand_unit_cone(rng, sun_from, 30.0)
        p_s = self.config.d_start * u
        want0 = RB._rand_unit_cone(rng, -u, 2.0)
        R_s = RB._rot_between(self.bore_b, want0)
        R_s = Rotation.from_rotvec(want0 * float(rng.uniform(-np.pi, np.pi))).as_matrix() @ R_s
        kd.qpos[M.cq : M.cq + 7] = [*p_s, *RB._q_wxyz(R_s)]
        kd.qvel[M.cv : M.cv + 6] = 0.0
        kd.qpos[M.arm] = self.q_stow
        kd.qpos[M.gq] = M.q_open

        randomize_bench_mount(self.backend, M.rig, [M.bcam, M.lid], self.servicer.sensors, rng, seed)
        self.backend.forward()

        # Init M and ActuatorBus shared nav/state
        M.rng = rng
        M.R_rel = self.client.r_rel.copy()
        de = rng.normal(size=3)
        q_iq = km.body_iquat[M.bid_t]
        Rq = Rotation.from_quat([q_iq[1], q_iq[2], q_iq[3], q_iq[0]]).as_matrix()
        I_est = Rq @ np.diag(km.body_inertia[M.bid_t] * (1.0 + rng.uniform(-self.client.i_err_frac, self.client.i_err_frac, 3))) @ Rq.T
        M.nav = RM.RelNav(M.com_true + self.client.com_err_m * de / np.linalg.norm(de), I_est)
        self.bus.reset_episode(seed)
        # Keep M's actuator objects in sync for RM.Mission capture phase & final telemetry
        M.pulses = self.bus.pulses
        M.wheels = self.bus.wheels
        M.unloading, M.u, M.t_ctrl, M.t_lidar = False, np.zeros(self.bus.rcs.n), -1.0, 0.0
        M.thr_on, M.s, M.sdot = True, self.client.r_hold3_m, 0.0
        M.F_cmd, M.T_cmd, M.tau_thr = np.zeros(3), np.zeros(3), np.zeros(3)
        M.plume, M.fixes, M.fix_fail = 0, 0, 0
        M.last_fix_ok = 0.0
        M.pend, M.lat_rng = [], np.random.default_rng([seed, 772])
        M.flog, M.fk, M.n_law, M.fdiag = [], {}, 0, []
        M.t_bad = None
        M.err = (0.0, 0.0)
        M.innov, M.n_rej_run, M.n_rej, M.anomaly = [], 0, 0, None

        self.row = dict(seed=seed, backend=self.config.backend, earth_bg=self.config.earth_bg, spin_deg_s=round(spin, 2), phases={})
        orb = self.backend.orbit_info()
        if orb is not None:
            self.row["orbit_init"] = orb

        r_true0 = kd.xpos[M.bid_t] - kd.subtree_com[M.bid_c]
        dp0 = np.clip(rng.normal(0.0, 1.0, 3), -3.0, 3.0)
        dv0 = np.clip(rng.normal(0.0, 0.003, 3), -0.01, 0.01)
        tracker = FarNavTracker(
            r_true0 + dp0,
            dv0,
            sp=3.0,
            sv=0.01,
            orbit_n=0.0,
            perception=self.client.perception,
            lat_s=self.config.lat_s,
            lat_j_s=self.config.lat_j_s,
            rng=np.random.default_rng([seed, 771, 9]),
            bus_net=self.bus_net,
        )
        return MissionState(tracker=tracker, roll_ref=R_s[:, 2].copy())

    def finish(self, result: str) -> Dict[str, Any]:
        self.M.bad |= self.bus.bad_contacts
        self.row["result"] = result
        self.row["contacts_bad"] = sorted(self.M.bad)
        self.row["plume_pulses"] = int(self.bus.plume_pulses + self.M.plume)
        self.row["prop_g"] = round(self.bus.pulses.prop_kg * 1e3, 1)
        self.row["wheel_fill_max"] = round(self.bus.wheels.fill(), 3)
        self.row["sim_s"] = round(float(self.M.kd.time), 1)
        self.row["wall_s"] = round(time.time() - self.t_wall0, 1)
        return self.row
