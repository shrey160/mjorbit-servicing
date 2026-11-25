"""Unified batched GPU reach environment supporting full mjorbit_warp orbital physics and plain MJWarp."""
from __future__ import annotations

import math
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
from _paths import RUNS_DIR, SIM_SRC, SRC_DIR as P7, find_weight  # noqa: E402

# CRITICAL on Linux/WSL: import mjorbit before torch so libstdc++ resolves from pixi.
import mjorbit  # noqa: E402, F401
import mjorbit_warp as mjow  # noqa: E402
import mujoco  # noqa: E402
import mujoco_warp as mjw  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

from arm_torch import ArmFK, frame, mat_to_quat, quat_to_mat, rot_from_rotvec, rotvec, section  # noqa: E402
from common import (  # noqa: E402
    DEFAULT_DUTY,
    HOLD_AVG,
    PH_CART,
    PH_CLOSE,
    PH_DESC,
    PH_DONE,
    PH_HOLD,
    PH_LOAD,
    PH_NAME,
    PH_SETTLE,
    W_MAX,
    CaptureParams,
    MissionConfig,
    StereoNoiseModel,
    classify_outcome,
    extract_model_indices,
    make_orbital_orbits,
    patch_model_for_warp,
    thermal_guard,
)
from policy import build_obs_torch  # noqa: E402

H_DT = 0.01  # truth history sampling interval (s) for sensor latency


@wp.kernel
def _add_xfrc_kernel(
    dst: wp.array2d(dtype=wp.spatial_vector),
    src: wp.array2d(dtype=wp.spatial_vector),
):
    w, b = wp.tid()
    dst[w, b] = dst[w, b] + src[w, b]


def _strip_mjorbit_xml(xml_text: str) -> str:
    cleaned = re.sub(r"<mjorbit\b[^>]*>.*?</mjorbit>", "", xml_text, flags=re.DOTALL)
    cleaned = re.sub(r"<mjorbit\b[^/>]*/>", "", cleaned)
    return "\n".join(
        ln for ln in cleaned.splitlines() if "<mjorbit " not in ln and "use_gravity_gradient" not in ln
    )


class GpuReachEnv:
    """Batched GPU reach-to-capture environment with mjorbit_warp or plain_warp physics and CUDA-graph stepping."""

    def __init__(
        self,
        starts: Sequence[Dict[str, Any]],
        cells: Optional[Sequence[Dict[str, Any]]] = None,
        *,
        backend: str = "mjorbit_warp",
        moving: bool = True,
        hold_check: bool = False,
        noise: Optional[StereoNoiseModel] = None,
        params: Optional[CaptureParams] = None,
        cfg: Optional[MissionConfig] = None,
        seed: int = 0,
        duty: float = DEFAULT_DUTY,
        o_max: float = 0.008,
        a_vel: float = 0.008,
        orbit_dt: float = 0.02,
        nconmax: int = 160,
        njmax: int = 800,
        trim: Any = None,
        diag: bool = False,
    ):
        if backend not in ("mjorbit_warp", "plain_warp"):
            raise ValueError(f"Unsupported backend: {backend!r}")
        if not starts:
            raise ValueError("starts must be non-empty")

        self.starts = list(starts)
        self.backend = backend
        self.moving = bool(moving)
        self.hold_check = bool(hold_check)
        self.duty = float(duty)
        self.o_max = float(o_max)
        self.a_vel = float(a_vel)
        self.orbit_dt = float(orbit_dt)
        self.diag = bool(diag)
        self.dev = "cuda"

        self.cfg = cfg if cfg is not None else MissionConfig(backend="mjorbit" if backend == "mjorbit_warp" else "plain")
        if params is not None:
            self.params = params
        else:
            self.params = CaptureParams.from_specs(cfg=self.cfg, track_mm=12.0 if self.moving else 5.0)
        if self.moving and noise is None:
            orb_json = find_weight("stereo_net_in+out_orb.json")
            self.noise = (
                StereoNoiseModel.from_json(orb_json, t_refit_s=self.cfg.lidar_dt)
                if orb_json.exists()
                else StereoNoiseModel.preset("stereo_plain", t_refit_s=self.cfg.lidar_dt)
            )
        else:
            self.noise = noise

        if cells is None:
            cells = [dict(cell="bench", w_ax=0.0, w_tr=0.0, tilt_r=0.0, scale=0.0, base=i) for i in range(len(self.starts))]
        self.cells = list(cells)
        self.N = len(self.cells)
        N = self.N
        f32, f64 = torch.float32, torch.float64

        # --- Build MuJoCo / Warp model & data once (reused across reset() calls)
        if backend == "mjorbit_warp":
            from mjorbit.spec import MjoSpec

            rng_orb = np.random.default_rng(seed)
            orbits = make_orbital_orbits(N, self.cfg, rng_orb)
            xml_txt = Path(self.params.scene_xml_path).read_text(encoding="utf-8")
            xml_txt = re.sub(r'noslip_iterations="\d+"', 'noslip_iterations="0"', xml_txt)
            spec = MjoSpec.from_xml_string(xml_txt)
            spec.mjorbit.orbit_dt = self.orbit_dt
            spec.mjorbit.use_j2 = True
            spec.mjorbit.use_gravity_gradient = True
            spec.mjorbit.use_drag = False
            spec.mjorbit.use_srp = False
            spec.mjorbit.use_magnetic = False
            self.mjo_model = mjow.MjoModel.from_host_model(spec.compile(mj_timestep=0.002))
            km = self.mjo_model.mj_model
            patch_model_for_warp(km, self.params, trim=trim)
            self.mjo_model.warp_model = mjw.put_model(km)
            self.wm = self.mjo_model.warp_model
            self.wm.eq_data = wp.array(
                np.repeat(km.eq_data[None], N, axis=0).astype(np.float32),
                dtype=self.wm.eq_data.dtype,
            )
            self.mjo_data = self.mjo_model.make_data(orbit=orbits, nworld=N, nconmax=nconmax, njmax=njmax)
            self.wd = self.mjo_data.warp_data
            self.cd = self.mjo_data.core_data
            self._r_eci = wp.to_torch(self.cd.orbit_R_eci)
            self._v_eci = wp.to_torch(self.cd.orbit_V_eci)
            self._orbit_t0 = wp.to_torch(self.cd.orbit_t0)
            self._orbit_t = wp.to_torch(self.cd.orbit_t)
            self._orbit_seg_dur = wp.to_torch(self.cd.orbit_segment_duration)
        else:
            raw = _strip_mjorbit_xml(Path(self.params.scene_xml_path).read_text(encoding="utf-8"))
            km = mujoco.MjModel.from_xml_string(raw)
            km.opt.integrator = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
            patch_model_for_warp(km, self.params, trim=trim)
            self.mjo_model = None
            self.mjo_data = None
            self.cd = None
            self.wm = mjw.put_model(km)
            self.wm.eq_data = wp.array(
                np.repeat(km.eq_data[None], N, axis=0).astype(np.float32),
                dtype=self.wm.eq_data.dtype,
            )
            self.wd = mjw.make_data(km, nworld=N, nconmax=nconmax, njmax=njmax)

        self.km = km
        self.dt = float(km.opt.timestep)
        self.ext_xfrc_wp = (
            wp.zeros((N, km.nbody), dtype=wp.spatial_vector)
            if (backend == "mjorbit_warp" and self.hold_check)
            else None
        )
        self.ext_xfrc = wp.to_torch(self.ext_xfrc_wp) if self.ext_xfrc_wp is not None else None

        # Zero-copy CUDA Torch views of warp_data & model eq_data
        T = wp.to_torch
        wd = self.wd
        self.qpos, self.qvel, self.ctrl = T(wd.qpos), T(wd.qvel), T(wd.ctrl)
        self.act = T(wd.act) if km.na > 0 else None
        self.qacc_warmstart = T(wd.qacc_warmstart)
        self.xfrc, self.eqa, self.eqm = T(wd.xfrc_applied), T(wd.eq_active), T(self.wm.eq_data)
        self.sens = T(wd.sensordata)

        # Cache decoded unique start arrays on CUDA for zero-overhead reset()
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        d1 = mujoco.MjData(km)
        cols: Dict[str, List[np.ndarray]] = dict(qpos=[], qvel=[], act=[], qacc_warmstart=[], ctrl=[])
        for s in self.starts:
            mujoco.mj_setState(km, d1, s["state"], spec)
            for k in cols:
                cols[k].append(getattr(d1, k).copy())
        self._start_bank = {
            k: torch.tensor(np.asarray(v), dtype=f32, device=self.dev)
            for k, v in cols.items()
            if getattr(d1, k).size > 0
        }

        # Pre-upload initial starts, compile/warm up kernels, and capture CUDA step graph once
        base_idx0 = torch.tensor([int(c.get("base", i % len(self.starts))) for i, c in enumerate(self.cells)],
                                 dtype=torch.long, device=self.dev)
        self._upload_starts(base_idx0)
        self.forward()
        for _ in range(4):
            self._raw_step()
        self._upload_starts(base_idx0)
        self.forward()
        wp.synchronize()
        with wp.ScopedCapture() as cap:
            self._raw_step()
        self.graph = cap.graph
        wp.synchronize()

        # --- Model indices & kinematics constants (direct from Specs + MjModel)
        idx = extract_model_indices(km, self.params)
        to_t = lambda x, dt_=f32: torch.tensor(np.asarray(x), dtype=dt_, device=self.dev)  # noqa: E731
        self.aq = to_t(idx["arm_q"], torch.long)
        self.av = to_t(idx["arm_v"], torch.long)
        self.aa = to_t(idx["act_arm"], torch.long)
        self.ag = to_t(idx["ag"], torch.long)
        self.gq = to_t(idx["gq"], torch.long)
        self.cq, self.cv = int(idx["cq"]), int(idx["cv"])
        self.tq, self.tv = int(idx["tq"]), int(idx["tv"])
        self.bid_c, self.bid_t = int(idx["bid_c"]), int(idx["bid_t"])
        self.kp, self.kdv = to_t(self.params.kp), to_t(self.params.kdv)
        self.lim = to_t(km.actuator_ctrlrange[idx["act_arm"], 1])
        self.ev = to_t(idx["ev"], torch.long)
        self.ev_list = [int(x) for x in idx["ev"]]
        self.s_touch = to_t(idx["s_touch"], torch.long)
        self.s_wf = to_t(idx["s_wf"], torch.long)
        self.q_open = to_t(self.params.q_open)
        self.sq_cmd = to_t(
            [self.params.jaw_close_signs[0] * self.params.q_squeeze,
             self.params.jaw_close_signs[1] * self.params.q_squeeze],
            f64,
        )
        self.g_side = to_t(idx["g_side"], torch.int8)
        self.g_tool = to_t(idx["g_tool"], torch.bool)
        self.g_ring = to_t(idx["g_ring"], torch.bool)
        self.A = ArmFK(
            km,
            site=self.params.ee_site,
            base=self.params.servicer_body_name,
            arm_joints=idx["arm_q"],
            jlo=idx["jlo"],
            jhi=idx["jhi"],
            device=self.dev,
        )
        self.H_TOP, self.R_R = float(self.params.h_top), float(self.params.r_grasp)
        self.C_B = to_t(self.params.c_b, f64)
        self.N_B = to_t(self.params.n_b, f64)
        self.r_com = to_t(km.body_ipos[self.bid_t], f64)
        Rq = quat_to_mat(to_t(km.body_iquat[self.bid_t], f64)[None])[0]
        self.I_b = Rq @ torch.diag(to_t(km.body_inertia[self.bid_t], f64)) @ Rq.T

        # Reset episode-specific state (reuses allocated Warp/CUDA graph cleanly)
        self.reset(cells=self.cells, seed=seed)

    # ------------------------------------------------------------------------------------------------------ Warp / step
    def _upload_starts(self, base_idx: torch.Tensor) -> None:
        self.eqa.zero_()
        self.xfrc.zero_()
        if self.ext_xfrc is not None:
            self.ext_xfrc.zero_()
        for k, bank in self._start_bank.items():
            getattr(self, k)[:] = bank[base_idx]

    def _raw_step(self) -> None:
        if self.backend == "mjorbit_warp":
            if self.hold_check and self.ext_xfrc_wp is not None:
                mj_dt = self.mjo_model.opt.timestep
                orb_dt = (
                    self.mjo_model.orbit_dt
                    if (self.mjo_model.orbit_dt is not None and self.mjo_model.orbit_dt > 0.0)
                    else mj_dt
                )
                mjow.core_gpu.refresh_core(self.mjo_model, self.mjo_data)
                mjw.step1(self.wm, self.wd)
                mjow.core_gpu.assemble_step_and_propagate(self.mjo_model, self.mjo_data, mj_dt=mj_dt, orbit_dt=orb_dt)
                wp.launch(
                    _add_xfrc_kernel,
                    dim=(self.N, self.km.nbody),
                    inputs=[self.wd.xfrc_applied, self.ext_xfrc_wp],
                )
                mjw.step2(self.wm, self.wd)
            else:
                mjow.mjo_step(self.mjo_model, self.mjo_data)
        else:
            mjw.step(self.wm, self.wd)

    def forward(self) -> None:
        if self.backend == "mjorbit_warp":
            mjow.mjo_forward(self.mjo_model, self.mjo_data)
        else:
            mjw.forward(self.wm, self.wd)

    # ------------------------------------------------------------------------------------------------------ reset
    def reset(self, cells: Optional[Sequence[Dict[str, Any]]] = None, seed: int = 0) -> None:
        if cells is not None:
            if len(cells) != self.N:
                raise ValueError(f"reset() expects {self.N} cells, got {len(cells)}")
            self.cells = list(cells)
        self.rng = np.random.default_rng(seed)
        torch.manual_seed(seed)

        base_idx_list = [int(c.get("base", i % len(self.starts))) for i, c in enumerate(self.cells)]
        base_idx = torch.tensor(base_idx_list, dtype=torch.long, device=self.dev)
        self._upload_starts(base_idx)

        if self.backend == "mjorbit_warp":
            orbits = make_orbital_orbits(self.N, self.cfg, self.rng)
            self._r_eci[:] = torch.tensor(np.stack([o.R_eci for o in orbits]), dtype=torch.float32, device=self.dev)
            self._v_eci[:] = torch.tensor(np.stack([o.V_eci for o in orbits]), dtype=torch.float32, device=self.dev)
            self._orbit_t0[:] = torch.tensor([o.t for o in orbits], dtype=torch.float64, device=self.dev)
            self._orbit_t.zero_()
            self._orbit_seg_dur.zero_()
        self.forward()

        f64, dev, N = torch.float64, self.dev, self.N
        base_starts = [self.starts[i] for i in base_idx_list]
        t64 = lambda x: torch.tensor(np.asarray(x), dtype=f64, device=dev)  # noqa: E731
        self.pin = t64(np.array([s["pin"] for s in base_starts]))
        self.bp = self.pin[:, :3].clone()
        self.bR = quat_to_mat(self.pin[:, 3:7])
        self.c = t64(np.array([s["c"] for s in base_starts]))
        self.n = t64(np.array([s["n"] for s in base_starts]))
        u, v = frame(self.n)
        a1 = t64(np.array([s["a1"] for s in base_starts]))
        self.rad = torch.cos(a1)[:, None] * u + torch.sin(a1)[:, None] * v

        self.k_glob = 0
        self._last_est_k = -1
        self.offset = torch.zeros(N, 3, dtype=f64, device=dev)
        self.v_peak = torch.zeros(N, dtype=f64, device=dev)
        self.a_w = torch.zeros(N, 3, dtype=f64, device=dev)
        self.a_v = torch.zeros(N, 3, dtype=f64, device=dev)
        self.bad0 = t64([bool(s.get("bad0", False)) for s in base_starts]).bool()

        if self.moving:
            self._setup_motion(base_starts)
        else:
            self._pin()
            self.forward()
            self._init_state()

    def _setup_motion(self, base_starts: Sequence[Dict[str, Any]]) -> None:
        N, dev, f64, rng = self.N, self.dev, torch.float64, self.rng
        noise = self.noise if self.noise is not None else StereoNoiseModel.preset("stereo_plain")
        t64 = lambda x: torch.tensor(np.asarray(x), dtype=f64, device=dev)  # noqa: E731

        sc = t64([c["scale"] for c in self.cells])
        self.scale = sc

        tq, Rc, com = self._client_pose_com()
        n_w = Rc @ self.N_B
        a_rand = torch.tensor(rng.normal(size=(N, 3)), dtype=f64, device=dev)
        perp = a_rand - (a_rand * n_w).sum(-1, keepdim=True) * n_w
        perp = perp / perp.norm(dim=-1, keepdim=True).clamp(min=1e-12)
        w0 = (
            torch.deg2rad(t64([c["w_ax"] for c in self.cells]))[:, None]
            * n_w
            * t64(rng.choice([-1.0, 1.0], N))[:, None]
            + torch.deg2rad(t64([c["w_tr"] for c in self.cells]))[:, None] * perp
        )
        self.qvel[:, self.tv + 3 : self.tv + 6] = (Rc.transpose(1, 2) @ w0[:, :, None]).squeeze(-1).float()
        self.qvel[:, self.tv : self.tv + 3] = (-torch.cross(w0, Rc @ self.r_com, dim=-1)).float()

        bR0 = quat_to_mat(self.pin[:, 3:7])
        self.st_R = Rc.transpose(1, 2) @ bR0
        self.st_p = (Rc.transpose(1, 2) @ (self.pin[:, :3] - tq[:, :3])[:, :, None]).squeeze(-1)
        self.g_R, self.g_c, self.g_w = Rc.clone(), com.clone(), w0.clone()

        # Smooth 3-sinusoid station-keeping error (0.02-0.2 Hz)
        freq = t64(rng.uniform(0.02, 0.2, (N, 6, 3)))
        self.st_f = 2.0 * math.pi * freq
        self.st_ph = t64(rng.uniform(0.0, 2.0 * math.pi, (N, 6, 3)))
        sig_st = torch.cat(
            [torch.full((3,), noise.sig_st_m), torch.full((3,), math.radians(noise.sig_sta_deg))]
        ).to(dev).double()
        self.st_A = (sig_st[None, :, None] * sc[:, None, None]) * math.sqrt(2.0 / 3.0)
        self.e_st = torch.zeros(N, 6, dtype=f64, device=dev)

        # Estimated ring pose in estimated body frame (calibrated StereoNet_orb or preset mixture)
        u, v = frame(self.N_B[None].repeat(N, 1))
        a1 = t64([s["a1"] for s in base_starts])
        rad_b = torch.cos(a1)[:, None] * u + torch.sin(a1)[:, None] * v
        tan_b = torch.cross(self.N_B[None].repeat(N, 1), rad_b, dim=-1)

        d_rad = t64(noise.sample_mixture(rng, N, noise.rad_core_m, noise.rad_tail_m)) * sc
        d_tan = t64(noise.sample_mixture(rng, N, noise.tan_core_m, noise.tan_tail_m)) * sc
        d_ax = noise.ax_mu_m * torch.clamp(sc, max=1.0) + t64(rng.normal(0.0, noise.sig_ax_m, N)) * sc
        d = d_rad[:, None] * rad_b + d_tan[:, None] * tan_b + d_ax[:, None] * self.N_B[None]

        tilt_r_deg = noise.sample_mixture(rng, N, noise.tilt_rad_core_deg, noise.tilt_rad_tail_deg)
        if noise.use_cell_tilt:
            tilt_r = torch.deg2rad(t64([c["tilt_r"] for c in self.cells]) * t64(rng.choice([-1.0, 1.0], N)) + t64(tilt_r_deg) * sc)
        else:
            tilt_r = torch.deg2rad(t64(tilt_r_deg) * sc)
        tilt_t = torch.deg2rad(t64(noise.sample_mixture(rng, N, noise.tilt_tan_core_deg, noise.tilt_tan_tail_deg)) * sc)
        Rt = rot_from_rotvec(tilt_r[:, None] * rad_b + tilt_t[:, None] * tan_b + 1e-12)
        self.cb = self.C_B[None] + d
        self.nb = Rt @ self.N_B
        self.radb = rad_b - (rad_b * self.nb).sum(-1, keepdim=True) * self.nb
        self.radb = self.radb / self.radb.norm(dim=-1, keepdim=True).clamp(min=1e-12)

        # Lidar AR(1) tracking errors and initial estimate
        self.e_p = t64(rng.normal(0.0, 1.0, (N, 3))) * (noise.sig_p_m * sc)[:, None]
        self.e_a = t64(rng.normal(0.0, 1.0, (N, 3))) * (math.radians(noise.sig_a_deg) * sc)[:, None]
        self.e_w = t64(rng.normal(0.0, 1.0, (N, 3))) * (math.radians(noise.sig_w_deg_s) * sc)[:, None]
        self.t_fit = -1e9
        self.t_anchor = torch.zeros(N, dtype=f64, device=dev)
        self.hist: List[Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]] = []
        self.hist_t = -1e9
        self._last_est_k = -1
        self._refit(force=True)
        self._update_estimate()

        # Move arm via IK to estimated H_PRE before planning H_CLOSE cart path
        q0 = self.qpos[:, self.aq].double()
        p, R = self.sect(torch.full((N,), self.params.h_pre, dtype=f64, device=dev))
        q, ep, er = self.A.ik(self.bp, self.bR, p, R, q0, 100)
        self.start_ok = (ep < 1e-3) & (er < 0.1)
        self.qpos[:, self.aq] = torch.where(self.start_ok[:, None], q, q0).float()
        self.qvel[:, self.av] = 0.0
        self._pin()
        self.forward()
        self._init_state()
        for i in torch.nonzero(~self.start_ok).flatten().tolist():
            self._finish(i, "abort: no start pose")

    def _init_state(self) -> None:
        N, dev, f64 = self.N, self.dev, torch.float64
        z = lambda dt_=f64: torch.zeros(N, dtype=dt_, device=dev)  # noqa: E731
        self.phase = torch.full((N,), PH_CART, dtype=torch.long, device=dev)
        self.result: List[Optional[str]] = [None] * N
        self.info: List[Dict[str, Any]] = [{} for _ in range(N)]
        self.k_tick = torch.zeros(N, dtype=torch.long, device=dev)
        self.q_tgt = self.qpos[:, self.aq].double().clone()
        self.qd_ff = torch.zeros(N, 6, dtype=f64, device=dev)
        self.vff_f = torch.zeros(N, 6, dtype=f64, device=dev)
        self.grip = self.q_open[None].double().repeat(N, 1)
        self.guard_on = torch.ones(N, dtype=torch.bool, device=dev)
        self.guard = self.sens[:, self.s_wf].double().clone()
        self.trip = torch.full((N,), -1, dtype=torch.long, device=dev)
        self.bad = torch.zeros(N, dtype=torch.bool, device=dev)
        self.bad_phase = torch.full((N,), -1, dtype=torch.long, device=dev)
        self.peak = torch.zeros(N, 4, dtype=f64, device=dev)
        self.h = z()
        self.n_wait = torch.zeros(N, dtype=torch.long, device=dev)
        self.at_top = torch.ones(N, dtype=torch.bool, device=dev)
        self.p_cmd = torch.zeros(N, 3, dtype=f64, device=dev)
        self.p_tick = torch.zeros(N, 3, dtype=f64, device=dev)
        self.k_close = torch.zeros(N, dtype=torch.long, device=dev)
        self.k_phase = torch.zeros(N, dtype=torch.long, device=dev)
        self.load_ax = torch.zeros(N, dtype=torch.long, device=dev)
        self.rel0: Optional[Tuple[torch.Tensor, torch.Tensor]] = None
        self.first_tick = torch.zeros(N, dtype=torch.bool, device=dev)
        self.trk_max = torch.zeros(N, dtype=f64, device=dev)
        self.slip = torch.zeros(N, 2, dtype=f64, device=dev)
        self.t_sum = torch.zeros(N, 4, dtype=f64, device=dev)
        self.t_n = torch.zeros(N, dtype=torch.long, device=dev)
        self.v_ff = torch.zeros(N, 3, dtype=f64, device=dev)
        self.n_lost = torch.zeros(N, dtype=torch.long, device=dev)
        self._plan_cart()

    # ------------------------------------------------------------------------------------------------------ kinematics
    def _client_pose_com(self) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        tq = self.qpos[:, self.tq : self.tq + 7].double()
        Rc = quat_to_mat(tq[:, 3:7])
        com = tq[:, :3] + (Rc @ self.r_com)
        return tq, Rc, com

    def com_vel(self) -> torch.Tensor:
        _, Rc, _ = self._client_pose_com()
        w = (Rc @ self.qvel[:, self.tv + 3 : self.tv + 6].double()[:, :, None]).squeeze(-1)
        return self.qvel[:, self.tv : self.tv + 3].double() + torch.cross(w, Rc @ self.r_com, dim=-1)

    def _truth(self) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        tq, Rc, com = self._client_pose_com()
        w = (Rc @ self.qvel[:, self.tv + 3 : self.tv + 6].double()[:, :, None]).squeeze(-1)
        return tq[:, :3], Rc, com, w, self.qvel[:, self.tv : self.tv + 3].double() + torch.cross(w, Rc @ self.r_com, dim=-1)

    def _site(self, q: Optional[torch.Tensor] = None) -> Tuple[torch.Tensor, torch.Tensor]:
        q = self.qpos[:, self.aq].double() if q is None else q
        p, R, _, _ = self.A.fk(self.bp, self.bR, q)
        return p, R

    def sect(self, h: torch.Tensor, idx: Optional[torch.Tensor] = None) -> Tuple[torch.Tensor, torch.Tensor]:
        if idx is None:
            return section(self.c, self.n, self.rad, h, self.H_TOP, self.R_R)
        return section(self.c[idx], self.n[idx], self.rad[idx], h, self.H_TOP, self.R_R)

    def seat(self) -> Dict[str, torch.Tensor]:
        tq, Rc, _ = self._client_pose_com()
        cc = tq[:, :3] + (Rc @ self.C_B)
        nn = Rc @ self.N_B
        p, Rg = self._site()
        v = p - cc
        h = (v * nn).sum(-1)
        rv = v - h[:, None] * nn
        r = rv.norm(dim=-1)
        ang = lambda a, b: torch.rad2deg(torch.acos(((a * b).sum(-1) / a.norm(dim=-1) / b.norm(dim=-1)).clamp(-1, 1)))  # noqa: E731
        return dict(
            rad_mm=(r - self.R_R) * 1e3,
            h_mm=(h - self.H_TOP) * 1e3,
            tilt_deg=ang(Rg[:, :, 0], -nn),
            yaw_deg=ang(Rg[:, :, 2], rv),
        )

    def _rel(self) -> Tuple[torch.Tensor, torch.Tensor]:
        tq, Rc, _ = self._client_pose_com()
        p, Rg = self._site()
        Rt = Rc.transpose(1, 2)
        return (Rt @ (p - tq[:, :3])[:, :, None]).squeeze(-1), Rt @ Rg

    @property
    def t_now(self) -> float:
        return self.k_glob * self.dt

    # ------------------------------------------------------------------------------------------------------ estimate / pin
    def _refit(self, force: bool = False) -> None:
        noise = self.noise
        assert noise is not None
        t = self.t_now
        if not force and t - self.t_fit < noise.t_refit_s - 1e-9:
            return
        sc = self.scale[:, None]
        if not force:
            rho = math.exp(-noise.t_refit_s / noise.tau_e_s)
            k = math.sqrt(max(0.0, 1.0 - rho * rho))
            rn = lambda: torch.randn(self.N, 3, dtype=torch.float64, device=self.dev)  # noqa: E731
            self.e_p = rho * self.e_p + k * rn() * noise.sig_p_m * sc
            self.e_a = rho * self.e_a + k * rn() * math.radians(noise.sig_a_deg) * sc
            self.e_w = rho * self.e_w + k * rn() * math.radians(noise.sig_w_deg_s) * sc
        _, Rc, com, w, vc = self._truth()
        t_a = torch.full((self.N,), t, dtype=torch.float64, device=self.dev)
        if noise.lat_s > 0.0 and self.hist:
            lat = noise.lat_s + (torch.rand(self.N, dtype=torch.float64, device=self.dev) * 2.0 - 1.0) * noise.lat_j_s
            k_hist = torch.clamp(torch.round(lat / H_DT).long(), 0, len(self.hist) - 1)
            ar = torch.arange(self.N, device=self.dev)
            Hs = [torch.stack([h[j] for h in self.hist]) for j in range(4)]
            Rc, com, w, vc = Hs[0][k_hist, ar], Hs[1][k_hist, ar], Hs[2][k_hist, ar], Hs[3][k_hist, ar]
            t_a = t - k_hist.double() * H_DT
        self.a_R = rot_from_rotvec(self.e_a + 1e-12) @ Rc
        self.a_c = com + self.e_p
        self.a_w = w + self.e_w
        self.a_v = vc
        self.t_fit = t
        self.t_anchor = t_a if noise.prop_now else torch.full_like(t_a, t)

    def _update_estimate(self) -> None:
        if not self.moving or self._last_est_k == self.k_glob:
            return
        self._last_est_k = self.k_glob
        noise = self.noise
        assert noise is not None
        if noise.lat_s > 0.0 and self.t_now - self.hist_t >= H_DT - 1e-9:
            _, Rc, com, w, vc = self._truth()
            self.hist.insert(0, (Rc, com, w, vc))
            del self.hist[int(round((noise.lat_s + noise.lat_j_s) / H_DT)) + 2 :]
            self.hist_t = self.t_now
        self._refit()
        dt_ = (self.t_now - self.t_anchor)[:, None]
        Rw = rot_from_rotvec(self.a_w * dt_ + 1e-12) @ self.a_R
        cw = self.a_c + self.a_v * dt_
        self.e_com = cw
        self.e_R = Rw
        po = cw - (Rw @ self.r_com)
        self.c = po + (Rw @ self.cb[:, :, None]).squeeze(-1)
        self.n = (Rw @ self.nb[:, :, None]).squeeze(-1)
        self.rad = (Rw @ self.radb[:, :, None]).squeeze(-1)

    def point_vel(self, p: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
        if not self.moving:
            return torch.zeros_like(p)
        return self.a_v[idx] + torch.cross(self.a_w[idx], p - self.e_com[idx], dim=-1)

    def _ghost_step(self) -> None:
        dt = self.dt

        def wdot(R: torch.Tensor, w: torch.Tensor) -> torch.Tensor:
            Iw = R @ self.I_b @ R.transpose(1, 2)
            return torch.linalg.solve(
                Iw, -torch.cross(w, (Iw @ w[:, :, None]).squeeze(-1), dim=-1)[:, :, None]
            ).squeeze(-1)

        w_m = self.g_w + 0.5 * dt * wdot(self.g_R, self.g_w)
        R_m = rot_from_rotvec(0.5 * dt * self.g_w + 1e-15) @ self.g_R
        self.g_w = self.g_w + dt * wdot(R_m, w_m)
        self.g_R = rot_from_rotvec(dt * w_m + 1e-15) @ self.g_R

    def _pin(self) -> None:
        if not self.moving:
            self.qpos[:, self.cq : self.cq + 7] = self.pin.float()
            self.qvel[:, self.cv : self.cv + 6] = 0.0
            return
        if self.k_glob > 0:
            self._ghost_step()
        t = self.t_now
        ph = self.st_f * t + self.st_ph
        self.e_st = (self.st_A * torch.sin(ph)).sum(-1)
        de = (self.st_A * self.st_f * torch.cos(ph)).sum(-1)
        Rg, cg, w = self.g_R, self.g_c, self.g_w
        Rb = Rg @ self.st_R @ rot_from_rotvec(self.e_st[:, 3:] + 1e-12)
        po = cg - (Rg @ self.r_com)
        pb = po + (Rg @ self.st_p[:, :, None]).squeeze(-1) + (Rg @ self.e_st[:, :3, None]).squeeze(-1)
        self.bp, self.bR = pb, Rb
        self.qpos[:, self.cq : self.cq + 3] = pb.float()
        self.qpos[:, self.cq + 3 : self.cq + 7] = mat_to_quat(Rb).float()
        vb = torch.cross(w, pb - cg, dim=-1) + (Rg @ de[:, :3, None]).squeeze(-1)
        self.qvel[:, self.cv : self.cv + 3] = vb.float()
        self.qvel[:, self.cv + 3 : self.cv + 6] = ((Rb.transpose(1, 2) @ w[:, :, None]).squeeze(-1) + de[:, 3:]).float()
        if self.k_glob > 0:
            vc = self.com_vel()
            live = (self.phase >= PH_DESC) & (self.phase <= PH_HOLD)
            self.v_peak = torch.where(live, torch.maximum(self.v_peak, vc.norm(dim=-1) * 1e3), self.v_peak)

    # ------------------------------------------------------------------------------------------------------ cart / phases
    def _plan_cart(self) -> None:
        N, dev = self.N, self.dev
        q_now = self.qpos[:, self.aq].double()
        p0, R0 = self._site(q_now)
        p1, R1 = self.sect(torch.full((N,), self.params.h_close, dtype=torch.float64, device=dev))
        dR = rotvec(R1 @ R0.transpose(1, 2))
        dist, ang = (p1 - p0).norm(dim=-1), dR.norm(dim=-1)
        Nw = torch.maximum(
            torch.maximum(torch.full_like(dist, 10.0), torch.floor(dist / 0.004)),
            torch.floor(torch.rad2deg(ang) / 1.0),
        ).long()
        assert int(Nw.max()) <= W_MAX, int(Nw.max())
        qs = torch.zeros(N, W_MAX, 6, dtype=torch.float64, device=dev)
        ok = torch.ones(N, dtype=torch.bool, device=dev)
        q = q_now.clone()
        for j in range(1, int(Nw.max()) + 1):
            act = j <= Nw
            tj = (j / Nw.double()).clamp(max=1.0)
            Rt = rot_from_rotvec(tj[:, None] * dR) @ R0
            qn, ep, er = self.A.ik(self.bp, self.bR, p0 + tj[:, None] * (p1 - p0), Rt, q, 60)
            ok = ok & ~(act & ((ep > 2e-3) | (er > 0.2)))
            q = torch.where(act[:, None], qn, q)
            qs[:, j - 1] = q
        dt = self.dt
        T_lin = torch.maximum(dist / 0.05, ang / math.radians(10.0))
        n_min = torch.clamp(torch.floor(T_lin / dt / Nw.double()), min=1).long()
        prev = torch.cat([q_now[:, None], qs[:, :-1]], 1)
        nseg = torch.ceil((qs - prev).abs().amax(-1) / (self.params.qd_max_rad_s * dt)).long()
        nseg = torch.maximum(nseg, n_min[:, None])
        jj = torch.arange(W_MAX, device=dev)[None]
        nseg = torch.where(jj < Nw[:, None], nseg, torch.zeros_like(nseg))
        self.c_qs, self.c_prev, self.c_n, self.c_Nw = qs, prev, nseg, Nw
        self.c_seg = torch.zeros(N, dtype=torch.long, device=dev)
        self.c_k = torch.zeros(N, dtype=torch.long, device=dev)
        for i in torch.nonzero(~ok).flatten().tolist():
            self._finish(i, "abort: path")

    def _finish(self, i: int, res: str) -> None:
        if self.result[i] is None:
            self.result[i] = res
            self.phase[i] = PH_DONE

    def touches(self) -> torch.Tensor:
        return self.sens[:, self.s_touch].double()

    def _set_phase(self, m: torch.Tensor, ph: int) -> None:
        self.phase = torch.where(m, torch.full_like(self.phase, ph), self.phase)
        self.k_phase = torch.where(m, torch.zeros_like(self.k_phase), self.k_phase)

    def _contacts(self) -> None:
        n = int(wp.to_torch(self.wd.nacon)[0])
        if n == 0:
            return
        g = wp.to_torch(self.wd.contact.geom)[:n].long()
        w = wp.to_torch(self.wd.contact.worldid)[:n].long()
        s1, s2 = self.g_side[g[:, 0]], self.g_side[g[:, 1]]
        sv = (s1 > 0) & (s2 > 0) & (s1 != s2)
        tool = (self.g_tool[g[:, 0]] & self.g_ring[g[:, 1]]) | (self.g_tool[g[:, 1]] & self.g_ring[g[:, 0]])
        badc = sv & ~tool
        if bool(badc.any()):
            nb = torch.zeros(self.N, dtype=torch.bool, device=self.dev)
            nb[w[badc]] = True
            new = nb & ~self.bad & (self.phase != PH_DONE)
            self.bad_phase = torch.where(new, self.phase, self.bad_phase)
            if bool(new.any()):
                nm = lambda x: mujoco.mj_id2name(self.km, mujoco.mjtObj.mjOBJ_GEOM, int(x)) or f"#{int(x)}"  # noqa: E731
                pos = wp.to_torch(self.wd.contact.pos)[:n]
                dist = wp.to_torch(self.wd.contact.dist)[:n]
                st = self.seat()
                for i in torch.nonzero(new).flatten().tolist():
                    self.info[i]["seat_bad"] = {k: round(float(v[i]), 2) for k, v in st.items()}
                    js = torch.nonzero(badc & (w == i)).flatten().tolist()
                    self.info[i]["bad_contact"] = dict(
                        ph=PH_NAME[int(self.phase[i])],
                        k_phase=int(self.k_phase[i]),
                        h_mm=round(float(self.h[i]) * 1e3, 1),
                        pairs=[
                            (nm(g[j, 0]), nm(g[j, 1]), [round(float(x), 4) for x in pos[j]], round(float(dist[j]) * 1e3, 2))
                            for j in js[:4]
                        ],
                    )
            self.bad |= new

    def physics_step(self) -> None:
        q, qd = self.qpos[:, self.aq], self.qvel[:, self.av]
        if self.params.ff_enabled:
            self.vff_f = self.vff_f + (
                self.qd_ff.clamp(-self.params.vff_max, self.params.vff_max) - self.vff_f
            ) * (self.dt / self.params.tau_ff)
            vff = self.vff_f.float()
        else:
            vff = 0.0
        u = self.kp * (self.q_tgt.float() - q) + self.kdv * (vff - qd)
        self.ctrl[:, self.aa] = torch.maximum(torch.minimum(u, self.lim), -self.lim)
        self.ctrl[:, self.ag] = self.grip.float()
        wp.capture_launch(self.graph)
        self.k_glob += 1
        self._pin()
        self._contacts()
        t = self.touches()
        live = self.phase != PH_DONE
        self.peak = torch.where((live & (self.phase >= PH_DESC))[:, None], torch.maximum(self.peak, t), self.peak)
        avg = (self.phase == PH_HOLD) & (self.k_phase >= int(1.0 / self.dt) - HOLD_AVG)
        self.t_sum = torch.where(avg[:, None], self.t_sum + t, self.t_sum)
        self.t_n = torch.where(avg, self.t_n + 1, self.t_n)
        g = self.guard_on & (self.trip < 0) & live
        if bool(g.any()):
            tm, ti = t.max(-1)
            wf = (self.sens[:, self.s_wf].double() - self.guard).norm(dim=-1)
            tr = torch.where(
                tm > self.params.touch_n,
                ti,
                torch.where(wf > self.params.ft_n, torch.full_like(ti, 4), torch.full_like(ti, -1)),
            )
            new = g & (tr >= 0)
            for i in torch.nonzero(new).flatten().tolist():
                self.info[i].setdefault("trips", []).append(
                    dict(
                        ph=PH_NAME[int(self.phase[i])],
                        touch=[round(float(x), 2) for x in t[i]],
                        wf=round(float(wf[i]), 2),
                        k=int(self.k_phase[i]),
                    )
                )
            self.trip = torch.where(new, tr, self.trip)
            self.k_tick = torch.where(new, torch.zeros_like(self.k_tick), self.k_tick)
        self.k_phase += 1

    def control(self) -> None:
        self._update_estimate()
        dt, ph, N, dev = self.dt, self.phase, self.N, self.dev
        m = ph == PH_CART
        if bool(m.any()):
            trip = m & (self.trip >= 0)
            for i in torch.nonzero(trip).flatten().tolist():
                nm = (list(self.params.touch_names) + ["wrist_ft"])[int(self.trip[i])]
                self._finish(i, f"abort: contact in the approach ({nm})")
            m = m & ~trip
            ar = torch.arange(N, device=dev)
            seg = self.c_seg.clamp(max=W_MAX - 1)
            n_s = self.c_n[ar, seg]
            fin = m & ((seg >= self.c_Nw) | (n_s == 0))
            if bool(fin.any()):
                last = self.c_qs[ar, (self.c_Nw - 1).clamp(min=0)]
                self.q_tgt = torch.where(fin[:, None], last, self.q_tgt)
                self.qd_ff = torch.where(fin[:, None], torch.zeros_like(self.qd_ff), self.qd_ff)
                self._set_phase(fin, PH_SETTLE)
            mm = m & ~fin
            if bool(mm.any()):
                k = self.c_k + 1
                qa, qb = self.c_prev[ar, seg], self.c_qs[ar, seg]
                qt = qa + (qb - qa) * (k.double() / n_s.clamp(min=1).double())[:, None]
                self.q_tgt = torch.where(mm[:, None], qt, self.q_tgt)
                self.qd_ff = torch.where(
                    mm[:, None], (qb - qa) / (n_s.clamp(min=1).double() * self.dt)[:, None], self.qd_ff
                )
                adv = mm & (k >= n_s)
                self.c_k = torch.where(mm, torch.where(adv, torch.zeros_like(k), k), self.c_k)
                self.c_seg = torch.where(adv, self.c_seg + 1, self.c_seg)

        m = (self.phase == PH_SETTLE) & ((self.k_phase >= int(0.5 / dt)) | (self.trip >= 0))
        if bool(m.any()):
            self._start_descent(m)
        m = (self.phase == PH_DESC) & (self.k_tick == 0)
        if bool(m.any()):
            self._descent_tick(m)
        m = (self.phase == PH_CLOSE) & (self.k_tick == 0)
        if bool(m.any()):
            self._close_tick(m)
        m = (self.phase == PH_HOLD) & (self.k_phase >= int(1.0 / dt))
        if bool(m.any()):
            self._end_hold(m)
        m = self.phase == PH_LOAD
        if bool(m.any()):
            self._load(m)

    def _start_descent(self, m: torch.Tensor) -> None:
        idx = torch.nonzero(m).flatten()
        p, _ = self._site()
        self.p_cmd[idx] = p[idx]
        self.h[idx] = self.params.h_close
        self.at_top[idx] = True
        self.n_wait[idx] = 0
        self.n_lost[idx] = 0
        self.v_ff[idx] = 0.0
        self.guard[idx] = self.sens[idx][:, self.s_wf].double()
        self.trip[idx] = -1
        self.guard_on[idx] = True
        self.peak[idx] = 0.0
        self.q_tgt[idx] = self.qpos[idx][:, self.aq].double()
        self.qd_ff[idx] = 0.0
        s = self.seat()
        for i in idx.tolist():
            self.info[i]["seat_pre"] = {k: round(float(v[i]), 2) for k, v in s.items()}
        self._set_phase(m, PH_DESC)
        self.k_tick[idx] = 0
        self.first_tick = self.first_tick | m

    def _descent_tick(self, m: torch.Tensor) -> None:
        dt = self.dt
        prev = m & ~self.first_tick
        self.first_tick = self.first_tick & ~m
        if bool(prev.any()):
            p, _ = self._site()
            trk = (p - self.p_tick).norm(dim=-1)
            self.trk_max = torch.where(prev, torch.maximum(self.trk_max, trk), self.trk_max)
            bad_trk = prev & (trk > self.params.track_mm / 1e3)
            for i in torch.nonzero(bad_trk).flatten().tolist():
                self.info[i]["trk_abort"] = dict(
                    h_mm=round(float(self.h[i]) * 1e3, 1),
                    n_wait=int(self.n_wait[i]),
                    at_top=bool(self.at_top[i]),
                    gap_mm=round(
                        float((self.sect(self.h[i : i + 1], torch.tensor([i], device=self.dev))[0][0] - self.p_cmd[i]).norm())
                        * 1e3,
                        2,
                    ),
                )
                self._finish(i, f"abort: arm tracking ({float(trk[i]) * 1e3:.1f} mm)")
            tr = prev & ~bad_trk & (self.trip >= 0)
            for i in torch.nonzero(tr).flatten().tolist():
                nm = (list(self.params.touch_names) + ["wrist_ft"])[int(self.trip[i])]
                self.info[i]["descent"] = dict(stop=nm, h_mm=round(float(self.h[i]) * 1e3, 1))
                if nm == "palm":
                    self._start_close(i)
                else:
                    self._finish(i, f"abort: descent ({nm})")
            m = m & (self.phase == PH_DESC)
        if not bool(m.any()):
            return
        end = m & (self.h <= -self.params.over)
        for i in torch.nonzero(end).flatten().tolist():
            self.info[i]["descent"] = dict(stop="none", h_mm=round(float(self.h[i]) * 1e3, 1))
            self._finish(i, "abort: descent (no contact)")
        m = m & ~end
        self.n_wait = torch.where(m, self.n_wait + 1, self.n_wait)
        late = m & self.at_top & (self.n_wait * 5 * dt > self.params.catch_s)
        for i in torch.nonzero(late).flatten().tolist():
            self._finish(i, "abort: descent (grasp point not caught up)")
        m = m & ~late
        lost = m & (self.n_lost * 5 * dt > self.params.catch_s)
        for i in torch.nonzero(lost).flatten().tolist():
            self._finish(i, "abort: descent (lost the grasp point)")
        m = m & ~lost
        slow = m & (self.n_wait * 5 * dt > self.params.t_desc_max)
        for i in torch.nonzero(slow).flatten().tolist():
            self._finish(i, "abort: descent (time)")
        m = m & ~slow
        if not bool(m.any()):
            return
        idx = torch.nonzero(m).flatten()
        h = self.h[idx]
        p_cmd = self.p_cmd[idx]
        n_, r_ = self.n[idx], self.rad[idx]
        o = self.offset[idx]
        o_w = o[:, :1] * r_ + o[:, 1:2] * torch.cross(n_, r_, dim=-1) + o[:, 2:] * n_
        p_t, R = self.sect(h, idx)
        if self.params.catch_ff:
            vf = self.point_vel(p_t, idx)
            nv = vf.norm(dim=-1, keepdim=True)
            vf = vf * (self.params.v_ff_max / nv.clamp(min=self.params.v_ff_max))
            self.v_ff[idx] = vf
            p_cmd = p_cmd + vf * 5 * dt
        p_t = p_t + o_w
        gap = p_t - p_cmd
        gn = gap.norm(dim=-1)
        step_max = self.params.v_catch * dt * 5
        caught = gn <= self.params.catch_mm / 1e3
        v = torch.where(h > self.params.h_slow, torch.full_like(h, self.params.v_desc), torch.full_like(h, self.params.v_slow))
        h = torch.where(caught, h - v * dt * 5, h)
        p_t2, R2 = self.sect(h, idx)
        p_t2 = p_t2 + o_w
        gap = torch.where(caught[:, None], p_t2 - p_cmd, gap)
        R = torch.where(caught[:, None, None], R2, R)
        gn = gap.norm(dim=-1)
        p = p_cmd + torch.where((gn <= step_max)[:, None], gap, gap * (step_max / gn.clamp(min=1e-12))[:, None])
        q, ep, er = self.A.ik(self.bp[idx], self.bR[idx], p, R, self.q_tgt[idx], 20)
        bad = (ep > 1e-3) | (er > 0.1)
        for j in torch.nonzero(bad).flatten().tolist():
            self._finish(int(idx[j]), "abort: IK in the descent")
        okk = ~bad
        ii = idx[okk]
        self.h[ii] = h[okk]
        self.at_top[ii] = self.at_top[ii] & ~caught[okk]
        self.n_lost[ii] = torch.where(caught[okk], torch.zeros_like(self.n_lost[ii]), self.n_lost[ii] + (~self.at_top[ii]).long())
        self.p_cmd[ii] = p[okk]
        self.p_tick[ii] = p[okk]
        self.qd_ff[ii] = (q[okk] - self.q_tgt[ii]) / (5 * dt)
        self.q_tgt[ii] = q[okk]
        self.k_tick[ii] = 5

    def _start_close(self, i: int) -> None:
        self.guard_on[i] = False
        self.trip[i] = -1
        p, _ = self._site()
        p0, _ = self.sect(torch.zeros(1, dtype=torch.float64, device=self.dev), torch.tensor([i], device=self.dev))
        self.h[i] = float(((p[i] - p0[0]) * self.n[i]).sum())
        d = self.sq_cmd - self.q_open.double()
        T_ramp = float(d.abs().max()) / self.params.v_jaw
        self.n_ramp = max(1, int(T_ramp / self.dt))
        self.n_tot = self.n_ramp + int(2.0 / self.dt)
        self.d_jaw = d
        self.k_close[i] = 0
        self.phase[i] = PH_CLOSE
        self.k_phase[i] = 0
        self.k_tick[i] = 0
        self.info[i]["close_s"] = round(T_ramp + 2.0, 2)

    def _close_tick(self, m: torch.Tensor) -> None:
        dt = self.dt
        done = m & (self.k_close >= self.n_tot)
        if bool(done.any()):
            idx = torch.nonzero(done).flatten()
            jv = self.qpos[idx][:, self.gq]
            for j, e in enumerate(self.ev_list):
                self.eqm[idx, e, 0] = jv[:, j]
            self.eqa[idx[:, None], self.ev[None]] = True
            self.grip[idx] = self.sq_cmd
            self.qd_ff[idx] = 0.0
            for i, row in zip(idx.tolist(), jv.tolist()):
                self.info[i]["jaw_mm"] = [round(x * 1e3, 2) for x in row]
            self._set_phase(done, PH_HOLD)
        m = m & ~done
        if not bool(m.any()):
            return
        idx = torch.nonzero(m).flatten()
        fp = self.touches()[idx, 0]
        h = self.h[idx] - self.params.v_follow * 5 * dt * ((self.params.f_palm - fp) / self.params.f_palm).clamp(-1.0, 1.0)
        p, R = self.sect(h, idx)
        q, _, _ = self.A.ik(self.bp[idx], self.bR[idx], p, R, self.q_tgt[idx], 20)
        k = self.k_close[idx]
        frac = ((k + 5).double() / self.n_ramp).clamp(max=1.0)
        self.h[idx] = h
        self.qd_ff[idx] = 0.0
        self.q_tgt[idx] = q
        self.grip[idx] = self.q_open.double()[None] + self.d_jaw[None] * frac[:, None]
        self.k_close[idx] = k + 5
        self.k_tick[idx] = 5

    def _end_hold(self, m: torch.Tensor) -> None:
        idx = torch.nonzero(m).flatten()
        t = self.t_sum / self.t_n.clamp(min=1)[:, None]
        s = self.seat()
        vcli = self.com_vel().norm(dim=-1) * 1e3
        for i in idx.tolist():
            self.info[i]["closed"] = dict(
                seat={k: round(float(v[i]), 2) for k, v in s.items()},
                touch={k: round(float(t[i, j]), 1) for j, k in enumerate(self.params.touch_names)},
                client_v_mm_s=round(float(vcli[i]), 2),
            )
        if not self.hold_check:
            for i in idx.tolist():
                self._classify(i, None)
            return
        p, R = self._rel()
        if self.rel0 is None:
            self.rel0 = (p.clone(), R.clone())
        self.rel0[0][idx], self.rel0[1][idx] = p[idx], R[idx]
        self.load_ax[idx] = 0
        self._set_phase(m, PH_LOAD)

    def _load(self, m: torch.Tensor) -> None:
        dt = self.dt
        n_on, n_off = int(2.0 / dt), int(0.5 / dt)
        kp = self.k_phase
        ax = self.load_ax
        on = m & (kp < n_on)
        w = torch.zeros(self.N, 6, dtype=self.xfrc.dtype, device=self.dev)
        w[torch.arange(self.N, device=self.dev), 3 + ax.clamp(max=2)] = self.params.load_nm
        target_buf = self.ext_xfrc if self.ext_xfrc is not None else self.xfrc
        target_buf[:, self.bid_t] = torch.where(
            on[:, None], w, torch.where(m[:, None], torch.zeros_like(w), target_buf[:, self.bid_t])
        )
        blk = m & (kp >= n_on + n_off)
        if bool(blk.any()):
            p, R = self._rel()
            assert self.rel0 is not None
            p0, R0 = self.rel0
            dp = (p - p0).norm(dim=-1) * 1e3
            dang = torch.rad2deg(rotvec(R0.transpose(1, 2) @ R).norm(dim=-1))
            self.slip[:, 0] = torch.where(blk, torch.maximum(self.slip[:, 0], dp), self.slip[:, 0])
            self.slip[:, 1] = torch.where(blk, torch.maximum(self.slip[:, 1], dang), self.slip[:, 1])
            self.load_ax = torch.where(blk, ax + 1, ax)
            self.k_phase = torch.where(blk, torch.zeros_like(kp), self.k_phase)
            fin = blk & (self.load_ax >= 3)
            for i in torch.nonzero(fin).flatten().tolist():
                self.info[i]["load"] = dict(
                    slip_mm=round(float(self.slip[i, 0]), 2), slip_deg=round(float(self.slip[i, 1]), 2)
                )
                self._classify(i, self.info[i]["load"])
            if bool(fin.any()):
                self.eqa[torch.nonzero(fin).flatten()[:, None], self.ev[None]] = False

    def _classify(self, i: int, load: Optional[Dict[str, float]]) -> None:
        c = self.info[i]["closed"]
        t, s = c["touch"], c["seat"]
        self.info[i]["peak"] = {f"pad_{k}": round(float(self.peak[i, j]), 1) for j, k in enumerate(self.params.touch_names)}
        grip = t["in"] > 5 and (t["hook"] > 5 or t["lip"] > 5) and (t["palm"] > 5 or t["lip"] > 5)
        seated = (
            abs(s["rad_mm"] - self.params.seat0_rad_mm) < self.params.seat_mm
            and abs(s["h_mm"] - self.params.seat0_h_mm) < self.params.seat_mm
            and s["tilt_deg"] < self.params.seat_deg
        )
        held = load is None or (load["slip_mm"] < self.params.slip_mm and load["slip_deg"] < self.params.slip_deg)
        res = (
            "grip"
            if grip and seated and held
            else "grip, slips"
            if grip and seated
            else "grip, off seat"
            if grip
            else "no grip"
        )
        self._finish(i, res)

    # ------------------------------------------------------------------------------------------------------ unified rollout
    def run(
        self,
        policy: Any = None,
        *,
        max_steps: int = 40000,
        dec: int = 50,
        deterministic: bool = False,
        collect: bool = False,
        early_stop_frac: float = 1.0,
    ) -> Tuple[Optional[Dict[str, torch.Tensor]], List[str]]:
        """Run one synchronous batched episode, optionally querying G3Policy at 10 Hz during descent."""
        s_prev: Optional[torch.Tensor] = None
        a_prev = torch.zeros(self.N, 3, dtype=torch.float64, device=self.dev)
        buf: Dict[str, List[torch.Tensor]] = dict(obs=[], act=[], logp=[], val=[], valid=[])
        tb = time.time()
        for k in range(max_steps):
            tb = thermal_guard(k, tb, duty=self.duty)
            if policy is not None and (k % dec) == 0:
                self._update_estimate()
                m = self.phase == PH_DESC
                if bool(m.any()):
                    p_t, _ = self.sect(self.h)
                    p_site, _ = self._site()
                    wf = self.sens[:, self.s_wf].double() - self.guard
                    x, s_now = build_obs_torch(
                        rad=self.rad,
                        n=self.n,
                        p_target=p_t,
                        p_site=p_site,
                        p_cmd=self.p_cmd,
                        offset=self.offset,
                        s_prev=s_prev,
                        a_prev=a_prev,
                        a_w=self.a_w,
                        h=self.h,
                        at_top=self.at_top,
                        n_wait=self.n_wait,
                        dt=self.dt,
                        dec=dec,
                        catch_s=self.params.catch_s,
                        touches=self.touches(),
                        wf_delta=wf,
                        o_max=self.o_max,
                    )
                    s_prev = s_now
                    a, logp, val = policy.act(x, deterministic=deterministic)
                    if collect:
                        buf["obs"].append(x)
                        buf["act"].append(a)
                        buf["logp"].append(logp)
                        buf["val"].append(val)
                        buf["valid"].append(m.clone())
                    a64 = torch.where(m[:, None], a.double(), torch.zeros_like(a_prev))
                    self.offset = (self.offset + a64 * (self.a_vel * dec * self.dt)).clamp(-self.o_max, self.o_max)
                    a_prev = a64
            self.control()
            if early_stop_frac >= 1.0:
                if bool((self.phase == PH_DONE).all()):
                    break
            elif (k % dec) == 0 and float((self.phase == PH_DONE).float().mean()) >= early_stop_frac:
                break
            self.physics_step()
            self.k_tick = (self.k_tick - 1).clamp(min=0)

        for i in range(self.N):
            if self.result[i] is None:
                self.result[i] = "timeout"
        cls = [classify_outcome(self.result[i], bool(self.bad[i]) or bool(self.bad0[i])) for i in range(self.N)]
        rollout = {k: torch.stack(v) for k, v in buf.items()} if (collect and buf["obs"]) else None
        return rollout, cls
