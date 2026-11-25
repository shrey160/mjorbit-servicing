"""Shared constants, spec-backed capture parameters, noise calibration, and GPU utilities for P7 RL."""
from __future__ import annotations

import itertools
import json
import math
import pickle
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
from _paths import RUNS_DIR as RUNS, SIM_SRC, SRC_DIR as P7, ensure_runs_dir, find_weight  # noqa: E402
from specs import ClientSpec, MissionConfig, ServicerSpec, client_p7, servicer_rev2  # noqa: E402

# Phase indices matching GPU batched rollout state machine
PH_CART = 0
PH_SETTLE = 1
PH_DESC = 2
PH_CLOSE = 3
PH_HOLD = 4
PH_LOAD = 5
PH_DONE = 6
PH_NAME = ("cart", "settle", "descent", "close", "hold", "load", "done")

HOLD_AVG = 100
TOUCH_PAD = 0.001
W_MAX = 64

# Canonical earth constants (upstream mjorbit src/cpp/src/gravity.cc)
GM_EARTH_KM3_S2 = 398600.4418
R_EARTH_KM = 6378.137

# Default thermal / duty guardrails (80 C pause, 72 C resume, 87 C hard stop, half duty)
DEFAULT_DUTY = 0.5
TEMP_PAUSE_C = 80
TEMP_RESUME_C = 72
TEMP_STOP_C = 87


@dataclass(frozen=True)
class CaptureParams:
    """Single source of truth for reach/capture parameters backed by ClientSpec, ServicerSpec, MissionConfig."""

    # Client ring & seat geometry
    c_b: Tuple[float, float, float]
    n_b: Tuple[float, float, float]
    h_top: float
    r_grasp: float
    seat_nominal_rad_mm: float
    seat_nominal_h_mm: float
    client_body_name: str

    # Servicer arm & tool
    servicer_body_name: str
    scene_xml_path: str
    arm_joints: Tuple[str, ...]
    arm_acts: Tuple[str, ...]
    kp: Tuple[float, ...]
    kdv: Tuple[float, ...]
    qd_max_rad_s: float
    grip_joints: Tuple[str, ...]
    grip_acts: Tuple[str, ...]
    lock_eqs: Tuple[str, ...]
    q_open: Tuple[float, float]
    q_squeeze: float
    jaw_close_signs: Tuple[float, float]
    ee_site: str
    touch_names: Tuple[str, ...]
    wrist_ft_name: str
    tool_bodies: Tuple[str, ...]
    cam_geoms: Tuple[str, ...]
    tool_friction: float

    # Mission timing & orbital parameters
    orbit_alt_km: float
    orbit_inc_deg: float
    sun_phase_deg: float
    lat_s: float
    lat_j_s: float
    lidar_dt: float

    # Reach / descent / close / hold thresholds (canonical reach_bench constants)
    h_pre: float = 0.15
    h_close: float = 0.075
    over: float = 0.025
    v_desc: float = 0.010
    h_slow: float = 0.015
    v_slow: float = 0.003
    v_catch: float = 0.030
    catch_mm: float = 1.0
    catch_s: float = 3.0
    v_ff_max: float = 0.060
    t_desc_max: float = 30.0
    touch_n: float = 1.0
    ft_n: float = 2.0
    f_palm: float = 5.0
    v_follow: float = 0.005
    v_jaw: float = 0.010
    tau_ff: float = 0.15
    vff_max: float = 0.5
    load_nm: float = 3.0
    seat_mm: float = 3.0
    seat_deg: float = 1.5
    slip_mm: float = 3.0
    slip_deg: float = 3.0
    seat0_rad_mm: float = 1.2
    seat0_h_mm: float = 0.0
    track_mm: float = 12.0
    ff_enabled: bool = True
    catch_ff: bool = True

    @classmethod
    def from_specs(
        cls,
        client: Optional[ClientSpec] = None,
        servicer: Optional[ServicerSpec] = None,
        cfg: Optional[MissionConfig] = None,
        *,
        track_mm: float = 12.0,
        ff_enabled: bool = True,
        catch_ff: bool = True,
    ) -> "CaptureParams":
        c = client if client is not None else client_p7()
        s = servicer if servicer is not None else servicer_rev2()
        m = cfg if cfg is not None else MissionConfig()
        return cls(
            c_b=tuple(float(x) for x in c.ring.c_b),  # type: ignore[arg-type]
            n_b=tuple(float(x) for x in c.ring.n_b),  # type: ignore[arg-type]
            h_top=float(c.ring.h_top),
            r_grasp=float(c.ring.r_grasp),
            seat_nominal_rad_mm=float(c.ring.seat_nominal_rad_mm),
            seat_nominal_h_mm=float(c.ring.seat_nominal_h_mm),
            client_body_name=str(c.body_name),
            servicer_body_name=str(s.body_name),
            scene_xml_path=str(s.scene_xml_path),
            arm_joints=tuple(s.arm.joint_names),
            arm_acts=tuple(s.arm.act_names),
            kp=tuple(float(x) for x in s.arm.kp),
            kdv=tuple(float(x) for x in s.arm.kdv),
            qd_max_rad_s=float(s.arm.qd_max_rad_s),
            grip_joints=tuple(s.tool.grip_joints),
            grip_acts=tuple(s.tool.grip_acts),
            lock_eqs=tuple(s.tool.lock_eqs),
            q_open=(float(s.tool.q_open[0]), float(s.tool.q_open[1])),
            q_squeeze=float(s.tool.q_squeeze),
            jaw_close_signs=(float(s.tool.jaw_close_signs[0]), float(s.tool.jaw_close_signs[1])),
            ee_site=str(s.tool.ee_site),
            touch_names=tuple(s.tool.touch_names),
            wrist_ft_name=str(s.tool.wrist_ft_name),
            tool_bodies=tuple(s.tool.tool_bodies),
            cam_geoms=tuple(s.tool.cam_geoms),
            tool_friction=float(s.tool.friction),
            orbit_alt_km=float(m.orbit_alt_km),
            orbit_inc_deg=float(m.orbit_inc_deg),
            sun_phase_deg=float(m.sun_phase_deg),
            lat_s=float(m.lat_s),
            lat_j_s=float(m.lat_j_s),
            lidar_dt=float(m.lidar_dt),
            seat0_rad_mm=float(c.ring.seat_nominal_rad_mm),
            seat0_h_mm=float(c.ring.seat_nominal_h_mm),
            track_mm=float(track_mm),
            ff_enabled=bool(ff_enabled),
            catch_ff=bool(catch_ff),
        )


def patch_model_for_warp(km: mujoco.MjModel, params: CaptureParams, trim: Any = None) -> None:
    """Apply Warp float32 / tool friction / noslip=0 overrides directly to an MjModel."""
    km.opt.noslip_iterations = 0
    km.opt.gravity[:] = 0.0
    prefixes = ("ring_wall", "ring_lip", "pad_u", "pad_w", "pad_l", "hook_finger")
    exact = ("palm", "tool_housing", "tool_neck")
    for g in range(km.ngeom):
        nm = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if nm.startswith(prefixes) or nm in exact:
            km.geom_friction[g, 0] = params.tool_friction
    for k in params.touch_names:
        sid = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, f"touch_{k}")
        if sid >= 0:
            km.site_size[sid] += TOUCH_PAD
    if trim is not None:
        trim(km)


def extract_model_indices(km: mujoco.MjModel, params: CaptureParams) -> Dict[str, Any]:
    """Read joint, actuator, sensor, equality, and body/geom class indices directly from km (no EGL renderers)."""
    jid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    bid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
    gid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
    aid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_ACTUATOR, n)  # noqa: E731
    sid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SENSOR, n)  # noqa: E731
    eid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_EQUALITY, n)  # noqa: E731

    arm_jids = [jid(j) for j in params.arm_joints]
    arm_q = [int(km.jnt_qposadr[j]) for j in arm_jids]
    arm_v = [int(km.jnt_dofadr[j]) for j in arm_jids]
    act_arm = [aid(a) for a in params.arm_acts]
    gq = [int(km.jnt_qposadr[jid(j)]) for j in params.grip_joints]
    ag = [aid(a) for a in params.grip_acts]

    cj = jid("chaser_joint")
    tj = jid("target_joint")
    cq, cv = int(km.jnt_qposadr[cj]), int(km.jnt_dofadr[cj])
    tq, tv = int(km.jnt_qposadr[tj]), int(km.jnt_dofadr[tj])
    bid_c, bid_t = bid(params.servicer_body_name), bid(params.client_body_name)

    jr = km.jnt_range[arm_jids]
    jlo, jhi = jr[:, 0] + 0.02, jr[:, 1] - 0.02

    s_touch = [int(km.sensor_adr[sid(f"touch_{k}")]) for k in params.touch_names]
    wf_adr = int(km.sensor_adr[sid(params.wrist_ft_name)])
    s_wf = [wf_adr, wf_adr + 1, wf_adr + 2]
    ev = [eid(n) for n in params.lock_eqs]

    tool_b = {bid(n) for n in params.tool_bodies}
    cam_g = gid(params.cam_geoms[0]) if params.cam_geoms else -1
    root_c = int(km.body_rootid[bid_c])
    root_t = int(km.body_rootid[bid_t])

    g_side = np.zeros(km.ngeom, np.int8)
    g_tool = np.zeros(km.ngeom, bool)
    g_ring = np.zeros(km.ngeom, bool)
    for g in range(km.ngeom):
        b = int(km.geom_bodyid[g])
        g_side[g] = 1 if int(km.body_rootid[b]) == root_c else (2 if int(km.body_rootid[b]) == root_t else 0)
        g_tool[g] = (b in tool_b) and (g != cam_g)
        gn = mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        g_ring[g] = gn.startswith("ring_")

    return dict(
        arm_q=arm_q,
        arm_v=arm_v,
        act_arm=act_arm,
        gq=gq,
        ag=ag,
        cq=cq,
        cv=cv,
        tq=tq,
        tv=tv,
        bid_c=bid_c,
        bid_t=bid_t,
        jlo=jlo,
        jhi=jhi,
        s_touch=s_touch,
        s_wf=s_wf,
        ev=ev,
        g_side=g_side,
        g_tool=g_tool,
        g_ring=g_ring,
    )


def _solve_gaussian_mixture(p50: float, p95: float, p_tail: float = 0.15) -> Tuple[float, float]:
    """Fit zero-mean 2-component Gaussian mixture (1-p_tail)*N(0,s1^2) + p_tail*N(0,s2^2) to |X| p50 & p95."""
    p50 = max(float(p50), 1e-9)
    p95 = max(float(p95), p50 * 2.91)

    def cdf(x: float, s1: float, s2: float) -> float:
        rt2 = math.sqrt(2.0)
        return (1.0 - p_tail) * math.erf(x / max(s1 * rt2, 1e-12)) + p_tail * math.erf(x / max(s2 * rt2, 1e-12))

    # Alternating 1D bisection on s1 (matched at p50) and s2 (matched at p95)
    s1 = p50 / 0.77
    s2 = max(p95 / 1.04, 1.5 * s1)
    for _ in range(30):
        lo, hi = 1e-6 * p50, 10.0 * p50
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if cdf(p50, mid, s2) > 0.50:
                lo = mid
            else:
                hi = mid
        s1 = 0.5 * (lo + hi)

        lo, hi = s1, 20.0 * p95
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if cdf(p95, s1, mid) > 0.95:
                lo = mid
            else:
                hi = mid
        s2 = 0.5 * (lo + hi)
    return float(s1), float(s2)


@dataclass(frozen=True)
class StereoNoiseModel:
    """Calibrated perception error & refit/station uncertainty model."""

    name: str
    rad_core_m: float
    rad_tail_m: float
    tan_core_m: float
    tan_tail_m: float
    tilt_rad_core_deg: float
    tilt_rad_tail_deg: float
    tilt_tan_core_deg: float
    tilt_tan_tail_deg: float
    p_tail: float = 0.15
    use_cell_tilt: bool = False
    ax_mu_m: float = 5.0e-3
    sig_ax_m: float = 7.0e-3
    sig_p_m: float = 2.0e-3
    sig_a_deg: float = 0.10
    sig_w_deg_s: float = 0.02
    tau_e_s: float = 20.0
    sig_st_m: float = 1.5e-3
    sig_sta_deg: float = 0.15
    t_refit_s: float = 1.0
    lat_s: float = 0.15
    lat_j_s: float = 0.02
    prop_now: bool = True

    @classmethod
    def from_json(cls, path: Path | str, *, t_refit_s: float = 1.0, p_tail: float = 0.15) -> "StereoNoiseModel":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        p50 = d["err_p50"]
        p95 = d["err_p95"]
        rc, rt = _solve_gaussian_mixture(p50[0] * 1e-3, p95[0] * 1e-3, p_tail=p_tail)
        tc, tt = _solve_gaussian_mixture(p50[1] * 1e-3, p95[1] * 1e-3, p_tail=p_tail)
        trc, trt = _solve_gaussian_mixture(p50[3], p95[3], p_tail=p_tail)
        ttc, ttt = _solve_gaussian_mixture(p50[4], p95[4], p_tail=p_tail)
        return cls(
            name=Path(path).stem,
            rad_core_m=rc,
            rad_tail_m=rt,
            tan_core_m=tc,
            tan_tail_m=tt,
            tilt_rad_core_deg=trc,
            tilt_rad_tail_deg=trt,
            tilt_tan_core_deg=ttc,
            tilt_tan_tail_deg=ttt,
            p_tail=p_tail,
            use_cell_tilt=False,
            t_refit_s=float(t_refit_s),
        )

    @classmethod
    def preset(cls, kind: str = "stereo_plain", *, t_refit_s: float = 1.0) -> "StereoNoiseModel":
        if kind == "stereo_plain":
            return cls(
                name="stereo_plain",
                rad_core_m=0.35e-3,
                rad_tail_m=0.35e-3,
                tan_core_m=0.55e-3,
                tan_tail_m=0.55e-3,
                tilt_rad_core_deg=0.18,
                tilt_rad_tail_deg=0.70,
                tilt_tan_core_deg=0.18,
                tilt_tan_tail_deg=0.70,
                p_tail=0.10,
                use_cell_tilt=False,
                t_refit_s=float(t_refit_s),
            )
        if kind == "net":
            return cls(
                name="net",
                rad_core_m=2.5e-3,
                rad_tail_m=2.5e-3,
                tan_core_m=3.0e-3,
                tan_tail_m=3.0e-3,
                tilt_rad_core_deg=0.60,
                tilt_rad_tail_deg=0.60,
                tilt_tan_core_deg=0.60,
                tilt_tan_tail_deg=0.60,
                p_tail=0.0,
                use_cell_tilt=True,
                t_refit_s=float(t_refit_s),
            )
        raise ValueError(f"Unknown noise preset: {kind!r}")

    def sample_mixture(self, rng: np.random.Generator, n: int, core: float, tail: float) -> np.ndarray:
        if self.p_tail <= 0.0 or tail == core:
            return rng.normal(0.0, core, n)
        sig = np.where(rng.uniform(size=n) < self.p_tail, tail, core)
        return rng.normal(0.0, 1.0, n) * sig


def load_starts(path: Path | str | None = None) -> List[Dict[str, Any]]:
    p = Path(path) if path is not None else find_weight("g1_starts.pkl")
    return [r for r in pickle.loads(p.read_bytes()) if "state" in r]


def classify_outcome(res: Optional[str], bad: bool) -> str:
    r = res or "timeout"
    if bad:
        return "collision"
    if r == "grip":
        return "captured"
    if r.startswith("abort") or r == "timeout":
        return "safe abort"
    return "failed close"


def make_eval_grid(
    w_ax: Sequence[float] = (0.0, 2.0, 4.0),
    w_tr: Sequence[float] = (0.0, 0.5, 1.0),
    tilt_r: Sequence[float] = (0.0, 1.0, 2.0, 3.0),
    scales: Sequence[float] = (1.0,),
    per: int = 37,
) -> List[Dict[str, Any]]:
    cells: List[Dict[str, Any]] = []
    for wa, wt, tr, sc in itertools.product(w_ax, w_tr, tilt_r, scales):
        key = json.dumps(dict(w_ax=wa, w_tr=wt, tilt_r=tr, scale=sc))
        cells.extend(dict(cell=key, w_ax=float(wa), w_tr=float(wt), tilt_r=float(tr), scale=float(sc)) for _ in range(per))
    return cells


def make_orbital_orbits(nworld: int, cfg: MissionConfig, rng: np.random.Generator) -> List[Any]:
    """Generate per-world OrbitInit values matching MjorbitBackend circular LEO SSO dynamics."""
    from mjorbit.config import OrbitInit

    r_km = R_EARTH_KM + float(cfg.orbit_alt_km)
    v_km_s = math.sqrt(GM_EARTH_KM3_S2 / r_km)
    inc = math.radians(float(cfg.orbit_inc_deg))
    ci, si = math.cos(inc), math.sin(inc)

    raans = rng.uniform(0.0, 2.0 * math.pi, nworld)
    u_angs = rng.uniform(0.0, 2.0 * math.pi, nworld)
    t_epochs = rng.uniform(0.0, 365.0 * 86400.0, nworld)

    orbits = []
    for i in range(nworld):
        ca, sa = math.cos(float(raans[i])), math.sin(float(raans[i]))
        cu, su = math.cos(float(u_angs[i])), math.sin(float(u_angs[i]))
        node = np.array([ca, sa, 0.0], dtype=float)
        perp = np.array([-sa * ci, ca * ci, si], dtype=float)
        r_hat = cu * node + su * perp
        v_hat = -su * node + cu * perp
        orbits.append(OrbitInit(R_eci=r_km * r_hat, V_eci=v_km_s * v_hat, t=float(t_epochs[i])))
    return orbits


def gpu_temp() -> int:
    import subprocess

    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout.split()[0]
        return int(out)
    except Exception:  # noqa: BLE001
        return -1


class GPUThermalStop(RuntimeError):
    """Raised when GPU temperature reaches the hard stop threshold."""


def thermal_guard(
    step: int,
    tb: float,
    *,
    duty: float = DEFAULT_DUTY,
    t_pause: int = TEMP_PAUSE_C,
    t_resume: int = TEMP_RESUME_C,
    t_stop: int = TEMP_STOP_C,
    check_every: int = 200,
    temp_every: int = 2000,
) -> float:
    """Enforce half-duty pacing and pause/abort thermal limits for GPU runs."""
    if step <= 0 or (step % check_every) != 0:
        return tb
    import torch

    torch.cuda.synchronize()
    if 0.0 < duty < 1.0:
        time.sleep(max(0.0, time.time() - tb) * (1.0 / duty - 1.0))
    if (step % temp_every) == 0:
        temp = gpu_temp()
        if temp >= t_stop:
            raise GPUThermalStop(f"GPU temperature {temp} C >= hard stop {t_stop} C")
        if temp >= t_pause:
            print(f"GPU {temp} C >= {t_pause} C: pausing until <= {t_resume} C", flush=True)
            while temp > t_resume:
                time.sleep(10)
                temp = gpu_temp()
                if temp >= t_stop:
                    raise GPUThermalStop(f"GPU temperature {temp} C >= hard stop {t_stop} C")
    return time.time()
