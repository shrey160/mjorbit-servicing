"""Unit & integration checks for P7 modular specs, SimBackend (incl. mjorbit 10s parity + Earth/stars), and lidar_acq."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SRC_DIR = HERE.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
import _paths  # noqa: E402,F401

from gnc.sensors import SensorSuite, extract_client_boxes, randomize_bench_mount  # noqa: E402
from sim.backend import MjorbitBackend, PlainMujocoBackend  # noqa: E402
from sim.optics import apply_space_optics  # noqa: E402
from specs import ClientPerceptionSpec, ClientSpec, client_double_wing, client_p7, servicer_rev2  # noqa: E402
import cams  # noqa: E402
import lidar_acq  # noqa: E402
import lidar_nav  # noqa: E402


def test_keepout_smoothness():
    c1 = client_p7()
    c2 = client_double_wing()
    c0 = ClientSpec(
        name="no_wing",
        body_name=c1.body_name,
        xml_path=c1.xml_path,
        bus_radius_m=c1.bus_radius_m,
        r_bus_ko_m=3.5,
        r_hold3_m=c1.r_hold3_m,
        r_hold18_m=c1.r_hold18_m,
        r_station_m=c1.r_station_m,
        ring=c1.ring,
        wings=(),
        perception=ClientPerceptionSpec(acq_strategy="box_grid"),
        r_rel_mat=c1.r_rel_mat,
    )
    # 0-wing constant 3.5 m
    for th in np.linspace(0, np.pi, 37):
        d = np.array([np.cos(th), np.sin(th), 0.0])
        assert abs(c0.keepout_radius(d) - 3.5) < 1e-12

    # 1-wing monotonic & C^1 continuous (max step per 1 deg <= 0.15 m, no 2.5 m jump!)
    vals1 = [c1.keepout_radius(np.array([np.sin(th), np.cos(th), 0.0])) for th in np.radians(np.arange(0, 181))]
    assert abs(vals1[0] - 6.0) < 1e-6
    assert abs(vals1[-1] - 3.5) < 1e-6
    assert max(abs(vals1[i + 1] - vals1[i]) for i in range(len(vals1) - 1)) < 0.14

    # 2-wing symmetric at +Y (0 deg) and -Y (180 deg), 3.5 m at +X (90 deg)
    vals2 = [c2.keepout_radius(np.array([np.sin(th), np.cos(th), 0.0])) for th in np.radians(np.arange(0, 181))]
    assert abs(vals2[0] - 6.0) < 1e-6 and abs(vals2[-1] - 6.0) < 1e-6
    assert abs(vals2[90] - 3.5) < 1e-6
    assert max(abs(vals2[i + 1] - vals2[i]) for i in range(len(vals2) - 1)) < 0.14
    print("PASS: test_keepout_smoothness (0/1/2-wing)")


def test_servicer_spec():
    s = servicer_rev2()
    c = client_p7()
    busd = s.load_bus_dict()
    w_lim_deg = np.degrees(s.spin_limit_rad_s(c, mass_kg=float(busd["mass_kg"]) + 16.2))
    assert 1.8 < w_lim_deg <= 2.5
    q0 = np.asarray(s.arm.stow_q)
    qr = np.asarray(s.arm.ready_q)
    qa = qr.copy()
    wp = s.unfold_waypoints(q0, qr, qa)
    wr = s.restow_waypoints(qa, qr, q0)
    assert len(wp) == 4 and len(wr) == 4
    print(f"PASS: test_servicer_spec (spin_limit={w_lim_deg:.2f} deg/s)")


def test_mjorbit_parity_and_earth_render():
    import mjorbit
    from mjorbit.spec import MjoSpec

    s = servicer_rev2()
    c = client_p7()
    xml_path = s.scene_xml_path

    # 1) Exact 10 s two-body free-drift parity check: MjorbitBackend vs upstream mjorbit.mjo_step
    be = MjorbitBackend(xml_path, orbit_alt_km=550.0, orbit_inc_deg=97.6, earth_bg=True, wrench_refresh_dt=0.002)
    rng = np.random.default_rng(123)
    be.reset(rng=rng)

    # Set clean 10 m separation (off-axis so tidal gravity gradient & differential J2 are active!)
    bid_c = mujoco.mj_name2id(be.km, mujoco.mjtObj.mjOBJ_BODY, s.body_name)
    bid_t = mujoco.mj_name2id(be.km, mujoco.mjtObj.mjOBJ_BODY, c.body_name)
    cq = int(be.km.jnt_qposadr[be.km.body_jntadr[bid_c]])
    tq = int(be.km.jnt_qposadr[be.km.body_jntadr[bid_t]])
    cv = int(be.km.jnt_dofadr[be.km.body_jntadr[bid_c]])
    tv = int(be.km.jnt_dofadr[be.km.body_jntadr[bid_t]])

    be.kd.qpos[tq : tq + 3] = [0.0, 0.0, 0.0]
    be.kd.qpos[cq : cq + 3] = [20.0, 15.0, -10.0]
    be.kd.qvel[tv : tv + 6] = 0.0
    be.kd.qvel[cv : cv + 6] = 0.0
    be.kd.eq_active[:] = 0
    be.forward()

    # Upstream mjorbit reference with exact same OrbitInit and state
    orb0 = mjorbit.OrbitInit(
        R_eci=be.oracle_data.orbit.R_eci.copy(),
        V_eci=be.oracle_data.orbit.V_eci.copy(),
        t=float(be.oracle_data.orbit.t),
    )
    mjo_spec = MjoSpec.from_xml_path(str(xml_path))
    mjo_spec.mjorbit.use_j2 = True
    mjo_spec.mjorbit.use_gravity_gradient = True
    mjo_spec.mjorbit.use_drag = False
    mjo_spec.mjorbit.use_srp = False
    mjo_spec.mjorbit.use_magnetic = False
    mjo_m = mjo_spec.compile()
    mjo_d = mjo_m.make_data(orbit=orb0)
    mjo_d.qpos[:] = be.kd.qpos
    mjo_d.qvel[:] = be.kd.qvel
    mjorbit.mjo_forward(mjo_m, mjo_d)

    # Also compare vs PlainMujocoBackend over 10 s to verify nonzero tidal drift!
    pb = PlainMujocoBackend(xml_path)
    pb.reset(rng=np.random.default_rng(123))
    pb.kd.qpos[:] = be.kd.qpos
    pb.kd.qvel[:] = be.kd.qvel
    pb.kd.eq_active[:] = 0
    pb.forward()

    n_steps = int(round(10.0 / be.dt))
    for _ in range(n_steps):
        be.step_physics()
        mjorbit.mjo_step(mjo_m, mjo_d)
        pb.step_physics()

    pos_err_m = float(np.linalg.norm(be.kd.xpos[bid_c] - mjo_d.xpos[bid_c]))
    cli_err_m = float(np.linalg.norm(be.kd.xpos[bid_t] - mjo_d.xpos[bid_t]))
    tidal_diff_mm = float(np.linalg.norm(be.kd.xpos[bid_c] - pb.kd.xpos[bid_c])) * 1e3
    assert pos_err_m < 5e-3, f"Chaser 10s parity error too large: {pos_err_m}"
    assert cli_err_m < 5e-3, f"Client 10s parity error too large: {cli_err_m}"
    assert tidal_diff_mm > 0.1, f"Expected nonzero orbital tide delta vs zero-g, got {tidal_diff_mm} mm"

    # 2) Verify earth_bg=True RGB render (Earth limb visible!) vs clean lidar_points
    be.reset(rng=np.random.default_rng(77))
    # Place servicer looking slightly toward Earth-limb side of client
    be.kd.qpos[tq : tq + 7] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    r_hat = be.oracle_data.orbit.R_eci / np.linalg.norm(be.oracle_data.orbit.R_eci)
    # Place chaser at 12 m toward +R_eci looking back along -R_eci (Earth fills behind client!)
    p_s = 12.0 * r_hat
    cams.LIDAR_MAX = 60.0
    rig = cams.CamRig(be.km, res={"wrist_cam": 128, "bus_cam": 512})
    ss = SensorSuite(be, rig, s.sensors, bid_c)
    randomize_bench_mount(be, rig, [ss.bcam, ss.lid], s.sensors, np.random.default_rng(77), 77)
    from gnc.nav import point_R

    R_s = point_R(ss.bore_b, -r_hat)
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R_s).as_quat()
    be.kd.qpos[cq : cq + 7] = [*p_s, w, x, y, z]
    be.forward()

    img = ss.bus_cam_rgb()
    P_lid, _ = ss.lidar_points()
    assert img.shape == (512, 512, 3) and img.dtype == np.uint8
    # Verify textured Earth + stars appear in RGB background (much more lit pixels than client alone)
    frac_lit = float((img.mean(axis=2) > 15).mean())
    assert frac_lit > 0.10, f"Expected Earth limb visible in bus_cam behind client, got lit frac={frac_lit:.4f}"
    # Verify lidar max range stays < 25 m (never detects Earth at 119+ m)
    max_lid_r = float(np.linalg.norm(P_lid, axis=1).max()) if len(P_lid) else 0.0
    assert 5.0 < max_lid_r < 25.0, f"Lidar corrupted by Earth geom! max_r={max_lid_r}"

    # 3) Verify lidar_acq.acquire works cleanly with ClientSpec on P_lid
    lidar_nav.set_model(extract_client_boxes(be.km, bid_t))
    lid_id = rig.lidar_id
    Pl_nom, Rl_nom = ss.cam_nominal(lid_id)
    t_bus_l = Rl_nom.T @ (-Pl_nom)
    acq1 = lidar_acq.acquire(P_lid, t_bus_l, client_spec=c)
    acq2 = lidar_acq.acquire(P_lid, t_bus_l, client_spec=client_double_wing())
    assert acq1["ok"] and acq2["ok"]

    print(
        f"PASS: test_mjorbit_parity_and_earth_render "
        f"(10s parity: chaser={pos_err_m*1e3:.4f} mm, client={cli_err_m*1e3:.4f} mm, "
        f"tide_vs_zerog={tidal_diff_mm:.3f} mm, earth_lit_frac={frac_lit:.3f}, lid_max={max_lid_r:.2f} m)"
    )

    # 4) Verify upgraded Earth mesh, hollow atmo rings, Earthshine light, terminator phase, and optics
    assert mujoco.mj_name2id(be.render_km, mujoco.mjtObj.mjOBJ_MESH, "mjo_earth_mesh") >= 0
    assert mujoco.mj_name2id(be.render_km, mujoco.mjtObj.mjOBJ_MESH, "mjo_atmo_inner_mesh") >= 0
    assert mujoco.mj_name2id(be.render_km, mujoco.mjtObj.mjOBJ_MESH, "mjo_atmo_outer_mesh") >= 0
    assert mujoco.mj_name2id(be.render_km, mujoco.mjtObj.mjOBJ_LIGHT, "mjo_earthshine") >= 0

    be_term = MjorbitBackend(xml_path, orbit_alt_km=550.0, orbit_inc_deg=97.6, earth_bg=False, sun_phase_deg=65.0)
    be_term.reset(rng=np.random.default_rng(77))
    r_term = np.asarray(be_term.oracle_data.orbit.R_eci, dtype=float)
    s_term = np.asarray(be_term.oracle_data.env.sun_vector_eci, dtype=float)
    r_term /= np.linalg.norm(r_term)
    s_term /= np.linalg.norm(s_term)
    phase_deg = float(np.degrees(np.arccos(np.clip(np.dot(r_term, s_term), -1.0, 1.0))))
    assert 55.0 <= phase_deg <= 125.0, f"Unexpected terminator phase angle {phase_deg:.2f} deg"

    img_fx = apply_space_optics(img, rng=np.random.default_rng(11))
    assert img_fx.shape == img.shape and img_fx.dtype == np.uint8
    gray = img.mean(axis=2).astype(np.uint8)
    gray_fx = apply_space_optics(gray, rng=np.random.default_rng(11))
    assert gray_fx.shape == gray.shape and gray_fx.dtype == np.uint8
    print(f"PASS: test_realism_upgrades (phase={phase_deg:.1f} deg, optics RGB/gray ok)")


if __name__ == "__main__":
    test_keepout_smoothness()
    test_servicer_spec()
    test_mjorbit_parity_and_earth_render()
    print("ALL MODULAR TESTS PASSED")
    sys.stdout.flush()
    os._exit(0)
