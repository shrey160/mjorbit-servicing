"""Physics checks of servicer rev 2 (SPEC_7 D7) before it enters the P6 chain (brief section 8).

  bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/phys_rev2.py   (writes runs/phys_rev2.json)

A  mass properties: servicer mass, CoM, inertia (stowed and ready), against the 200 kg design point
B  envelope: stowed bounding box against ESPA (610 x 710 x 970 mm, depth from the aft port face); the
   current stow pose and the flattest collision-free stow pose found by search; bus-camera view blocked
   by the stowed arm (ray fan in the bus_cam field of view)
C  thrust authority: largest pure force / torque per axis and sign (linear program) about the servicer
   CoM and the stack CoM, all nozzles and worst single failure; the same wrenches flown in MuJoCo with
   the nozzle actuators (sign and magnitude check of the scene)
D  plume keep-out: nozzles whose 30 deg plume cone reaches the client (bounding sphere, any tumble
   phase) at 3 m and 1.8 m, and the arm (ready pose); authority left with those nozzles off
E  wheels: momentum and torque envelope per axis (all, single failure) against the detumble demand,
   and a MuJoCo run that checks the wheel model conserves total angular momentum (gyroscopic term)
F  propellant and delta-v
"""
from __future__ import annotations

import json
import sys
from itertools import product
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
from actuation import G0, Rcs, Wheels, load_bus  # noqa: E402
from make_scene_rev2 import OUT, plain_model  # noqa: E402

ARM_J = ["j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll"]
ESPA = dict(depth=0.710, y=0.305, z=0.4825)   # depth from the aft (port) face; half width / height
AFT_X = load_bus().get("port_face_x_m", -0.25)   # the separation ring's port face (depth is counted from it)
CLIENT_R = float(np.linalg.norm([0.75, 0.5, 0.5]))   # client box half-diagonal (wing and dish not included)
CONE = np.radians(30.0)

import os  # noqa: E402
m = plain_model(os.environ.get("P7_SCENE", OUT))  # P7_SCENE: a variant scene (arm layout trials)
d = mujoco.MjData(m)
bus = load_bus()
rcs, wh = Rcs(bus), Wheels(bus)
bid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i): i for i in range(m.nbody)}
CH, TG = bid["chaser"], bid["target"]
jadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM_J]
jrng = np.array([m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM_J])
cq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "chaser_joint")]
tq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")]
num = lambda n: np.array(m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, n)]:][: m.numeric_size[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, n)]])  # noqa: E731
STOW, READY = num("servicer_stow"), num("servicer_ready")


def in_chaser(b):
    while b != 0:
        if b == CH:
            return True
        b = m.body_parentid[b]
    return False


SUB = [b for b in range(m.nbody) if in_chaser(b)]
ARMB = [b for b in SUB if b != CH]


def pose(q, client_far=True):
    """Chaser at the origin, identity attitude, arm at q; client 50 m away."""
    mujoco.mj_resetData(m, d)
    d.qpos[cq:cq + 7] = [0, 0, 0, 1, 0, 0, 0]
    d.qpos[tq:tq + 7] = [50, 0, 0, 1, 0, 0, 0] if client_far else d.qpos[tq:tq + 7]
    for a, v in zip(jadr, q):
        d.qpos[a] = v
    mujoco.mj_forward(m, d)


def mass_props():
    M = sum(m.body_mass[b] for b in SUB)
    c = sum(m.body_mass[b] * d.xipos[b] for b in SUB) / M
    I = np.zeros((3, 3))
    for b in SUB:
        R = d.ximat[b].reshape(3, 3)
        r = d.xipos[b] - c
        I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
    return M, c, I


def geom_points(g):
    """Points bounding geom g in the world frame (mesh vertices or the AABB corners)."""
    R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        return v @ R.T + p
    c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
    corners = np.array(list(product(*[(-1, 1)] * 3))) * h + c
    return corners @ R.T + p


def bbox(bodies):
    pts = np.vstack([geom_points(g) for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and m.geom_rgba[g, 3] > 0])
    return pts.min(0), pts.max(0)


# ---- collision-free stow search ------------------------------------------------------------------------
COLG = [g for g in range(m.ngeom) if m.geom_bodyid[g] in SUB and (m.geom_contype[g] or m.geom_conaffinity[g]
                                                                 or m.geom_bodyid[g] in ARMB and m.geom_rgba[g, 3] > 0
                                                                 and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH)]


def _adjacent(b1, b2):
    return b1 == b2 or m.body_parentid[b1] == b2 or m.body_parentid[b2] == b1 or \
        (m.body_parentid[b1] == m.body_parentid[b2] and m.body_parentid[b1] in ARMB)


PAIRS = [(g1, g2) for i, g1 in enumerate(COLG) for g2 in COLG[i + 1:]
         if not _adjacent(m.geom_bodyid[g1], m.geom_bodyid[g2])]


def _structural():
    """Pairs already touching in the ready pose (shoulder module on the bus face, the cardan wrist housing,
    forearm end in the wrist): mounting overlaps of the model, not stow clashes; left out."""
    pose(READY)
    ft = np.zeros(6)
    return {(g1, g2) for g1, g2 in PAIRS if mujoco.mj_geomDistance(m, d, g1, g2, 0.2, ft) < 0.001}


def clearance(q):
    pose(q)
    ft = np.zeros(6)
    return min(mujoco.mj_geomDistance(m, d, g1, g2, 0.2, ft) for g1, g2 in PAIRS if (g1, g2) not in STRUCT)


STRUCT = _structural()


def stow_cost(q, margin=0.01):
    pose(q)
    lo, hi = bbox(SUB)
    depth = hi[0] - AFT_X
    pen = max(0.0, margin - clearance(q)) * 50
    pen += max(0.0, hi[1] - ESPA["y"]) * 20 + max(0.0, -lo[1] - ESPA["y"]) * 20
    pen += max(0.0, hi[2] - ESPA["z"]) * 20 + max(0.0, -lo[2] - ESPA["z"]) * 20
    pen += max(0.0, (np.abs(np.asarray(q)) - jrng[:, 1]).max()) * 50
    return depth + pen


def search_stow(n=3000, seed=0):
    rng = np.random.default_rng(seed)
    best = []
    for _ in range(n):
        q = rng.uniform(jrng[:, 0], jrng[:, 1])
        best.append((stow_cost(q), q))
    best.sort(key=lambda t: t[0])
    out = []
    for c0, q0 in best[:8] + [(stow_cost(STOW), STOW)]:
        r = minimize(stow_cost, q0, method="Nelder-Mead", options=dict(maxiter=1500, xatol=1e-4, fatol=1e-5))
        out.append((r.fun, r.x))
    out.sort(key=lambda t: t[0])
    return out[0][1]


def cam_blocked(q, n=24):
    """Fraction of a ray fan over the bus_cam field of view that hits the arm first."""
    pose(q)
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "bus_cam")
    R, p = d.cam_xmat[cid].reshape(3, 3), d.cam_xpos[cid]
    t = np.tan(np.radians(m.cam_fovy[cid] / 2))
    hit, geomid = 0, np.zeros(1, np.int32)
    for u, v in product(np.linspace(-t, t, n), np.linspace(-t, t, n)):
        ray = R @ np.array([u, v, -1.0])
        ray /= np.linalg.norm(ray)
        dist = mujoco.mj_ray(m, d, p, ray, None, 1, CH, geomid)
        if dist > 0 and m.geom_bodyid[geomid[0]] in ARMB:
            hit += 1
    return hit / n ** 2


# ---- authority -------------------------------------------------------------------------------------------
AX = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]


def authority(ref, enabled=None):
    return {f"{'+' if s > 0 else '-'}{AX[i]}": round(rcs.max_pure(i, ref, s, enabled), 3) for i in range(6) for s in (1, -1)}


def worst_single(ref):
    w = {}
    for k in range(rcs.n):
        en = np.ones(rcs.n, bool)
        en[k] = False
        for key, v in authority(ref, en).items():
            w[key] = min(w.get(key, 1e9), v)
    return w


def fly_wrench(q, ref_body, M, I):
    """Fly each pure unit-ish wrench in MuJoCo with the nozzle actuators for 0.5 s; compare the body-frame
    linear and angular velocity change with B u / M and I^-1 tau."""
    noz = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "test_rcs_" + n[4:]) for n in rcs.names]
    rows = []
    for i in range(6):
        e = np.zeros(6)
        e[i] = 0.5 if i < 3 else 0.3
        u = rcs.allocate(e, ref_body)
        pose(q)
        d.ctrl[:] = 0
        d.ctrl[noz] = u * rcs.F
        steps = int(0.5 / m.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(m, d)
        v = d.qvel[cq:cq + 3].copy()          # world linear velocity of the chaser frame origin
        w = d.qvel[cq + 3:cq + 6].copy()      # body-frame angular velocity
        R = d.xmat[CH].reshape(3, 3)
        mujoco.mj_subtreeVel(m, d)
        vcom = R.T @ (d.subtree_linvel[CH])   # CoM velocity in the body frame
        exp_v = (rcs.B(ref_body) @ u)[:3] / M * 0.5
        exp_w = np.linalg.solve(I, (rcs.B(ref_body) @ u)[3:]) * 0.5
        rows.append(dict(wrench=AX[i], asked=e[i], made=np.round(rcs.B(ref_body) @ u, 3).tolist(),
                         dv_com=np.round(vcom, 5).tolist(), dv_exp=np.round(exp_v, 5).tolist(),
                         dw=np.round(w, 5).tolist(), dw_exp=np.round(exp_w, 5).tolist()))
    return rows


# ---- plume -----------------------------------------------------------------------------------------------
def plume_margin_sphere(c, R):
    """Smallest angle [deg] between a plume centreline and the client's bounding sphere, over all nozzles."""
    a = []
    for k in range(rcs.n):
        v = np.asarray(c) - rcs.P[k]
        dist = np.linalg.norm(v)
        a.append(np.degrees(np.arccos(np.clip(-rcs.D[k] @ v / dist, -1, 1)) - np.arcsin(min(1.0, R / dist))))
    return round(float(min(a)), 1)


def plume_hits_sphere(c, R):
    """Nozzles whose plume cone (half angle CONE about -force dir) reaches a sphere (centre c, radius R)."""
    bad = []
    for k in range(rcs.n):
        v = np.asarray(c) - rcs.P[k]
        dist = np.linalg.norm(v)
        ang = np.arccos(np.clip(-rcs.D[k] @ v / dist, -1, 1))
        if ang - np.arcsin(min(1.0, R / dist)) < CONE:
            bad.append(rcs.names[k])
    return bad


def plume_hits_self(q, half=np.radians(15.0), n=7, reach=3.0):
    """Nozzles whose plume core (rays within `half` of the axis, from 3 cm past the nozzle base) hits any
    servicer geom (bus parts, panels, sensors, arm) other than its own nozzle and bracket."""
    pose(q)
    bad, gid = {}, np.zeros(1, np.int32)
    for k in range(rcs.n):
        ax = -rcs.D[k]
        u = np.cross(ax, [1, 0, 0] if abs(ax[0]) < 0.9 else [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(ax, u)
        p0 = rcs.P[k] + 0.03 * ax
        for a, b in product(np.linspace(-1, 1, n), repeat=2):
            if a * a + b * b > 1:
                continue
            ray = ax + np.tan(half) * (a * u + b * v)
            ray /= np.linalg.norm(ray)
            dist = mujoco.mj_ray(m, d, p0, ray, None, 1, -1, gid)
            if 0 < dist < reach and m.geom_bodyid[gid[0]] in SUB:
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid[0]) or "?"
                if rcs.names[k] not in name and "bracket" not in name:
                    bad.setdefault(rcs.names[k], set()).add(name)
    return {k: sorted(v) for k, v in bad.items()}


def plume_hits_arm(q):
    pose(q)
    pts = np.vstack([geom_points(g) for g in range(m.ngeom) if m.geom_bodyid[g] in ARMB and m.geom_rgba[g, 3] > 0])
    bad = []
    for k in range(rcs.n):
        v = pts - rcs.P[k]
        cosang = (v @ -rcs.D[k]) / np.linalg.norm(v, axis=1)
        if (cosang > np.cos(CONE)).any():
            bad.append(rcs.names[k])
    return bad


def main():
    out = {}
    # A
    pose(STOW)
    Ms, cs, Is = mass_props()
    pose(READY)
    Mr, cr, Ir = mass_props()
    out["A_mass"] = dict(servicer_kg=round(Ms, 2), bus_kg=bus["mass_kg"], com_stow_m=np.round(cs, 4).tolist(),
                         com_ready_m=np.round(cr, 4).tolist(), I_stow_diag=np.round(np.diag(Is), 2).tolist(),
                         I_ready_diag=np.round(np.diag(Ir), 2).tolist(), design_point_kg=200)
    print("A", out["A_mass"], flush=True)
    # B
    lo, hi = (pose(STOW), bbox(SUB))[1]
    q_flat = search_stow()
    pose(q_flat)
    lo2, hi2 = bbox(SUB)
    out["B_envelope"] = dict(
        espa=ESPA, current_stow=dict(q=STOW.round(3).tolist(), depth_m=round(hi[0] - AFT_X, 3),
                                     y=[round(lo[1], 3), round(hi[1], 3)], z=[round(lo[2], 3), round(hi[2], 3)],
                                     clearance_m=round(clearance(STOW), 4), cam_blocked=round(cam_blocked(STOW), 3)),
        flat_stow=dict(q=q_flat.round(3).tolist(), depth_m=round(hi2[0] - AFT_X, 3),
                       y=[round(lo2[1], 3), round(hi2[1], 3)], z=[round(lo2[2], 3), round(hi2[2], 3)],
                       clearance_m=round(clearance(q_flat), 4), cam_blocked=round(cam_blocked(q_flat), 3)),
        bus_only_depth_m=round(bus["bbox_m"][3] - AFT_X, 3))
    print("B", out["B_envelope"], flush=True)
    # C
    M_c, x_c = 300.0, 0.25 + 1.3 + 0.75
    stack_ref = (Mr * cr + M_c * np.array([x_c, 0, 0])) / (Mr + M_c)
    out["C_authority"] = dict(
        about_servicer_com=authority(cr), worst_single_failure=worst_single(cr),
        stack_com_m=np.round(stack_ref, 3).tolist(), about_stack_com=authority(stack_ref),
        flown=fly_wrench(READY, cr, Mr, Ir))
    print("C", {k: v for k, v in out["C_authority"].items() if k != "flown"}, flush=True)
    for r in out["C_authority"]["flown"]:
        print("   flown", r, flush=True)
    # D
    face = 0.25
    D = {}
    for dist in (8.0, 3.0, 1.8):
        c = np.array([face + dist + 0.75, 0, 0])        # client CoM: its ring face `dist` ahead of the +X face
        bad = plume_hits_sphere(c, CLIENT_R)
        en = np.array([n not in bad for n in rcs.names])
        D[f"{dist} m"] = dict(blocked=bad, min_plume_angle_deg=plume_margin_sphere(c, CLIENT_R), authority_left=authority(cr, en))
    D["arm_ready_blocked"] = plume_hits_arm(READY)
    D["self_impingement_ready"] = plume_hits_self(READY)
    D["self_impingement_stow"] = plume_hits_self(STOW)
    out["D_plume"] = D
    print("D", json.dumps(D), flush=True)
    # E
    dirs = dict(x=[1, 0, 0], y=[0, 1, 0], z=[0, 0, 1], yz=[0, 1, 1], xy=[1, 1, 0])
    env = {k: round(wh.envelope(v), 2) for k, v in dirs.items()}
    env1 = {k: round(min(wh.envelope(v, failed=(i,)) for i in range(4)), 2) for k, v in dirs.items()}
    tq = {k: round(wh.torque_envelope(v), 3) for k, v in dirs.items()}
    w2 = np.radians(2.0)
    mu = Mr * M_c / (Mr + M_c)
    demand = {f"d {dd} m": round(mu * dd * dd * w2 + 2.8, 1) for dd in (1.8, 2.0, 2.5)}
    out["E_wheels"] = dict(h_envelope_nms=env, h_envelope_single_failure=env1, torque_envelope_nm=tq,
                           detumble_demand_nms=demand, conservation=wheel_conservation(Ir))
    print("E", out["E_wheels"], flush=True)
    # F
    prop = next(p["mass_kg"] for p in bus["parts"] if p["name"] == "propellant")
    m0 = Ms
    out["F_prop"] = dict(propellant_kg=prop, isp_s=bus["isp_s"], dv_mps=round(bus["isp_s"] * G0 * np.log(m0 / (m0 - prop)), 1),
                         accel_per_axis_mm_s2=round(1000 * out["C_authority"]["about_servicer_com"]["+Fx"] / Mr, 2))
    print("F", out["F_prop"], flush=True)
    p = ensure_runs_dir() / "phys_rev2.json"
    p.parent.mkdir(exist_ok=True)
    p.write_text(json.dumps(out, indent=1, default=float))
    print("wrote", p)


def wheel_conservation(I_ready, T=30.0):
    """Free servicer, a torque command through the wheel model for 10 s then hold (no command), while the
    body turns: total angular momentum (body + wheels, world frame) should stay constant."""
    pose(READY)
    w = Wheels(bus)
    d.qvel[cq + 3:cq + 6] = np.radians([0.5, -1.0, 0.8])
    dt = m.opt.timestep
    Hs = []
    for k in range(int(T / dt)):
        t = k * dt
        R = d.xmat[CH].reshape(3, 3)
        om = d.qvel[cq + 3:cq + 6].copy()
        cmd = np.array([0.1, 0.15, -0.12]) if t < 10 else np.zeros(3)
        tau, _ = w.command(cmd, om, dt)
        d.xfrc_applied[CH, 3:] = R @ tau   # the arm joints are free (passive); their momentum is in the subtree
        mujoco.mj_step(m, d)
        if k % 500 == 0:
            mujoco.mj_subtreeVel(m, d)
            Hb = d.subtree_angmom[CH].copy()
            Hs.append(Hb + R @ w.H())
    Hs = np.array(Hs)
    return dict(H_total_start=np.round(Hs[0], 4).tolist(), H_total_end=np.round(Hs[-1], 4).tolist(),
                drift_nms=round(float(np.linalg.norm(Hs[-1] - Hs[0])), 5), wheel_h_end=np.round(w.h, 3).tolist())


if __name__ == "__main__":
    main()
