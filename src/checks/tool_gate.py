"""P7.2 gate (SPEC_7 section 13, step 4, 21 Nov): the ring tool grips the flange of a FIXED client; the grip holds
test loads at mu 0.4 and 0.2 without slipping; the D5 sensors read sensibly.

Scene: make_scene_rev2.OUT (client_p7 + rev-2 servicer with the ring tool), plain MuJoCo, no gravity. The client is
pinned in place (position and velocity reset each step). The servicer faces it (its +X face to the client's +X face, bus centre 0.75 m beyond the
ring top) and is pinned during placement and closing. The arm is set by IK so the grip site sits on the ring's
lowest section (azimuth -90 deg: radially out = -z), 1 mm off the ring top, jaws open; the arm then holds that
pose with a joint PD. Close: the jaw drives go to the squeeze targets (hook along its cam path, inboard
jaw radially). Load: the servicer pin is released and a wrench is applied to the bus (stack-move class x3 and
more: force 5 N per axis, torque 3 N m per axis, each sign, 3 s each, the grip carries it to the fixed client).
Recorded: tool slip on the ring (grip site relative to the ring section frame, mm / deg), jaw positions, drive
forces, touch per face, wrist force / torque, with friction 0.4 (nominal) and 0.2 (stress) on the tool and ring.

  bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/tool_gate.py
"""
import json
import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
from make_scene_rev2 import OUT  # noqa: E402

ARM_J = ["j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll"]
RING_X, RING_R = 0.81, 0.294          # client frame: ring top plane, wall mid radius (client_p7)
AZ = -np.pi / 2                        # grasp section: the ring's lowest point


def model(mu):
    txt = "\n".join(ln for ln in OUT.read_text().splitlines() if "<mjorbit " not in ln and "use_gravity_gradient" not in ln)
    m = mujoco.MjModel.from_xml_string(txt)
    m.opt.gravity[:] = 0.0
    for g in range(m.ngeom):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if nm.startswith(("ring_wall", "ring_lip", "pad_u", "pad_w", "pad_l", "hook_finger")) or nm in ("palm", "tool_housing", "tool_neck"):
            m.geom_friction[g, 0] = mu
    return m


def ids(m):
    J = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    return dict(arm_q=[m.jnt_qposadr[J(j)] for j in ARM_J], arm_v=[m.jnt_dofadr[J(j)] for j in ARM_J],
                cq=m.jnt_qposadr[J("chaser_joint")], tq=m.jnt_qposadr[J("target_joint")],
                grip=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site"),
                chaser=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chaser"),
                act_arm=[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"tau{i}") for i in range(1, 7)],
                act_u=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "grip_u_pos"),
                act_l=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "grip_l_pos"))


def ring_frame():
    rad = np.array([0.0, np.cos(AZ), np.sin(AZ)])
    x = np.array([-1.0, 0.0, 0.0])                # tool x: into the ring top (client -x)
    R = np.column_stack([x, np.cross(rad, x), rad])
    return np.array([RING_X, 0, 0]) + RING_R * rad, R


def ik(m, d, I, p_des, R_des, q0, iters=300):
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    q = q0.copy()
    for _ in range(iters):
        d.qpos[I["arm_q"]] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        p, R = d.site_xpos[I["grip"]], d.site_xmat[I["grip"]].reshape(3, 3)
        e = np.r_[p_des - p, Rotation.from_matrix(R_des @ R.T).as_rotvec()]
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
            break
        mujoco.mj_jacSite(m, d, jp, jr, I["grip"])
        J = np.vstack([jp, jr])[:, I["arm_v"]]
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
    return q, float(np.linalg.norm(e[:3])), float(np.degrees(np.linalg.norm(e[3:])))


def sens(m, d, n):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)
    return d.sensordata[m.sensor_adr[i]:m.sensor_adr[i] + m.sensor_dim[i]].copy()


def run(mu):
    m = model(mu)
    d = mujoco.MjData(m)
    I = ids(m)
    p_g, R_g = ring_frame()
    # servicer: bus centre 0.75 m beyond the ring top, +X face to the client, turned 180 deg about z
    d.qpos[I["tq"]:I["tq"] + 7] = [0, 0, 0, 1, 0, 0, 0]
    d.qpos[I["cq"]:I["cq"] + 7] = [RING_X + 0.25 + 0.75, 0, 0, 0, 0, 0, 1]
    q_open = np.array([m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, "grip_open")] + k] for k in range(2)])
    sq = float(m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, "grip_squeeze")]])
    gq = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ("grip_u", "grip_l")]
    d.qpos[gq] = q_open
    q, ep, er = ik(m, d, I, p_g - 0.001 * R_g[:, 0], R_g, np.array([0.0, 0.3, -0.6, 0.3, 0.0, 0.0]))
    d.qpos[I["arm_q"]] = q
    mujoco.mj_forward(m, d)
    # pins (kinematic: position and velocity reset after every step = an immovable body for the contacts)
    tv = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")]
    cv = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "chaser_joint")]
    pin = {"client": (I["tq"], tv, d.qpos[I["tq"]:I["tq"] + 7].copy()), "bus": (I["cq"], cv, d.qpos[I["cq"]:I["cq"] + 7].copy())}
    pinned = {"client", "bus"}
    out = dict(mu=mu, ik_err_mm=round(ep * 1000, 2), ik_err_deg=round(er, 3), q_arm=np.round(q, 3).tolist())
    kp = np.array([400, 400, 300, 60, 60, 60.0])
    kdv = 2 * 0.9 * np.sqrt(kp * np.array([3, 3, 1.5, 0.2, 0.2, 0.2]))

    def step(n, grip_cmd, wrench=None):
        for _ in range(n):
            d.ctrl[I["act_arm"]] = np.clip(kp * (q - d.qpos[I["arm_q"]]) - kdv * d.qvel[I["arm_v"]], -m.actuator_ctrlrange[I["act_arm"], 1],
                                           m.actuator_ctrlrange[I["act_arm"], 1])
            d.ctrl[[I["act_u"], I["act_l"]]] = grip_cmd
            d.xfrc_applied[I["chaser"]] = 0.0 if wrench is None else wrench
            mujoco.mj_step(m, d)
            for k in pinned:
                qa, va, q0_ = pin[k]
                d.qpos[qa:qa + 7] = q0_
                d.qvel[va:va + 6] = 0.0
            mujoco.mj_forward(m, d)

    def rel():
        p, R = d.site_xpos[I["grip"]].copy(), d.site_xmat[I["grip"]].reshape(3, 3).copy()
        return R_g.T @ (p - p_g), Rotation.from_matrix(R_g.T @ R).as_rotvec()

    dt = m.opt.timestep
    step(int(0.5 / dt), q_open)
    if os.environ.get("P7_GATE_CONTACTS"):
        pq, rq = rel()
        print(f"  mu {mu} before close: seat mm {np.round(pq * 1e3, 2)} deg {np.round(np.degrees(rq), 3)}", flush=True)
    squeeze = np.array([-sq, sq])
    step(int(3.0 / dt), squeeze)
    p0, r0 = rel()
    if os.environ.get("P7_GATE_CONTACTS"):
        print(f"  mu {mu} after close: seat mm {np.round(p0 * 1e3, 2)} deg {np.round(np.degrees(r0), 3)} arm q err (mrad) "
              f"{np.round((d.qpos[I['arm_q']] - q) * 1e3, 2)}", flush=True)
    if os.environ.get("P7_GATE_CONTACTS"):          # diagnostic: every contact on the tool after the close
        gn = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"#{g}"  # noqa: E731
        bn = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])  # noqa: E731
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = bn(c.geom1), bn(c.geom2)
            if not ({b1, b2} & {"wrist3", "jaw_l", "jaw_u"}):
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            print(f"  mu {mu} contact {gn(c.geom1)}({b1}) - {gn(c.geom2)}({b2}) dist {c.dist * 1e3:6.2f} mm "
                  f"Fn {f6[0]:7.1f} N at {np.round(c.pos * 1e3, 1)}", flush=True)
        if os.environ["P7_GATE_CONTACTS"] == "stop":
            raise SystemExit(0)
    out["closed"] = dict(jaw_u=round(float(d.qpos[gq[0]]) * 1000, 2), jaw_l=round(float(d.qpos[gq[1]]) * 1000, 2),
                         drive_u_N=round(float(sens(m, d, "jaw_u_frc")[0]), 1), drive_l_N=round(float(sens(m, d, "jaw_l_frc")[0]), 1),
                         touch_palm=round(float(sens(m, d, "touch_palm")[0]), 1), touch_in=round(float(sens(m, d, "touch_in")[0]), 1),
                         touch_hook=round(float(sens(m, d, "touch_hook")[0]), 1), touch_lip=round(float(sens(m, d, "touch_lip")[0]), 1),
                         seat_mm=np.round(p0 * 1000, 2).tolist(), seat_deg=np.round(np.degrees(r0), 3).tolist())
    # 21 Nov (tool v3): the drives are non-backdrivable lead screws (D7.5). The force-limited servo closes the jaws;
    # then each drive locks where it stalled: the scene's joint equality constraints lock_u / lock_l (implicit, stiff)
    # switched on at the stalled positions, so a load can no longer push a jaw back (the soft servo, 10 N/mm on the
    # inboard jaw, let 3 N m loads open it); the contacts stay compressed, so the closing preload stays.
    # P7_GATE_LOCK=0 keeps the soft servo (the v1 / v2 behaviour).
    hold = squeeze
    if os.environ.get("P7_GATE_LOCK", "1") == "1":
        q_lock = d.qpos[gq].copy()
        f_lock = [round(float(sens(m, d, "jaw_u_frc")[0]), 1), round(float(sens(m, d, "jaw_l_frc")[0]), 1)]
        for nm, qv in zip(("lock_u", "lock_l"), q_lock):
            e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, nm)
            m.eq_data[e, :5] = [qv, 0, 0, 0, 0]
            d.eq_active[e] = 1
        out["lock"] = dict(q_mm=np.round(q_lock * 1e3, 2).tolist(), preload_N=f_lock)
    # release the bus; load it
    pinned.discard("bus")
    step(int(1.0 / dt), hold)
    loads = []
    for k in range(6):
        for sgn in (1, -1):
            w = np.zeros(6)
            w[k] = sgn * (5.0 if k < 3 else 3.0)
            pa, ra = rel()
            ja = d.qpos[gq].copy()
            step(int(3.0 / dt), hold, w)
            pb, rb = rel()
            ft_f, ft_t = sens(m, d, "wrist_force"), sens(m, d, "wrist_torque")
            loads.append(dict(axis="FxFyFzTxTyTz"[2 * k:2 * k + 2], sign=sgn,
                              slip_mm=round(float(np.linalg.norm(pb - pa)) * 1000, 3),
                              slip_deg=round(float(np.degrees(np.linalg.norm(rb - ra))), 4),
                              jaw_move_mm=np.round((d.qpos[gq] - ja) * 1e3, 2).tolist(),
                              touch=[round(float(sens(m, d, n)[0]), 1) for n in ("touch_palm", "touch_in", "touch_hook", "touch_lip")],
                              wrist_F=np.round(ft_f, 2).tolist(), wrist_T=np.round(ft_t, 2).tolist()))
            step(int(1.0 / dt), hold)
    pe, re = rel()
    out["loads"] = loads
    out["slip_max_mm"] = max(x["slip_mm"] for x in loads)
    out["slip_max_deg"] = max(x["slip_deg"] for x in loads)
    out["drift_total_mm"] = round(float(np.linalg.norm(pe - p0)) * 1000, 3)
    out["drift_total_deg"] = round(float(np.degrees(np.linalg.norm(re - r0))), 4)
    return out


if __name__ == "__main__":
    res = [run(mu) for mu in (0.4, 0.2)]
    for r in res:
        print(json.dumps({k: v for k, v in r.items() if k != "loads"}))
        for x in r["loads"]:
            print("   ", x)
    (ensure_runs_dir() / "tool_gate.json").write_text(json.dumps(res, indent=1))
