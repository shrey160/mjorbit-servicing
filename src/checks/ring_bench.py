"""P7.3 bench: ring centre and axis from bus_lidar at 8, 3 and 1.8 m (SPEC_7 section 5 and the P7.3 gate).

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/ring_bench.py [--n 25]

Scene: make_scene_rev2.OUT (servicer rev 2 with the ring tool, client_p7). Per case (bench seeds 7300000 + i, not a
mission split): the client at a random attitude; the servicer's lidar on a line within 15 deg of the ring axis at the
range D (lidar to ring centre), its boresight on the ring centre with a 2 deg pointing error and a random roll; arm
stowed at 8 m, ready at 3 and 1.8 m. Sensor model as the P6 chain: CamRig.lidar (range noise 5-20 mm + 0.5%, dropout,
edge and grazing dropout), bus_cam + bus_lidar on one optical bench with a +-10 mm / +-1 deg error (+ 1 mm / 0.1 deg
each, as --mount-bench), the servicer's own bodies masked (segmentation render, as netest._lidar_points). The prior
is the truth in the nominal lidar frame plus a prior error (8 m: 100 mm / 4 deg; 3 and 1.8 m: 50 mm / 2 deg, the P6
close-range errors), then the P6 box fit (lidar_nav.fit on the client's unrotated box geoms), then ring_fit.fit.

Errors: fit = against the truth in the true sensor frame (what the fit itself gets wrong); total = against the truth
in the nominal frame the servicer believes (adds the mount error, common to every bus sensor). Writes
runs/ring_bench.json.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import RUNS_DIR, ensure_runs_dir  # noqa: E402
from make_scene_rev2 import OUT, plain_model  # noqa: E402
from cams import CamRig  # noqa: E402
import lidar_nav  # noqa: E402
import ring_fit  # noqa: E402

ARM_J = ["j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll"]
PRIOR = {8.0: (0.10, 4.0), 3.0: (0.05, 2.0), 1.8: (0.05, 2.0)}
CONE_DEG, POINT_DEG = 15.0, 2.0
MOUNT_M, MOUNT_DEG, OWN_M, OWN_DEG = 0.010, 1.0, 0.001, 0.1


def _rand_unit_cone(rng, axis, deg):
    """Uniform direction within deg of axis."""
    ct = rng.uniform(np.cos(np.radians(deg)), 1.0)
    ph = rng.uniform(0, 2 * np.pi)
    u, v = ring_fit._frame(axis)
    st = np.sqrt(1 - ct * ct)
    return ct * axis + st * (np.cos(ph) * u + np.sin(ph) * v)


def _rot_between(a, b):
    """Rotation taking unit a to unit b."""
    v = np.cross(a, b)
    s, c = np.linalg.norm(v), float(a @ b)
    if s < 1e-12:
        return np.eye(3) if c > 0 else Rotation.from_rotvec(np.pi * ring_fit._frame(a)[0]).as_matrix()
    return Rotation.from_rotvec(v / s * np.arctan2(s, c)).as_matrix()


def _q_wxyz(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return [w, x, y, z]


class World:
    """The bench scene: model, sensors, and per-case placement (also used by ring_cam.py)."""

    def __init__(self):
        km = plain_model(OUT)
        km.vis.global_.offwidth = max(km.vis.global_.offwidth, 1024)
        km.vis.global_.offheight = max(km.vis.global_.offheight, 1024)
        self.km, self.kd = km, mujoco.MjData(km)
        B = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
        J = lambda n: km.jnt_qposadr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_JOINT, n)]  # noqa: E731
        self.bid_c, self.bid_t = B("chaser"), B("target")
        self.cq, self.tq = J("chaser_joint"), J("target_joint")
        self.arm = [J(j) for j in ARM_J]
        num = lambda n: km.numeric_data[km.numeric_adr[mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_NUMERIC, n)]:][:6].copy()  # noqa: E731
        self.poses = dict(stow=num("servicer_stow"), ready=num("servicer_ready"))
        self.rig = CamRig(km, res={"wrist_cam": 256, "bus_cam": 512})
        self.lid = self.rig.lidar_id
        self.bcam = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, "bus_cam")
        assert km.cam_bodyid[self.lid] == self.bid_c and km.cam_bodyid[self.bcam] == self.bid_c
        self.base = dict(pos=km.cam_pos.copy(), quat=km.cam_quat.copy())
        self.dirs = lidar_nav.ray_dirs(self.rig.lres, km.cam_fovy[self.lid])
        root = km.body_rootid[self.bid_c]
        self.self_geoms = np.array([g for g in range(km.ngeom) if km.body_rootid[km.geom_bodyid[g]] == root])
        self.seg = mujoco.Renderer(km, self.rig.lres, self.rig.lres)
        self.seg.enable_segmentation_rendering()
        boxes = []
        for g in range(km.ngeom):
            if (km.geom_bodyid[g] == self.bid_t and km.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
                    and np.allclose(np.abs(km.geom_quat[g]), [1, 0, 0, 0], atol=1e-6) and km.geom_contype[g] != 0):
                boxes.append((km.geom_pos[g].copy(), km.geom_size[g].copy()))
        lidar_nav.set_model(boxes)     # the client's design boxes (bus, wing panels, radiator)
        self.n_boxes = len(boxes)

    def cam_nominal(self, cid):
        """Nominal (believed) camera pose in the world: the chaser pose with the design mount."""
        kd = self.kd
        Rb, pb = kd.xmat[self.bid_c].reshape(3, 3), kd.xpos[self.bid_c]
        q = self.base["quat"][cid]
        Rc = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        return pb + Rb @ self.base["pos"][cid], Rb @ Rc

    def place(self, seed, D):
        """One case: client attitude, servicer placement, bench mount error, lidar noise. Returns the truth."""
        km, kd, rig = self.km, self.kd, self.rig
        rng = np.random.default_rng([seed, int(D * 10)])
        km.cam_pos[:] = self.base["pos"]
        km.cam_quat[:] = self.base["quat"]
        mujoco.mj_resetData(km, kd)
        Rc = Rotation.random(random_state=rng.integers(2**31)).as_matrix()
        pc = np.zeros(3)
        c_w, n_w = pc + Rc @ ring_fit.C_B, Rc @ ring_fit.N_B
        u = _rand_unit_cone(rng, n_w, CONE_DEG)
        pL = c_w + D * u
        # servicer attitude: lidar boresight (-z of the camera) on the ring centre, pointing error, random roll
        q = self.base["quat"][self.lid]
        Rl_b = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()   # lidar in the chaser frame
        bore_b = -Rl_b[:, 2]
        want = _rand_unit_cone(rng, -u, POINT_DEG)
        Rs = _rot_between(bore_b, want)
        Rs = Rotation.from_rotvec(want * rng.uniform(-np.pi, np.pi)).as_matrix() @ Rs
        ps = pL - Rs @ self.base["pos"][self.lid]
        kd.qpos[self.tq:self.tq + 7] = [*pc, *_q_wxyz(Rc)]
        kd.qpos[self.cq:self.cq + 7] = [*ps, *_q_wxyz(Rs)]
        kd.qpos[self.arm] = self.poses["stow" if D > 5 else "ready"]
        # bench mount error (bus_cam + bus_lidar), as p63_loop._bench_mounts
        ids = [self.bcam, self.lid]
        o = 0.5 * (self.base["pos"][ids[0]] + self.base["pos"][ids[1]])
        Rb = Rotation.from_rotvec(np.radians(rng.uniform(-MOUNT_DEG, MOUNT_DEG, 3)))
        tb = rng.uniform(-MOUNT_M, MOUNT_M, 3)
        for c in ids:
            Ro = Rotation.from_rotvec(np.radians(rng.uniform(-OWN_DEG, OWN_DEG, 3)))
            km.cam_pos[c] = o + Rb.apply(self.base["pos"][c] - o) + tb + rng.uniform(-OWN_M, OWN_M, 3)
            qq = self.base["quat"][c]
            x, y, z, w = (Rb * Ro * Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]])).as_quat()
            km.cam_quat[c] = [w, x, y, z]
        rig.l_sig0, rig.l_drop = rng.uniform(0.005, 0.02), rng.uniform(0.01, 0.05)
        rig._nrng = np.random.default_rng([seed, int(D * 10), 7])
        rig.gain, rig.noise = rng.uniform(0.75, 1.25), rng.uniform(0.0, 3.0)
        mujoco.mj_forward(km, kd)
        self.rng = rng
        return dict(Rc=Rc, pc=pc, c_w=c_w, n_w=n_w)

    def lidar_points(self):
        """Lidar points in the sensor frame, own bodies masked; and the self-masked pixel count."""
        img = self.rig.lidar(self.kd)
        self.seg.update_scene(self.kd, camera=self.lid, scene_option=self.rig.opt)
        sg = self.seg.render()
        own = (sg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(sg[..., 0], self.self_geoms)
        img = np.where(own, 0, img)
        return lidar_nav.points(img, self.dirs), int(own.sum())

    def frames(self, cid):
        """(nominal pos, R), (true pos, R) of a bus camera in the world."""
        kd = self.kd
        return self.cam_nominal(cid), (kd.cam_xpos[cid].copy(), kd.cam_xmat[cid].reshape(3, 3).copy())


def _ang(a, b):
    return float(np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def case(W, seed, D):
    tr = W.place(seed, D)
    P, own = W.lidar_points()
    (pn, Rn), (pa, Ra) = W.frames(W.lid)
    truth = {}
    for k, (p0, R0) in dict(nom=(pn, Rn), act=(pa, Ra)).items():
        truth[k] = (R0.T @ (tr["c_w"] - p0), R0.T @ tr["n_w"])
    # prior: truth in the nominal frame + error
    sp, sa = PRIOR[D]
    rng = W.rng
    Rp_true = Rn.T @ tr["Rc"]
    tp_true = Rn.T @ (tr["pc"] - pn)
    dv = rng.normal(size=3)
    da = rng.normal(size=3)
    t0 = tp_true + sp * dv / np.linalg.norm(dv) * rng.uniform(0.3, 1.0)
    R0 = Rotation.from_rotvec(np.radians(sa) * da / np.linalg.norm(da) * rng.uniform(0.3, 1.0)).as_matrix() @ Rp_true
    res = dict(seed=seed, D=D, n_pts=int(len(P)), own_px=own)
    ests = dict(prior=(t0 + R0 @ ring_fit.C_B, R0 @ ring_fit.N_B))
    b = lidar_nav.fit(P, R0, t0)
    res["box_ok"] = bool(b["ok"])
    if b["ok"]:
        ests["box"] = (b["t"] + b["R"] @ ring_fit.C_B, b["R"] @ ring_fit.N_B)
        Rb_, tb_ = b["R"], b["t"]
    else:
        Rb_, tb_ = R0, t0
    r = ring_fit.fit(P, Rb_, tb_)
    res.update(ring_ok=bool(r["ok"]), why=r["why"], n_top=r["n_top"], ring_pts=r["n_pts"],
               rad_mm=round(float(r["rad"]) * 1e3, 1) if np.isfinite(r["rad"]) else None,
               rms_mm=round(float(r["rms"]) * 1e3, 2) if np.isfinite(r["rms"]) else None,
               sig_mm=round(float(r.get("sig", np.nan)) * 1e3, 1))
    if r["ok"]:
        ests["ring"] = (r["c"], r["n"])
        res["ring_sig_mm"] = round(float(np.sqrt(np.trace(r["Cc"]))) * 1e3, 1) if r["Cc"] is not None else None
    for k, (c, n) in ests.items():
        for f, (ct, nt) in truth.items():
            e = c - ct
            res[f"{k}_{f}_mm"] = round(float(np.linalg.norm(e)) * 1e3, 1)
            res[f"{k}_{f}_ax_mm"] = round(float(abs(e @ nt)) * 1e3, 1)     # along the ring axis
            res[f"{k}_{f}_deg"] = round(_ang(n, nt), 2)
    res["_est"] = {k: (pn + Rn @ c, Rn @ n) for k, (c, n) in ests.items()}    # world, as the servicer believes
    res["_truth"] = (tr["c_w"], tr["n_w"])
    ct, nt = truth["act"]       # the true ring as the servicer sees it through its (mis-mounted) bench
    res["_truth_b"] = (pn + Rn @ ct, Rn @ nt)
    return res


def summarise(rows):
    out = {}
    for D in sorted({r["D"] for r in rows}, reverse=True):
        rs = [r for r in rows if r["D"] == D]
        s = dict(n=len(rs), box_ok=sum(r["box_ok"] for r in rs), ring_ok=sum(r["ring_ok"] for r in rs),
                 fails=sorted({r["why"] for r in rs if not r["ring_ok"]}))
        for k in ("prior", "box", "ring"):
            for f in ("act", "nom"):
                for q in ("mm", "deg"):
                    v = [r[f"{k}_{f}_{q}"] for r in rs if f"{k}_{f}_{q}" in r]
                    if v:
                        s[f"{k}_{f}_{q}"] = dict(p50=round(float(np.median(v)), 2), p95=round(float(np.percentile(v, 95)), 2),
                                                 max=round(float(np.max(v)), 2))
        out[str(D)] = s
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--d", type=float, nargs="*", default=[8.0, 3.0, 1.8])
    ap.add_argument("--out", default=str(RUNS_DIR / "ring_bench.json"))
    a = ap.parse_args()
    W = World()
    print(f"client boxes in the box model: {W.n_boxes}", flush=True)
    rows = []
    for D in a.d:
        for i in range(a.start, a.start + a.n):
            r = case(W, 7300000 + i, D)
            rows.append(r)
            print(f"D {D:3.1f} seed {r['seed']} pts {r['n_pts']:5d} own {r['own_px']:4d} top {r['n_top']:4d} | "
                  f"prior {r['prior_act_mm']:6.1f} box {r.get('box_act_mm', float('nan')):6.1f} "
                  f"ring {r.get('ring_act_mm', float('nan')):6.1f} mm ({r.get('ring_act_deg', float('nan')):4.2f} deg) "
                  f"nom {r.get('ring_nom_mm', float('nan')):6.1f} mm  step {r['rad_mm']} sig {r.get('sig_mm')}  {r['why']}", flush=True)
    s = summarise(rows)
    print(json.dumps(s, indent=1))
    rows = [{k: v for k, v in r.items() if not k.startswith("_")} for r in rows]
    out_p = Path(a.out)
    out_p.parent.mkdir(parents=True, exist_ok=True)
    out_p.write_text(json.dumps(dict(summary=s, rows=rows), indent=1))


if __name__ == "__main__":
    main()
