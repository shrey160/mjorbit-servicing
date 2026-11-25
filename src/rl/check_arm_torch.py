"""Check rl/arm_torch.py against MuJoCo: FK and Jacobian at random joint angles and bus poses, IK against _ik1."""
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
import _paths  # noqa: E402,F401
import reach_bench as RBN  # noqa: E402
from arm_torch import ArmFK, quat_to_mat  # noqa: E402

B = RBN.Bench()
km, I = B.km, B.I
dev = "cuda" if torch.cuda.is_available() else "cpu"
A = ArmFK(km, arm_joints=I["arm_q"], jlo=B.jlo, jhi=B.jhi, device=dev)
rng = np.random.default_rng(0)
d = mujoco.MjData(km)
N = 64
qs = rng.uniform(B.jlo, B.jhi, (N, 6))
bps, bRs, ps, Rs, Js = [], [], [], [], []
jp, jr = np.zeros((3, km.nv)), np.zeros((3, km.nv))
for i in range(N):
    d.qpos[:] = km.qpos0
    d.qpos[B.cq:B.cq + 3] = rng.normal(size=3)
    qq = rng.normal(size=4)
    d.qpos[B.cq + 3:B.cq + 7] = qq / np.linalg.norm(qq)
    d.qpos[I["arm_q"]] = qs[i]
    mujoco.mj_kinematics(km, d)
    mujoco.mj_comPos(km, d)
    bps.append(d.xpos[B.bid_c].copy()); bRs.append(d.xmat[B.bid_c].reshape(3, 3).copy())
    ps.append(d.site_xpos[I["grip"]].copy()); Rs.append(d.site_xmat[I["grip"]].reshape(3, 3).copy())
    mujoco.mj_jacSite(km, d, jp, jr, I["grip"])
    Js.append(np.vstack([jp, jr])[:, I["arm_v"]])
T = lambda x: torch.tensor(np.array(x), dtype=torch.float64, device=dev)  # noqa: E731
p, R, ax, an = A.fk(T(bps), T(bRs), T(qs))
J = A.jac(p, ax, an)
print("fk pos err max (m)", float((p - T(ps)).norm(dim=-1).max()))
print("fk rot err max", float((R - T(Rs)).abs().max()))
print("jac err max", float((J - T(Js)).abs().max()))
# IK: targets from known q, start from a perturbed q; compare with _ik1
q0 = qs + rng.normal(0, 0.05, qs.shape)
qi, ep, er = A.ik(T(bps), T(bRs), T(ps), T(Rs), T(q0), 60)
ref = []
for i in range(8):
    d.qpos[B.cq:B.cq + 3] = bps[i]
    B.kd.qpos[:] = d.qpos
    # _ik1 runs on the bench's scratch copy of kd: set the same bus pose there
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(bRs[i]).as_quat()
    B.kd.qpos[B.cq:B.cq + 7] = [*bps[i], w, x, y, z]
    qc, epc, erc = RBN._ik1(B, ps[i], Rs[i], q0[i], 60)
    ref.append((qc, epc, erc))
print("ik ep max (m)", float(ep.max()), "er max (deg)", float(er[er < 900].max()) if (er < 900).any() else None,
      "outside", int((er >= 900).sum()))
print("ik vs _ik1 q diff max (8 cases)", max(float(np.abs(qi[i].cpu().numpy() - ref[i][0]).max()) for i in range(8)),
      "cpu ep", max(r[1] for r in ref))
