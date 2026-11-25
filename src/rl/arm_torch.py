"""Batched (torch) kinematics of the rev-2 arm for the GPU reach (SPEC_7 section 24, G1).

The arm chain from the servicer bus (`chaser`) to the grip site, read from the MuJoCo model: forward kinematics on a
given bus pose (the bus is pinned per world), the site Jacobian over the 6 arm joints (hinges: a x (p - anchor), a),
and the damped least-squares IK of reach_bench._ik1 (step cap 0.3 rad, damping 1e-4, the joint-range wrap). Every
function works on a batch of worlds (leading dim N). Checked against MuJoCo by rl/check_arm_torch.py.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
import torch


def quat_to_mat(q):
    """(..., 4) wxyz -> (..., 3, 3)."""
    w, x, y, z = q.unbind(-1)
    return torch.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1).reshape(q.shape[:-1] + (3, 3))


def mat_to_quat(R):
    """(N, 3, 3) -> (N, 4) wxyz (Shepperd: the largest of the four candidates, so no small divisor)."""
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    cand = torch.stack([tr, R[:, 0, 0], R[:, 1, 1], R[:, 2, 2]], -1)
    k = cand.argmax(-1)
    q = torch.zeros(R.shape[0], 4, dtype=R.dtype, device=R.device)
    r = torch.sqrt((1 + 2 * cand.gather(1, k[:, None]).squeeze(1) - tr).clamp(min=1e-12))
    s = 0.5 / r
    m0, m1, m2, m3 = (k == 0), (k == 1), (k == 2), (k == 3)
    q[:, 0] = torch.where(m0, 0.5 * r, (R[:, 2, 1] - R[:, 1, 2]) * s * (~m0) + 0)
    q[:, 0] = torch.where(m1, (R[:, 2, 1] - R[:, 1, 2]) * s, q[:, 0])
    q[:, 0] = torch.where(m2, (R[:, 0, 2] - R[:, 2, 0]) * s, q[:, 0])
    q[:, 0] = torch.where(m3, (R[:, 1, 0] - R[:, 0, 1]) * s, q[:, 0])
    q[:, 1] = torch.where(m0, (R[:, 2, 1] - R[:, 1, 2]) * s, torch.where(m1, 0.5 * r, torch.where(
        m2, (R[:, 0, 1] + R[:, 1, 0]) * s, (R[:, 0, 2] + R[:, 2, 0]) * s)))
    q[:, 2] = torch.where(m0, (R[:, 0, 2] - R[:, 2, 0]) * s, torch.where(m1, (R[:, 0, 1] + R[:, 1, 0]) * s, torch.where(
        m2, 0.5 * r, (R[:, 1, 2] + R[:, 2, 1]) * s)))
    q[:, 3] = torch.where(m0, (R[:, 1, 0] - R[:, 0, 1]) * s, torch.where(m1, (R[:, 0, 2] + R[:, 2, 0]) * s, torch.where(
        m2, (R[:, 1, 2] + R[:, 2, 1]) * s, 0.5 * r)))
    q = q * torch.where(q[:, :1] < 0, -1.0, 1.0)
    return q / q.norm(dim=-1, keepdim=True)


def axis_angle_mat(a, th):
    """a (3,) unit axis (constant), th (N,) -> (N, 3, 3) (Rodrigues)."""
    K = torch.tensor([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=th.dtype, device=th.device)
    s, c = torch.sin(th)[:, None, None], torch.cos(th)[:, None, None]
    return torch.eye(3, dtype=th.dtype, device=th.device) + s * K + (1 - c) * (K @ K)


def rotvec(R):
    """(N, 3, 3) -> (N, 3) rotation vector (log map; the IK errors are small, but it is exact up to ~pi)."""
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    c = ((tr - 1) * 0.5).clamp(-1.0, 1.0)
    th = torch.acos(c)
    v = torch.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], -1)
    s = torch.sin(th)
    k = torch.where(s.abs() > 1e-6, th / (2 * s.clamp(min=1e-12)), 0.5 + th * th / 12)
    return v * k[:, None]


def rot_from_rotvec(v):
    th = v.norm(dim=-1).clamp(min=1e-12)
    a = v / th[:, None]
    K = torch.zeros(v.shape[0], 3, 3, dtype=v.dtype, device=v.device)
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -a[:, 2], a[:, 1], a[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -a[:, 0], -a[:, 1], a[:, 0]
    s, c = torch.sin(th)[:, None, None], torch.cos(th)[:, None, None]
    return torch.eye(3, dtype=v.dtype, device=v.device) + s * K + (1 - c) * (K @ K)


class ArmFK:
    def __init__(self, km, site="grip_site", base="chaser", arm_joints=None, jlo=None, jhi=None, device="cuda",
                 dtype=torch.float64):
        self.dev, self.dt = device, dtype
        sid = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, site)
        bid0 = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, base)
        chain = []
        b = int(km.site_bodyid[sid])
        while b != bid0:
            chain.append(b)
            b = int(km.body_parentid[b])
            assert b != 0, "site not below the base body"
        chain.reverse()
        T = lambda x: torch.tensor(np.asarray(x, float), dtype=dtype, device=device)  # noqa: E731
        self.bodies = []
        self.qadr = []
        for b in chain:
            js = []
            for j in range(km.body_jntadr[b], km.body_jntadr[b] + km.body_jntnum[b]):
                assert km.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE, "only hinges on the arm chain"
                js.append((km.jnt_axis[j].copy(), T(km.jnt_pos[j]), int(km.jnt_qposadr[j]), float(km.qpos0[km.jnt_qposadr[j]])))
                self.qadr.append(int(km.jnt_qposadr[j]))
            self.bodies.append((T(km.body_pos[b]), quat_to_mat(T(km.body_quat[b])), js))
        if arm_joints is not None:
            assert self.qadr == list(arm_joints), (self.qadr, arm_joints)
        self.site_p = T(km.site_pos[sid])
        self.site_R = quat_to_mat(T(km.site_quat[sid]))
        self.jlo = T(jlo) if jlo is not None else None
        self.jhi = T(jhi) if jhi is not None else None

    def fk(self, bp, bR, q):
        """bp (N,3), bR (N,3,3) bus pose; q (N,6). -> site p (N,3), R (N,3,3), joint axes (N,6,3), anchors (N,6,3)."""
        p, R = bp, bR
        axes, anchors = [], []
        k = 0
        for pos, Rl, js in self.bodies:
            p = p + (R @ pos[:, None]).squeeze(-1)
            R = R @ Rl
            for ax, jp, _, q0 in js:
                anc = p + (R @ jp[:, None]).squeeze(-1)
                a_w = (R @ torch.tensor(ax, dtype=self.dt, device=self.dev)[:, None]).squeeze(-1)
                R = R @ axis_angle_mat(ax, q[:, k] - q0)
                p = anc - (R @ jp[:, None]).squeeze(-1)
                axes.append(a_w)
                anchors.append(anc)
                k += 1
        ps = p + (R @ self.site_p[:, None]).squeeze(-1)
        return ps, R @ self.site_R, torch.stack(axes, 1), torch.stack(anchors, 1)

    def jac(self, ps, axes, anchors):
        Jp = torch.cross(axes, ps[:, None, :] - anchors, dim=-1)       # (N, 6, 3)
        return torch.cat([Jp, axes], -1).transpose(1, 2)              # (N, 6 rows, 6 joints)

    def ik(self, bp, bR, p_des, R_des, q0, iters):
        """reach_bench._ik1 batched: -> q (N,6), position error (m), rotation error (deg; 999 outside the ranges)."""
        q = q0.clone()
        eye = torch.eye(6, dtype=self.dt, device=self.dev) * 1e-4
        done = torch.zeros(q.shape[0], dtype=torch.bool, device=self.dev)
        e = torch.ones(q.shape[0], 6, dtype=self.dt, device=self.dev)
        for _ in range(iters):
            ps, Rs, ax, an = self.fk(bp, bR, q)
            e_new = torch.cat([p_des - ps, rotvec(R_des @ Rs.transpose(1, 2))], -1)
            e = torch.where(done[:, None], e, e_new)
            done = done | ((e[:, :3].norm(dim=-1) < 1e-5) & (e[:, 3:].norm(dim=-1) < 1e-4))
            if bool(done.all()):
                break
            J = self.jac(ps, ax, an)
            dq = (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2) + eye, e[:, :, None])).squeeze(-1)
            sc = (0.3 / dq.abs().amax(-1).clamp(min=1e-12)).clamp(max=1.0)
            q = torch.where(done[:, None], q, q + dq * sc[:, None])
        if self.jlo is not None:
            two = 2 * math.pi
            for _ in range(2):
                q = torch.where((q > self.jhi) & (q - two >= self.jlo), q - two, q)
                q = torch.where((q < self.jlo) & (q + two <= self.jhi), q + two, q)
            inside = ((q >= self.jlo) & (q <= self.jhi)).all(-1)
        else:
            inside = torch.ones(q.shape[0], dtype=torch.bool, device=self.dev)
        ep = e[:, :3].norm(dim=-1)
        er = torch.where(inside, torch.rad2deg(e[:, 3:].norm(dim=-1)), torch.full_like(ep, 999.0))
        return q, ep, er


def frame(n):
    """ring_fit._frame batched: (N,3) -> u, v (N,3)."""
    a = torch.zeros_like(n)
    use_z = n[:, 2].abs() < 0.9
    a[:, 2] = use_z.to(n.dtype)
    a[:, 0] = (~use_z).to(n.dtype)
    u = torch.cross(n, a, dim=-1)
    u = u / u.norm(dim=-1, keepdim=True)
    return u, torch.cross(n, u, dim=-1)


def section(c, n, rad, h, h_top, ring_r):
    """Bench.section batched: grip-site target (N,3) and tool frame (N,3,3) at height h (N,) above the ring top."""
    x = -n
    R = torch.stack([x, torch.cross(rad, x, dim=-1), rad], -1)
    return c + (h_top + h)[:, None] * n + ring_r * rad, R
