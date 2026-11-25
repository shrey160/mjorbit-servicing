"""Shared G3 residual policy network, observation builder, and CPU controller wrapper."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import torch
from torch import nn

NOBS = 28


class G3Policy(nn.Module):
    """Dual Tanh MLP actor-critic predicting 3D section-frame residual offset velocity."""

    def __init__(self, obs_dim: int = NOBS, hidden: int = 128):
        super().__init__()
        self.pi = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 3),
        )
        self.v = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1),
        )
        nn.init.zeros_(self.pi[-1].weight)
        nn.init.zeros_(self.pi[-1].bias)
        self.log_std = nn.Parameter(torch.full((3,), -1.0))

    def dist(self, x: torch.Tensor) -> torch.distributions.Normal:
        return torch.distributions.Normal(self.pi(x), self.log_std.exp())

    def value(self, x: torch.Tensor) -> torch.Tensor:
        return self.v(x).squeeze(-1)

    @torch.no_grad()
    def act(self, x: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        d = self.dist(x)
        a = (d.mean if deterministic else d.sample()).clamp(-1.0, 1.0)
        return a, d.log_prob(a).sum(-1), self.value(x)

    def save(self, path: Path | str, meta: Optional[Dict[str, Any]] = None) -> None:
        torch.save({"ac": self.state_dict(), "meta": dict(meta or {})}, Path(path))

    @classmethod
    def load(cls, path: Path | str, device: str = "cpu") -> Tuple["G3Policy", Dict[str, Any]]:
        ckpt = torch.load(Path(path), map_location=device, weights_only=False)
        sd = ckpt["ac"] if isinstance(ckpt, dict) and "ac" in ckpt else ckpt
        meta = dict(ckpt.get("meta", {})) if isinstance(ckpt, dict) else {}
        model = cls().to(device)
        model.load_state_dict(sd)
        model.eval()
        return model, meta


def build_obs_torch(
    *,
    rad: torch.Tensor,
    n: torch.Tensor,
    p_target: torch.Tensor,
    p_site: torch.Tensor,
    p_cmd: torch.Tensor,
    offset: torch.Tensor,
    s_prev: Optional[torch.Tensor],
    a_prev: torch.Tensor,
    a_w: torch.Tensor,
    h: torch.Tensor,
    at_top: torch.Tensor,
    n_wait: torch.Tensor,
    dt: float,
    dec: int,
    catch_s: float,
    touches: torch.Tensor,
    wf_delta: torch.Tensor,
    o_max: float,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Build exact 28-dim float32 G3 observation tensor for a batch of N worlds (N=1 on CPU)."""
    Bm = torch.stack([rad, torch.cross(n, rad, dim=-1), n], dim=-1)  # (N, 3, 3) world <- section
    BmT = Bm.transpose(-2, -1)
    s_now = p_target + (Bm @ offset.unsqueeze(-1)).squeeze(-1)
    if s_prev is None:
        vel = torch.zeros_like(s_now)
    else:
        vel = (s_now - s_prev) / max(dec * dt, 1e-9)

    def loc(v: torch.Tensor) -> torch.Tensor:
        return (BmT @ v.unsqueeze(-1)).squeeze(-1)

    x = torch.cat(
        [
            loc(s_now - p_cmd) / 0.01,
            loc(p_site - p_cmd) / 0.005,
            loc(vel) / 0.03,
            loc(a_w) / math.radians(4.0),
            (h / 0.05).unsqueeze(-1),
            at_top.to(offset.dtype).unsqueeze(-1),
            (n_wait * (5.0 * dt / catch_s)).unsqueeze(-1),
            (touches / 5.0).clamp(max=4.0),
            (wf_delta / 2.0).clamp(-4.0, 4.0),
            offset / o_max,
            a_prev,
        ],
        dim=-1,
    )
    return x.float(), s_now


class G3Controller:
    """Single-env stateful CPU wrapper matching GPU G3 rollout cadence (10 Hz residual offset)."""

    def __init__(
        self,
        policy: G3Policy,
        *,
        dec: int = 50,
        a_vel: float = 0.008,
        o_max: float = 0.008,
        catch_s: float = 3.0,
        dt: float = 0.002,
    ):
        self.policy = policy
        self.dec = int(dec)
        self.a_vel = float(a_vel)
        self.o_max = float(o_max)
        self.catch_s = float(catch_s)
        self.dt = float(dt)
        self.ticks_per_dec = max(1, self.dec // 5)
        self.reset()

    @classmethod
    def load(cls, path: Optional[Path | str], *, dt: float = 0.002, catch_s: float = 3.0) -> Optional["G3Controller"]:
        if path is None or str(path).strip() == "":
            return None
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"G3 policy checkpoint not found: {p}")
        policy, meta = G3Policy.load(p, device="cpu")
        dec = int(meta.get("dec", 50))
        a_vel = float(meta.get("a_vel", 0.005 if not meta else 0.008))
        o_max = float(meta.get("o_max", 0.006 if not meta else 0.008))
        return cls(policy, dec=dec, a_vel=a_vel, o_max=o_max, catch_s=catch_s, dt=dt)

    def reset(self) -> None:
        self.k_desc = 0
        self.offset = np.zeros(3, dtype=np.float64)
        self.s_prev: Optional[torch.Tensor] = None
        self.a_prev = torch.zeros(1, 3, dtype=torch.float64)

    def world_offset(self, n: np.ndarray, rad: np.ndarray) -> np.ndarray:
        """Convert current stored section-frame offset to world frame."""
        t_axis = np.cross(n, rad)
        return float(self.offset[0]) * rad + float(self.offset[1]) * t_axis + float(self.offset[2]) * n

    def step(
        self,
        *,
        rad: np.ndarray,
        n: np.ndarray,
        p_target: np.ndarray,
        p_site: np.ndarray,
        p_cmd: np.ndarray,
        a_w: np.ndarray,
        h: float,
        at_top: bool,
        n_wait: int,
        touches: np.ndarray,
        wf_delta: np.ndarray,
    ) -> np.ndarray:
        """Called every 100 Hz descent tick; updates residual offset at 10 Hz and returns world-frame offset."""
        if (self.k_desc % self.ticks_per_dec) == 0:
            t64 = lambda arr: torch.tensor(np.asarray(arr, dtype=np.float64)[None], dtype=torch.float64)  # noqa: E731
            off_t = t64(self.offset)
            x, s_now = build_obs_torch(
                rad=t64(rad),
                n=t64(n),
                p_target=t64(p_target),
                p_site=t64(p_site),
                p_cmd=t64(p_cmd),
                offset=off_t,
                s_prev=self.s_prev,
                a_prev=self.a_prev,
                a_w=t64(a_w),
                h=torch.tensor([float(h)], dtype=torch.float64),
                at_top=torch.tensor([bool(at_top)], dtype=torch.bool),
                n_wait=torch.tensor([float(n_wait)], dtype=torch.float64),
                dt=self.dt,
                dec=self.dec,
                catch_s=self.catch_s,
                touches=t64(touches),
                wf_delta=t64(wf_delta),
                o_max=self.o_max,
            )
            self.s_prev = s_now
            a, _, _ = self.policy.act(x, deterministic=True)
            a64 = a.to(torch.float64)
            step_m = self.a_vel * (self.dec * self.dt)
            new_off = (off_t + a64 * step_m).clamp(-self.o_max, self.o_max)
            self.offset = new_off[0].numpy().copy()
            self.a_prev = a64
        self.k_desc += 1
        return self.world_offset(n, rad)
