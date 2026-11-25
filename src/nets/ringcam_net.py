"""Wrist-camera ring net (P7.6 lead, user 22 Nov): from the wrist camera image at the pre-reach pose (128 px grey,
the bar-mission camera spec), the ring centre's offset from the estimate in the estimated section frame (radial,
tangential, axial; metres) and a per-axis standard deviation (Gaussian NLL), so the reach can gate on it.

  train:  bash tools/rlrun.sh python ../projects/p7_unprepared_client/nets/ringcam_net.py --epochs 30
          (data: runs/ringcam_data.npz from checks/gen_ringcam.py; last 2000 samples held out; writes
           runs/ringcam_net.pt and runs/ringcam_net.json)
  use:    RingCamNet.load(path).predict(img_uint8_128x128) -> (d (3,), sigma (3,)) in metres
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import RUNS_DIR, ensure_runs_dir, find_weight  # noqa: E402

SCALE = 0.03          # metres per output unit


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        c = lambda i, o: nn.Sequential(nn.Conv2d(i, o, 5, 2, 2), nn.BatchNorm2d(o), nn.ReLU())  # noqa: E731
        self.f = nn.Sequential(c(1, 16), c(16, 32), c(32, 64), c(64, 64), c(64, 64))      # 128 -> 4
        self.h = nn.Sequential(nn.Flatten(), nn.Linear(64 * 4 * 4, 128), nn.ReLU(), nn.Linear(128, 6))

    def forward(self, x):
        o = self.h(self.f(x))
        return o[:, :3], o[:, 3:].clamp(-8, 4)      # mean (scaled), log variance (scaled units)


class RingCamNet:
    def __init__(self, net, device="cpu"):
        self.net, self.dev = net.to(device).eval(), device

    @classmethod
    def load(cls, path=None, device="cpu"):
        n = Net()
        p = Path(path) if path else find_weight("ringcam_net.pt")
        n.load_state_dict(torch.load(p, map_location=device, weights_only=True))
        return cls(n, device)

    @torch.no_grad()
    def predict(self, img):
        x = torch.as_tensor(np.asarray(img, np.float32)[None, None] / 255.0 - 0.5, device=self.dev)
        m, lv = self.net(x)
        return m[0].cpu().numpy() * SCALE, np.exp(0.5 * lv[0].cpu().numpy()) * SCALE


def main():
    ensure_runs_dir()
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--val", type=int, default=2000)
    a = ap.parse_args()
    d = np.load(RUNS_DIR / "ringcam_data.npz")
    X = torch.as_tensor(d["img"][:, None].astype(np.float32) / 255.0 - 0.5)
    Y = torch.as_tensor(d["y"] / SCALE)
    Xt, Yt, Xv, Yv = X[:-a.val], Y[:-a.val], X[-a.val:], Y[-a.val:]
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0)
    net = Net().to(dev)
    opt = torch.optim.AdamW(net.parameters(), 2e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=a.epochs * ((len(Xt) + a.bs - 1) // a.bs))
    t0 = time.time()
    for ep in range(a.epochs):
        net.train()
        perm = torch.randperm(len(Xt))
        for i in range(0, len(Xt), a.bs):
            idx = perm[i:i + a.bs]
            x, y = Xt[idx].to(dev), Yt[idx].to(dev)
            g = 1 + 0.15 * torch.randn(len(x), 1, 1, 1, device=dev)          # brightness / contrast jitter
            x = (x + 0.5) * g - 0.5 + 0.02 * torch.randn_like(x)
            m, lv = net(x)
            loss = (0.5 * (lv + (y - m) ** 2 / lv.exp())).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
        net.eval()
        with torch.no_grad():
            ms, ls = zip(*[net(Xv[i:i + 500].to(dev)) for i in range(0, len(Xv), 500)])
            m, lv = torch.cat(ms).cpu(), torch.cat(ls).cpu()
        e = (m - Yv).abs() * SCALE * 1e3
        print(f"ep {ep:2d} loss {loss.item():.3f} val |err| mm p50 {np.round(e.median(0).values.numpy(), 1)} "
              f"p95 {np.round(np.percentile(e.numpy(), 95, axis=0), 1)}  {time.time() - t0:.0f}s", flush=True)
    sig = (0.5 * lv).exp() * SCALE * 1e3
    z = ((m - Yv).abs() * SCALE * 1e3 / sig).numpy()
    out = dict(val_n=len(Xv), err_mm_p50=np.round(np.median(e.numpy(), 0), 2).tolist(),
               err_mm_p95=np.round(np.percentile(e.numpy(), 95, 0), 2).tolist(),
               label_mm_p50=np.round(np.median((Yv.abs() * SCALE * 1e3).numpy(), 0), 2).tolist(),
               sigma_mm_p50=np.round(np.median(sig.numpy(), 0), 2).tolist(),
               within_2sigma=np.round((z < 2).mean(0), 3).tolist(), epochs=a.epochs, train_s=round(time.time() - t0))
    print(json.dumps(out))
    torch.save(net.cpu().state_dict(), RUNS_DIR / "ringcam_net.pt")
    (RUNS_DIR / "ringcam_net.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
