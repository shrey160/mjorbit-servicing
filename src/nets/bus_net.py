"""Far-range bus_cam orbital bearing detector (predicts bus center in 64x64 RGB ROI + uncertainty)."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from PIL import Image

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import RUNS_DIR, ensure_runs_dir, find_weight  # noqa: E402

CROP_SZ = 64


def gpu_temp():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"], text=True
        )
        return int(out.strip().splitlines()[0])
    except Exception:
        return 0


def thermal_guard(t_pause=80, t_resume=72, t_stop=87):
    t = gpu_temp()
    if t >= t_stop:
        raise SystemExit(f"GPU {t} C >= {t_stop} C: stopping")
    if t >= t_pause:
        print(f"GPU {t} C: pausing until {t_resume} C", flush=True)
        while gpu_temp() > t_resume:
            time.sleep(5)
        print(f"GPU {gpu_temp()} C: resuming", flush=True)


class Net(nn.Module):
    def __init__(self, cin=3):
        super().__init__()
        b = lambda a, o, s=2: [nn.Conv2d(a, o, 3, s, 1, bias=False), nn.BatchNorm2d(o), nn.GELU()]  # noqa: E731
        self.f = nn.Sequential(*b(cin, 24), *b(24, 48), *b(48, 96), *b(96, 128), nn.AdaptiveAvgPool2d(1), nn.Flatten())
        self.h = nn.Sequential(nn.Linear(128, 64), nn.GELU(), nn.Linear(64, 4))

    def forward(self, x):
        o = self.h(self.f(x))
        return o[:, :2], o[:, 2:].clamp(-8.0, 4.0)


class BusNet:
    def __init__(self, net, device="cpu"):
        self.net, self.dev = net.to(device).eval(), device

    @classmethod
    def load(cls, device="cpu", path=None):
        p = Path(path) if path else find_weight("bus_net_orb.pt")
        n = Net(3)
        n.load_state_dict(torch.load(p, map_location=device, weights_only=True))
        return cls(n, device)

    @torch.no_grad()
    def predict(self, crop_rgb_u8):
        crop = Image.fromarray(crop_rgb_u8).resize((CROP_SZ, CROP_SZ), Image.BILINEAR)
        arr = np.transpose(np.asarray(crop, dtype=np.float32), (2, 0, 1)) / 255.0 - 0.5
        x = torch.as_tensor(arr[None], device=self.dev)
        m, lv = self.net(x)
        return m[0].cpu().numpy(), np.exp(0.5 * lv[0].cpu().numpy())


def main():
    ensure_runs_dir()
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(RUNS_DIR / "buscam_data_orb.npz"))
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--val", type=int, default=1500)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--out-prefix", default="bus_net_orb")
    ap.add_argument("--init-from", default=None)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--duty", type=float, default=0.375)
    ap.add_argument("--t-pause", type=int, default=80)
    ap.add_argument("--t-resume", type=int, default=72)
    ap.add_argument("--t-stop", type=int, default=87)
    a = ap.parse_args()

    d = np.load(a.data)
    X = torch.as_tensor(np.ascontiguousarray(d["img"]))
    Y = torch.as_tensor(d["y"], dtype=torch.float32)
    f32 = lambda x: x.float() / 255.0 - 0.5  # noqa: E731
    if a.val <= 0 or a.val >= len(X):
        Xt, Yt, Xv, Yv = X, Y, X, Y
    else:
        Xt, Yt, Xv, Yv = X[:-a.val], Y[:-a.val], X[-a.val:], Y[-a.val:]

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0)
    net = Net(3).to(dev)
    t0 = time.time()

    if a.eval_only:
        if a.init_from:
            net.load_state_dict(torch.load(a.init_from, map_location=dev, weights_only=True))
        net.eval()
        with torch.no_grad():
            ms, ls = zip(*[net(f32(Xv[i:i + 500].to(dev))) for i in range(0, len(Xv), 500)])
            m, lv = torch.cat(ms).cpu(), torch.cat(ls).cpu()
        e = (m - Yv).abs().numpy()
        sig = (0.5 * lv).exp().numpy()
        z = e / sig
        out = dict(
            val_n=len(Xv),
            units="normalized ROI [-1, 1] (1.0 = 3.5 m)",
            err_p50=np.round(np.median(e, 0), 4).tolist(),
            err_p95=np.round(np.percentile(e, 95, 0), 4).tolist(),
            sigma_p50=np.round(np.median(sig, 0), 4).tolist(),
            within_2sigma=np.round((z < 2).mean(0), 3).tolist(),
            eval_only=True,
        )
        print(json.dumps(out))
        return

    opt = torch.optim.AdamW(net.parameters(), a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=a.epochs * ((len(Xt) + a.bs - 1) // a.bs))
    ck = RUNS_DIR / f"{a.out_prefix}.ckpt"
    ep0 = 0
    if ck.exists():
        c = torch.load(ck, map_location=dev, weights_only=False)
        net.load_state_dict(c["net"])
        opt.load_state_dict(c["opt"])
        sched.load_state_dict(c["sched"])
        torch.set_rng_state(c["rng"].cpu())
        ep0 = c["ep"] + 1
    elif a.init_from:
        net.load_state_dict(torch.load(a.init_from, map_location=dev, weights_only=True))

    for ep in range(ep0, a.epochs):
        net.train()
        perm = torch.randperm(len(Xt))
        for k, i in enumerate(range(0, len(Xt), a.bs)):
            if k % 25 == 0:
                thermal_guard(a.t_pause, a.t_resume, a.t_stop)
            tb = time.time()
            b = perm[i:i + a.bs]
            x, y = f32(Xt[b].to(dev)), Yt[b].to(dev)
            g = 1 + 0.15 * torch.randn(len(x), 1, 1, 1, device=dev)
            x = (x + 0.5) * g - 0.5 + 0.02 * torch.randn_like(x)
            m, lv = net(x)
            loss = (0.5 * (lv + (y - m) ** 2 / lv.exp())).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            if a.duty < 1.0:
                if dev == "cuda":
                    torch.cuda.synchronize()
                time.sleep((time.time() - tb) * (1.0 / a.duty - 1.0))

        net.eval()
        with torch.no_grad():
            ms, ls = zip(*[net(f32(Xv[i:i + 500].to(dev))) for i in range(0, len(Xv), 500)])
            m, lv = torch.cat(ms).cpu(), torch.cat(ls).cpu()
        e = (m - Yv).abs().numpy()
        if ep % 5 == 4 or ep == a.epochs - 1:
            print(
                f"GPU {gpu_temp()} C | bus ep {ep:2d} loss {loss.item():.3f} val |err| p50 {np.round(np.median(e, 0), 4)} "
                f"p95 {np.round(np.percentile(e, 95, 0), 4)}  {time.time() - t0:.0f}s",
                flush=True,
            )
            tmp = ck.with_suffix(".tmp")
            torch.save(dict(net=net.state_dict(), opt=opt.state_dict(), sched=sched.state_dict(), rng=torch.get_rng_state(), ep=ep), tmp)
            tmp.replace(ck)

    sig = (0.5 * lv).exp().numpy()
    z = e / sig
    out = dict(
        val_n=len(Xv),
        units="normalized ROI [-1, 1] (1.0 = 3.5 m)",
        err_p50=np.round(np.median(e, 0), 4).tolist(),
        err_p95=np.round(np.percentile(e, 95, 0), 4).tolist(),
        sigma_p50=np.round(np.median(sig, 0), 4).tolist(),
        within_2sigma=np.round((z < 2).mean(0), 3).tolist(),
        epochs=a.epochs,
        train_s=round(time.time() - t0),
    )
    print(json.dumps(out))
    torch.save(net.cpu().state_dict(), RUNS_DIR / f"{a.out_prefix}.pt")
    (RUNS_DIR / f"{a.out_prefix}.json").write_text(json.dumps(out, indent=1))
    ck.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
