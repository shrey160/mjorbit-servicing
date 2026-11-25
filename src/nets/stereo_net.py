"""Wrist stereo nets (SPEC_7 s25, 23 Nov): the ring net (ringcam_net.py) generalised to several views stacked as
input channels and five outputs: the ring centre's offset from the estimate in the estimated section frame (radial,
tangential, axial; m) and the true axis' tilt along the estimated radial and tangential (rad), each with a sigma
(Gaussian NLL). One architecture, trained on the same samples per view set, so the views are compared like for like:
  in        the inboard camera only (mono: the ring net's role, on the new camera pose)
  in+out    the stereo module's two cameras (50 mm apart along the jaws' travel)
  in+mot    motion stereo (inboard camera, then again 40 mm radially out)

  train:  bash tools/rlrun.sh python ../projects/p7_unprepared_client/nets/stereo_net.py --views in+out --epochs 30
          (data: runs/stereo_data.npz from checks/gen_stereo.py; last 2000 held out; writes runs/stereo_net_<views>.pt
           and .json)
  use:    StereoNet.load(views).predict(imgs_uint8 (k, 128, 128)) -> (y (5,), sigma (5,))
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn

if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _paths import RUNS_DIR, ensure_runs_dir, find_weight  # noqa: E402

SCALE = np.array([0.03, 0.03, 0.03, 0.03, 0.03], np.float32)   # metres / radians per output unit
VIEW_IDX = {"in": 0, "out": 1, "mot": 2}


class Net(nn.Module):
    def __init__(self, cin):
        super().__init__()
        c = lambda i, o: nn.Sequential(nn.Conv2d(i, o, 5, 2, 2), nn.BatchNorm2d(o), nn.ReLU())  # noqa: E731
        self.f = nn.Sequential(c(cin, 32), c(32, 48), c(48, 64), c(64, 64), c(64, 64))      # 128 -> 4
        self.h = nn.Sequential(nn.Flatten(), nn.Linear(64 * 4 * 4, 192), nn.ReLU(), nn.Linear(192, 10))

    def forward(self, x):
        o = self.h(self.f(x))
        return o[:, :5], o[:, 5:].clamp(-8, 4)


def gpu_temp():
    try:
        return int(subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                                  capture_output=True, text=True, timeout=10).stdout.strip().splitlines()[0])
    except Exception:  # noqa: BLE001
        return -1


def thermal_guard(t_pause, t_resume, t_stop):
    """23 Nov (the laptop shut down at the start of the first GPU run): pause while the GPU is hot."""
    t = gpu_temp()
    if t >= t_stop:
        raise SystemExit(f"GPU {t} C >= {t_stop} C: stopped")
    if t >= t_pause:
        print(f"GPU {t} C: pausing until {t_resume} C", flush=True)
        while t > t_resume:
            time.sleep(10)
            t = gpu_temp()
            if t >= t_stop:
                raise SystemExit(f"GPU {t} C >= {t_stop} C: stopped")
        print(f"GPU {t} C: resuming", flush=True)
    return t


def idx(views):
    return [VIEW_IDX[v] for v in views.split("+")]


class StereoNet:
    def __init__(self, net, views, device="cpu"):
        self.net, self.views, self.dev = net.to(device).eval(), views, device

    @classmethod
    def load(cls, views, device="cpu", suffix=""):
        n = Net(len(idx(views)))
        p = find_weight(f"stereo_net_{views}{suffix}.pt")
        if suffix and not p.exists():
            p = find_weight(f"stereo_net_{views}.pt")
        n.load_state_dict(torch.load(p, map_location=device, weights_only=True))
        return cls(n, views, device)

    @torch.no_grad()
    def predict(self, imgs):
        x = torch.as_tensor(np.asarray(imgs, np.float32)[None] / 255.0 - 0.5, device=self.dev)
        m, lv = self.net(x)
        return m[0].cpu().numpy() * SCALE, np.exp(0.5 * lv[0].cpu().numpy()) * SCALE


def main():
    ensure_runs_dir()
    ap = argparse.ArgumentParser()
    ap.add_argument("--views", default="in+out")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--val", type=int, default=2000)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--data", default=str(RUNS_DIR / "stereo_data.npz"))
    ap.add_argument("--init-from", default=None)
    ap.add_argument("--out-prefix", default=None)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--duty", type=float, default=1.0, help="GPU duty cycle: sleep (1/duty - 1) x each batch's time")
    ap.add_argument("--t-pause", type=int, default=80)
    ap.add_argument("--t-resume", type=int, default=72)
    ap.add_argument("--t-stop", type=int, default=87)
    a = ap.parse_args()
    prefix = a.out_prefix or f"stereo_net_{a.views}"
    d = np.load(a.data)
    ii = idx(a.views)
    X = torch.as_tensor(np.ascontiguousarray(d["img"][:, ii]))       # uint8; scaled per batch (memory)
    f32 = lambda x: x.float() / 255.0 - 0.5  # noqa: E731
    Y = torch.as_tensor(d["y"] / SCALE)
    if a.val <= 0 or a.val >= len(X):
        Xt, Yt, Xv, Yv = X, Y, X, Y
    else:
        Xt, Yt, Xv, Yv = X[:-a.val], Y[:-a.val], X[-a.val:], Y[-a.val:]
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0)
    net = Net(len(ii)).to(dev)
    t0 = time.time()
    unit = np.array([1e3, 1e3, 1e3, np.degrees(1), np.degrees(1)])     # mm, mm, mm, deg, deg
    if a.eval_only:
        if a.init_from:
            net.load_state_dict(torch.load(a.init_from, map_location=dev, weights_only=True))
        net.eval()
        with torch.no_grad():
            ms, ls = zip(*[net(f32(Xv[i:i + 500].to(dev))) for i in range(0, len(Xv), 500)])
            m, lv = torch.cat(ms).cpu(), torch.cat(ls).cpu()
        e = (m - Yv).abs().numpy() * SCALE * unit
        sig = (0.5 * lv).exp().numpy() * SCALE * unit
        z = e / sig
        lab = Yv.abs().numpy() * SCALE * unit
        out = dict(views=a.views, val_n=len(Xv), units="radial tangential axial mm, tilt_rad tilt_tan deg",
                   err_p50=np.round(np.median(e, 0), 2).tolist(), err_p95=np.round(np.percentile(e, 95, 0), 2).tolist(),
                   label_p50=np.round(np.median(lab, 0), 2).tolist(), sigma_p50=np.round(np.median(sig, 0), 2).tolist(),
                   within_2sigma=np.round((z < 2).mean(0), 3).tolist(), eval_only=True)
        print(json.dumps(out))
        return
    opt = torch.optim.AdamW(net.parameters(), a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=a.epochs * ((len(Xt) + a.bs - 1) // a.bs))
    # checkpoint every 5 epochs (23 Nov: the laptop shut down twice mid-run); a rerun resumes from it
    ck = RUNS_DIR / f"{prefix}.ckpt"
    ep0 = 0
    if ck.exists():
        c = torch.load(ck, map_location=dev, weights_only=False)
        net.load_state_dict(c["net"])
        opt.load_state_dict(c["opt"])
        sched.load_state_dict(c["sched"])
        torch.set_rng_state(c["rng"].cpu())
        ep0 = c["ep"] + 1
        print(f"{prefix}: resumed after epoch {c['ep']}", flush=True)
    elif a.init_from:
        net.load_state_dict(torch.load(a.init_from, map_location=dev, weights_only=True))
        print(f"{prefix}: initialized from {a.init_from}", flush=True)
    for ep in range(ep0, a.epochs):
        net.train()
        perm = torch.randperm(len(Xt))
        for k, i in enumerate(range(0, len(Xt), a.bs)):
            if k % 25 == 0:
                thermal_guard(a.t_pause, a.t_resume, a.t_stop)
            tb = time.time()
            b = perm[i:i + a.bs]
            x, y = f32(Xt[b].to(dev)), Yt[b].to(dev)
            g = 1 + 0.15 * torch.randn(len(x), 1, 1, 1, device=dev)          # brightness / contrast jitter
            x = (x + 0.5) * g - 0.5 + 0.02 * torch.randn_like(x)
            m, lv = net(x)
            loss = (0.5 * (lv + (y - m) ** 2 / lv.exp())).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            if a.duty < 1.0:
                torch.cuda.synchronize() if dev == "cuda" else None
                time.sleep((time.time() - tb) * (1.0 / a.duty - 1.0))
        net.eval()
        with torch.no_grad():
            ms, ls = zip(*[net(f32(Xv[i:i + 500].to(dev))) for i in range(0, len(Xv), 500)])
            m, lv = torch.cat(ms).cpu(), torch.cat(ls).cpu()
        e = (m - Yv).abs().numpy() * SCALE * unit
        if ep % 5 == 4 or ep == a.epochs - 1:
            print(f"GPU {gpu_temp()} C | {a.views} ep {ep:2d} loss {loss.item():.3f} val |err| p50 {np.round(np.median(e, 0), 2)} "
                  f"p95 {np.round(np.percentile(e, 95, 0), 2)}  {time.time() - t0:.0f}s", flush=True)
            tmp = ck.with_suffix(".tmp")
            torch.save(dict(net=net.state_dict(), opt=opt.state_dict(), sched=sched.state_dict(),
                            rng=torch.get_rng_state(), ep=ep), tmp)
            tmp.replace(ck)
    sig = (0.5 * lv).exp().numpy() * SCALE * unit
    z = e / sig
    lab = Yv.abs().numpy() * SCALE * unit
    out = dict(views=a.views, val_n=len(Xv), units="radial tangential axial mm, tilt_rad tilt_tan deg",
               err_p50=np.round(np.median(e, 0), 2).tolist(), err_p95=np.round(np.percentile(e, 95, 0), 2).tolist(),
               label_p50=np.round(np.median(lab, 0), 2).tolist(), sigma_p50=np.round(np.median(sig, 0), 2).tolist(),
               within_2sigma=np.round((z < 2).mean(0), 3).tolist(), epochs=a.epochs, train_s=round(time.time() - t0))
    print(json.dumps(out))
    torch.save(net.cpu().state_dict(), RUNS_DIR / f"{prefix}.pt")
    (RUNS_DIR / f"{prefix}.json").write_text(json.dumps(out, indent=1))
    ck.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
