"""Composite a 25 fps 5-view MP4 video from a recorded replay directory.

Usage (via uv with pillow + imageio-ffmpeg):
  uv run --with pillow --with imageio-ffmpeg python viz/export_video.py \
      --dir runs/replay_p7_earth_v2 --fps 25
"""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Dict, List, Tuple

import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont

BG = (11, 15, 20)
PANEL = (18, 25, 34)
LINE = (35, 48, 64)
FG = (223, 231, 239)
MUTED = (138, 154, 171)

PHASE_COLORS: Dict[str, Tuple[int, int, int]] = {
    "far_hold": (53, 92, 125),
    "far_app": (47, 116, 156),
    "unfold": (82, 137, 181),
    "acq8": (76, 149, 108),
    "xfer_to_hold3": (42, 157, 143),
    "pull_in3": (106, 176, 76),
    "hold3": (212, 155, 40),
    "to1.8": (224, 159, 62),
    "hold1.8": (231, 111, 81),
    "to_station": (217, 93, 57),
    "settle": (200, 75, 49),
    "to pre-reach": (181, 101, 118),
    "to pre-close": (157, 78, 221),
    "descent": (199, 125, 255),
    "close": (224, 170, 255),
    "hold": (79, 209, 139),
    "detumble": (56, 176, 0),
    "unlock_open": (88, 196, 255),
    "retract": (78, 168, 222),
    "retreat_restow": (83, 144, 217),
}


def _load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for p in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "C:/Windows/Fonts/consola.ttf",
    ):
        if Path(p).is_file():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def _draw_pill(draw: ImageDraw.ImageDraw, xy: Tuple[int, int], text: str, font, bg=BG, fg=FG, pad=(8, 4)) -> int:
    x, y = xy
    bb = draw.textbbox((0, 0), text, font=font)
    tw, th = bb[2] - bb[0], bb[3] - bb[1]
    w, h = tw + 2 * pad[0], th + 2 * pad[1]
    draw.rounded_rectangle([x, y, x + w, y + h], radius=5, fill=bg, outline=LINE, width=1)
    draw.text((x + pad[0], y + pad[1] - bb[1]), text, font=font, fill=fg)
    return w


def _build_timeline_strip(frames: List[dict], width: int, height: int) -> Image.Image:
    im = Image.new("RGB", (width, height), (9, 13, 18))
    draw = ImageDraw.Draw(im)
    n = len(frames)
    i = 0
    while i < n:
        j = i + 1
        while j < n and frames[j]["phase"] == frames[i]["phase"]:
            j += 1
        x0 = int(round(i / n * width))
        x1 = int(round(j / n * width))
        col = PHASE_COLORS.get(frames[i]["phase"], (69, 90, 100))
        col_dim = tuple(int(c * 0.82) for c in col)
        draw.rectangle([x0, 0, max(x0 + 1, x1 - 1), height - 1], fill=col_dim)
        i = j
    return im


def export_video(replay_dir: Path, out_mp4: Path, fps: int = 25) -> None:
    manifest = json.loads((replay_dir / "manifest.json").read_text())
    frames = manifest["frames"]
    n = len(frames)
    t_end = frames[-1]["t"]

    # Canvas: 1200 x 680 (even dimensions for H.264 yuv420p)
    W, H = 1200, 680
    pad = 16
    gap = 14
    side = 264
    center = 2 * side + gap  # 542
    total_grid_w = 2 * side + center + 2 * gap  # 1098
    x_left = (W - total_grid_w) // 2
    x_ctr = x_left + side + gap
    x_right = x_ctr + center + gap
    y_top = 18
    y_bot_side = y_top + side + gap

    bar_x, bar_y, bar_w, bar_h = x_left, y_top + center + 16, total_grid_w, 64
    strip_h = 22
    strip_img = _build_timeline_strip(frames, bar_w - 24, strip_h)

    f_lbl = _load_font(12)
    f_hud = _load_font(14)
    f_bar = _load_font(13)

    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [
        ffmpeg, "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{W}x{H}", "-pix_fmt", "rgb24", "-r", str(fps),
        "-i", "-",
        "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p",
        "-crf", "18", "-preset", "medium", "-movflags", "+faststart",
        str(out_mp4),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None

    frames_dir = replay_dir / "frames"
    resamp_lin = Image.Resampling.BILINEAR
    resamp_near = Image.Resampling.NEAREST

    for idx, f in enumerate(frames):
        fn = f["file"]
        canvas = Image.new("RGB", (W, H), BG)
        draw = ImageDraw.Draw(canvas)

        im_lid = Image.open(frames_dir / "lidar" / fn).convert("RGB").resize((side, side), resamp_near)
        im_bus = Image.open(frames_dir / "bus" / fn).convert("RGB").resize((side, side), resamp_lin)
        im_3rd = Image.open(frames_dir / "third" / fn).convert("RGB").resize((center, center), resamp_lin)
        im_stl = Image.open(frames_dir / "stereo_l" / fn).convert("RGB").resize((side, side), resamp_lin)
        im_str = Image.open(frames_dir / "stereo_r" / fn).convert("RGB").resize((side, side), resamp_lin)

        panels = [
            (x_left, y_top, side, im_lid, "BUS LiDAR (range)"),
            (x_left, y_bot_side, side, im_bus, "BUS CAM (optical)"),
            (x_ctr, y_top, center, im_3rd, "3RD-PERSON ORBIT VIEW (550 km LEO)"),
            (x_right, y_top, side, im_stl, "WRIST STEREO L"),
            (x_right, y_bot_side, side, im_str, "WRIST STEREO R"),
        ]
        for px, py, sz, im, lbl in panels:
            canvas.paste(im, (px, py))
            draw.rectangle([px - 1, py - 1, px + sz, py + sz], outline=LINE, width=1)
            _draw_pill(draw, (px + 8, py + 8), lbl, f_lbl, bg=(11, 15, 20))

        # Center HUD (top-right of 3rd-person view)
        ph = f["phase"]
        ph_col = PHASE_COLORS.get(ph, (240, 180, 41))
        t_txt = f"t = {f['t']:.1f} s"
        r_txt = f"range: {f['range_m']:.2f} m"
        bb_t = draw.textbbox((0, 0), t_txt, font=f_hud)
        bb_p = draw.textbbox((0, 0), ph, font=f_lbl)
        row1_w = (bb_t[2] - bb_t[0]) + (bb_p[2] - bb_p[0]) + 34
        rx = x_ctr + center - row1_w - 10
        ry = y_top + 10
        draw.rounded_rectangle([rx, ry, rx + row1_w, ry + 26], radius=5, fill=BG, outline=LINE, width=1)
        draw.text((rx + 8, ry + 5), t_txt, font=f_hud, fill=FG)
        px_chip = rx + (bb_t[2] - bb_t[0]) + 16
        draw.rounded_rectangle([px_chip, ry + 4, px_chip + (bb_p[2] - bb_p[0]) + 12, ry + 22], radius=8, fill=ph_col)
        draw.text((px_chip + 6, ry + 6), ph, font=f_lbl, fill=(11, 15, 20))

        bb_r = draw.textbbox((0, 0), r_txt, font=f_hud)
        r_w = (bb_r[2] - bb_r[0]) + 16
        draw.rounded_rectangle([x_ctr + center - r_w - 10, ry + 32, x_ctr + center - 10, ry + 56], radius=5, fill=BG, outline=LINE, width=1)
        draw.text((x_ctr + center - r_w - 2, ry + 36), r_txt, font=f_hud, fill=FG)

        # Bottom timeline bar
        draw.rounded_rectangle([bar_x, bar_y, bar_x + bar_w, bar_y + bar_h], radius=8, fill=PANEL, outline=LINE, width=1)
        status_txt = f"{f['t']:.1f} / {t_end:.1f} s   ·   {ph}   ·   prop {f['prop_g']:.1f} g   ·   rw {f['wheel_fill']*100:.1f}%"
        draw.text((bar_x + 12, bar_y + 9), status_txt, font=f_bar, fill=FG)
        sx, sy = bar_x + 12, bar_y + 32
        canvas.paste(strip_img, (sx, sy))
        head_x = sx + int(round(idx / max(n - 1, 1) * (bar_w - 24)))
        draw.rectangle([head_x - 1, sy - 3, head_x + 1, sy + strip_h + 2], fill=(255, 255, 255))

        proc.stdin.write(canvas.tobytes())

    proc.stdin.close()
    err = proc.stderr.read().decode("utf-8", errors="replace") if proc.stderr else ""
    rc = proc.wait()
    if rc != 0:
        raise RuntimeError(f"ffmpeg failed ({rc}):\n{err[-800:]}")
    print(f"VIDEO SAVED: {out_mp4} ({n} frames @ {fps} fps, {out_mp4.stat().st_size / (1024*1024):.2f} MB)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--fps", type=int, default=25)
    a = ap.parse_args()
    out_mp4 = a.out or (a.dir / f"replay_{a.fps}fps.mp4")
    export_video(a.dir, out_mp4, fps=a.fps)


if __name__ == "__main__":
    main()
