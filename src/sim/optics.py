"""Opt-in orbital camera optical post-processing (bloom, auto-exposure roll-off, shot & read noise)."""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.ndimage import gaussian_filter


def apply_space_optics(
    img_u8: np.ndarray,
    rng: Optional[np.random.Generator] = None,
    *,
    auto_exposure: bool = True,
    bloom: bool = True,
    shot_noise: float = 1.2,
    read_noise: float = 1.5,
) -> np.ndarray:
    """Apply physical space-camera optical effects to an RGB or grayscale uint8 image."""
    arr = np.asarray(img_u8, dtype=np.float32)
    is_rgb = arr.ndim == 3

    if bloom:
        hi = np.maximum(arr - 215.0, 0.0)
        sig_tight = (1.6, 1.6, 0.0) if is_rgb else 1.6
        sig_wide = (4.5, 4.5, 0.0) if is_rgb else 4.5
        arr = arr + 0.50 * gaussian_filter(hi, sigma=sig_tight, mode="nearest") + 0.25 * gaussian_filter(
            hi, sigma=sig_wide, mode="nearest"
        )

    if auto_exposure:
        lum = arr.mean(axis=2) if is_rgb else arr
        lit = lum[lum > 8.0]
        if lit.size > max(16, int(0.005 * lum.size)):
            p95 = float(np.percentile(lit, 95.0))
            ae_gain = float(np.clip(190.0 / max(p95, 25.0), 0.72, 1.32))
            arr = arr * ae_gain
        # Soft shoulder roll-off above 204 DN (preserves crisp highlight detail on sunlit MLI/Earth limb)
        x = arr / 255.0
        knee = 0.80
        arr = np.where(x > knee, knee + 0.20 * np.tanh((x - knee) / 0.35), x) * 255.0

    if shot_noise > 0.0 or read_noise > 0.0:
        r = rng if rng is not None else np.random.default_rng(0)
        if shot_noise > 0.0:
            arr = arr + r.normal(0.0, float(shot_noise), size=arr.shape).astype(np.float32) * np.sqrt(
                np.maximum(arr, 0.0) / 255.0
            )
        if read_noise > 0.0:
            arr = arr + r.normal(0.0, float(read_noise), size=arr.shape).astype(np.float32)

    return np.clip(np.rint(arr), 0, 255).astype(np.uint8)
