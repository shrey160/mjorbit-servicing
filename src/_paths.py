"""Centralized path resolution and import bootstrapping for mjorbit-servicing."""
from __future__ import annotations

import os
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent
REPO_ROOT = SRC_DIR.parent
ASSETS_DIR = REPO_ROOT / "assets"
WEIGHTS_DIR = REPO_ROOT / "weights"
RUNS_DIR = REPO_ROOT / "runs"

_pbr_env = os.environ.get("MJORBIT_PBR_DIR", "").strip()
if _pbr_env:
    PBR_ASSETS_DIR = Path(_pbr_env).expanduser().resolve()
elif (ASSETS_DIR / "pbr_assets").is_dir():
    PBR_ASSETS_DIR = ASSETS_DIR / "pbr_assets"
else:
    PBR_ASSETS_DIR = RUNS_DIR / "pbr_assets"

_sim_env = os.environ.get("MJORBIT_DIR", "").strip()
if _sim_env:
    SIM_DIR = Path(_sim_env).expanduser().resolve()
else:
    SIM_DIR = REPO_ROOT / "simulator"
SIM_SRC = SIM_DIR / "src"

_SUBDIRS = ("gnc", "perception", "servicer", "checks", "mission", "nets", "rl", "specs", "sim", "phases", "viz")


def find_weight(filename: str | Path) -> Path:
    """Resolve a checkpoint or calibration artifact from runs/ first, then weights/."""
    p = Path(filename)
    if p.is_absolute() or p.exists():
        return p
    run_cand = RUNS_DIR / p.name
    if run_cand.exists():
        return run_cand
    wt_cand = WEIGHTS_DIR / p.name
    if wt_cand.exists():
        return wt_cand
    return run_cand


def ensure_runs_dir() -> Path:
    """Create and return the runs output directory."""
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    return RUNS_DIR


def bootstrap() -> None:
    """Idempotently add src packages and upstream mjorbit simulator/src onto sys.path."""
    paths = [SRC_DIR, *(SRC_DIR / sub for sub in _SUBDIRS)]
    if SIM_SRC.is_dir():
        paths.append(SIM_SRC)
    for p in paths:
        sp = str(p)
        if sp not in sys.path:
            sys.path.insert(0, sp)


bootstrap()
