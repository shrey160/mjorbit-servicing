#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${MJORBIT_DIR:-$REPO_ROOT/simulator}"
PIXI_BIN="${PIXI:-$(command -v pixi || true)}"
if [[ -z "$PIXI_BIN" && -x "$HOME/.pixi/bin/pixi" ]]; then
  PIXI_BIN="$HOME/.pixi/bin/pixi"
fi

if [[ -z "$PIXI_BIN" ]]; then
  echo "Error: pixi not found on PATH or at \$HOME/.pixi/bin/pixi" >&2
  exit 1
fi

MANIFEST="$SIM_DIR/pixi.toml"
if [[ ! -f "$MANIFEST" ]]; then
  MANIFEST="$SIM_DIR/pyproject.toml"
fi

if [[ ! -f "$MANIFEST" ]]; then
  cat >&2 <<EOF
Error: mjorbit simulator not found at: $SIM_DIR
Either initialize the submodule:
  git submodule update --init --recursive
Or point MJORBIT_DIR at an existing mjorbit checkout:
  export MJORBIT_DIR=/path/to/mjorbit
EOF
  exit 1
fi

export MJORBIT_DIR="$SIM_DIR"
export PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export CMAKE_GENERATOR="${CMAKE_GENERATOR:-Ninja}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1

exec "$PIXI_BIN" run --manifest-path "$MANIFEST" -e "${MJORBIT_ENV:-rl}" --locked "$@"
