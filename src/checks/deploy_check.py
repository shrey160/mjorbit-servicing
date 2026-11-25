"""Does the arm get out of its launch stow? (21 Nov, user: "won't it get stuck while getting ready?")

Steps the arm from servicer_stow to servicer_ready (joint-space path, N steps; --path linear or staged: first the
shoulder pitch and elbow swing the folded arm out, then the wrist unfolds, then the rest) and records at each step
the smallest distance from the arm (and tool) to (a) the launch restraints (launch_* geoms: visual only in the sim, so
measured here), (b) the bus and the other arm parts (phys_rev2.clearance: non-adjacent collision geoms). The HDRM pins
are taken as pulled (launch_*pin* left out). Prints the minimum of each along the path and where.

  bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/deploy_check.py [--path staged]
"""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
import phys_rev2 as P  # noqa: E402

m, d = P.m, P.d
name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""  # noqa: E731
REST = [g for g in range(m.ngeom) if name(g).startswith("launch_") and "pin" not in name(g)]
ARMG = [g for g in range(m.ngeom) if m.geom_bodyid[g] in P.ARMB and m.geom_rgba[g, 3] > 0]
num = lambda n: m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, n)]:][:6].copy()  # noqa: E731


def gname(g):
    return f"{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])}/{name(g) or g}"


def rest_dist(q):
    P.pose(q)
    ft = np.zeros(6)
    best = (1.0, "", "")
    for a in ARMG:
        for b in REST:
            dd = mujoco.mj_geomDistance(m, d, a, b, 0.1, ft)
            if dd < best[0]:
                best = (dd, gname(a), name(b))
    return best


def path(q0, q1, kind, n):
    if kind == "linear":
        return [q0 + (q1 - q0) * t for t in np.linspace(0, 1, n)]
    # staged: (1) shoulder pitch + elbow swing the folded arm out (wrist kept folded), (2) wrist pitch unfolds,
    # (3) everything to ready
    qa = q0.copy()
    qa[1], qa[2] = q0[1] + 0.6, q0[2] - 0.9
    qb = qa.copy()
    qb[3] = 0.0
    out = []
    for a, b in ((q0, qa), (qa, qb), (qb, q1)):
        out += [a + (b - a) * t for t in np.linspace(0, 1, n // 3)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="staged", choices=["linear", "staged"])
    ap.add_argument("--n", type=int, default=90)
    a = ap.parse_args()
    q0, q1 = num("servicer_stow"), num("servicer_ready")
    rows = []
    for i, q in enumerate(path(q0, q1, a.path, a.n)):
        dr, ga, gb = rest_dist(q)
        dc = float(P.clearance(q))
        rows.append(dict(i=i, q=np.round(q, 3).tolist(), rest_mm=round(dr * 1e3, 1), rest_pair=f"{ga} - {gb}",
                         clear_mm=round(dc * 1e3, 1)))
    rmin = min(rows, key=lambda r: r["rest_mm"])
    cmin = min(rows[1:], key=lambda r: r["clear_mm"])
    out = dict(path=a.path, stow=q0.tolist(), ready=q1.tolist(), restraints=[name(g) for g in REST],
               min_to_restraints=rmin, min_clearance_after_start=cmin,
               first_steps=[(r["rest_mm"], r["clear_mm"]) for r in rows[:12]])
    print(json.dumps(out, indent=1))
    (ensure_runs_dir() / f"deploy_{a.path}.json").write_text(json.dumps(dict(summary=out, rows=rows), indent=1))


if __name__ == "__main__":
    main()
