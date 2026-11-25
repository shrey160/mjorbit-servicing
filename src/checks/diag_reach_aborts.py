"""R0 diagnosis (SPEC_7 section 21): why the ring mission's reach aborts (tune seeds 1, 2, 5, 6 with --cam net).
Per episode, during the reach phases: every lidar fix (time, kind, how far it moved the tracked grasp point), the
arm tracking error |grip site - commanded section point| after every descent step, the first off-tool contacts and
the wrist-force trip. Truth is read for the record only.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/diag_reach_aborts.py --seeds 2
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from _paths import ensure_runs_dir  # noqa: E402
import ring_mission as RM  # noqa: E402
import torch  # noqa: E402

torch.set_num_threads(1)
REACH = ("to pre-close", "descent", "close")


def run(seed):
    M = RM.Mission()
    if RM.NET[0] is None:
        from ringcam_net import RingCamNet
        RM.NET[0] = RingCamNet.load()
    RM.USE_CAM[0] = "net"
    log = dict(fixes=[], trk=[], trips=[])
    last_p = [None]

    f0 = RM.Mission.fix
    def fix(self):
        g0 = self.track(*self._diag_E) if getattr(self, "Eb", None) is not None and hasattr(self, "_diag_E") else None
        k = f0(self)
        if self.phase in REACH and g0 is not None:
            g1 = self.track(*self._diag_E)
            log["fixes"].append(dict(t=round(self.kd.time, 2), phase=self.phase, kind=k,
                                     jump_mm=round(float(np.linalg.norm(g1[0][0] - g0[0][0])) * 1e3, 2),
                                     tilt_jump_deg=round(float(np.degrees(np.arccos(np.clip(g1[0][1] @ g0[0][1], -1, 1)))), 3)))
        return k
    M.fix = fix.__get__(M)

    t0 = RM.Mission.track
    def track(self, E, rad):
        self._diag_E = (E, rad)
        return t0(self, E, rad)
    M.track = track.__get__(M)

    s0 = M.section
    def section(est, rad, h):
        r = s0(est, rad, h)
        last_p[0] = r[0]
        return r
    M.section = section

    st0 = RM.Mission.step
    import mujoco
    from tool_gate import sens
    def snap(self, tag):
        km, kd = self.km, self.kd
        cs = []
        for i in range(kd.ncon):
            c = kd.contact[i]
            b1, b2 = km.geom_bodyid[c.geom1], km.geom_bodyid[c.geom2]
            if (b1 in self.srv_b and b2 in self.cli_b) or (b2 in self.srv_b and b1 in self.cli_b):
                f6 = np.zeros(6)
                mujoco.mj_contactForce(km, kd, i, f6)
                gn = lambda g: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or f"#{g}"  # noqa: E731
                cs.append((gn(c.geom1), gn(c.geom2), round(float(f6[0]), 2), round(float(c.dist) * 1e3, 2)))
        g = kd.site_xpos[self.I["grip"]]
        log.setdefault("snaps", []).append(dict(
            tag=tag, t=round(kd.time, 3), contacts=cs, wrist_N=np.round(sens(km, kd, "wrist_force"), 2).tolist(),
            guard=None if self.guard is None else np.round(self.guard, 2).tolist(),
            cmd_err_mm=None if last_p[0] is None else round(float(np.linalg.norm(g - last_p[0])) * 1e3, 2),
            arm_qvel=round(float(np.linalg.norm(kd.qvel[self.I["arm_v"]])), 4)))
    def step(self, q, g, n=1):
        ph0 = self.phase
        st0(self, q, g, n)
        if self.phase == "descent" and not any(s["tag"] == "descent0" for s in log.get("snaps", [])):
            snap(self, "descent0")
        if ph0 == "to pre-close" and self.trip is not None and not any(s["tag"] == "preclose_trip" for s in log.get("snaps", [])):
            snap(self, "preclose_trip")
        if self.phase == "descent" and last_p[0] is not None:
            trk = float(np.linalg.norm(self.kd.site_xpos[self.I["grip"]] - last_p[0])) * 1e3
            log["trk"].append((round(self.kd.time, 3), round(trk, 2)))
            log["seat_end"] = self.seat()            # truth, record only: where the grip site is on the true ring
        if self.trip is not None and (not log["trips"] or log["trips"][-1][1] != self.trip):
            log["trips"].append((round(self.kd.time, 3), self.trip, self.phase))
    M.step = step.__get__(M)

    ag0 = RM.Mission.arm_guard
    def arm_guard(self, on):
        if self.phase == "descent" and on:
            snap(self, "pre_descent")
        ag0(self, on)
    M.arm_guard = arm_guard.__get__(M)

    r = RM.episode(M, 7600000 + seed)
    trk = log["trk"]
    big = [x for x in trk if x[1] > 5.0]
    out = dict(seed=seed, result=r["result"], fixes=log["fixes"], trips=log["trips"][:3],
               trk_max=max(trk, key=lambda x: x[1]) if trk else None, trk_n=len(trk),
               trk_first_over5=big[0] if big else None, trk_tail=trk[-6:],
               bad=r.get("contacts_bad"), descent=r.get("descent"), seat_pre=r.get("seat_pre"),
               E1=r.get("E1"), spin=r.get("spin_deg_s"), snaps=log.get("snaps"), seat_end=log.get("seat_end"))
    print(json.dumps(out, default=float), flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[2])
    a = ap.parse_args()
    rows = [run(s) for s in a.seeds]
    (ensure_runs_dir() / "diag_reach_aborts.json").write_text(json.dumps(rows, indent=1, default=float))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
