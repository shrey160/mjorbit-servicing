"""Truth audit of the ring mission harness (P7.6; as P6's checks/truth_audit.py): which lines that touch the TRUE
client state run in an episode, in which phase and from which function.

  MUJOCO_GL=egl bash tools/rlrun.sh python ../projects/p7_unprepared_client/checks/ring_truth_audit.py [--seeds 0 1]

Static part: every line of the harness sources matching TRUTH_RE (the client body / joint ids, the true ring, the
true COM, the hand-over prior). Dynamic part: sys.monitoring LINE events on those lines only, counted per (line,
phase, function). Each function is classed by hand below (ALLOWED: setup / hand-over, sensor simulation, scoring and
records); anything else is listed as UNCLASSED for review. Writes runs/ring_truth_audit.txt.
"""
from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
P7 = HERE.parent
if str(P7) not in sys.path:
    sys.path.insert(0, str(P7))
from _paths import ensure_runs_dir  # noqa: E402
import ring_mission as RM  # noqa: E402

FILES = [P7 / "mission" / "ring_mission.py", HERE / "reach_bench.py", HERE / "ring_bench.py", HERE / "ring_cam.py",
         P7 / "perception" / "ring_fit.py"]
TRUTH_RE = re.compile(r"bid_t\b|\btq\b|\btv\b|com_true|truth|prior0|cli_b|c_body|qpos\[self\.tq|qvel\[self\.tv")
ALLOWED = {
    # setup and the hand-over from the P6 chain (the client's case, the first fix's prior)
    "place_episode": "setup / hand-over", "__init__": "setup", "_approach_pose": "setup (design station)",
    "place_station": "setup (bench)",
    # sensor simulation (rendering, contact physics bookkeeping)
    "lidar_points": "sensor sim", "wrist_check": "sensor sim", "sun": "sensor sim",
    # scoring and records
    "seat": "scoring", "truth_ring": "scoring", "est_err": "scoring", "done": "record", "_contacts": "record (collision list)",
    "episode": "scoring (E0 / E1 / rate error, see lines)", "case": "bench (not in the mission)",
}
# single lines in functions that otherwise act on own data only (classified by line, so a new truth read in the same
# function still shows up)
LINE_OK = {
    ("fix", "self.prior0"): "hand-over prior (the first fix only)",
    ("reach_close", "client_v_mm_s="): "record (client push in the row)",
}
TOOL = 4
_lines = {}
_src = {}
_rec = defaultdict(int)
M = [None]


def _scan():
    for f in FILES:
        hits = set()
        for i, ln in enumerate(f.read_text().splitlines(), 1):
            code = ln.split("#", 1)[0]
            if TRUTH_RE.search(code):
                hits.add(i)
                _src[(str(f), i)] = ln.strip()
        _lines[str(f)] = hits


def _line(code, ln):
    f = code.co_filename
    if f in _lines and ln in _lines[f]:
        ph = getattr(M[0], "phase", "?") if M[0] is not None else "?"
        _rec[(f, ln, ph, code.co_name)] += 1
    else:
        return sys.monitoring.DISABLE


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[0, 1])
    ap.add_argument("--multirate", action="store_true",
                    help="SPEC_7 section 23: 10 Hz fixes, 2 Hz refit over <= 40 fixes, monitor on, the ring net")
    a = ap.parse_args()
    if a.multirate:
        import torch
        torch.set_num_threads(1)
        from ringcam_net import RingCamNet
        RM.LIDAR_DT, RM.FIT_DT[0], RM.FIT_MAX[0], RM.INNOV[0] = 0.1, 0.5, 40, "on"
        RM.USE_CAM[0], RM.NET[0] = "net", RingCamNet.load()
    _scan()
    mon = sys.monitoring
    mon.use_tool_id(TOOL, "ring_truth_audit")
    mon.register_callback(TOOL, mon.events.LINE, _line)
    mon.set_events(TOOL, mon.events.LINE)
    M[0] = RM.Mission()
    res = [RM.episode(M[0], 7600000 + s)["result"] for s in a.seeds]
    mon.set_events(TOOL, 0)
    by = defaultdict(lambda: defaultdict(int))
    for (f, ln, ph, fn), n in _rec.items():
        by[(f, ln, fn)][ph] += n
    out = [f"ring mission truth audit, seeds {a.seeds}: {res}", ""]
    uncl = []
    for (f, ln, fn), phs in sorted(by.items(), key=lambda x: (x[0][0], x[0][1])):
        cls = ALLOWED.get(fn) or next((v for (f_, s_), v in LINE_OK.items() if f_ == fn and s_ in _src[(f, ln)]),
                                      "UNCLASSED")
        row = f"{Path(f).name}:{ln:<4} {fn:<16} {cls:<34} {dict(phs)}  | {_src[(f, ln)][:90]}"
        out.append(row)
        if cls == "UNCLASSED":
            uncl.append(row)
    out += ["", f"UNCLASSED ({len(uncl)}):"] + uncl
    txt = "\n".join(out)
    print(txt)
    (ensure_runs_dir() / "ring_truth_audit.txt").write_text(txt)
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
