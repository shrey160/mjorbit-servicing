"""Build script for servicer_2_draft.pdf (single-column research preprint).

Focuses cleanly on the Dual-Arm 15" ESPA Servicer baseline, reference COTS/custom
hardware, FreeCAD capture poses, MuJoCo verification, and the Ultra-Shallow
Hosted-Payload In-Orbit Demonstration (IOD) concept.
"""
import shutil
import subprocess
from pathlib import Path

from PIL import Image

HERE = Path(__file__).resolve().parent
DOCS_DIR = HERE.parent
MEDIA_V4 = DOCS_DIR / "media" / "servicer_2_draft"
MEDIA_IOD = DOCS_DIR / "media" / "arm_iod_demo"
MEDIA_THERM = DOCS_DIR / "media" / "thermal_audit"
FIG_DIR = HERE / "figures"
HTML_PATH = HERE / "servicer_2_draft.html"
PDF_PATH = HERE / "servicer_2_draft.pdf"


def crop_and_pad(
    src_path: Path,
    dst_path: Path,
    target_w: int = 1200,
    target_h: int = 800,
    tol: int = 10,
    pad_frac: float = 0.03,
) -> None:
    """Auto-crop uniform background padding and center cleanly on a 3:2 canvas."""
    im = Image.open(src_path).convert("RGB")
    w, h = im.size
    bg = im.getpixel((0, 0))
    px = im.load()

    x0, y0, x1, y1 = w, h, 0, 0
    for y in range(h):
        row_has_fg = False
        for x in range(w):
            r, g, b = px[x, y]
            if abs(r - bg[0]) > tol or abs(g - bg[1]) > tol or abs(b - bg[2]) > tol:
                row_has_fg = True
                if x < x0:
                    x0 = x
                if x > x1:
                    x1 = x
        if row_has_fg:
            if y < y0:
                y0 = y
            if y > y1:
                y1 = y

    if x1 <= x0 or y1 <= y0:
        cropped = im
    else:
        cw, ch = x1 - x0 + 1, y1 - y0 + 1
        pad_x = max(8, int(cw * pad_frac))
        pad_y = max(8, int(ch * pad_frac))
        cropped = im.crop((
            max(0, x0 - pad_x),
            max(0, y0 - pad_y),
            min(w, x1 + 1 + pad_x),
            min(h, y1 + 1 + pad_y),
        ))

    cw, ch = cropped.size
    scale = min((target_w * 0.94) / cw, (target_h * 0.94) / ch)
    nw, nh = max(1, int(round(cw * scale))), max(1, int(round(ch * scale)))
    resized = cropped.resize((nw, nh), Image.Resampling.LANCZOS)

    canvas = Image.new("RGB", (target_w, target_h), bg)
    canvas.paste(resized, ((target_w - nw) // 2, (target_h - nh) // 2))
    canvas.save(dst_path, format="PNG", optimize=True)


def prepare_figures() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    v4_files = [
        "01_stowed_iso_front.png",
        "02_stowed_iso_rear.png",
        "03_stowed_front_face.png",
        "04_stowed_side_elevation.png",
        "scale_servicer_front.png",
        "scale_servicer_side.png",
        "05_deployed_24in_geo_elevation.png",
        "06_deployed_24in_geo_iso.png",
        "07_deployed_15in_elevation.png",
        "08_deployed_8in_elevation.png",
        "09_deployed_37in_geo_elevation.png",
        "10_deployed_24in_large_offset_iso.png",
        "11_closeup_arm_chain_deployed.png",
        "12_closeup_wrist_gripper_locked.png",
        "13_closeup_gripper_open.png",
        "15_deployed_24in_open_approach_elevation.png",
        "dual_arm_mj_capture.png",
        "dual_arm_mj_closeup.png",
    ]
    for fname in v4_files:
        src = MEDIA_V4 / fname
        assert src.exists(), f"Missing asset: {src}"
        crop_and_pad(src, FIG_DIR / fname, tol=8 if "mj_" in fname else 10, pad_frac=0.03)

    iod_files = [
        "01_stowed_iso_front.png",
        "02_stowed_side_profile.png",
        "03_rear_bay_cutaway.png",
        "04_deployed_nominal_elevation.png",
        "05_deployed_nominal_iso.png",
        "06_deployed_tilted_5deg_pitch.png",
        "scale_iod_front.png",
        "scale_iod_side.png",
    ]
    for fname in iod_files:
        src = MEDIA_IOD / fname
        assert src.exists(), f"Missing IOD asset: {src}"
        crop_and_pad(src, FIG_DIR / f"iod_{fname}", tol=10, pad_frac=0.03)

    shutil.copy2(MEDIA_THERM / "orbital_transient_profiles.png", FIG_DIR / "therm_orbital_profiles.png")
    therm_crops = [
        ("cad_v4_internals_cutaway.png", "therm_cad_v4_cutaway.png", 10),
        ("cad_iod_internals_cutaway.png", "therm_cad_iod_cutaway.png", 10),
        ("fem_v4_full_rpo_hot.png", "therm_fem_v4_full_rpo_hot.png", 8),
        ("fem_v4_full_geo_cold.png", "therm_fem_v4_full_geo_cold.png", 8),
        ("fem_iod_full_isolated_hot.png", "therm_fem_iod_full_iso_hot.png", 8),
        ("fem_arm_hot_capture.png", "therm_fem_arm_hot_capture.png", 8),
    ]
    for src_name, dst_name, tol in therm_crops:
        src = MEDIA_THERM / src_name
        assert src.exists(), f"Missing thermal asset: {src}"
        crop_and_pad(src, FIG_DIR / dst_name, target_w=1320, target_h=880, tol=tol, pad_frac=0.03)


HTML_DOC = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<title>Dual Arm Servicer Draft and Hosted Payload Arm Demo</title>
<style>
  @page {
    size: A4;
    margin: 15mm 16mm 17mm 16mm;
    @top-left {
      content: "TECHNICAL DESIGN DRAFT";
      font-family: "Segoe UI", Arial, sans-serif;
      font-size: 7.4pt;
      letter-spacing: 0.06em;
      color: #64748b;
    }
    @top-right {
      content: "DUAL ARM SERVICER & HOSTED PAYLOAD DEMO";
      font-family: "Segoe UI", Arial, sans-serif;
      font-size: 7.4pt;
      letter-spacing: 0.05em;
      color: #64748b;
    }
    @bottom-center {
      content: "Page " counter(page) " of " counter(pages);
      font-family: "Segoe UI", Arial, sans-serif;
      font-size: 7.8pt;
      color: #475569;
    }
  }
  @page :first {
    @top-left { content: none; }
    @top-right { content: none; }
  }

  * { box-sizing: border-box; }
  body {
    font-family: Georgia, "Times New Roman", serif;
    font-size: 9.4pt;
    line-height: 1.44;
    color: #1e293b;
    margin: 0;
    padding: 0;
  }

  /* Header / Preprint Title Block */
  .doc-tag {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 8pt;
    font-weight: 700;
    letter-spacing: 0.10em;
    text-transform: uppercase;
    color: #1b365d;
    border-bottom: 2px solid #1b365d;
    padding-bottom: 4px;
    margin-bottom: 9px;
  }
  .doc-title {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 18pt;
    font-weight: 700;
    line-height: 1.2;
    color: #0f172a;
    margin: 0 0 5px 0;
  }
  .doc-subtitle {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 9.8pt;
    color: #475569;
    margin: 0 0 7px 0;
  }
  .doc-meta {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 8pt;
    color: #64748b;
    margin-bottom: 10px;
  }

  .abstract-box {
    background: #f4f7fb;
    border-left: 3.5px solid #1b365d;
    padding: 9px 12px;
    margin: 0 0 11px 0;
    border-radius: 0 4px 4px 0;
  }
  .abstract-label {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 8.2pt;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: #1b365d;
    margin-bottom: 3px;
  }
  .abstract-box p {
    margin: 0 0 4px 0;
    font-size: 8.9pt;
    line-height: 1.4;
  }
  .keywords {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 7.9pt;
    color: #334155;
    margin-top: 3px;
  }

  /* Headings */
  h1, h2 {
    font-family: "Segoe UI", Arial, sans-serif;
    color: #1b365d;
    break-after: avoid;
    page-break-after: avoid;
  }
  h1 {
    font-size: 12.2pt;
    font-weight: 700;
    border-bottom: 1.2px solid #cbd5e1;
    padding-bottom: 3px;
    margin: 0 0 6px 0;
    break-before: page;
    page-break-before: always;
  }
  .front-teaser {
    margin: 14px 0 6px 0;
    break-inside: avoid;
    page-break-inside: avoid;
  }
  h2 {
    font-size: 10pt;
    font-weight: 700;
    color: #1e3a5f;
    margin: 8px 0 3.5px 0;
  }
  p {
    margin: 0 0 5.5px 0;
    text-align: justify;
  }
  ul, ol {
    margin: 3px 0 6px 0;
    padding-left: 18px;
  }
  li {
    margin-bottom: 2.5px;
    text-align: justify;
  }

  .callout {
    background: #f8fafc;
    border: 1px solid #cbd5e1;
    border-left: 3.5px solid #2b6cb0;
    padding: 8px 11px;
    margin: 8px 0 4px 0;
    border-radius: 3px;
    font-size: 8.8pt;
    line-height: 1.4;
    break-inside: avoid;
    page-break-inside: avoid;
  }
  .callout strong { color: #1b365d; }

  /* Booktabs-style tables */
  .table-block {
    margin: 7px 0 9px 0;
    break-inside: avoid;
    page-break-inside: avoid;
  }
  .table-caption {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 8.2pt;
    color: #334155;
    margin-bottom: 3.5px;
    line-height: 1.3;
  }
  .table-caption strong { color: #0f172a; }
  table {
    width: 100%;
    border-collapse: collapse;
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 7.7pt;
    line-height: 1.28;
  }
  thead th {
    background: #f1f5f9;
    color: #0f172a;
    font-weight: 700;
    text-align: left;
    padding: 4px 5.5px;
    border-top: 1.6px solid #1b365d;
    border-bottom: 1.1px solid #475569;
  }
  tbody td {
    padding: 3.5px 5.5px;
    border-bottom: 0.5px solid #e2e8f0;
    vertical-align: top;
  }
  tbody tr:last-child td {
    border-bottom: 1.5px solid #1b365d;
  }
  .badge-cots, .badge-custom, .badge-hybrid {
    display: inline-block;
    font-size: 6.9pt;
    font-weight: 700;
    padding: 1px 4px;
    border-radius: 2px;
    white-space: nowrap;
  }
  .badge-cots   { color: #065f46; background: #d1fae5; }
  .badge-custom { color: #92400e; background: #fef3c7; }
  .badge-hybrid { color: #1e40af; background: #dbeafe; }
  .pass { font-weight: 700; color: #047857; }
  .num  { text-align: right; white-space: nowrap; }

  /* Figures */
  .figure-block {
    margin: 7px 0 9px 0;
    break-inside: avoid;
    page-break-inside: avoid;
  }
  .fig-grid-2x2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 5px;
  }
  .fig-grid-1x2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
  }
  .fig-grid-1x3 {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 5px;
  }
  .panel {
    border: 1px solid #cbd5e1;
    border-radius: 3px;
    overflow: hidden;
    background: #ffffff;
  }
  .panel img {
    display: block;
    width: 100%;
    height: auto;
  }
  .panel-label {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 7.2pt;
    color: #334155;
    background: #f8fafc;
    border-top: 1px solid #e2e8f0;
    padding: 2.5px 4px;
    text-align: center;
    white-space: nowrap;
  }
  .fig-caption {
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 8.1pt;
    color: #334155;
    margin-top: 4.5px;
    line-height: 1.34;
    text-align: justify;
  }
  .fig-caption strong { color: #0f172a; }
</style>
</head>
<body>

<div class="doc-tag">Technical Design Draft</div>
<div class="doc-title">Dual Arm Servicer Draft and Hosted Payload Arm Demo</div>
<div class="doc-subtitle">System Specifications, Reference Parts Matrix, CAD Capture Poses, MuJoCo Verification, Hosted-Payload IOD Concept &amp; Preliminary Thermal Audit</div>
<div class="doc-meta">Working Baseline Draft &middot; November 2026 &middot; Subject to Iterative Design Updates</div>

<div class="abstract-box">
  <div class="abstract-label">Abstract</div>
  <p>This draft outlines the current dual-arm 15&Prime; ESPA orbital servicer baseline together with an initial hosted-payload flight demonstration concept and a preliminary thermal audit. The full servicer folds two compact 6-DOF manipulators with intersecting 3-DOF Cardan-Roll wrists, 6-axis force/torque sensors, wrist stereo cameras, and &plusmn;18&nbsp;mm form-closure Marman grippers flush inside recessed centerline bays (704.3 &times; 606.0 &times; 937.4&nbsp;mm stowed, 188.5&nbsp;kg with subsystem margins, CG&nbsp;=&nbsp;357.7&nbsp;mm). Analytical sizing and closed-loop MuJoCo simulations show clean capture across 8&Prime;&ndash;37&Prime; payload adapter rings at 480&ndash;500&nbsp;mm standoff, clearance past 350&nbsp;mm GEO apogee thruster bells, tolerance to 5&deg;&nbsp;+&nbsp;5&deg; 3D target tilt, and plume-free reaction-wheel detumble at 2&nbsp;&deg;/s. To validate the arms in orbit before committing a full free-flying bus, the second half of this document defines a 273&nbsp;mm-deep hosted-payload testbed (~45.6&nbsp;kg) that deploys a 15&Prime; target ring plate on four rollable CFRP booms, followed by multi-node orbital transient and coupled 3D thermomechanical FEM thermal verification across LEO, GEO, and hosted-payload boundary modes.</p>
  <div class="keywords"><strong>Keywords:</strong> Orbital Servicing &middot; Dual-Arm Manipulator &middot; 15&Prime; ESPA &middot; Cardan-Roll Wrist &middot; Marman Gripper &middot; Rollable CFRP Booms &middot; Hosted Payload &middot; Preliminary Thermal Audit</div>
</div>

<div class="front-teaser">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/06_deployed_24in_geo_iso.png" alt="Dual-arm 15 inch ESPA servicer preview" />
      <div class="panel-label">Dual-Arm 15&Prime; ESPA Servicer (24&Prime; GEO ring capture)</div>
    </div>
    <div class="panel">
      <img src="figures/iod_05_deployed_nominal_iso.png" alt="Hosted payload arm demo preview" />
      <div class="panel-label">Hosted-Payload Arm Demo (15&Prime; boom-deployed IOD)</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Concept Preview.</strong> Orthographic previews of the two spacecraft baselines examined in this draft: the full 15&Prime; ESPA dual-arm servicer capturing a 24&Prime; GEO client ring past a central apogee thruster bell (left), and the 273&nbsp;mm-deep hosted-payload in-orbit demonstration testbed exercising identical 6-DOF Cardan-Roll arm hardware on a rollable CFRP-boom target plate (right). Detailed specifications, CAD poses, MuJoCo physics checks, and the preliminary thermal audit follow in Sections&nbsp;1&ndash;5.</div>
</div>

<!-- ====================================================================== -->
<h1>1. Servicer Overview, Specifications &amp; Reference Hardware</h1>

<p>The servicer is sized directly around standard 15&Prime; ESPA secondary payload limits while packing full two-arm redundancy, rendezvous sensing, attitude control, and green propulsion inside a symmetric 640 &times; 600 &times; 910&nbsp;mm bus on a 42.2&nbsp;mm Rocket Lab MLB15.000 separation ring (total mass 161.7&nbsp;kg base, 188.5&nbsp;kg with subsystem margins, CG at 357.7&nbsp;mm &le; 508.0&nbsp;mm). Table&nbsp;1 summarizes the key specifications against the ESPA envelope.</p>

<div class="table-block">
  <div class="table-caption"><strong>Table 1.</strong> Dual-arm servicer specifications compared against standard 15&Prime; ESPA rideshare limits.</div>
  <table>
    <thead>
      <tr>
        <th style="width:24%">Subsystem / Metric</th>
        <th style="width:24%">15&Prime; ESPA Requirement</th>
        <th style="width:52%">Current Dual-Arm Servicer Specification</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Stowed Envelope</strong></td>
        <td>711 &times; 610 &times; 965&nbsp;mm</td>
        <td><strong>704.3 &times; 606.0 &times; 937.4&nbsp;mm</strong> (+6.7 X, +4.0 Y, +27.6 Z&nbsp;mm margin)</td>
      </tr>
      <tr>
        <td><strong>Mass &amp; Center of Gravity</strong></td>
        <td>&le; 200 / 220&nbsp;kg, CG &le; 508&nbsp;mm</td>
        <td><strong>161.7&nbsp;kg base / 188.5&nbsp;kg w/ MGA</strong> (193.1&nbsp;kg @ flat 20%), <strong>CG = 357.7&nbsp;mm</strong></td>
      </tr>
      <tr>
        <td><strong>Manipulator Architecture</strong></td>
        <td>Flush stowage, 15&nbsp;g launch load</td>
        <td>2&times; 6-DOF recessed arms (248 + 208&nbsp;mm links + 128&nbsp;mm wrist/palm), 2&times; NEA&nbsp;9102 locks (10.8&times; FoS)</td>
      </tr>
      <tr>
        <td><strong>Wrist, F/T &amp; Gripper</strong></td>
        <td>Zero-backlash capture &amp; hold</td>
        <td>Intersecting 3-DOF Cardan-Roll wrist + ATI Mini45-Ti F/T + &plusmn;18&nbsp;mm stroke Marman jaws</td>
      </tr>
      <tr>
        <td><strong>Supported Client Rings</strong></td>
        <td>Standard spacecraft adapters</td>
        <td><strong>8&Prime;, 15&Prime;, 24&Prime;, and 37&Prime; rings</strong> @ 450&ndash;500&nbsp;mm standoff (+100&ndash;150&nbsp;mm past 350&nbsp;mm LAE nozzle)</td>
      </tr>
      <tr>
        <td><strong>RPO Sensors</strong></td>
        <td>Unprepared client tracking</td>
        <td>Recessed +Y ASC DragonEye LiDAR &amp; IRIS cam, 2&times; deck ST-16RT2, 2&times; 50&nbsp;mm wrist stereo bars</td>
      </tr>
      <tr>
        <td><strong>Propulsion &amp; Wheels</strong></td>
        <td>Fault-tolerant &amp; plume-safe</td>
        <td>16 &times; 1&nbsp;N HPGP (55&deg; fwd / 30&deg; aft, 6/6 rank on any 1-fail) + 4 &times; RW-12 (&beta; = 65&deg;, 2&nbsp;&deg;/s wheel detumble)</td>
      </tr>
      <tr>
        <td><strong>Earth 1g Ground Use</strong></td>
        <td>Bench checkout without offload</td>
        <td>Continuous 1g operation (~26.6&nbsp;N&middot;m shoulder, ~11.9&nbsp;N&middot;m elbow self-weight; ~1&ndash;2&nbsp;kg cont / ~4&ndash;6&nbsp;kg peak/arm)</td>
      </tr>
    </tbody>
  </table>
</div>

<h2>1.1 Key Mechanical, Sensor &amp; Bus Features</h2>
<ul>
  <li><strong>Hollow-Shaft 6-DOF Arms:</strong> Each arm begins at a bulkhead-mounted Base-Roll drive (q0) inside 135&nbsp;mm front cavities flanking the centerline (Y = &plusmn;70&nbsp;mm, Z = &plusmn;245&nbsp;mm), preventing ring-center kinematic singularities. Double-supported straddle clevises carry Harmonic Drive CSD-20-2A (shoulder, 28/57&nbsp;N&middot;m) and CSD-17-2A (elbow, 16/37&nbsp;N&middot;m) gearsets paired with Kollmorgen TBM2G frameless motors, routing wiring cleanly through internal bores. Distal forearms terminate in a compact intersecting 3-DOF Cardan-Roll wrist (three CSD-14-2A + TBM2G-05008 drives with power-off holding brakes) bolted to an ATI Mini45 Titanium F/T wafer and &plusmn;18&nbsp;mm radial-stroke Marman jaws.</li>
  <li><strong>Recessed External Sensors:</strong> Far-range rendezvous optics (ASC DragonEye flash LiDAR and 3D PLUS IRIS camera) are tucked into the +Y cheek ahead of the intact 20&nbsp;mm front shear bulkhead, protruding only 12&nbsp;mm proud. Two ST-16RT2 star trackers sit recessed in the +Z deck tilted &plusmn;20&deg; outward for 70&deg; solar exclusion clearance. Close-range capture relies on 50&nbsp;mm global-shutter stereo bars + LED lamps mounted on the -Y cheek of each gripper opposite the wrist pitch drum.</li>
  <li><strong>Reference Hardware &mdash; Procure vs. Build:</strong> Table&nbsp;2 separates off-the-shelf (COTS) procurements from parts intended to be CNC-machined or built in-house, alongside practical alternatives.</li>
</ul>

<div class="table-block">
  <div class="table-caption"><strong>Table 2.</strong> Reference parts list showing COTS purchases versus custom-machinable parts and candidate alternatives.</div>
  <table>
    <thead>
      <tr>
        <th style="width:18%">Component Group</th>
        <th style="width:14%">Source Path</th>
        <th style="width:35%">Baseline Reference Hardware</th>
        <th style="width:33%">COTS / Build Alternatives</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Shoulder &amp; Elbow</strong></td>
        <td><span class="badge-hybrid">COTS + CNC</span></td>
        <td>CSD-20-2A / CSD-17-2A gearsets + TBM2G-06013 / 06008 motors + CNC Ti/Al straddle clevises</td>
        <td>Harmonic Drive SHF-20/17-2UH housed units (+1.5&nbsp;kg); Maxon EC Frameless DT 60</td>
      </tr>
      <tr>
        <td><strong>3-DOF Cardan Wrist</strong></td>
        <td><span class="badge-hybrid">COTS + CNC</span></td>
        <td>3&times; CSD-14-2A + TBM2G-05008 + power-off brakes + CNC Cardan spider &amp; housings</td>
        <td>Harmonic Drive SHF-14-2UH (heavier); Maxon EC Frameless DT 50</td>
      </tr>
      <tr>
        <td><strong>Wrist F/T Sensor</strong></td>
        <td><span class="badge-cots">COTS</span></td>
        <td>ATI Mini45 Titanium 6-axis wafer (&Oslash;45 &times; 15.7&nbsp;mm, &plusmn;1,160&nbsp;N axial, &plusmn;20&nbsp;N&middot;m)</td>
        <td>ATI Mini40; Bota SensONE; custom strain-gauged Ti-6Al-4V spoked flexure</td>
      </tr>
      <tr>
        <td><strong>Marman Gripper</strong></td>
        <td><span class="badge-custom">Custom Build</span></td>
        <td>CNC Ti/7075 palm + twin linear rails, trapezoidal lead screws, 20&deg; undercut hook &amp; inboard finger</td>
        <td>Commercial miniature linear guides (THK/IKO, Braycote relubed) + CNC jaws</td>
      </tr>
      <tr>
        <td><strong>Wrist Stereo Bar</strong></td>
        <td><span class="badge-hybrid">COTS Sensors</span></td>
        <td>2&times; OV9281 or PYTHON 300 global-shutter sensors + narrow-band LED on 50&nbsp;mm custom PCB</td>
        <td>Basler dart / IDS board cameras + bandpass filter &amp; compact optics</td>
      </tr>
      <tr>
        <td><strong>Links &amp; HDRMs</strong></td>
        <td><span class="badge-hybrid">COTS + Tube</span></td>
        <td>Roll-wrapped CFRP tubes + bonded Ti sockets; 2&times; EBAD NEA 9102 non-pyro nuts (17.8&nbsp;kN)</td>
        <td>EBAD Frangibolt FD04 (2&ndash;3&times;/arm) or TiNi Pin Puller 2545; Al tubes for lab EM</td>
      </tr>
      <tr>
        <td><strong>Bus Avionics &amp; ACS</strong></td>
        <td><span class="badge-cots">COTS</span></td>
        <td>DragonEye, 3D PLUS IRIS, 2&times; ST-16RT2, 4&times; RW-12, 16&times; ECAPS 1N HPGP, MLB15.000</td>
        <td>Redwire Argus, Sodern Auriga, Bradford W18, Nammo 1N, Beyond Gravity PAS&nbsp;381S</td>
      </tr>
    </tbody>
  </table>
</div>

<!-- ====================================================================== -->
<h1>2. Servicer CAD Showcase &amp; Multi-Target Capture Poses</h1>

<p>All geometry was constructed as valid solid B-Rep parts in FreeCAD and verified to <code>0.00&nbsp;mm&sup3;</code> solid overlap across every stowed and deployed configuration. Figure&nbsp;1 shows the exterior stowed vehicle inside the 15&Prime; ESPA volume; Figure&nbsp;2 shows true-to-scale front and side comparisons alongside a 175&nbsp;cm human and an 18&nbsp;cm banana; Figures&nbsp;3 and&nbsp;4 show multi-target ring capture across <code>8&Prime;&ndash;37&Prime;</code> adapters (minimum <code>+21.1&nbsp;mm</code> reach margin, <code>+100&ndash;150&nbsp;mm</code> GEO nozzle clearance, and asymmetric lateral offset compensation); and Figure&nbsp;5 details the articulated 6-DOF linkage and <code>&plusmn;18&nbsp;mm</code> open-stroke Marman gripper.</p>

<div class="figure-block">
  <div class="fig-grid-2x2">
    <div class="panel">
      <img src="figures/01_stowed_iso_front.png" alt="Stowed 3/4 front isometric" />
      <div class="panel-label">(a) Stowed 3/4 front isometric (recessed bays + +Y optics)</div>
    </div>
    <div class="panel">
      <img src="figures/02_stowed_iso_rear.png" alt="Stowed 3/4 rear isometric" />
      <div class="panel-label">(b) Aft 3/4 isometric (MLB15.000 port + recessed star trackers)</div>
    </div>
    <div class="panel">
      <img src="figures/03_stowed_front_face.png" alt="Stowed front face" />
      <div class="panel-label">(c) Forward +X face (Y = &plusmn;70&nbsp;mm bays + R115&nbsp;mm arc reliefs)</div>
    </div>
    <div class="panel">
      <img src="figures/04_stowed_side_elevation.png" alt="Stowed side elevation" />
      <div class="panel-label">(d) Side elevation (704.3&nbsp;mm depth &le; 711&nbsp;mm ESPA limit)</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 1. Stowed external views of the dual-arm servicer.</strong> Both arms stow flush inside the 135&nbsp;mm front cavities, forward 55&deg; RCS pods sink inside corner chamfers, and rendezvous optics recess cleanly into the +Y side wall.</div>
</div>

<div class="figure-block">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/scale_servicer_front.png" alt="Dual arm servicer front view with human and banana for scale" />
      <div class="panel-label">(a) Front view: 18&nbsp;cm banana | servicer (60.6&times;93.7&nbsp;cm) | 175&nbsp;cm human</div>
    </div>
    <div class="panel">
      <img src="figures/scale_servicer_side.png" alt="Dual arm servicer side view with human and banana for scale" />
      <div class="panel-label">(b) Side view: 18&nbsp;cm banana | servicer (70.4&times;93.7&nbsp;cm) | 175&nbsp;cm human</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 2. True-to-scale reference (Dual-Arm Servicer).</strong> Orthographic front and side elevations placing an ~18&nbsp;cm banana (left) and a 175&nbsp;cm human reference (right) alongside the stowed 15&Prime; ESPA servicer.</div>
</div>

<div class="figure-block">
  <div class="fig-grid-2x2">
    <div class="panel">
      <img src="figures/08_deployed_8in_elevation.png" alt="8 inch ring capture" />
      <div class="panel-label">(a) 8&Prime; (203&nbsp;mm) ring @ +480&nbsp;mm standoff (+21.1&nbsp;mm margin)</div>
    </div>
    <div class="panel">
      <img src="figures/07_deployed_15in_elevation.png" alt="15 inch ring capture" />
      <div class="panel-label">(b) 15&Prime; (381&nbsp;mm) ring @ +480&nbsp;mm standoff (+44.0&nbsp;mm margin)</div>
    </div>
    <div class="panel">
      <img src="figures/05_deployed_24in_geo_elevation.png" alt="24 inch GEO capture" />
      <div class="panel-label">(c) 24&Prime; (610&nbsp;mm) + 350&nbsp;mm GEO nozzle (+130&nbsp;mm gap)</div>
    </div>
    <div class="panel">
      <img src="figures/09_deployed_37in_geo_elevation.png" alt="37 inch GEO capture" />
      <div class="panel-label">(d) 37&Prime; (940&nbsp;mm) + 350&nbsp;mm GEO nozzle @ +450&nbsp;mm (+100&nbsp;mm gap)</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 3. Multi-target capture poses (+X upward).</strong> Dual opposed wrists grip every standard adapter diameter cleanly outside protruding central GEO thruster cones with 0.0&deg; roll alignment error and 0.00&nbsp;mm&sup3; FreeCAD interference.</div>
</div>

<div class="figure-block">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/06_deployed_24in_geo_iso.png" alt="24 inch GEO 3/4 iso" />
      <div class="panel-label">(a) 24&Prime; GEO capture 3/4 view (nozzle clearance + dual grasp)</div>
    </div>
    <div class="panel">
      <img src="figures/10_deployed_24in_large_offset_iso.png" alt="24 inch large offset capture" />
      <div class="panel-label">(b) Asymmetric off-center capture (+80&nbsp;Y, &minus;50&nbsp;Z&nbsp;mm offset)</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 4. Isometric and off-axis dual-arm capture.</strong> Independent bulkhead q0 base-roll actuators (here +1.9&deg; / &minus;37.6&deg;) absorb large lateral ring offsets without expending attitude propellant.</div>
</div>

<div class="figure-block">
  <div class="fig-grid-2x2">
    <div class="panel">
      <img src="figures/11_closeup_arm_chain_deployed.png" alt="Full 6-DOF arm chain closeup" />
      <div class="panel-label">(a) Complete 6-DOF arm chain, Cardan wrist &amp; F/T wafer</div>
    </div>
    <div class="panel">
      <img src="figures/15_deployed_24in_open_approach_elevation.png" alt="Open approach 24 inch" />
      <div class="panel-label">(b) Open-jaw approach 22&nbsp;mm ahead of ring (&plusmn;18&nbsp;mm open)</div>
    </div>
    <div class="panel">
      <img src="figures/13_closeup_gripper_open.png" alt="Gripper open closeup" />
      <div class="panel-label">(c) Open-jaw close-up (retracted finger &amp; 20&deg; wedge hook)</div>
    </div>
    <div class="panel">
      <img src="figures/12_closeup_wrist_gripper_locked.png" alt="Gripper locked closeup" />
      <div class="panel-label">(d) Locked Marman clamp + Mini45 F/T + wrist stereo bar</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 5. Arm hardware and Marman gripper operation.</strong> Twin purely radial lead-screw carriages open &plusmn;18&nbsp;mm during final approach to clear the 15&nbsp;mm Marman ring flange cleanly, then clamp inward for positive mechanical form closure.</div>
</div>

<!-- ====================================================================== -->
<h1>3. Multibody Physics Verification (MuJoCo)</h1>

<p>To verify contact stability beyond static CAD, the complete 191.10&nbsp;kg servicer and a 300.10&nbsp;kg free-floating client (144-segment 24&Prime; Marman ring) were simulated in MuJoCo (dt = 2&nbsp;ms, &mu; = 0.4) with full RCS thruster, reaction-wheel, and lead-screw constraints. On asynchronous contact, the wrists apply 0.8&nbsp;N active F/T admittance feather touch before locking form-closure at 1.0&nbsp;N axial preload and engaging holding brakes for rigid stack detumble. All five benchmark checks passed with zero bad collisions and zero braking plume impingement (&ge; 30&deg; keep-out).</p>

<div class="table-block">
  <div class="table-caption"><strong>Table 3.</strong> Closed-loop MuJoCo physics verification summary across static parity, blind tilt, spin sync, and high-load holds.</div>
  <table>
    <thead>
      <tr>
        <th style="width:18%">Benchmark</th>
        <th style="width:27%">Test Scenario</th>
        <th style="width:25%">Measured Contact &amp; Wrist Action</th>
        <th style="width:20%">Detumble / RW / Prop</th>
        <th class="num" style="width:10%">Bad / Status</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>A. Static Parity</strong></td>
        <td>191.10&nbsp;kg, 5&deg;p+5&deg;y IK, all 16 1-fail RCS</td>
        <td>Max FK/IK err: (0.0000&nbsp;mm, 0.0000&deg;)</td>
        <td>1-fail min &minus;Fx = 1.721&nbsp;N (&gt;1.5&nbsp;N)</td>
        <td class="num pass">0 / PASS</td>
      </tr>
      <tr>
        <td><strong>B. Blind Async</strong></td>
        <td>Blind 2&deg; tilt + 15&nbsp;mm Y off + 3&nbsp;N&middot;m hold</td>
        <td>Touch 4.65&nbsp;s / 6.85&nbsp;s; A1/A2 palm 224/321&nbsp;N</td>
        <td>2.91&nbsp;g prop, 0.4% RW fill</td>
        <td class="num pass">0 / PASS</td>
      </tr>
      <tr>
        <td><strong>C. Roll Sync</strong></td>
        <td>1.0&nbsp;&deg;/s client spin + RW roll sync</td>
        <td>6.01&nbsp;s sync; A1/A2 lip 358/376&nbsp;N</td>
        <td>21.4&nbsp;s detumble, 14.5% RW, 0.51&nbsp;g</td>
        <td class="num pass">0 / PASS</td>
      </tr>
      <tr>
        <td><strong>D. Wheels Detumble</strong></td>
        <td>2.0&nbsp;&deg;/s oblique spin, 0 thruster pulses</td>
        <td>6.00 / 6.01&nbsp;s; A1/A2 lip 379/382&nbsp;N</td>
        <td>71.5&nbsp;s, <strong>69.3% RW fill</strong> (&lt;80%), 1.52&nbsp;g</td>
        <td class="num pass">0 / PASS</td>
      </tr>
      <tr>
        <td><strong>E. 3D High-Load</strong></td>
        <td>5&deg; pitch + 5&deg; yaw + 5.0&nbsp;N&middot;m hold torque</td>
        <td>Wrists rotated up to &plusmn;25.9&deg;; lip 398/338&nbsp;N</td>
        <td>0.89&nbsp;g prop, 0.1% RW fill</td>
        <td class="num pass">0 / PASS</td>
      </tr>
    </tbody>
  </table>
</div>

<div class="figure-block">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/dual_arm_mj_capture.png" alt="MuJoCo full capture" />
      <div class="panel-label">(a) Full 3D capture on 300.1&nbsp;kg free-floating target</div>
    </div>
    <div class="panel">
      <img src="figures/dual_arm_mj_closeup.png" alt="MuJoCo gripper closeup" />
      <div class="panel-label">(b) Marman 20&deg; wedge form closure + Mini45 F/T + Cardan wrist</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 6. MuJoCo physics verification frames.</strong> Form-closure jaw geometry generates 224&ndash;392&nbsp;N palm seating force and 317&ndash;398&nbsp;N undercut lip preload without clipping during approach.</div>
</div>

<!-- ====================================================================== -->
<h1>4. Initial In-Orbit Demo: Ultra-Shallow Hosted-Payload Concept</h1>

<h2>4.1 Concept Motivation &amp; Architectural Trade</h2>
<p>Rather than launching a complete propulsive spacecraft for initial flight validation, the flight dual arms can be tested far more rapidly as a compact <strong>hosted payload</strong> mounted to an upper-stage orbital platform or orbital transfer vehicle. Precedents such as DLR <em>ROKVISS</em>, GITAI <em>S1/S2</em>, ISRO POEM-4 <em>RRM-TD</em>, and Cosmoserve <em>Mission Embrace</em> highlight two practical design rules adopted here: (1) use motorized, reversible deployment rather than one-shot spring releases so target deployment cannot stall mid-stroke; and (2) include an independent optical witness camera alongside joint telemetry for unambiguous verification.</p>

<h2>4.2 Ultra-Shallow Chassis (273.0 &times; 520.0 &times; 840.0&nbsp;mm, ~45.6&nbsp;kg)</h2>
<p>Removing propulsion, tanks, RCS pods, reaction wheels, star trackers, and long-range bus optics cuts overall depth by <strong>61.2%</strong> and volume by <strong>70.2%</strong> while keeping the <strong>exact flight 6-DOF arm bays and drives</strong> untouched:</p>
<ul>
  <li><strong>0&ndash;10&nbsp;mm:</strong> Rear host mounting baseplate with four corner titanium bolt bushings (28&nbsp;V DC power + Ethernet/CAN umbilical) and flush &minus;Z 210 &times; 480&nbsp;mm (0.101&nbsp;m&sup2;) OSR thermal radiator pocket on isolated host interfaces.</li>
  <li><strong>10&ndash;90&nbsp;mm (Rear Deployer &amp; Avionics Bay):</strong> Houses two independent boom motors/shafts (tangent drum axes at Z = &plusmn;123&nbsp;mm peeling tangentially into Z = &plusmn;155&nbsp;mm chutes), four &Oslash;64&nbsp;mm hub / &Oslash;70&nbsp;mm flange spool drums at (&plusmn;210, &plusmn;123)&nbsp;mm, the aft end of the two q0 Base-Roll drives (26&nbsp;mm clear of boom motors), host-deck-bolted payload computer / 14&times; FOC drives, and PDU.</li>
  <li><strong>90&ndash;245&nbsp;mm (Structural Bulkhead &amp; Recessed Front Bay):</strong> Preserves the 20&nbsp;mm shear bulkhead, exact 135&nbsp;mm dual-arm bays (X_stow &le; 244&nbsp;mm), four rigid composite boom guide chutes inside the solid outer cheeks, a 22&nbsp;mm annular nesting groove for the stowed target ring, and a tiny &Oslash;12&nbsp;mm centerline witness camera at (Y = 0, Z = 0).</li>
  <li><strong>245&ndash;273&nbsp;mm (Stowed Dummy Target Plate):</strong> A 480 &times; 440 &times; 8&nbsp;mm aluminum plate carrying a machined 15&Prime; (&Oslash;381&nbsp;mm) Marman ring sits on four perimeter cup-cone HDRM standoffs, doubling as a protective front cover over the stowed arms (38.0&nbsp;kg base / 45.6&nbsp;kg with +20% MGA, CG = 141.5&nbsp;mm, 65&ndash;110&nbsp;W peak power during capture).</li>
</ul>

<h2>4.3 Rollable CFRP Booms, Independent Tilt Motors &amp; Wrist-Stereo ConOps</h2>
<p>The dummy plate deploys forward to +480&nbsp;mm capture standoff using <strong>four &Oslash;20 &times; 0.24&nbsp;mm bistable CFRP slit-tube booms</strong> (coiled like solar-array or tape-spring booms on &Oslash;64&nbsp;mm drums at 0.375% strain across 3.2 wraps). Because thin-shell booms have a ~100&ndash;120&nbsp;mm root transition length where stiffness develops, routing each boom through a 135&nbsp;mm internal cheek chute ensures it exits the chassis front face 100% rigid. Conservative portal-frame estimates yield 8.9&nbsp;kN/m lateral stiffness, <strong>8.2&nbsp;Hz</strong> first mode (&gt; 8&times; control loop), 0.56&nbsp;mm deflection under 5&nbsp;N touch, and 4,082&nbsp;N Euler buckling capacity. Driving the upper and lower boom motors independently with a 27.0&nbsp;mm payout delta tilts the plate by <strong>+5.0&deg; pitch</strong> in orbit to test full 6-DOF Cardan wrist alignment.</p>

<p>With no bus camera/LiDAR bay, capture relies solely on the 50&nbsp;mm wrist stereo bars (&sigma;_Z = 0.58&nbsp;mm at 400&nbsp;mm &rarr; 0.15&nbsp;mm at 200&nbsp;mm &rarr; <strong>0.036&nbsp;mm at 100&nbsp;mm</strong>) with integrated LED illumination (operable in eclipse), Mini45 F/T admittance, and the centerline witness camera. On orbit, the demo executes a five-step orbital test cycle: (1) release plate HDRMs &amp; extend booms to +480&nbsp;mm; (2) release arm locks &amp; acquire ring pose via wrist stereo; (3) open-jaw approach &amp; F/T feather-touch clamp/hold/release; (4) differential upper/lower boom payout for +5&deg; tilted asynchronous capture; and (5) controlled retraction for repeat tests.</p>

<div class="figure-block">
  <div class="fig-grid-1x3">
    <div class="panel">
      <img src="figures/iod_01_stowed_iso_front.png" alt="IOD stowed iso" />
      <div class="panel-label">(a) Stowed hosted payload (front 3/4 view)</div>
    </div>
    <div class="panel">
      <img src="figures/iod_02_stowed_side_profile.png" alt="IOD stowed side profile" />
      <div class="panel-label">(b) Stowed side profile (273.0&nbsp;mm depth)</div>
    </div>
    <div class="panel">
      <img src="figures/iod_03_rear_bay_cutaway.png" alt="IOD rear bay cutaway" />
      <div class="panel-label">(c) Cutaway: boom motors, spools &amp; avionics</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 7. Ultra-shallow IOD stowed configuration and rear deployer cutaway.</strong> Both independent boom motor rails (orange cylinders at Z = &plusmn;155&nbsp;mm), four &Oslash;70&nbsp;mm spools, central avionics, and aft q0 drives pack cleanly inside 80&nbsp;mm depth with 0.00&nbsp;mm&sup3; overlap.</div>
</div>

<div class="figure-block">
  <div class="fig-grid-1x3">
    <div class="panel">
      <img src="figures/iod_04_deployed_nominal_elevation.png" alt="IOD deployed nominal elevation" />
      <div class="panel-label">(a) Nominal 15&Prime; deployment (+480&nbsp;mm)</div>
    </div>
    <div class="panel">
      <img src="figures/iod_05_deployed_nominal_iso.png" alt="IOD deployed nominal iso" />
      <div class="panel-label">(b) 3/4 iso: 4&times; booms &amp; witness cam</div>
    </div>
    <div class="panel">
      <img src="figures/iod_06_deployed_tilted_5deg_pitch.png" alt="IOD deployed tilted +5 deg" />
      <div class="panel-label">(c) Differential +5&deg; pitch capture test</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 8. Deployed on-orbit IOD demonstration modes.</strong> Solid-kernel FreeCAD verification confirms 0.00&nbsp;mm&sup3; interference across stowed packaging, nominal capture (reach 412.0/456&nbsp;mm), &plusmn;18&nbsp;mm open-jaw approach (+458&nbsp;mm), and +5.0&deg; differential-boom capture (reach 423.5 / 391.5&nbsp;mm).</div>
</div>

<div class="figure-block">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/iod_scale_iod_front.png" alt="Hosted payload demo front view with human and banana for scale" />
      <div class="panel-label">(a) Front view: 18&nbsp;cm banana | demo (52.0&times;84.0&nbsp;cm) | 175&nbsp;cm human</div>
    </div>
    <div class="panel">
      <img src="figures/iod_scale_iod_side.png" alt="Hosted payload demo side view with human and banana for scale" />
      <div class="panel-label">(b) Side view: 18&nbsp;cm banana | demo (27.3&times;84.0&nbsp;cm) | 175&nbsp;cm human</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 9. True-to-scale reference (Hosted-Payload Arm Demo).</strong> Orthographic front and side views showing the ultra-shallow 27.3&nbsp;cm-deep stowed testbed centered between an ~18&nbsp;cm banana (left) and a 175&nbsp;cm human (right).</div>
</div>

<!-- ====================================================================== -->
<h1>5. Preliminary Thermal Audit (Orbital Network &amp; 3D Coupled FEM)</h1>

<p>This section presents a <strong>preliminary thermal audit</strong> of the updated internal CAD packaging for both the Dual-Arm 15&Prime; ESPA Servicer and the Arm IOD Demo (with <code>0.00&nbsp;mm&sup3;</code> B-Rep internal interference). The study combines a 12-node / 6-node 4th-order Runge-Kutta orbital transient network grounded in ECSS-E-ST-31C, NASA MSFC TM-2001-211221, NASA GSFC GOLD Rules, and Gilmore&rsquo;s <em>Spacecraft Thermal Control Handbook</em> with one-way coupled 3D <code>Gmsh</code> + <code>CalculiX v2.22</code> thermomechanical FEM models. Results are marked preliminary because full Monte Carlo ray-traced external self-shadowing and thermal-vacuum (TVAC) hardware correlation remain to be performed in Phase&nbsp;B/C.</p>

<h2>5.1 Assumptions, Specifications, Limits &amp; Boundary Conditions</h2>
<ul>
  <li><strong>Compute &amp; Solar Power Architecture:</strong> Flight-critical bus control runs continuously on a rad-hard supervisor OBC (4&ndash;6&nbsp;W), while AI rendezvous inference runs on a power-gated <strong>NVIDIA Jetson Orin NX 16GB (25&nbsp;W active)</strong>. Body-mounted 0.40&nbsp;m&sup2;/side GaAs arrays deliver 118.1&nbsp;W peak sun-normal but only <strong>53.0&nbsp;W LEO 550&nbsp;km orbit-average</strong> usable power (94.6&nbsp;W GEO). Therefore, Orin&nbsp;NX is switched <strong>OFF</strong> during cruise, trimming cruise bus load to <strong>32.5&nbsp;W</strong> (+20.5&nbsp;W recharge surplus). A 20-min RPO capture (263.0&nbsp;W peak, including ~11% / 29&nbsp;W PCDU DC-DC heat) plus 5-min wheel detumble (182.5&nbsp;W) consumes <strong>102.9&nbsp;Wh</strong> (10.3% of the 1,000&nbsp;Wh Li-ion battery), recharging within <strong>3.5 LEO orbits (5.6&nbsp;hr)</strong>.</li>
  <li><strong>Thermal Control Hardware &amp; Modeling Conservatisms:</strong> Heat is rejected through a right-sized <strong>520 &times; 375&nbsp;mm (0.195&nbsp;m&sup2;) &minus;Z OSR radiator</strong> on the servicer and a flush <strong>210 &times; 480&nbsp;mm (0.101&nbsp;m&sup2;) &minus;Z OSR pocket</strong> on the IOD chassis (&alpha;_BOL&nbsp;=&nbsp;0.07, &alpha;_EOL&nbsp;=&nbsp;0.16, &epsilon;&nbsp;=&nbsp;0.80). High-flux +Y DragonEye LiDAR heat (65&nbsp;W) routes via a bolted pocket saddle and embedded CCHP heat pipe (~5.5&nbsp;W/K) directly to the &minus;Z coldplate; the 1,000&nbsp;Wh battery sits on low-k G10 isolators (0.25&nbsp;W/K). Per Gilmore, large bus Gold Kapton MLI is modeled at &epsilon;*&nbsp;=&nbsp;0.03&ndash;0.04 whereas the narrow 0.095&nbsp;m&sup2; tubular arm/wrist MLI sleeves take an edge-loss penalty of <strong>&epsilon;*&nbsp;=&nbsp;0.07 (hot) / 0.08 (cold)</strong>; bolted vacuum coldplate interfaces use 18&nbsp;W/K (servicer) and 5.5&nbsp;W/K (IOD); survival heaters obey NASA GOLD Rules (<strong>&le;&nbsp;75%</strong> cold orbit-average duty cycle).</li>
</ul>

<div class="table-block">
  <div class="table-caption"><strong>Table 4.</strong> Orbital environmental boundary conditions and component operating temperature limits.</div>
  <table>
    <thead>
      <tr>
        <th style="width:23%">Boundary / Hardware Item</th>
        <th style="width:37%">Hot Case Parameters (EOL / Max Load)</th>
        <th style="width:40%">Cold Case Parameters (BOL / Min Load / Limits)</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>LEO 550&nbsp;km SSO (97.6&deg;)</strong></td>
        <td>S&nbsp;=&nbsp;1418&nbsp;W/m&sup2;, Alb&nbsp;=&nbsp;0.35, OLR&nbsp;=&nbsp;265&nbsp;W/m&sup2;, 0&nbsp;min eclipse; exact F_side&nbsp;=&nbsp;0.2578 &amp; worst-case RPO nadir tilt <strong>F_nadir&nbsp;=&nbsp;0.8474</strong></td>
        <td>S&nbsp;=&nbsp;1322&nbsp;W/m&sup2;, Alb&nbsp;=&nbsp;0.18, OLR&nbsp;=&nbsp;206&nbsp;W/m&sup2;, <strong>35.6&nbsp;min</strong> eclipse; safe load 47.4&nbsp;W w/ heaters vs. 53.0&nbsp;W solar avg</td>
      </tr>
      <tr>
        <td><strong>GEO Transfer / Station</strong></td>
        <td>S&nbsp;=&nbsp;1418&nbsp;W/m&sup2;, 23.5&deg; solstice sun grazing &minus;Z OSR, Alb/OLR&nbsp;=&nbsp;0</td>
        <td>S&nbsp;=&nbsp;1322&nbsp;W/m&sup2;, <strong>72&nbsp;min</strong> equinox umbra eclipse, cold soak w/ thermostatted heaters (58.2&nbsp;W vs. 94.6&nbsp;W GEO solar)</td>
      </tr>
      <tr>
        <td><strong>Hosted IOD Boundaries</strong></td>
        <td>Conductive host +45&nbsp;&deg;C (G&nbsp;=&nbsp;12&nbsp;W/K) or Isolated host +40&nbsp;&deg;C (G&nbsp;=&nbsp;0.10&nbsp;W/K &le;&nbsp;4.0&nbsp;W NASA TICD); 15-min burst (69&nbsp;W total)</td>
        <td>Host &minus;20&nbsp;&deg;C (Conductive or Isolated); Orin&nbsp;NX OFF in idle/safe; 10&nbsp;W wrist &amp; 20&nbsp;W electronics heaters (&le;&nbsp;75% duty)</td>
      </tr>
      <tr>
        <td><strong>Critical Thermal Limits</strong></td>
        <td>Battery &le;&nbsp;40&nbsp;&deg;C, DragonEye &le;&nbsp;40&nbsp;&deg;C, PEPT-230 tank &le;&nbsp;50&nbsp;&deg;C, Avionics &le;&nbsp;60&nbsp;&deg;C, ATI&nbsp;Mini45 &le;&nbsp;60&nbsp;&deg;C, <strong>Orin&nbsp;NX SoC T_j &le;&nbsp;99&nbsp;&deg;C</strong></td>
        <td><strong>PEPT-230 &ge;&nbsp;+10&nbsp;&deg;C</strong> (ADN precip.), <strong>Battery &ge;&nbsp;0&nbsp;&deg;C</strong>, <strong>ATI&nbsp;Mini45 &ge;&nbsp;&minus;5&nbsp;&deg;C</strong>, RCS &ge;&nbsp;&minus;10&nbsp;&deg;C, Avionics &ge;&nbsp;&minus;20&nbsp;&deg;C (Orin startup &ge;&nbsp;&minus;25&nbsp;&deg;C); ECSS &plusmn;10&nbsp;&deg;C target</td>
      </tr>
    </tbody>
  </table>
</div>

<div class="figure-block">
  <div class="fig-grid-1x2">
    <div class="panel">
      <img src="figures/therm_cad_v4_cutaway.png" alt="v4.2 internal thermal packaging cutaway" />
      <div class="panel-label">(a) Servicer cutaway: coldplate, CCHP strap, G10 battery &amp; RW-12 truss</div>
    </div>
    <div class="panel">
      <img src="figures/therm_cad_iod_cutaway.png" alt="IOD internal thermal packaging cutaway" />
      <div class="panel-label">(b) IOD rear bay: bolted Orin/FOC/PDU, tangent drums &amp; &minus;Z OSR</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 10. Verified internal packaging &amp; heat-rejection architecture.</strong> In (a), high-power PCDU, OBC/Orin&nbsp;NX, and 14&times; Elmo FOC drives bolt directly to the &minus;Z coldplate beneath the 0.195&nbsp;m&sup2; OSR while the Li-ion battery isolates thermally via G10 standoffs; the +Y DragonEye couples down to &minus;Z via a bulkhead pass-through CCHP. In (b), IOD electronics bolt directly to the host mounting deck with a flush 0.101&nbsp;m&sup2; &minus;Z OSR pocket for isolated hosts.</div>
</div>

<h2>5.2 Orbital Transient Experiments</h2>

<p>Cyclic multi-orbit RK4 simulations were run across five boundary pairs (Table&nbsp;5, Figure&nbsp;11) until orbit-to-orbit equilibrium, followed by the 20-min RPO + 5-min detumble peak load (servicer) or 15-min capture burst (IOD). <strong>Zero hard temperature limit violations occurred across all scenarios</strong>, every zone in LEO achieves &ge;&nbsp;+10&nbsp;&deg;C ECSS margin, and the highest heater orbit-average duty cycle across all cold cases is <strong>59.7%</strong> (&le;&nbsp;75% GOLD limit).</p>

<div class="table-block">
  <div class="table-caption"><strong>Table 5.</strong> Multi-node orbital transient results across LEO, GEO, and hosted-payload boundary modes (zero limit failures).</div>
  <table>
    <thead>
      <tr>
        <th style="width:24%">Scenario Pair</th>
        <th style="width:18%">Cold Heater Avg / Max Duty</th>
        <th style="width:40%">Key Nodal Extremes (Cold Min .. Hot Max, &deg;C) &amp; Orin T_j</th>
        <th class="num" style="width:18%">Failures / Status</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Servicer LEO Side-View</strong><br/>(F_side&nbsp;=&nbsp;0.2578)</td>
        <td>23.2&nbsp;W avg<br/>(42.6% wrist)</td>
        <td>Bus &minus;6.3..13.6, Avionics &minus;6.2..16.7, Batt 10.0..19.1, Tank 20.0..21.5, DragonEye &minus;7.2..21.1, Mini45 6.0..9.9</td>
        <td class="num pass">0 / ALL &ge; 10&deg;C</td>
      </tr>
      <tr>
        <td><strong>Servicer LEO Nadir RPO</strong><br/>(Worst F_nadir&nbsp;=&nbsp;0.8474)</td>
        <td>23.2&nbsp;W avg<br/>(42.6% wrist)</td>
        <td>&minus;Z OSR &minus;7.2..15.8, <strong>Avionics &minus;6.2..17.9</strong> (Orin T_j&nbsp;&approx;&nbsp;35.4&nbsp;&deg;C), <strong>DragonEye &minus;7.2..22.0</strong>, Mini45 6.0..10.0</td>
        <td class="num pass">0 / ALL &ge; 10&deg;C</td>
      </tr>
      <tr>
        <td><strong>Servicer GEO Sol. / Equin.</strong><br/>(23.5&deg; sun / 72-m shadow)</td>
        <td>32.9&nbsp;W avg<br/>(45.4% tank)</td>
        <td>Bus &minus;7.9..8.3, Avionics &minus;7.7..11.7, SolarPanels &minus;82.8..28.8, DragonEye &minus;8.0..15.9, Mini45 6.0..10.1</td>
        <td class="num pass">0 / PASS</td>
      </tr>
      <tr>
        <td><strong>IOD Conductive Host</strong><br/>(+45 / &minus;20&nbsp;&deg;C, G&nbsp;=&nbsp;12&nbsp;W/K)</td>
        <td>6.0&nbsp;W avg<br/>(59.7% wrist)</td>
        <td>Chassis &minus;20.6..43.0, <strong>Orin/FOC &minus;19.9..48.1</strong> (T_j&nbsp;&approx;&nbsp;65.6&nbsp;&deg;C), PDU &minus;20.2..44.7, Mini45 6.0..31.1</td>
        <td class="num pass">0 / +5.1&deg;C cold</td>
      </tr>
      <tr>
        <td><strong>IOD Isolated Host TICD</strong><br/>(G&nbsp;=&nbsp;0.10&nbsp;W/K + 0.101&nbsp;m&sup2; OSR)</td>
        <td>5.9&nbsp;W avg<br/>(58.8% wrist)</td>
        <td>Chassis &minus;21.5..12.4, Orin/FOC &minus;20.6..19.2, Mini45 6.0..9.5, <strong>Host Leak&nbsp;=&nbsp;2.99&nbsp;W &le;&nbsp;4.0&nbsp;W</strong></td>
        <td class="num pass">0 / TICD PASS</td>
      </tr>
    </tbody>
  </table>
</div>

<div class="figure-block">
  <div class="panel">
    <img src="figures/therm_orbital_profiles.png" alt="Orbital and hosted payload transient thermal profiles" />
  </div>
  <div class="fig-caption"><strong>Figure 11. Orbital transient temperature histories across key regimes.</strong> <em>Top-left:</em> Starting from +10.2&nbsp;&deg;C LEO cruise, the 20-min nadir-tilt RPO burst drives DragonEye to 22.0&nbsp;&deg;C and avionics to 17.9&nbsp;&deg;C while the bus mass and 0.195&nbsp;m&sup2; OSR dissipate the pulse cleanly after detumble. <em>Top-right &amp; bottom-left:</em> Thermostatted Kapton heaters regulate the propellant tank (20.0&ndash;21.5&nbsp;&deg;C), battery (10&ndash;14&nbsp;&deg;C), and wrist Mini45 (6&ndash;10&nbsp;&deg;C) through both 35.6-min LEO and 72-min GEO eclipses without exceeding 45.4% duty cycle. <em>Bottom-right:</em> Under thermally isolated host mounting, the IOD Orin&nbsp;NX/FOC stack rises only +9.7&nbsp;&deg;C during a 15-min burst while holding host conductive leak below 3.0&nbsp;W.</div>
</div>

<h2>5.3 Full-Assembly 3D Coupled FEM Experiments</h2>

<p>To verify spatial hotspots, contact gradients, and thermal-expansion (CTE) deflections not visible in lumped nodes, full 3D multi-material <code>CompSolid</code> meshes (Al-6061, Al-7075, Ti-6Al-4V, CFRP, G10, CCHP) were built with <strong>both deployed 6-DOF arms attached directly to the front bulkheads</strong> (plus full optical suite on the servicer) and solved in <code>CalculiX v2.22</code> (<code>*COUPLED TEMPERATURE-DISPLACEMENT</code>) using one-way orbital equilibrium boundary sinks (Figure&nbsp;12, Table&nbsp;6).</p>

<div class="figure-block">
  <div class="fig-grid-2x2">
    <div class="panel">
      <img src="figures/therm_fem_v4_full_rpo_hot.png" alt="Full v4.2 bus plus attached arms and optics 20-min nadir RPO FEM" />
      <div class="panel-label">(a) Servicer + arms + optics: 20-min Nadir RPO (10.2..18.1&nbsp;&deg;C)</div>
    </div>
    <div class="panel">
      <img src="figures/therm_fem_v4_full_geo_cold.png" alt="Full v4.2 bus plus attached arms 72-min GEO cold eclipse FEM" />
      <div class="panel-label">(b) Servicer + arms: 72-min GEO cold eclipse (1.1..13.3&nbsp;&deg;C)</div>
    </div>
    <div class="panel">
      <img src="figures/therm_fem_iod_full_iso_hot.png" alt="Full IOD chassis plus attached arms 15-min isolated burst FEM" />
      <div class="panel-label">(c) IOD + arms: 15-min isolated burst (9.0..15.8&nbsp;&deg;C)</div>
    </div>
    <div class="panel">
      <img src="figures/therm_fem_arm_hot_capture.png" alt="High-resolution deployed arm sunlit capture FEM" />
      <div class="panel-label">(d) Arm chain detail: Mini45 &Delta;T = 1.4&nbsp;&deg;C, CTE = 0.025&nbsp;mm</div>
    </div>
  </div>
  <div class="fig-caption"><strong>Figure 12. 3D CalculiX coupled thermal-structural FEM contours (temperature in K on colorbars; 10&deg;&ndash;18&nbsp;&deg;C range across operational bursts).</strong> In (a), the +Y DragonEye aperture bezel glows warmly at 17.0&nbsp;&deg;C directly above the cooler IRIS camera barrel (12.0&nbsp;&deg;C), transferring heat across the internal CCHP path (green patch on the +Y cheek) down to the &minus;Z OSR radiator; both forward-reaching arms show smooth gradients from 18.1&nbsp;&deg;C active elbows down to the cooler 10.7&nbsp;&deg;C bulkhead mounts. In (b), joint/wrist heaters under &epsilon;*&nbsp;=&nbsp;0.08 MLI hold the Mini45 sensors at +5.7..+6.3&nbsp;&deg;C while the cold bus drops to +1.1&nbsp;&deg;C. In (c), rear-bay Orin&nbsp;NX/FOC heat (13.2&nbsp;&deg;C center spot) spreads cleanly into the ultra-shallow chassis and &minus;Z OSR pocket (9.0&nbsp;&deg;C) with deployed arms at 14.5&nbsp;&deg;C. Panel (d) confirms a minimal 1.4&nbsp;&deg;C axial gradient across the ATI Mini45 wafer.</div>
</div>

<div class="table-block">
  <div class="table-caption"><strong>Table 6.</strong> Summary of 3D <code>Gmsh</code> + <code>CalculiX v2.22</code> coupled thermomechanical FEM verification runs.</div>
  <table>
    <thead>
      <tr>
        <th style="width:27%">3D FEM Assembly &amp; Case</th>
        <th style="width:15%">Mesh (Nodes / Tets)</th>
        <th style="width:43%">Observed Regional Temperatures &amp; Critical Gradients</th>
        <th class="num" style="width:15%">Max CTE Shift</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Servicer + 2 Arms + Optics</strong><br/>LEO 20-m Nadir RPO (T0&nbsp;=&nbsp;10.2&nbsp;&deg;C)</td>
        <td>10,170 / 33,287<br/>(28 solids)</td>
        <td>Overall 10.2..18.1&nbsp;&deg;C; <strong>DragonEye 12.8..17.0&nbsp;&deg;C</strong>, IRIS 11.9..12.0&nbsp;&deg;C, ST 10.4&nbsp;&deg;C, Avionics 11.2..12.2&nbsp;&deg;C, Mini45 16.7..16.8&nbsp;&deg;C</td>
        <td class="num">0.0464&nbsp;mm</td>
      </tr>
      <tr>
        <td><strong>Servicer + 2 Arms + Optics</strong><br/>GEO 72-m Eclipse (2.7&nbsp;K sink)</td>
        <td>10,170 / 33,287<br/>(28 solids)</td>
        <td>Overall 1.1..13.3&nbsp;&deg;C; Bus 1.1..6.6&nbsp;&deg;C, DragonEye 1.6..2.1&nbsp;&deg;C, Elbow 10.5..13.3&nbsp;&deg;C, <strong>Mini45 5.7..6.3&nbsp;&deg;C</strong></td>
        <td class="num">0.0271&nbsp;mm</td>
      </tr>
      <tr>
        <td><strong>IOD Chassis + 2 Arms</strong><br/>Isolated 15-m Burst (T0&nbsp;=&nbsp;9.5&nbsp;&deg;C)</td>
        <td>6,049 / 19,778<br/>(18 solids)</td>
        <td>Overall 9.0..15.8&nbsp;&deg;C; <strong>Orin/OBC 11.9..13.2&nbsp;&deg;C</strong>, FOC 11.2..12.5&nbsp;&deg;C, PDU 9.6..10.3&nbsp;&deg;C, Mini45 14.4..14.5&nbsp;&deg;C</td>
        <td class="num">0.0244&nbsp;mm</td>
      </tr>
      <tr>
        <td><strong>IOD Conductive Host (+45&nbsp;&deg;C)</strong><br/>Steady Peak Burst (54.5&nbsp;W)</td>
        <td>Chassis CompSolid</td>
        <td>Overall 43.7..45.7&nbsp;&deg;C; <strong>Orin/OBC 45.0..45.7&nbsp;&deg;C (Est. T_j&nbsp;&approx;&nbsp;63.2&nbsp;&deg;C &lt;&nbsp;99&nbsp;&deg;C)</strong>, FOC/PDU 45.0..45.4&nbsp;&deg;C</td>
        <td class="num">0.0094&nbsp;mm</td>
      </tr>
      <tr>
        <td><strong>Detailed 6-DOF Arm Chain</strong><br/>Sunlit Capture / GEO 72-m Eclipse</td>
        <td>1,970 / 6,670<br/>(8 solids)</td>
        <td>Capture: 13.0..18.7&nbsp;&deg;C (<strong>Mini45 16.4..17.9&nbsp;&deg;C, &Delta;T&nbsp;=&nbsp;1.4&nbsp;&deg;C</strong>); GEO Eclipse (bulkhead &minus;7.9&nbsp;&deg;C): Shoulder &minus;6.0&nbsp;&deg;C, <strong>Mini45 12.3..20.9&nbsp;&deg;C</strong></td>
        <td class="num">0.0249&nbsp;mm</td>
      </tr>
    </tbody>
  </table>
</div>

<h2>5.4 Preliminary Verdict &amp; Verification Caveats</h2>
<ul>
  <li><strong>Preliminary Audit Verdict &mdash; PASS (Orin NX Baseline Closed):</strong> Both spacecraft geometries, internal layouts, and the shared 6-DOF Cardan-Roll arm satisfy all thermal limits with zero hard failures, &le;&nbsp;59.7% survival heater duty cycles, &le;&nbsp;0.047&nbsp;mm peak CTE distortion, and &gt;&nbsp;33&nbsp;&deg;C Orin&nbsp;NX junction margin even under worst-case LEO nadir-tilt RPO and +45&nbsp;&deg;C conductive host interfaces.</li>
  <li><strong>Mandatory Operational Constraints:</strong> Because LEO body-mounted solar generation averages 53.0&nbsp;W (versus 47.4&nbsp;W cold-eclipse survival), flight software must <strong>power-gate Orin&nbsp;NX OFF</strong> outside RPO/capture windows and keep <strong>both arms stowed inside the recessed front bays</strong> during cruise and safe mode. On a &minus;20&nbsp;&deg;C conductive IOD host, command a 2&ndash;3&nbsp;min heater pre-warm prior to Orin&nbsp;NX boot.</li>
  <li><strong>Required Pre-PDR/CDR Verification:</strong> Follow this preliminary audit with (1) Monte Carlo ray-traced geometric radiative modeling (ESATAN-TMS / Thermal Desktop) capturing deployed-arm self-shadowing across the &plusmn;Y arrays and &minus;Z OSR, and (2) TVAC thermal balance correlation of the DragonEye CCHP saddle, small-blanket arm MLI, and custom Orin&nbsp;NX die-to-coldplate thermal transfer plate.</li>
</ul>

<div class="callout">
  <strong>Next Engineering Milestones:</strong> With both full-servicer and IOD FreeCAD assemblies verified at 0.00&nbsp;mm&sup3; overlap, MuJoCo servicer capture checks A&ndash;E passing, and this preliminary orbital + 3D FEM thermal audit complete, near-term steps are: (1) model boom flexure, differential tilt, and wrist-stereo closed-loop capture in a dedicated IOD MuJoCo scene; (2) build a 1g bench engineering arm + boom unit to validate F/T admittance and stereo calibration; (3) complete ray-traced GMM/TVAC correlation alongside environmental vibration testing; and (4) advance the 45.6&nbsp;kg hosted IOD toward flight readiness.
</div>

</body>
</html>
"""


def build_pdf() -> None:
    prepare_figures()
    HTML_PATH.write_text(HTML_DOC, encoding="utf-8")

    browsers = [
        Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe"),
        Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"),
    ]
    browser = next((b for b in browsers if b.exists()), None)
    assert browser is not None, "Neither Chrome nor Edge found for headless PDF generation"

    cmd = [
        str(browser),
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--no-pdf-header-footer",
        "--allow-file-access-from-files",
        f"--print-to-pdf={PDF_PATH}",
        HTML_PATH.as_uri(),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or not PDF_PATH.exists():
        raise RuntimeError(f"PDF build failed ({res.returncode}): {res.stderr}")
    print(f"BUILT HTML: {HTML_PATH}")
    print(f"BUILT PDF : {PDF_PATH} ({PDF_PATH.stat().st_size / 1024:.1f} KB)")


if __name__ == "__main__":
    build_pdf()
