# Project 7: unprepared client (no grapple bar), draft spec

Status: DRAFT plan, 19 Nov 2026. Nothing built. Starts after Project 6 (plan steps 3-6, the full-mission batch, then P6.4 faults and P6.5 test split). Research: `learning/RESEARCH_SERVICING.md`, "Unprepared clients (19 Nov 2026)". User decisions 19 Nov: **grip the launch adapter ring** (the industry standard), **lidar fit first**, **modelled contact sensors** (D5); gripper on the arm recommended (D2). What carries over and what changes: section 11.

## 1. Question and claim

**Today (Project 6):** the servicer captures a silent, tumbling client by a grapple bar that was put there for it (type C: non-cooperative, prepared).

**Question (user, 19 Nov):** can it capture a client that was never built for servicing (type D: non-cooperative, unprepared) by what existing satellites already have, the launch adapter ring, with the same safety? (The engine nozzle, as MEV-1/2 docked on still clients, stays a later option: the model carries one.)

**Claim:** sim-to-sim only, as in Projects 4-6. Success and safety as Project 6 (captured, no unintended contact, no abort, push <= 2 mm/s; any collision is unsafe; safe aborts are good results; about 80% captured with the rest safe aborts is a good result). New for contact: a peak load limit on the ring (section 7).

## 2. Scope

- **In:** a client model without the bar, with an aft face carrying a standard adapter ring (and small thrusters; a nozzle probe is a later option on a larger client); ring jaws on the arm; perception of the ring; the last approach and the grasp; the grip and contact monitors.
- **Carries over unchanged:** the far leg, the observation, the axis estimate and transfer, the fly-in planner, station keeping, the safety monitors and aborts, post-capture detumble, identification and the stack move. They do not depend on what the grip point is (the pre-grasp point is design data in the client frame).
- **Out:** spin above 2 deg/s, contactless detumbling, real images, refuelling, the nozzle probe (later option).
- **Frozen code:** as now (p3_capture / p4_vision modules not edited, P5 unchanged); P7 in its own folder, reusing P6 by import or subclass.

## 3. Client model (P7.1)

**Decided 19 Nov (user): keep the 300 kg class** (option a: a small satellite with a standard
ring, which our ESPA-class servicer and arm can handle) and edit the client to match that class;
GEO scale is D6, later. A GEO comsat is 3,000-6,500 kg with a 2-3 m body, 25-45 m arrays and a
1194 / 1666 mm ring: it would need an MEV-class servicer (about 2 t), not ours.

The P7 client, `client_p7` (models/clients/build_client_p7.py, a variant of `client_mid`, which already
is the current 1.5 x 1.0 x 1.0 m, 300 kg box made realistic):
- **no grapple bar**; the grasp face is +X (aft);
- **launch adapter ring 610 mm (24-inch class)** on +X with a clamp lip: the class small satellites
  use (8-24 inch separation rings; the 937 mm clamp band is rated to about 2.3 t, oversized for
  300 kg), and it leaves room on the 1.0 m face;
- **no large engine**: a 300 kg satellite has no 400 N apogee engine; one small (22 N class)
  thruster inside the ring and four small thrusters near the face corners, outside the ring (clutter
  the lidar fit and the clearance monitors must handle);
- kept from `client_mid`: the one 3-panel solar wing on +Y (breaks the 4-fold symmetry about the
  ring axis, sets a keep-out the approach must respect), the high-gain dish on +Z, two star
  trackers, the radiator on -Y;
- collision geometry: the bus box plus primitives for the ring; mass properties from the parts.

Spin cases: as now (0-2 deg/s about any body axis, slight nutation). The ring is off the spin axis
for most cases, so the grasp point co-rotates as the bar did; the fly-in already handles a
body-fixed point.

## 4. End effector: ring jaws on the arm (P7.2)

- The two-jaw gripper re-shaped to straddle the flange: one jaw on the ring's outer edge, one on the inner, the palm on the separation face (the NRL / Northrop ring gripper layout), closing in two stages (first light contact, then clamp), constraining all six degrees of freedom. A short section of the ring is bar-like, so most of the P3-P5 grasp logic (reach, align, close, hold) carries over; the jaw opening and pad shape change.
- **Grasp point on the ring:** any azimuth works; pick the one with the best access when the fly-in is planned (nearest the servicer's approach, clear of the client's other parts, the wrist camera's view open). New: the client frame point is chosen from the estimate, not fixed by the design.
- **On the arm (D2, recommended; section 9):** the gripper stays on the existing 6-joint arm.

## 5. Perception (P7.3; lidar fit first, user 19 Nov)

- **Lidar:** fit a circle (the ring's outer and inner edges, with the face plane) to the aft-face points, giving the ring centre and axis; the existing box fit gives the roll about the axis and is the prior. Classical, no training. Known pitfall to test: concentric circles (ring and nozzle exit) and inner / outer edges confused (DLR interface-ring work); the fit models both edges and the nozzle so they cannot be swapped.
- **Camera:** the corner network keeps the box pose (retrain `ckf_net` only if the missing bar and the new face spoil it); a ring ellipse check from the bus camera.
- **Close range (wrist camera):** the flange edge in view for the final jaw alignment; the P4/P5 palm network is bar-specific and does not apply. A small learned detector only if the lidar and an edge fit are not enough (CPU / short GPU runs only).

## 6. Guidance (P7.4)

- Pre-grasp point: off the chosen ring section along the face normal, at the distance the arm needs (about 1.8 m from the face as today), synchronised with the client (the existing fly-in to a body-fixed point).
- Reach: the arm brings the open jaws over the flange, then closes (as the bar grasp, re-tuned for the flange).
- Aborts: as now; plus an arm-load monitor during the reach and close (section 7).

## 7. Contact and grasp (P7.5)

- **Grasp-boundary map first, offline:** the jaws closing on the flange of a free-floating, spinning client in MuJoCo over position offset, angle, closing speed and relative rate (as the Apollo capture-boundary studies did for probes); record a good grip, peak force, client push. Gate: a region wide enough for the measured close-range errors (P6: hand-over 22-45 mm, attitude about 1-2 deg).
- **Load limit:** peak force and torque on the flange and the arm joints below set limits (arm peak torques are in the scene: 60 / 60 / 40 / 12 / 12 / 12 N m; a ring load limit was not found, take a conservative number and record it).
- **Grip and contact monitors:** what they may read is decision D5 (section 9).

## 8. Stages and gates

| Stage | Work | Gate |
|---|---|---|
| P7.0 | research and this plan (19 Nov); user decisions D1-D5 | D1, D4, D5 decided; D2 (arm) and D3 proposed |
| P7.1 | client model (ring + nozzle, no bar), physics check against rev 1 | mass, inertia, contacts sane; P6 chain still runs to pre-grasp on it |
| P7.2 | ring jaws on the arm, actuators, sensors (D5) | grips the flange of a fixed client; sensors read sensibly |
| P7.3 | ring perception (lidar circle fit; camera checks) | centre and axis errors at 8 m and 1.8 m small enough for P7.4 |
| P7.4 | grasp-point choice, pre-grasp and reach on the estimate | reaches the flange on the estimate, no truth |
| P7.5 | grasp-boundary map, then the closed-loop grasp | boundary covers the measured errors; loads under the limits |
| P7.6 | 25 tune seeds (full chain), truth audit, then once on the test split | about 80% captured, the rest safe aborts, 0 collisions |

One change, one ~25-seed run, record, move on (as P6). At most 6 workers, one job at a time, long runs detached in WSL.

## 9. Decisions

- **D1, grip point: the launch adapter ring** (user, 19 Nov: the industry standard). The nozzle probe stays a later option; the model carries a nozzle.
- **D2, gripper on the arm or on the bus: arm recommended** (user asked 19 Nov whether the arm's reach makes it better). Reasons: (1) reach: the bus keeps about 1.8 m from the client face instead of flying its own front face to within centimetres of a tumbling client's ring, so the bus stays outside the keep-out zone the monitors defend; (2) dexterity: six joints align the jaws to the flange whatever the bus attitude, and follow the ring section as the client turns; (3) compliance: the arm's joint torque control is the buffer that lowers the impact peak (contact literature), and its joint torques are a load measurement; (4) it is what flown and planned ring captures use (the RSGS arm on MRV, e.Deorbit, MDA's ring tool), and it keeps the P3-P6 chain. Costs: the arm must follow the tumbling client before the grasp (e.Deorbit's joint-torque concern), and an arm-held grip is softer than a docked joint (e.Deorbit clamped after grasp; we hold on the arm as with the bar, and can add a clamp later). A bus-mounted gripper (MEV-style docking) suits a client holding still, not a tumbling one.
- **D3, spin cases:** keep today's (0-2 deg/s, any body axis) (proposed).
- **D6, client scale: 300 kg class** (user, 19 Nov; section 3). GEO scale (4-5 t, 1194 / 1666 mm ring, an MEV-class servicer) is a later project.
- **D4, perception: lidar fit first** (user, 19 Nov); learned detector only if needed.
- **D5, what the grip and contact logic may read: modelled sensors, option (b)** (user, 19 Nov: the end effector changes anyway, so do the sensing properly). Details below and in section 11.
- **D7, servicer rev 2 (user, 20 Nov; all seven as recommended in `reports/Servicer rev 2 research for P7.md`, brief section 7):**
  1. Launch class: **ESPA standard 15 in port, ~200 kg wet** (mass properties from parts; ESPA Heavy, same port, if the
     port limit proves 180 kg). Arm stows in a flat fold (FreeCAD: 645 / 710 mm depth); pods recessed into the bus edges.
  2. Propulsion: **16 x 1 N green monoprop (HPGP class, ~220 s)** in four mid-depth edge pods, each +X and -X (canted
     30 deg outward), inward Y, inward Z; allocation about the estimated CoM / stack CoM; ~4 kg for 40 m/s.
  3. Attitude: **4 x 12 N m s / 0.2 N m wheels in a pyramid, in every phase**; thruster-couple unloading at 80%; switch to
     25 N m s wheels-only detumble if the held-client allocation would fire at the client. Magnetorquers: note only.
  4. Arm: **one arm on the +X face**, shoulder 0.25 m below centre, 10 deg/s enforced; MEV-style stanchions only as an
     upgrade if the stack move shows compliance problems.
  5. Grip: **form-closure ring tool with hard jaw faces** (outboard + inboard jaws, palm datum, two-stage close,
     non-backdrivable lead screw); **mu 0.4 nominal, 0.2 stress, 0.8 jam check**.
  6. Fidelity: **model** mass from parts, bus collision, per-nozzle thrust (pulses, MIB, +-5% / 1-2 deg errors, propellant
     use), wheel limits everywhere, joint speed limit, non-backdrivable jaws, realistic mu, contact-sensor noise,
     free-drift monitor; **log** power per phase (no gating); **note** thermal, magnetorquers, joint stiffness, slosh.
  7. Approach: **holds 30 / 8 / 3 / 1.8 m, synchronise only from a new 3 m stop on the estimated spin axis**, pre-grasp
     near the spin axis; Clohessy-Wiltshire free-drift check per hold and leg against a >= 2 m keep-out; 10-15 deg
     corridor; no nozzle firing within 30 deg of the client inside 3 m; observation cap 1.5x p95 with back-out to 30 m;
     2-3 attempts with a propellant floor; automatic gates on covariance. 50-80 m inspection hold recorded as upstream.

### D5 in more detail: contact sensing

**What it is about.** The sim knows every contact by name: MuJoCo lists each contact with the two geoms involved. Today the grasp chain uses that directly (`grasp_sim.contacts`: normal force per pad on the bar, any other servicer-client contact; `unintended_hits` for the safety monitor and scoring). A real servicer gets no such labelled list. It has sensors: joint torque sensors in the arm (ours already assumes them for friction compensation), a wrist force / torque sensor (ours is drawn as a visual ring only), jaw motor position and current, and possibly pad touch sensors or proximity sensors in the jaws. The P6 audit marked this as an open truth input (item f): reading "pad touched bar, nothing else touched the client" from the contact list is like having perfect tactile sensing everywhere. It matters most for: deciding the grasp is made, the slip monitor in the hold (P6 plan step 4 reads the true bar today), and detecting an unintended touch early enough to abort.

**Options:**
- **(a) Keep the labelled contacts** as a stand-in for touch sensing, recorded as accepted. Least work; the claim then says "perfect contact sensing".
- **(b) Model the sensors and use only them:** MuJoCo `touch` sensors on the pad sites (normal force inside the pad, without knowing what was touched), a `force` / `torque` sensor at the wrist, jaw position and actuator force, arm joint torques; grasp made = both jaws stopped at the flange width with pad force on both; slip = jaw or wrist load drift and the arm's position relative to the latched ring estimate; unintended touch = a wrist or joint load step outside the grasp, or a touch sensor on the bus or arm hull. Scoring still reads the labelled contacts (scoring reads the truth by design).
- **(c) Mixed:** (b) for the grasp and slip logic, (a) for the early-abort touch detector until the hull touch sensors are modelled.

**Chosen: (b)** (user, 19 Nov), because it closes the last contact-related truth input and makes P6 plan step 4 (grip-slip monitor on own sensing) the same work, done once. Cost: a few sensors in the scene and thresholds tuned on the grasp-boundary map (P7.5); the risk is noisy loads during the close making the grasp-made test chatter (two-stage close helps).

## 10. Risks

Contact simulation fidelity (a thin flange, soft contact, jaw seating); no published ring load limit; the grasp point now chosen on the estimate (a wrong roll estimate puts the jaws on the wrong place: the ring is symmetric, so this only matters near other parts on the face); capture of a tumbling unprepared client has not flown; arm torques while following the tumble before the grasp (e.Deorbit concern); a spin axis across the ring axis is the hard case in the e.Deorbit studies; wall time (contact needs smaller steps near the grasp).

## 11. What carries over and what changes (inventory, 19 Nov)

Read from the current scene (`p3_capture/p4_espa_arm2_cam_scene.xml` with the rev-1 servicer visuals of `rev1_scene.servicer_text`) and the P3-P6 chain. Keep = unchanged; adapt = same code, new data or parameters; replace = new code for the same job; new = no counterpart today.

### 11.1 Servicer bus and its sensors: keep

| Item | Today | P7 |
|---|---|---|
| Bus | 0.5 x 0.6 x 0.9 m box, 133.55 kg; servicer 150 kg with the arm (ESPA class) | keep |
| Thrusters, wheels | 0.75 N per axis, 3 N m per axis | keep |
| bus_cam | 512 px, 40 deg, on the +x face | keep |
| bus_lidar | 128 x 128 flash, 45 deg, 60 m, on a shared optical bench with bus_cam (10 mm / 1 deg mount error) | keep: it is the ring sensor (circle fit, section 5) |
| Own state (star tracker / IMU class), orbit model, rev-1 visuals | as P6 | keep |
| Bus collision | the bus geom does NOT collide with the client today (only the arm does, class arm_col); the monitors keep it away | **adapt: make the bus collide with the client**, so a bus strike near the ring counts in scoring |

### 11.2 Arm: keep, with one structural change

| Item | Today | P7 |
|---|---|---|
| Kinematics | 6 revolute joints (shoulder yaw, pitch, elbow, wrist pitch, yaw, roll); upper link 0.55 m, forearm 0.45 m, grip point 0.13 m past the wrist roll | keep; check in P7.2 that the ring section (0.31 m off the face centre) is reachable from the pre-grasp station with margin, else move the station, not the arm |
| Actuators | joint torque motors, 60 / 60 / 40 / 12 / 12 / 12 N m; armature, friction after torque-sensor compensation, 10 deg/s joint speed flag; mass layout (arm 16.55 kg) | keep |
| Control | joint torque / impedance control from P3-P5, compliant wrist during the close, stow and abort moves | keep (re-tune gains only if the grasp-boundary map asks) |
| Wrist structure | palm, jaws and cameras are all in body `wrist3` | **adapt: insert a fixed body `ft` between `wrist3` and the gripper** so a MuJoCo force / torque sensor can measure the gripper load (a sensor reads the force between a body and its parent); same masses and kinematics, checked equal like the rev-1 servicer |
| Arm skin | arm geoms collide with the client; contacts are known only from the labelled list | **new: touch sites on the arm links** (D5) |

### 11.3 Gripper: keep the drive, change the fingers

| Item | Today | P7 |
|---|---|---|
| Drive | two parallel slide jaws, position servos (kp 400, kv 20, 30 N force limit), stroke giving up to about 0.17 m opening | keep the drive; the ring wall (about 20-40 mm) is well inside the stroke |
| Pads | flat 90 mm deep x 100 mm wide x 16 mm, both faces close on the bar | **change: pads at most about 45 mm deep** (the ring stands only about 60 mm proud of the face, so today's pads would hit the face before closing), slightly curved or V-faced to centre on the ring wall (a 100 mm pad on the 0.305 m radius has about 4 mm of sag) |
| Palm | box face behind the jaws | **change: the palm becomes a datum** that seats on the ring's top (the separation face), the third contact as on the NRL / Northrop ring gripper |
| Close sequence | one squeeze to a fixed command | **change: two stages**: close to light contact, check the sensors (grasp made), then clamp |
| Grip point | `grip_site` 0.13 m past the wrist roll, between the pads | adapt to the new pad mid-depth |
| wrist_cam (75 deg) and palm_cam (90 deg) with the palm light | used by the P4/P5 bar networks | keep the cameras (re-check their view of the flange at the new pad depth); the networks change (11.5) |

### 11.4 Sensors to add (D5, option b)

| Sensor | MuJoCo element | Used for |
|---|---|---|
| Pad touch, each jaw, and the palm face | `touch` on a site box over each pad face and the palm face (normal force inside the box, without knowing what touched it) | grasp made, pad lost |
| Wrist force / torque | `force` + `torque` at a site on the new `ft` body | contact onset during the reach, clamp load, slip (load drift in the hold), the load limit |
| Jaw position and force | `jointpos` on grip_u / grip_l, `actuatorfrc` on their servos | grasp made (both jaws stopped near the expected wall thickness), slip (jaw creep) |
| Arm joint torques | `actuatorfrc` on tau1-tau6 (an external-torque estimate from the arm model later, optional) | arm load limit, unexpected load during the reach |
| Arm and bus skin | `touch` sites on the arm links and the bus front face | unintended contact (abort) |

Noise, bias and rate are added in Python (MuJoCo's sensor `noise` attribute is not applied by the simulator), from datasheet-class numbers recorded as assumptions. Thresholds are tuned on the grasp-boundary map (P7.5), never on the episode seeds. **Scoring still reads the labelled contact list** (success, collisions): scoring reads the truth by design.

### 11.5 Software chain, by phase

| Phase / function | Today | P7 |
|---|---|---|
| Far leg (`nav/far_nav.py`), observation filters (`dyn_filter`), stop test (rate, settle), transfer onto the axis (P6 step 2), fly-in planner and speed limit, station keeping | P6 | **keep** |
| Safety monitors (predicted clearance, keep-out, attitude innovation, thrust residual), abort back-off on the estimate | P5 / P6 | keep; **adapt** the client geometry they use (boxes and radius from the model: ring and nozzle added, bar removed) |
| Lidar box fit (`nav/lidar_nav`, `model_from` reads the client's box geoms) | box + bar + posts | **adapt**: picks up the ring and nozzle boxes automatically; check the fit still converges |
| End resolution (which end of the 1.5 m box is which): bar flag, `--bar-chk`, `--bar-occl`, `--end-flip`, `SYM_BAR` | from the bar | **replace**: from the ring and nozzle (lidar fit residual of the two ends, camera ellipse); the symmetry left drops from 8 box copies to 4 (turns of 90 deg about the ring axis, since the 1.0 x 1.0 face is square) instead of the bar's 2. Harmless here: the ring is round and this box's inertia is the same in all 4, but other face features would break it on a real client |
| Corner network (`nets/ckf`, 8 corners + 2 bar ends) | rev-1 client | **adapt**: drop the bar-end keypoints (corners only, the lidar resolves the end), retrain on the new client only if the corners degrade (CPU / short GPU runs) |
| Close-range pose: `pose_v3` bar network + gated palm network v2 (`PalmPoseEst`, latched bar estimate) | bar-specific | **replace**: latched lidar ring fit (centre, axis) plus a classical ring-edge fit in the wrist camera for the last centimetres; a learned detector only if needed |
| Grasp target in the reach and approach | bar site pose from the estimate | **adapt**: ring section frame at the chosen azimuth (approach along the ring axis, jaws closing radially) |
| Grasp-point choice | fixed (the bar) | **new**: azimuth on the ring chosen when the fly-in is planned (access, view, clear of other parts) |
| Grasp made | both pads on the bar, from the labelled contacts (`grasp_sim.run` after `close_s`) | **replace**: sensors (11.4), two-stage close |
| Grip-slip monitor in the hold | true bar vs grip (`safety.py:210`) and pad contacts | **replace**: jaw creep, pad force loss, wrist load drift (this is also P6 plan step 4) |
| Unintended contact | any non-pad-on-bar contact from the labelled list | **replace** for the abort decision (skin touch, wrist / joint load steps); the labelled list stays for scoring |
| Post-capture detumble, identification, stack move, release / retreat | P6 steps 5-6 | keep (the grip point only changes the arm's hold pose) |

**Frozen-code note:** `grasp_sim.run` (P3, frozen) uses `contacts()` both to decide (grasp made, slip) and to score (success, both pads at the end). P7 must split the two in a subclass: decisions on the sensors, scoring on the labelled contacts. The P5 / P6 pattern (subclass, monitor wrappers) covers it; no P3 file is edited.

### 11.6 Client model and data

| Item | P7 |
|---|---|
| Client | new rev-2 model: rev-1 box and visuals, bar and posts removed, ring (box polygon) and nozzle (stacked cylinders) on one end face, mass properties recomputed |
| Scene copies | as `rev1_scene.select` (`--client rev2`), with the rev-1 servicer and the P7 gripper |
| Training data | `gen_ckp` (corner network) re-run on the rev-2 client only if needed; no other network data |
| Seeds and splits | the P6 tune / test seed bases, so P6 and P7 runs pair up seed by seed |

## 12. Servicer rev 2: model and physics check (20 Nov, D7)

**Built.** `models/servicer_rev_2/build_servicer_rev2.py` (FreeCAD; models/ untracked, regenerate) makes the bus
from 61 parts with datasheet-class masses x 1.2 growth allowance: **bus 180.0 kg, servicer 196.5 kg** with
the unchanged arm (design point 200). `servicer/make_scene_rev2.py` writes `models/servicer_rev_2/
servicer_rev2_scene.xml`: the P4 camera scene with the rev-2 bus on `chaser` (explicit inertial from the parts,
bus box colliding with the client as class `bus_col`, 16 nozzle sites + site-transmission motors `thr_NN`,
4 wheel sites) and `client_p7` as `target`; qpos layout unchanged. `servicer/actuation.py`: thruster
allocation (bounded least squares), max pure wrench (LP), on/off pulses with 20 ms minimum on-time, +-5 %
scale and 1 deg misalignment per nozzle, propellant count; wheel pyramid model (torque and momentum limits,
gyroscopic term, saturation). The scene keeps the P3-P6 ideal free-joint motors so the chain still loads;
wiring the new actuation into the chain is the next step.

**Layout (changed during the check and the render review with the user):**
- Thrusters: **8 corner clusters** on brackets at the corners of the +X and -X faces, each one X-force nozzle
  (firing out of the face, canted outward: **50 deg on the client face**, 30 deg aft) and one side nozzle along
  Y or Z (user choice 20 Nov from `checks/thruster_trade.py`: equal forces, better pitch / yaw couples and fuel
  than the first fix, 8 mounting points instead of 12; 12- and 8-nozzle layouts lose control or most couple
  authority after one failure). History: mid-depth edge pods put the X nozzles inside the bus firing along its
  side (render review); a 30 deg cant left the client inside the 30 deg plume keep-out at every range.
- Wheels: pyramid about X with beta 70 deg (more momentum about Y and Z, where the grasp lever puts it).
- Sensors at the +X face (lidar window and camera barrel at the scene's bus_lidar / bus_cam); 15 in
  separation ring (50 mm) on the aft face.

**Physics check** (`checks/phys_rev2.py`, `runs/phys_rev2.json`):

| Check | Result |
|---|---|
| Mass | servicer 196.5 kg; CoM 2-5 cm forward of the bus centre; I (ready) 19 / 28 / 21 kg m^2 |
| Thrust, pure force per axis | +Fx 3.46, -Fx 2.57, Fy 3.51, Fz 3.58 N (17.6 mm/s^2 vs ~3 needed); worst single failure >= 1.44 N |
| Thrust, pure couples | Mx 1.53, My 1.17, Mz 0.73 N m; worst single failure >= 0.30 N m |
| About the stack CoM (1.41 m ahead) | Fx 3.46 / 2.57 N; lateral only 0.52 (Y) / 0.72 (Z) N: move the stack along X (turn first) |
| Flown in MuJoCo | every pure wrench gives the predicted velocity / rate change (nozzle signs and sites correct) |
| Plume vs client (any tumble phase, bounding sphere) | no nozzle within 30 deg: closest plume edge 46 / 41 / 36 deg at 8 / 3 / 1.8 m |
| Plume vs own servicer (ray fan, 15 deg core) | no self-impingement, stowed or ready; none on the arm |
| Wheels | 22.6 N m s and 0.38 N m about Y / Z, 16.4 / 0.27 about X; one failed: 11.3 about Y / Z. Detumble demand 16 / 19 / 29 N m s at 1.8 / 2.0 / 2.5 m CoM separation: wheels to ~2.2 m, thrusters above |
| Wheel model | total angular momentum (body + wheels) conserved to 0.0005 N m s over 30 s of commanded torque |
| Propellant | 5.4 kg (with allowance), Isp 220 s: 60 m/s |
| **ESPA envelope** | **fails: stowed depth 0.86 m vs 0.71** (bus + ring 0.595 m). The arm's planar shoulder-pitch / elbow chain cannot fold flat: the forearm folds back onto the upper arm (elbow range +-175 deg) and the gripper stands off the face. Trials with an elbow side offset (0.11 m) or a shoulder base axis along X did not get below 0.78-0.81 m. **User (20 Nov): redesign the stow in P7.2 together with the ring tool** (its size sets the stow; ESPA standard kept) |

**In the chain** (`servicer/rev2_bus.py`, `mission/p7_batch.py`, `scripts/run_p7_batch.sh`): the P3-P6
controllers keep writing the ideal commands (world force clipped to 1 N per axis = `f_max`, body torque);
`rev2_bus.install()` wraps `mjo_step` in grasp_sim / stack_sim and, on a rev-2 scene, replaces them each
physics step with the nozzle allocation (per control step, about the servicer CoM; stack phases about the
chaser origin, as the P3 stack model), on/off pulses with a 50 ms minimum on-time (impulse accumulation),
per-nozzle +-5 % scale / 1 deg misalignment (as a correction wrench), and the wheel pyramid model (grasp
phases; the stack phases keep the P3 wheel model for now). The ideal commands are restored after each
step, so the controllers and filters see what they commanded. The thruster FDI (P3 `safety.py`, 0.04 N
over 1 s) is given the nominal force of the valves actually opened (on-board data); with the continuous
command it tripped `thr_resid` within 1 s. Scene with the P6 bar client: `.case_srv2_rev1_p4_espa_arm2_cam_scene.xml`
(`make_scene_rev2.build("bar")`). Test (P3 truth controller, seed 5): captured, min clearance 0.304 m and
push 0.29 mm/s (old bus: 0.304 m, 0.31 mm/s), 15.5 g propellant, 662 pulses; wall time about 2x (allocation).

**Full-mission smoke on rev 2** (`runs/p7r2_smoke`, seed 5000000, P6 `$CFG`, 20 Nov): **complete**: captured
(min clearance 0.300 m; P6 smoke 0.301), push 0.30 mm/s (0.41), client 386 kg identified to 0.9 % (0.09), final
position 3.0 cm / attitude 0.15 deg (3.1 cm / 0.06), release without contact, separation 5.15 m (5.15); 4138
pulses, 96 g propellant to the end of the hold, 186 g for the whole mission; wheels at most 4 % full (no unloading);
wall 551 s (479). Own-state inputs of the hook (servicer attitude and rate, servicer CoM from its own mass
model and joint angles, valve states) are the star-tracker / IMU class own state P6 already uses; no client
truth. Open: rev-2 wheel limits in the stack phases (P3 wheel model there), allocation about the estimated
stack CoM, propellant mass depletion, a 25-seed run, then P7.2 (ring tool and stow).

**CAD audit (20 Nov, user asked for a recheck)**: interference of every part pair in FreeCAD, containment of the
internal parts in the shell cavity, sensors against the scene cameras (`checks/sensor_view_rev2.py`: segmentation
renders), collision geometry against the visual model. Found and fixed:
- battery inside the propellant tank (2450 cm^3), wheel 3 through the deck and the IMU, wheel 2 through the
  transponder, wheel electronics and optical bench through the side wall, lidar and camera boxes overlapping, the
  harness a 30 cm solid cube through half the parts. New layout: deck at z = -120 m m; lower bay tank, battery,
  power unit, transponder, propellant lines; upper bay wheels (cluster raised to z = 0.13 m), IMU, avionics, wheel
  electronics; sensors behind the +X face; harness as looms on the inner walls. Remaining overlaps < 1 cm^3, all
  mounts (nozzles in their brackets, lens in its barrel); nothing outside the cavity.
- panel openings cut to the shape of the parts passing through (lidar head, camera barrel, star-tracker heads), so
  no camera looks into the bus.
- collision boxes added for everything outside the bus box (8 thruster clusters with their nozzles, arm base
  plate, launch locks, sensor hood, star-tracker baffles, separation ring).
- sensors: bus_cam and bus_lidar see no bus part, stowed or ready (as P6); wrist / palm cameras see the bus face
  only when stowed (as P6). An apparent internal part in the palm view was the grip_site marker (segmentation
  ids: sites and geoms share numbers; the check now filters by object type).
Mass properties after the re-layout: bus 180.0 kg, CoM (+1.0, -0.1, -1.5) cm, I (ready) 21 / 30 / 21 kg m^2;
`phys_rev2` re-run: all checks as before (forces 2.6-3.5 N, couples 0.73-1.53 N m, worst single failure
>= 1.59 N / 0.30 N m, plume clear, wheels unchanged). The full-mission smoke above ran on the layout before
this audit (inertia ~8 % lower); the 25-seed run will use the audited model.

**25-seed run on rev 2** (`runs/p7r2_25`, tune seeds 5000000-24, P6 `$CFG`, model after the CAD audit, 20 Nov):
**0 collisions; 19/25 captured; 12 missions complete**; 6 safe approach aborts (no_converge 3, rate_anom 2,
lidar_lock 1); after capture 6 grip aborts by the post-capture grip monitor (pad force loss: detumble 4,
identification 2; re-clamp and stop, safe); 1 release contact (5000017: the release touched the client). Completed
missions: final position <= 5.7 cm, attitude <= 0.49 deg, move target <= 5.1 cm, separation >= 5.14 m. Wheels
never saturated (fill <= 0.46), no unloading; propellant 0.11-0.36 kg per mission (60 m/s budget 5.4 kg). Not
directly comparable with `p63ap_net25` (21/25): here each seed's client is the E3 case (200-400 kg), as in
p6_batch; the P6 25-seed full-mission run was stopped, so there is no P6 baseline for the post-capture outcomes.
Open (next session): why 6 / 19 captured stacks lose pad force in detumble / id on rev 2 (heavier servicer,
pulsed thrust in the stack phase, P3 wheel model there) and the release contact on 5000017; the latched
estimate error reached 1.3 m on one release.

**Sensor-corner fix (after the run, user render review):** lidar window and its hood opening ended above the
camera barrel's bottom (overlap 490 mm^3) -> window and opening z 202..264, 8 mm under the barrel; square lens
plate through the round barrel -> disc; window now fills its opening; hood camera opening r 32.5; side nozzles
stand 4 mm further off the faces (one sank 2 mm into the panel); hood top lowered to 20 mm under the cluster;
star-tracker openings cut to the tilted head shape. Remaining part overlaps: nozzles seated in their brackets
only. Rebuilt; phys_rev2 and sensor_view_rev2 re-run: unchanged (forces 2.6-3.5 N, couples 0.74-1.53 N m, worst
single failure >= 1.57 N / 0.31 N m, plume clear 36 deg at 1.8 m, no self-impingement; bus_cam / bus_lidar see
no bus). The 25-seed run used the model before this fix (side nozzles 4 mm closer; visual parts).

**Post-capture investigation (21 Nov).** Tools: `mission/p7_batch.py` switches `P7_BRANCH` (the episode forks at
the arm brakes; the copy runs the post-capture phases and the release from the identical state, either on the
ideal P3-P6 motors (`ideal`) or with the nozzles but the Project 3 stack wheel limits (`p3wheels`)),
`P7_TRACE` (per-step post-capture trace: pad forces, commands, rates) and a release monitor (largest client
velocity change in one physics substep); `scripts/diag_branch.sh`; `checks/branch_compare.py`;
`checks/post_bench_rev2.py` (P3 grasp, no fly-in: did not reproduce the batch aborts).
- `runs/br25` (25 seeds, rebuilt scene, rev 2 vs ideal from the brakes): **0 collisions; 17/25 captured; 13
  complete**; 7 safe approach aborts (the rebuilt sensor corner changes the camera images, so the approach does
  not replay the 20 Nov run seed for seed: 5000004 / 05 now complete, 5000001 / 02 / 17 abort before capture);
  3 grip aborts in detumble (5000008 / 16 / 18; client retained, re-clamped, safe), 1 release contact (5000021).
  **On every captured seed the ideal-motor copy has the same outcome** (same aborts at the same time, same
  release contact, pad force minima within a few %): the rev-2 nozzles and wheel model are not the cause.
- **Stack wheel limits:** the stack phases still used the Project 3 wheel model (1 N m, 30 N m s per axis),
  which rev 2 cannot give. `rev2_bus.install` now sets it to the box the rev-2 pyramid can give on all three axes
  at once (`stack_wheel_box`: 0.137 / 0.188 / 0.188 N m, 8.2 / 11.3 / 11.3 N m s; rw_limit = 0.137, the scalar
  maneuver3d scales by), the Project 3 unloading couple (0.5 N m) above that. `runs/sw3` (the three abort seeds,
  rev-2 limits vs the copy on the Project 3 limits): detumble torque 0.2 instead of 1.1-1.2 N m, **the aborts
  stay** (pad force still falls ~1 N/s to 15 % and trips). So it is not the detumble load either.
- **Cause: the bar is not seated square when the jaws are pinned.** Jaw gap at the brakes (`pc_gap0_mm`, own
  encoders): aborts 14.97 / 15.48 / 15.75 mm, all 14 others 15.90-16.09 mm (bar 16 mm); the closest complete
  (5000013, 15.90 mm) fell to 0.24 of its grasp force. Pad force then decays almost linearly in time whatever
  the torque or spin (bar squeezed out of pinned jaws that cannot follow; every abort has hold shear 0.76-0.82,
  the bar bearing on the pad edges). The monitor catches it and the client is kept in all three. Not established
  whether it is more frequent than on rev 1 (no P6 post-capture baseline). Candidate fix (not done): a seat
  check before the brakes from the jaw encoders (gap below bar - 0.15 mm -> open, re-seat, re-squeeze, or stop).
- **Release:** the demo release (post_own.release, P6) pushes the client without a detected contact on some
  seeds: substep velocity jumps up to 1.3-1.5 mm/s (5000024: client 14-16 mm/s at the end, latched estimate
  1.3 m off; 5000005: 8 mm/s, end separation 4.5 m), same on the ideal motors. `unintended_hits` samples
  contacts only at the 100 Hz control steps, so short impacts during the jaw opening / retract are missed. Not
  rev 2 (the P6 bench saw 4 mm/s on 5002). Candidate fixes: contact check every substep; retract only after the
  pads are clear of the bar (jaw encoders), slower opening; lidar tracking of the client in the retreat.
- **Bar placement (truth diagnostic at the brakes, `runs/seat6`):** bar 4-9 mm towards the pad tips and turned
  -1.2..+0.7 deg about its axis in the aborts, the same (5.5-6.5 mm, -1.2..+0.1 deg) in the completes; the pads
  reach +-45 mm. So the bar is not shallow: only the jaw closure differs (up to 1 mm further on the same 9 N), a
  different contact pattern (edge contacts sinking deeper), not a geometric wedge.
- **Stack wheel limits** (`rev2_bus.STACK_WHEELS`, now off by default, `P7_STACK_WHEELS=1`): with the rev-2
  limits the 3D move misses its 0.5 deg attitude criterion (0.61-0.74 deg on 3 of 3 seeds, `runs/seat6`): the
  rev-2 wheels (0.14-0.19 N m on all axes at once) are too weak for the move as specified. Open decision.
- **Seat check** (`servicer/seat_check.py`, `P7_SEAT=1`, off by default): jaw closure from the encoders before
  the brakes; below 15.85 mm open to 3 mm clearance, move the gripper deeper by the pad centre-of-pressure offset,
  squeeze again (2 tries), else stop before any post-capture load (`grip_unseated`, re-clamp, client kept).
  `runs/seatfix3`: 5000018 re-seated in one try (15.75 -> 15.97 mm) and completed; 5000008 / 16 improved
  (-> 15.63 / 15.61 mm) but stayed below, stopped safely; no contact during the re-seat. The 25-seed run was
  stopped (user asked whether this was over-optimising; it was: the aborts were already safe and the threshold
  came from these seeds).
  **Decision 21 Nov (user): stop here, move on to P7.2.** Kept as optional code, not in the baseline.
- **Open items carried to later:** (1) rev-2 wheels vs the move's attitude criterion; (2) the release contact
  check samples at 100 Hz and misses short contacts (client pushed up to 16 mm/s); (3) unseated grasps abort
  safely after capture (3/17 in `runs/br25`).

## 13. P7.2: ring tool and arm stow (plan, 21 Nov)

Ring on client_p7 (models/clients/build_client_p7.py): 610 mm OD, 22 mm wall (r 283-305 mm), 60 mm proud of
the +X face; clamp lip at the top outer edge, 12 mm overhang (r 305-317) x 15 mm thick, so a 45 mm deep pocket
behind it. Tool topology from D7.5 (US 10,414,053 layout): **palm datum on the ring top (the separation
plane), inboard jaw on the inner wall, outboard hook jaw that closes radially under the lip and then draws the
ring onto the palm**; non-backdrivable lead-screw drives (position held unpowered); hard faces, mu 0.4 nominal /
0.2 stress / 0.8 jam check; pad depth into the flange <= 45 mm.
1. **Tool CAD** (FreeCAD, `models/servicer_rev_2/build_ring_tool.py`): palm, inboard jaw, hook jaw, two drives,
   wrist flange, wrist camera; inside the 140 x 200 x 160 mm envelope the stow check assumed; mass from parts;
   open-state clearance to the ring section >= 15 mm radially (the P6 hand-over error was 22-45 mm, so the open
   jaws plus the compliant wrist must take ~20 mm, checked in P7.5).
2. **Stow redesign with the tool**: flat fold on the +X face, gate depth <= 710 mm from the port face, width <=
   610, no part intersections, joint ranges reach the fold; bench view of the ring at 1.8 m unblocked.
3. **MuJoCo tool**: bodies, joints (inboard / hook radial slides, hook draw slide), non-backdrivable drives,
   D5 sensors (pad touch, wrist force/torque, jaw position and drive force); in make_scene_rev2 ("p7" client).
4. **Gate (P7.2)**: on a fixed client, the arm placed at the grasp pose by script closes both stages; the grip
   holds test wrenches (stack-move class, x3) at mu 0.4 and 0.2 without slip; the sensors read sensibly (touch
   only on the jaws in contact, F/T matches the applied wrench). No closed-loop grasp yet (P7.4 / P7.5).

**Step 1 done (21 Nov): ring tool concept** (`models/servicer_rev_2/build_ring_tool.py` -> ring_tool.FCStd / .step /
.json, parts/tool_*.stl; renders ring_tool_iso / _side / _section.png). Palm (Al) on the ring top, inboard jaw
(Ti, radial slide, 20 mm stroke), outboard hook jaw (Ti, radial slide 25 mm + draw slide 6 mm), two lead-screw
drive housings on the palm back, wrist flange; palm face 0.13 m past the wrist roll (same grip point as the P3-P6
kinematics); fingers reach 40 mm past the palm (<= 45). **2.96 kg**, CoM 75 mm past the flange, envelope with
the jaws open 170 x 90 x 134 mm (x, along the ring, radial). Clearance checks against the client_p7 ring section
(seated): open: inboard jaw 20 mm, hook finger 27 mm, hook tip 16 mm from the lip; closed: palm and inboard jaw in
contact, hook 2 mm off; drawn: hook tip on the lip underside; no overlaps in any state. Not checked: the ring's
curvature over the 80 mm jaw width (2.7 mm sag, ignored), lead-screw sizing, cabling, the wrist camera's view.

**Step 2, stow with a recessed bay (21 Nov, `checks/stow_bay.py`, `runs/stow_bay.json`):** variant scenes with the
shoulder sunk into a vertical bay in the +X face (bay y -0.29..0.11, z +-0.44) and the stow search. Bay alone
(no kinematic change): 0.801 m (60 mm deep) / 0.786 m (100 mm), barely better than 0.86: the in-plane elbow
(range +-175 deg) cannot fold the forearm flat over the upper arm. Bay 100 mm + elbow side offset 0.12 m (forearm
folds beside the upper arm): 0.736 m with the offset to -y, 0.845 m to +y; still over 0.71. The ESPA allowance on
the +X face is 160 mm (face at x 0.25 m, limit 0.41 m). **Decision 21 Nov (user): keep the arm as it is (on its base plate); the stow is an open launch item (does
not fit standard ESPA: 0.86 m vs 0.71).**

**Step 3 done (21 Nov): ring tool in the MuJoCo scene** (`make_scene_rev2.put_tool`, "p7" scene only; the bar scene
keeps the two-jaw gripper). Joints keep the gripper's names and order (grip_u = hook, grip_l = inboard jaw), so
the qpos layout is unchanged. The hook is one slide along a cam path (25 mm in, 6 mm up: under the lip, then it
draws the ring onto the palm; tip clears the lip edge by 2.8 mm on the way); inboard jaw a radial slide. Class
`tool` (mu 0.4). Drives: stiff position servos as non-backdrivable lead screws (hook kp 30000 / 300 N, inboard
10000 / 150 N); grip_open = 0.018 / -0.012, squeeze 0.012 (4 mm past contact). Sensors (D5): touch on the palm,
inboard jaw, hook-on-wall and hook-on-lip; wrist force / torque; jaw positions and drive forces. Servicer 197.0 kg.
phys_rev2 not re-run with the tool (0.5 kg heavier than the gripper).

**Step 4, the P7.2 gate** (`checks/tool_gate.py`, `runs/tool_gate.json`; client pinned, servicer placed at the
ring's lowest section by IK, close, then the bus released and loaded: 5 N per axis, 3 N m per axis, each sign,
3 s each). First try: the inboard jaw pushed the lip edge onto the hook finger (2 mm gap) and the hook never
reached the lip (no draw, palm not seated). Fix: finger 6 mm off the lip edge, hook tip 0.5 mm off the wall
(the radial clamp is the tip on the wall), hook drive stiffer than the inboard one. Then:
| | mu 0.4 | mu 0.2 |
|---|---|---|
| closed: palm / inboard / hook-wall / hook-lip touch | 187 / 48 / 112 / 88 N | 508 / 47 / 410 / 238 N |
| seat error | 0.6 mm, -2.3 deg roll | 1.9 mm, -2.3 deg roll |
| slip under 5 N forces | 0.15-1.0 mm | 0.5-1.5 mm (Fx+ 3.2 mm: hook briefly off the lip) |
| slip under 3 N m torques | <= 1.8 mm / 3.3 deg | <= 2.1 mm / 5.4 deg |
| left after all loads | 0.46 mm / 1.1 deg | 0.73 mm / 2.3 deg |
Sensors read sensibly (touch rises on the faces that take the load and goes to 0 on the hook-lip face when a
torque pries the hook off; wrist force / torque follow the applied wrench). **Gate partly met:** both stages
close and seat, forces held within ~1 mm, but the 3 N m torques (x3 the ~1 N m post-capture torques) turn the
tool on the ring by 2-5 deg, part of it permanent. Not tuned further (options, not done: two hooks spaced along
the ring, a wider palm, contact stiffness check).

**FreeCAD assembly and shoulder fix (21 Nov, user).** `models/servicer_rev_2/build_servicer_rev2_arm.py` ->
servicer_rev2_arm.FCStd (bus parts + arm in the poses ready / stow / grasp from the MuJoCo scene via
`checks/export_arm_geoms.py` -> arm_geoms.json + the ring tool at the wrist), servicer_rev2_arm_ready.step,
renders servicer_rev2_arm_*.png. It showed the scene's shoulder yaw / pitch housings (r 60 mm, centred 50 mm out)
sinking 10 mm into the bus face, MLI and the 8 mm arm base plate. **User: move it out.** `make_scene_rev2`: arm_base
at x 0.268 (was 0.25, both rev-2 scenes), shoulder bracket lengthened to sit on the plate. Arm-bus overlap now 0
(ready and stow); stowed depth 0.860 m. tool_gate re-run: mu 0.4 as before (seat 0.5 mm, slip <= 1.8 mm / 3.3
deg); mu 0.2 slip <= 2.5 mm / 5.4 deg, and right after the close the palm and lip faces read 0 (the earlier run:
508 / 238 N): at mu 0.2 the draw does not hold reliably. **The P7 batch chain (bar scene) also has the moved
shoulder: smoke one episode before the next 25-seed run; earlier p7r2 / br25 results used x 0.25.**

**Shoulder redesign (21 Nov, user).** The P4 shoulder was two 120 mm drums crossing at one point, the yaw drum drawn
50 mm out from its own axis (j1, along bus z, at the face), on a 90 mm block: not buildable, and a real yaw drive
does not fit an axis 10 mm above the plate. Now (`make_scene_rev2.build`, both rev-2 scenes): yaw axis 55 mm above the
arm base plate (arm_base x 0.313, was 0.25 in the P4 scene: the whole arm 63 mm further out); on the bus a flange
and a clevis (two lugs at |z| 120-135 mm about the axis, shaft stubs to the drums); on the yaw body two coaxial
drums (r 42 mm, |z| 65-115 mm) and a yoke (side plates beside the pitch drum, plates to the drums); pitch drum and
everything beyond unchanged. Joint axes, order and ranges unchanged; masses as before (3.0 kg on the yaw body).
FreeCAD assembly re-built: no arm-bus overlap; the only part overlaps are the joins (shaft in lug and drum, yoke
plates). Stowed depth 0.905 m (the existing stow pose; it also has the upper arm through the forearm and the top
yaw drum: the P4 stow pose is not collision-free, never was). tool_gate re-run: mu 0.4 slip <= 1.8 mm / 3.0 deg,
mu 0.2 <= 3.0 mm / 5.1 deg; touch readings at the close swing between runs (hard contacts chatter).

**Wrist, jaw guides and stow pose (21 Nov, user review of the CAD).**
- Tool vs wrist: the tool frame origin is the cardan wrist centre, inside the arm's 55 mm wrist ball, so the tool's
  flange, housing and hook drive sat inside the ball. Now the flange is on the ball (x 55-62), the housing x 62-110,
  both drives side by side at x 85 (grip point unchanged at 130). No tool-wrist overlap.
- Floating jaws: open, the jaw carriages hung free. Added guides on the tool body: a hook rail along the cam path
  (the carriage stays 0.08 mm off it over the whole 26 mm stroke) and an inboard rail under the palm. Tool 2.31 kg.
- Stow pose (`checks/stow_rev2.py`, `runs/stow_rev2.json`): the P4 pose put the upper arm through the forearm and
  the top yaw drum (clearance 0, 0.922 m deep, bus camera 8.5 % blocked). New rev-2 stow pose (both rev-2 scenes,
  `make_scene_rev2.STOW_Q`): collision-free (>= 7 mm, the wrist ball to the tool), 0.881 m deep (CAD box 0.889 m),
  camera unblocked. A flatter fold the search also found reaches 0.694 m (inside ESPA) but with 20 mm interference:
  a lead for the stow item, not pursued.
- tool_gate re-run on the final scene: mu 0.4 seats with all four faces loaded; mu 0.2 seats this run; slip as
  before. Servicer 196.3 kg.
- **The P7 batch scene changed (shoulder, stow pose, tool): smoke one episode before any new batch run.**

## 14. P7.3: ring perception, first pass (21 Nov)

User 21 Nov: go on to P7.3 (ring perception, camera checks), grip as it is. Rev-2 smoke on the changed scene first
(`runs/p73_smoke`, seed 5000000): complete, captured, 0 collisions.

**Lidar ring fit** (`perception/ring_fit.py`, classical, the P6 box fit is the prior): (A) robust face plane around the
ring -> axis; (B) matched filter in the face plane: the centre maximising the height step between the ring band
(r 0.283-0.317) and the bands beside it (a free circle fit to "ring-top" points failed at 3 and 8 m: with 20-60 mm
range noise the face leaks into the top band, radius 327-341 mm); (C) Gauss-Newton of the centre and axis on the
signed distance to the ring section model (face U wall U lip), wide first gate then the tight design gates. Checks:
step height 30-96 mm, within 150 mm / 10 deg of the box fit; else rejected (the box fit stands). No roll (the ring is
symmetric; roll from the box fit).

**Bench** (`checks/ring_bench.py`, `runs/ring_bench.json`; 25 cases per range, bench seeds 7300000+, P6 sensor model:
lidar noise and dropout, bus_cam + bus_lidar on one bench with +-10 mm / +-1 deg, own bodies masked; prior = truth +
100 mm / 4 deg at 8 m, 50 mm / 2 deg at 3 and 1.8 m, then the P6 box fit, then the ring fit). Errors of the ring
centre / axis; "fit" in the true sensor frame, "total" in the frame the servicer believes (adds the bench mount error):

| range | ring ok | box fit, fit p50 / p95 | ring fit, fit p50 / p95 | ring axis p95 | ring fit, total p95 |
|---|---|---|---|---|---|
| 8 m | 24/25 (1 model-fit fail) | 27.5 / 48.7 mm | 17.2 / 48.6 mm | 4.0 deg (box 1.7) | 157 mm, 4.0 deg |
| 3 m | 25/25 | 22.5 / 53.3 mm | 7.0 / 11.0 mm | 1.6 deg | 64 mm, 2.4 deg |
| 1.8 m | 24/25 (1 rejected: box fit 198 mm off) | 21.0 / 67.7 mm | 3.7 / 6.1 mm | 0.8 deg | 44 mm, 1.5 deg |

Reading: at 1.8 and 3 m the ring fit is 4-10x tighter than the box fit and safe when the prior is bad (rejected). At
8 m use the ring for the centre and the box for the axis. The total error is the +-1 deg bench mount error, common to
every bus sensor (P6 lived with it); for the grasp the jaws need the ring in the arm's frame, so the close-range
correction belongs to an arm sensor at the reach (P7.4), not to the bus lidar.

**Camera checks** (`checks/ring_cam.py`, `runs/ring_cam.json`; same cases, random sun over the sphere): an
edge-support threshold on one estimate did not separate right from wrong (the ring is the face's grey, its top edges
are faint; its shadow and the arm give strong edges), so the check is a measurement: the in-plane shift of the
projected ring top edges that best fits the image edges (own bodies masked), agree if <= 15 mm (40 mm at 8 m).
- bus_cam at 1.8 m: of 18 lit cases, a 40 mm-wrong start is caught 18/18 and corrected to p50 6.6 mm; good lidar fits
  agree 14/18 (4 false alarms). Usable as a cross-check before the reach.
- bus_cam at 3 and 8 m: not usable (locks onto the ring's shadow ~55-60 mm off; false alarms 50-90%).
- wrist_cam at 1.8 m (arm ready): not usable (oblique, partial view; 16/17 disagree). The smoke's two cases had shown
  it removing the 26 mm mount error, so the wrist view at the reach pose (closer, square-on) is the P7.4 lead.
- dark faces (sun behind the face): no light, 6-13 of 25 per range: the lidar fit stands alone.

**Gate P7.3 (centre and axis small enough for P7.4): met for the lidar at 1.8 m (6 mm / 0.8 deg p95 in the sensor
frame), with the bench mount error (44 mm p95) left to the arm-side correction in P7.4.** Not tuned further
(explore, don't over-optimise). Open: the wrist-camera flange check at the reach pose; camera roll / azimuth (the
ring gives none); the 8 m axis.

## 15. Arm look cleanup (21 Nov, user: "a bit clanky"; visual only, same joints)

Audit (FreeCAD, `servicer_rev2_arm.FCStd`): the shoulder was an oversized sheet-metal yoke (~280 mm tall, two yaw
drums 180 mm apart) on an 80 mm arm; the joint housings were three unrelated shapes (120 mm drum, 100 mm drum,
110 mm ball); bare tubes sat in the drums' sides; three wrist axes in one 110 mm ball (not buildable); the planar
elbow cannot fold flat (the stow problem; kinematic, left open).

**Done (user: visual only, same joints;** `servicer/make_scene_rev2.py`, both scenes**):** one family of two-part
actuator modules (stator half on the inboard body, rotor half on the outboard body, dark seam): shoulder yaw
(r 50 mm) on a saddle on the bus under the pitch module, pitch (r 58) on a single-sided round boss bracket, elbow
(r 50), wrist pitch (r 42) on the forearm's side, yaw knuckle (r 38), roll (r 40) to the tool flange; link tubes
72 / 60 mm with end collars, offset along the joint axes (upper arm y -5 mm, forearm y -67.5 mm, as cobots). Joint
frames, axes, ranges and per-body masses unchanged (arm_base 3.0, upper 3.6, fore 2.95, wrist1-3 1.5 kg); the new
primitives are also the collision shapes (arm_col). Notes: the arm does not self-collide in the sim (as before);
physically the wrist yaw would be limited (the roll module reaches the pitch rotor beyond ~+-40 deg), and the
shoulder bracket would meet the bus face near +-90 deg yaw (as the old yoke). Stow pose still clear (8 mm, 0.884 m);
a 0.825 m fold exists with the wrist yaw at 2.56 rad (not adopted: still over ESPA 0.71 m, near the yaw limit).
FreeCAD assembly rebuilt from the scene (`export_arm_geoms.py` -> `build_servicer_rev2_arm.py`), no arm-bus overlap;
render `models/servicer_rev_2/arm_v2_ready.png`.

**Secure launch stow (21 Nov, user: the stowed arm should sit securely, elbow in a cradle, gripper placed to hold,
then: widen the elbow and place the wrist so the arms are parallel).** `checks/stow_secure.py` (search, or
`stow_secure.py '[q]'` to evaluate a pose), `servicer/make_scene_rev2.py` (STOW_Q, STOW_ELBOW, STOW_TOOL, restraints):
- Stow pose q = [0, -pi/2, pi, pi, 0, 0]: upper arm vertical against the +X face, elbow folded 180 deg so the
  forearm comes straight back down beside it, wrist folded 180 deg so the tool points up the channel between the
  two tubes. Depth **0.725 m** from the port face (P7.2: 0.884 m; ESPA 0.71 m, 15 mm over), arm clearance 4 mm,
  bus camera view 2.6% blocked, everything inside the face outline.
- What it took (geometry, plus two joint ranges): in a parallel fold the wrist point (kinematic) lands on the upper
  arm's axis 0.10 m above the shoulder (L1 0.55 - L2 0.45), so the upper tube moved to y +0.08 and the elbow module
  widened to y -0.115..+0.115 (upper tube on one side, forearm at y -0.08 on the other, an 89 mm channel between);
  shoulder pitch module re-centred under the upper tube (bracket now on -y); shoulder pitch r 58 -> 55 mm, knuckle
  38 -> 36, roll 40 -> 35 (9-10 mm to the shoulder when folded); **elbow and wrist pitch ranges +-3.05 / +-2.6 ->
  +-pi** (a 180 deg fold; the tubes are in different planes, the folded tool 5 mm off the forearm tube).
- Launch restraints (static, on the bus; visual / CAD only, no contact, mission dynamics unchanged): elbow cradle =
  saddle pedestal off the face (3 mm off the elbow drum) + one cheek on -y with a hold-down release mechanism
  (pin-puller class) on the elbow axis (no +y cheek: the bus sensor hood, y 0.115-0.285, is there); gripper stow
  fixture = an 80 mm section of the 610 mm ring profile (wall + lip) on a 28 mm box beam off the face between the
  tubes, braced by a web at the face (user: the first 12 mm cantilevered plate looked flimsy),
  the ring tool closed on it (palm, inboard jaw, hook under the lip, as on the client's ring). CAD check: no
  arm-bus overlap, restraint gaps 1-3 mm; renders `models/servicer_rev_2/arm_v2_stow_iso.png`, `_stow_face.png`,
  `arm_v2_ready.png`, `arm_v2_stow_rest.png`. Mission smoke on the parallel stow (`runs/par_smoke`, seed 5000000):
  complete, captured, 0 collisions (also after the module redraw, `runs/arm_smoke`, and the first secure stow).

**Deployment from the stow (21 Nov, user: "won't it get stuck while getting ready?").** It would have, at the gripper:
the tool closed on the ring-section fixture cannot leave it (the fixture stays between the jaws in x even with them
open; the folded wrist is at its closest to the shoulder so it cannot move down along the tool axis; a sideways
wrist-yaw slide hits the tubes). The ring-section rest was removed. Every restraint now releases outward (+x), the
way the arm unfolds: one tall restraint plate on -y (the elbow and wrist pitch modules both end at y -0.115) with
pin-puller HDRMs on the elbow axis and on the wrist pitch axis, the elbow saddle pedestal, and a snubber pad
behind the tool housing (face side, y 0.018-0.034, clear of the hook rail); the jaws carry no launch load.
`checks/deploy_check.py` steps the arm from stow to ready (HDRM pins pulled) and measures the distance to the
restraints, the bus and the arm itself (`runs/deploy_*.json`):
- **staged** (shoulder pitch + elbow swing the folded arm out, then the wrist unfolds, then the rest to ready):
  clear; the tool leaves the snubber at once (0 mm at stow, 3 mm one step on, growing), arm clearance >= 4 mm.
  (Two 0.0 mm readings bus box vs palm are a MuJoCo box-box distance artefact: the true gap there is ~106 mm.)
- **linear** joint interpolation: **stuck**: the tool first moves toward the face, 11 mm into the snubber post.
So the deployment needs the staged order. Open: the mission chain (frozen P3 grasp_sim) starts the arm at
servicer_stow and moves it, and on an abort drives it back to the stow, in straight joint-space lines; the restraints
are visual only in the sim, so the runs are unaffected, but on hardware both moves must use the staged order (a P7
override when the chain is next touched). CAD: no arm-bus overlap, only the pins in their module sockets and a 1.5 mm^3 snubber
preload; renders `arm_v2_stow_iso.png`, `_stow_face.png`, `_stow_side.png`.

## 16. Ring tool rework v1 -> v3 (21 Nov, user: "take a closer look at the actual gripper"; then "full rework + gate";
then "review it again: the errors, how it opens, the crooked structures")

**Audit of v1** (`models/servicer_rev_2/build_ring_tool_v1.py`): the clamping principle is sound (palm on the ring top,
hook under the lip, inboard jaw on the inner wall), but: straight 60 mm jaws on the r 294 ring (1.5 mm sag, 3x the
hook's 0.5 mm gap: line contacts the tool can pivot on); jaw carriages running through the solid palm (16 / 12 cm^3);
rails beside instead of under the carriages (one 85 mm out of the tool), drives not connected; no lead-ins.

**v2** curved jaws, palm between the fingers, guides and screws, chamfers; but the hook still on an inclined slide
(rail, screw and end plate leaning 13.5 deg, 60 mm out of the tool) and a thin core on a large flange.

**v3** (`build_ring_tool.py`, 1.71 kg; renders `ring_tool_v3_iso.png`, `ring_tool_v3_open.png`):
- both jaws move **purely radially** on straight carriages riding rails on the front face of a full-height back
  housing (motors inside), lead screws with end bearings; open = inboard 20 mm in, hook 25 mm out;
- the axial draw comes from a **20 deg wedge face on the hook tip** that meets the lip's outer edge (Marman-clamp
  principle: radial squeeze on an inclined face -> axial preload), so no inclined slide;
- jaw faces concentric with the ring (inboard r 283, hook tip r 305, finger r 323), palm a plate between the fingers,
  4 mm lead-in chamfers on the finger tips; the tip face lands on the wall as the wedge reaches the lip edge (a
  0.5 mm tip gap left the radial clamp to the wedge alone: worse);
- CAD checks: no part overlaps in the open / closed states; closed: inboard jaw on the wall, tip on the wall and the
  wedge on the lip edge, palm on the ring top; open: all jaws 14-31 mm off the ring.
Sim (`make_scene_rev2.tool_block`): hook joint radial; jaw faces as 3 pads along the arc; wedge pads; client ring
collision regenerated with 144 segments per wall / lip (P7_RING_N; 48 straight segments sagged 0.7 mm); touch sites
widened to the pad edges; joint equality locks lock_u / lock_l (off until switched on).

**Gate findings (tool_gate.py):**
1. **Every earlier gate run was rolled 2.3 deg**: the grasp needs wrist roll = pi and j6 stopped at 3.1, so the arm
   held the tool against the joint stop and the jaws met the ring on one side. Wrist roll range now +-pi.
2. **The drive model let loads open the jaws**: the jaws were soft position servos (inboard 10 N/mm), so 3 N m pushed
   them back; the spec'd drives are non-backdrivable lead screws. The gate now closes with the force-limited servo,
   then locks each jaw where it stalled (implicit joint constraint; P7_GATE_LOCK=0 for the old behaviour).
3. Results, max over +-5 N per axis and +-3 N m per axis, 3 s each (runs/tool_gate_v3c.json; v1 numbers carry the
   2.3 deg roll and the soft drives):

| tool / model | mu | forces: slip mm / deg | torques: slip mm / deg | jaw motion under load |
|---|---|---|---|---|
| v1, soft drives, rolled | 0.4 | 0.98 / 0.31 | 1.83 / 3.20 | (not recorded) |
| v1, soft drives, rolled | 0.2 | 3.13 / 1.41 | 2.12 / 5.09 | |
| v3, soft drives, seated square | 0.4 | 1.04 / 1.26 | 6.40 / 11.57 | |
| v3, locked drives, seated square | 0.4 | 0.98 / 0.51 | 1.38 / 2.64 | <= 0.7 mm |
| v3, locked drives, seated square | 0.2 | 0.94 / 0.18 | 0.80 / 1.76 | <= 0.7 mm |

Gate (grips the flange of a fixed client, holds the loads, sensors sensible): met for v3 with the lead-screw lock;
the residual 1-2.6 deg under 3 N m is recorded, not tuned. The mission (bar scene, two-jaw gripper) is untouched
by the tool; the wrist roll range is shared (smoke `runs/tool_smoke`). Stow with v3: 0.727 m, 4 mm clearance, the
snubber now behind the taller back housing.

## 17. P7.4: reach on the estimate, first pass (22 Nov; user: "go ahead" with the P7.4 plan)

**Bench, not the mission chain yet.** The P6 mission chain still runs the bar client (its grasp, end resolution and
close-range nets are bar-specific, section 11.5), so P7.4 was done as a bench in the "p7" scene
(`checks/reach_bench.py`, `runs/reach_bench.json`; bench seeds 7400000+, 25 cases). Per case:
- **station:** client free (300 kg, at rest); servicer at the tool_gate grasp station (bus centre on the ring axis
  1.0 m beyond the ring top) with a random roll about the ring axis and a fly-in error (<= 40 mm, <= 2 deg), held there
  (pinned: ideal station keeping); bench mount error +-10 mm / +-1 deg and lidar noise as ring_bench; random sun.
  The arm arrives in an **approach pose** (grip site 0.30 m above the design ring top): at the station the ready pose
  reaches ~0.3 m into the client.
- **sense:** bus_lidar -> box fit (prior truth + 50 mm / 2 deg, as ring_bench) -> ring fit = E0. At the station the
  box fit can lock 0.1-0.2 m off (the lidar sees the face close up and partly), and then the ring fit fails its prior
  check (5/25); the ring fit is retried from the prior (the filter's prediction) and then succeeds (5/5).
- **choose:** the ring section nearest the shoulder, >= 40 deg from the solar wing (client +Y from the box fit), first
  that the IK reaches at all three heights; the azimuth is kept after the camera correction (re-choosing jumped it).
- **check:** straight-line move (IK along the line, joints <= 10 deg/s) to 0.15 m above the estimated section; wrist
  camera with a **wrist lamp** (the headlight at the camera; the P4 lighting has none) and the ring_cam shift search on
  the visible arc (+-40 mm) -> E1. Gates: no verdict -> abort; correction > 40 mm -> abort (lidar and camera disagree).
- **reach:** straight line to 30 mm above E1, then a guarded descent along the axis at 10 mm/s that stops on any touch
  > 3 N (15 mm past the estimated top at most), arm tracking monitor (grip site vs command > 5 mm -> abort);
  press 2 mm, close, lead-screw lock, 1 s hold, then a hold check (3 N m about each client axis, 2 s each).
- scored with the truth only: seat against the true ring, touch sensors, contacts outside tool-on-ring, push, slip.

**Result: 25/25 grips, 0 collisions, 0 aborts.**

| quantity | p50 | p95 | max |
|---|---|---|---|
| E0 (lidar at the station), in-plane error | 14.0 mm | 25.5 mm | 28.8 mm |
| E1 (after the wrist check), in-plane error | 3.2 mm | 10.9 mm | 14.9 mm |
| radial error at the start of the descent | 2.9 mm | 8.8 mm | 11.8 mm |
| axial error of E0 / E1 | | | 12 mm (the descent stops on touch) |
| seat after the close (radial, fixed-client seat 1.2 mm) | | | 2.8 mm; height <= 0.3 mm, tilt <= 0.5 deg |
| hold check slip (3 N m) | | | 1.6 mm / 0.37 deg |
| client push after the close | 4.1 mm/s | | 8.1 mm/s |

Camera verdicts: 22 disagree (a correction > 6 mm, all used), 3 agree. The jaws self-centred from up to 11.8 mm
radial error, so the camera correction may be more than the jaws need; whether E0 alone suffices is a P7.5
boundary-map question (not tested here). Grip test: the ring held radially between the inboard jaw and the hook (its
face or its wedge on the lip) and axially on the palm or the wedge; a first scoring rule that wanted the palm and the
hook face both loaded called seated grips failures (either datum carries the closing load in v3).

**Scene change:** wrist roll j6 range +-pi -> +-270 deg (`make_scene_rev2`; a usual cable-wrap range): the ring grasp
needs roll = pi, exactly on a +-pi stop, so a limit-respecting IK could not hold it. Shared with the bar scene: smoke
the mission before any batch run.

**Gate P7.4 (reaches the flange on the estimate, no truth): met on the bench.** Not covered (recorded): the bus is
pinned (no station-keeping error during the reach, no reaction from the arm on the bus); the client is at rest (no
residual relative rate after synchronisation; P7.5); arm kinematics perfect (no joint offset / link error); one light
setting for the lamp; the mission-chain integration (fly-in to this station on the estimate, latched 1.8 m ring fit,
the sensor-based grasp decision in a grasp_sim subclass) is P7.6 work.

**Revision (22 Nov, later; user: check what the P7.4 changes mean for the CAD).** The wrist roll range has no CAD
effect. The wrist camera did: it was a bare viewpoint at tool (-30, 0, +140) mm with no hardware, and a real 40 mm
camera there sits **47 mm inside the bus in the launch stow** (tool +z points at the bus there); the lamp did not
exist either (the bench had used the render headlight). A 40 mm test box at six mounts (stow / ready / grasp): only
the inboard side (-z) clears all three. **User: inboard mount.**
- **Camera + LED lamp unit** (`build_ring_tool.py`, group "cam", 0.40 kg; scene geom `cam_unit`, light `wrist_lamp`):
  54 x 44 x 36 mm, bolted to the housing's inboard end (tool z -101..-65), camera at (83, 0, -76) mm. The housing end
  (z -64) set the stow envelope, so the unit adds 37 mm: **stow depth 0.727 -> 0.764 m** (ESPA 0.71; a bracket further
  out would have made it 0.823). Tool 1.71 -> 2.12 kg. CAD: no overlaps between moving and fixed tool parts, 0 arm-bus overlap in
  stow and ready (servicer_rev2_arm.FCStd rebuilt; views servicer_rev2_arm_stow_*.png, *_grasp_isometric.png,
  ring_tool_v3_cam_*.png). Staged deploy: closest pairs unchanged (the unit is on the side away from the snubber).
- **View:** looking radially OUT from the inboard side (the mirror of the old view) the camera sees ~60 edge points
  on the near arc and made 6/25 wrong corrections (20-30 mm; narrowing the search did not help); looking IN across
  the ring (as the old view) it sees the far arc (~40 deg, sampled at 0.5 deg: ~75 points), which fixes the radial
  direction through the grasp section, the one the jaws need; the tangential error stays (harmless).
- **Guards added** (a wrong correction pushed the free client at 26 mm/s with no touch pad loaded: the hook tip, 40
  mm past the palm, hit the lip while still moving to the old 30 mm pre-close point): pre-close height from the jaw
  tips (75 mm above the estimated top); contact guard on every sim step during the moves near the client and the
  descent (touch pad > 1 N or wrist force step > 2 N; a free 300 kg client is pushed along by a few N); only a palm
  touch starts the close, anything else backs off.

Result with the inboard camera (`runs/reach_bench_b.json`, 25 cases): **21 grips, 4 safe aborts (camera no verdict 1,
camera / lidar disagree 1, hook-tip touch 1, wrist force 1; client pushed 0 mm/s), 0 collisions.** Radial error at
the start of the descent p50 6.1 / p95 12.4 mm (max 24.4, caught); seats as before; hold slip <= 2.3 mm / 2.4 deg;
client push after the close <= 12 mm/s. The 25/25 above was with the impossible outboard camera and without the
guards. Open: whether the inboard camera beats the lidar alone radially (a no-camera run would tell).

**Lidar alone vs the inboard camera (22 Nov, later; user: no-camera run).** Same 25 cases, path and guards, the
wrist check skipped (`reach_bench.py --no-cam`, `runs/reach_bench_nocam.json`):

| reach on | grips | safe aborts | collisions | radial error at the descent p50 / p95 / max | hold slip max |
|---|---|---|---|---|---|
| lidar alone | 20 | 5 (wrist force 4, hook tip 1; client pushed 0 mm/s) | 0 | 6.5 / 17.8 / 21.5 mm | 1.8 mm / 2.8 deg |
| lidar + inboard camera | 21 | 4 | 0 | 6.1 / 11.1 / 24.4 mm | 2.3 mm / 2.4 deg |

The jaws' radial capture window is about 12 mm (every grip <= 13.6 mm, every abort >= 11.8 mm). The camera tightens
the radial tail (p95 17.8 -> 11.1 mm) but adds one grip net: it fixes 4 lidar aborts (seeds 0, 5, 8, 10) and adds 3
of its own (no verdict, disagree, one bad correction). Either way the guards turn a miss into a safe abort. Reading:
the lidar at the station (mount error +-1 deg) is the limit; the camera is a modest gain, kept; a wider capture
window (hook opening, lead-in chamfers) or a second look after a back-off would buy more than camera tuning. Recorded,
not tuned (explore, don't over-optimise); P7.5 maps the window properly (offset, angle, relative rate).

## 18. P7.5: grasp-boundary map and the close on a free client, first pass (22 Nov; user: "go to P7.5")

**Map** (`checks/grasp_map.py`, `runs/grasp_map.json`): the P7.4 reach and close (`reach_bench.reach_close`) on a
free, 300 kg client, on an estimate = truth + one imposed error (the map's input; the reach reads only the
estimate and its own sensors); bench station 7400001. Loads from the start of the descent.

**The close had to change first.** The P7.4 close was a step command on the jaw servos with a 2 mm palm press. On a
free client that hid two problems:
1. *Step command:* the hook tip hit the wall 1.6 mm deep in one step (tens of kN of solver force). A lead-screw jaw
   has a bounded speed: the close now ramps at V_JAW (10 mm/s for the faster jaw).
2. *At a bounded speed the free client gets away:* (a) at equal jaw speeds the inboard jaw (6 mm less travel) arrived
   0.6 s first and pushed the ring 11 mm off the seat: both jaws now ramp over the same time and meet the ring
   together; (b) the palm touch at 10 mm/s plus the 2 mm press kicked the client away at 6.7 mm/s; the palm lost
   contact in 0.4 s and the jaws closed on a ring 7.6 mm below it. Now: a slow zone (3 mm/s) through +-15 mm around the
   estimated top (the axial error), no press, and **palm force accommodation** during the close (the arm steps along
   the axis to keep ~5 N on the palm touch pad: it follows the drifting client; own sensing only).
With these, every jaw speed grips (5, 10, 20, 40 mm/s and the old step).

| imposed error | result | note |
|---|---|---|
| radial offset -20, -16 mm | safe abort (hook-tip touch) | |
| radial offset -12 ... +16 mm | **grip**, seat within 1.5 mm | capture window ~ -12 / +16 mm (inboard jaw opens 20, hook 26) |
| radial offset +20 mm | safe abort (inboard-jaw touch) | |
| tilt about the tangent +-2 deg | grip | (the tool leans radially) |
| tilt about the tangent +4 deg | grip, off seat (2.7 deg left) | caught by the seat check |
| tilt about the tangent -4 deg | safe abort (no palm touch) | |
| tilt about the radial 2, 4 deg | **no grip** (palm on one corner, palm / lip unloaded) | the weak axis: the 80 mm palm does not comply in roll |
| spin about the ring axis 0.5, 1, 2 deg/s | grip | axial clamp load falls with rate (1.45 -> 0.32 kN) |
| transverse rate 0.25, 0.5, 1 deg/s | safe abort (hook, wrist force, no contact) | the face tilts 2.5+ deg during the slow descent |
| dr 8 + tilt 2 + transverse 0.5 | safe abort | |
| dr -8 + tilt -2 + spin 1 | grip | |

0 collisions, client push at an abort 0-0.35 mm/s. Loads: largest single contact 37-280 N; palm / lip pads (the wedge's
axial clamp pair) 1.3-1.9 kN over the 60 mm jaw (25-30 N/mm; a clamp band's line load is of that order, but no
published lip rating was found: recorded, not gated); arm joint torque <= 34% of the limits.

**P7.4 reach re-run with the new close** (`runs/reach_bench_c.json`): the inboard camera's tangential shifts (up to
30 mm, poorly observed from the far arc) turned the local radial ~6 deg and skewed the jaws (2 closes with low clamp
load); only the camera's **radial** correction is now applied. Result: **23 grips, 2 safe aborts (camera no verdict,
wrist force), 0 collisions**; radial error at the descent p50 5.8 / p95 12.1 mm; hold slip <= 1.5 mm / 0.67 deg
(3 N m); client push after the close <= 9.7 mm/s; palm / lip peak 2.1 / 1.8 kN; arm torque <= 37%.

**Gate P7.5 (the region covers the measured close-range errors; loads under the limits): met for position (window
-12 / +16 mm vs radial p95 12.1 mm) and for spin about the ring axis (<= 2 deg/s); marginal for tilt (lidar axis
error p95 ~1.4 deg vs a failure at 2 deg about the radial); NOT met for transverse relative rate (aborts from
0.25 deg/s); arm loads under the limits; the lip load has no rating to check against.** Not tuned (explore, don't
over-optimise). Leads, recorded: a palm that complies in roll (or two half-palms), and tracking the ring face during
the descent (arm servo on the lidar / camera, or a faster descent once synchronised) for transverse rates; the
synchronisation residual itself (P6 chain on the ring client) is the number that decides the second. Next: P7.6
(the mission chain on the ring client: fly-in to the station, the grasp decision on the sensors, 25 seeds).

## 19. P7.6: ring mission harness, first pass (22 Nov; user: "ring mission harness")

**Scope (user choice):** not the P6 chain (its P3-P6 classes carry ~200 bar-specific references) but a new episode
driver, `mission/ring_mission.py`: the capture on own sensing from the 3 m hold to a closed grip, with a free,
spinning client (0-2 deg/s about a random body axis, torque-free: it nutates) and a free servicer on its rev-2
actuators (16 nozzles with allocation, 50 ms minimum on-time, scale / misalignment errors; 4-wheel pyramid with
unloading). The far leg, the 30 / 8 m holds and the synchronisation from the 3 m stop are P6's results (the hand-over
errors: <= 50 mm, 2 deg, 3 mm/s, 0.1 deg/s), not re-run; the post-capture phases and the release are P6's, not re-run.
- **Nav (`RelNav`):** bus_lidar at 1 Hz; box fit beyond 2.2 m, ring fit at every range, both from the filter's
  prediction (first fix: the hand-over prior); a 20 s window fitted with Euler's equations (design inertia +-5%,
  design COM + 10 mm) and a constant COM velocity. Three findings on the way: (1) a ring fix has no roll: fitting
  it freely drifted the roll 7 deg; (2) carrying the predicted roll as a measurement, or a window mixing box and ring
  fixes, fitted a 0.2-0.3 deg/s false rate (22 deg of attitude error by the reach): the rate is fitted only from
  box-fix windows; (3) a frozen rate missed the nutation (9/25 did not settle): the rate is carried by Euler's
  equations. Rate error at the reach now <= 0.16 deg/s (p50 ~0.05).
- **Flight:** body-fixed station on the estimated ring axis (3.2 -> 2.0 -> 1.0 m beyond the ring top at 20 mm/s,
  holds 20 / 15 s, settle to 15 mm / 1 deg); translation PD with the rigid-motion feed-forward, attitude PD on the
  wheels; aborts: a station error > 0.15 m / 5 deg lasting 5 s, no fix for 15 s.
- **Grasp:** the P7.4 / P7.5 reach and close (`reach_bench.reach_close`), the grasp point carried with the client's
  estimated motion (`track()`); arm tracking monitor 12 mm (5 mm tripped on the moving target's lag). The grip
  decision on the touch pads; then the thrusters stop and the stack drifts 5 s.
- **Wrist camera off in the mission** (`--cam` on): here the section is always on the side away from the wing and the
  camera's correction moved a good lidar estimate 10-20 mm inboard, past the capture window (with it: 10 / 25).

**Result (`runs/ring_mission25.json`, seeds 7600000-24, 5 workers): 18 / 25 captured (72%), 7 safe (6 aborts: guarded
descent 4 (wrist force 3, no contact 1), arm tracking 1, station attitude 1; 1 close not confirmed by the touch pads),
0 collisions**, no nozzle firing toward the client inside 3 m, propellant 27-86 g per episode (more at higher spin),
client push after the capture <= 5 mm/s, the grip held through the 5 s drift (seat change <= 0.06 mm). Failures: two
lidar radial errors outside the window (+21, -27 mm), one slow-spin attitude drift (6.5 deg, seed 21), three guarded
stops at moderate errors.

**Truth audit (`checks/ring_truth_audit.py`, `runs/ring_truth_audit.txt`, 2 episodes):** every truth read is setup /
the hand-over prior (the first fix), sensor simulation, scoring or a record (collision list, client push); none in the
control loop.

**Gate P7.6 (about 80% captured, the rest safe aborts, 0 collisions): nearly met (72%, 0 collisions, every failure
safe).** Not covered, recorded: the far leg / holds / synchronisation on the ring client (P6 results used), the
post-capture phases and the release, bus navigation ideal (own attitude and motion known), orbital relative dynamics
neglected (minutes at < 3 m), one run of 25 tune seeds (no test split yet). Leads: the station-range radial error
(the mount error; the camera needs a view not dominated by the wing side), the slow-spin roll drift in ring-only mode,
a rolling palm and face tracking from P7.5.

## 20. P7.6: learned wrist-camera ring net in the mission (22 Nov; user: "finetune the model first, retrain it for this", "learned wrist-cam ring net"; camera "to the bar spec ... conservative, just for testing")

- **Camera:** the bar-mission spec (75 deg fovy, 128 px, grey), the inboard camera + lamp unit of section 17
  (`reach_bench.WRIST_RES = 128`).
- **Data (`checks/gen_ringcam.py`, seeds 7800000+, not a mission or test range):** 18000 images at the pre-reach pose
  (0.15 m) on a perturbed estimate (centre radial / tangential +-30 mm, axial +-15 mm, axis tilt <= 1.5 deg, azimuth
  +-40 deg from the shoulder side), bench station, sun, lamp 0.5-1.0, gain / noise; label = true minus estimated ring
  centre in the estimated section frame. `runs/ringcam_data.npz` (295 MB, untracked).
- **Net (`nets/ringcam_net.py`):** 5 strided conv blocks + FC, mean and log variance per axis (Gaussian NLL),
  brightness / noise jitter; 30 epochs, 63 s on the GPU (one short run). Held-out 2000: |err| p50 0.30 / 0.24 /
  0.49 mm, p95 1.5 / 1.1 / 2.3 mm (radial / tangential / axial); within 2 sigma 92 / 95 / 84%.
- **Mission use (`ring_mission.py --cam net`):** at the pre-reach pose one image -> the radial correction only (as
  section 17: the tangential one skewed the jaws); abort if radial sigma > 3 mm. The net is loaded inside each pool
  worker with one thread (loaded before the fork, torch hung the workers; unpinned BLAS ran 9 cores per worker).

**Result (`runs/ring_mission25_net.json`, the same tune seeds 7600000-24): 20 / 25 captured (80%), 5 safe aborts,
0 collisions** (lidar only: 18 / 25). Radial error at the reach: lidar |E0| up to 27 mm, after the net |E1| <= 4.8 mm,
mean -2.2 mm (a small inboard bias, all but one negative). Gained: seeds 3, 12, 17 (lidar off by 9-27 mm); lost:
seed 2 (arm tracking 14 mm, a moving-target lag, radial fine). The 5 aborts: arm tracking 2 (2, 5), guarded descent
2 (1: wrist force with radial -3 mm; 6: no contact, tangential ~21 mm), station attitude 1 (21, before the camera).

**Recorded, not fixed:** the sigma head is nearly constant (~0.55 mm), so the 3 mm gate never fires: not yet a
safety check; the axial output reads -8 to -20 mm in the mission (outside its +-15 mm training range: the mission's
pre-reach geometry differs from the bench; unused); the -2 mm radial bias; tangential error uncorrected (seed 6).

**Gate P7.6 (about 80% captured, the rest safe, 0 collisions): met on the tune seeds (20 / 25).** Not covered: a
test split (fresh seeds, frozen code), the items of section 19.

## 21. P7.7 plan (proposal, 22 Nov; user: "can we do RL, with the arm gripper", "understand how the previous ones were done, research and plan first"): a residual policy for the reach and close

Research: `learning/RESEARCH_SERVICING.md`, "RL for the arm and gripper in P7". Not started; decisions marked (U)
are the user's.

**What the evidence says.**
- P1 / P2: RL matched or slightly beat a weak classical base, and lost to a better one; residuals helped only under
  uncertainty; an observation shift broke one; the shield carried safety. P4 agreed learning in perception, not
  contact control (U: this plan reopens that).
- The 5 mission aborts (section 20) are not yet a learning target: arm tracking (seeds 2, 5) at only 0.2 deg/s
  spin, cause unknown; wrist force off the pads (1, 1.4 deg/s); a miss with 21 mm in-plane / 12 mm axial error
  (6, perception); station attitude (21, flight). 25 seeds is too few to train against anyway.
- Where a policy could matter is the **capture envelope** of section 18: tilt along the ring of 2 deg fails, a
  transverse rate >= 0.25 deg/s aborts, the window is -12 / +16 mm radial. Those are timing / alignment problems
  during contact, the kind residuals have helped with (insertion, synchronisation).
- Budget: the bench reach-to-lock is ~3 s wall per core (contact steps at ~1.3k/s); off-policy residual RL
  (ResFiT-style, ~1e5-5e5 decisions) is 0.5-2 h on 5 CPU workers; PPO at 1e7-1e8 steps is out (no sustained GPU).

**Plan.**
- **R0, base first (classical, before any RL):** diagnose seeds 1, 2, 5 and 6 (tracking error source, the
  off-pad contact, the axial miss); fix what is a bug. P2's lesson: a better base removed most of what the residual
  had learned.
- **R1, env (`rl/reach_env.py`, CPU MuJoCo, no new framework):**
  - start at the pre-reach pose (0.15 m) from saved states of the mission chain (`mj_getState` + the RelNav state),
    with the client free and spinning / nutating, the bus held on its estimated station;
  - per episode sampled: spin 0-3 deg/s, transverse rate up to 0.5 deg/s, ring tilt up to 3 deg, estimate errors
    from the mission's E1 spread plus margin, friction 0.3-0.5, sensor noise levels including near zero (r2's
    lesson);
  - policy at 25 Hz; **action** = a bounded correction on the grip-site Cartesian target (+-5 mm/s translation,
    +-1 deg/s rotation) on top of `reach_close`'s own path, actor initialised near zero, so a = 0 is today's reach;
    the close itself stays classical first (option later: the policy picks the close moment);
  - **observation, own sensors only:** grip site relative to the estimated section, the estimated client rate,
    wrist force minus its baseline, the 4 touch pads, jaw positions, phase and time, the last action;
  - **reward:** sparse +1 for a lock with the seat in the window (scored on the truth, training only), a small
    time cost; constraints as terminations (CaT): any off-tool contact, pad force or wrist force above limits, client
    push above 5 mm/s; a safe abort ends the episode at 0, not a penalty, so the policy is not pushed to gamble;
  - the existing guards stay outside the policy as the shield.
- **R2, check:** a = 0 reproduces the scripted reach on the env distribution (as P2's residual check).
- **R3, measure the envelope:** the scripted reach swept over tilt, transverse rate, spin and estimate error on the
  env (small, ~25 per cell); the gate is set from this, before training.
- **R4, train:** TD3 residual as ResFiT (critic ensemble, layer norm, update-to-data 2-4, n-step 3, zero-initialised
  actor), ~2e5-5e5 decisions; 5 CPU workers collect, one CPU learner (small MLPs); checkpointed and resumable.
  Library (U): a small own TD3 (~250 lines, no env change), or stable-baselines3 added to the pixi env.
- **R5, evaluate:** env validation seeds (25 per cell) policy vs a = 0, same draws; then the mission's 25 tune seeds
  with the policy (no worse than 20 / 25, 0 collisions); the test split once, at the end.

**Gate (draft, fixed at R3):** inside the extended envelope captured clearly above the scripted reach at 0 collisions
and no rise in peak pad force or client push; nominal mission not worse.

**Risks:** the policy learning sim-contact tricks (check at timestep 0.001 and friction outside the training range);
an observation shift (randomise noise, no covariance inputs); a residual that cannot fix what the base lacks
(ResFiT's own limit); small evaluations are noisy (25 per cell, report intervals).

## 22. P7.7 R0: the reach aborts diagnosed and fixed classically (22 Nov; user: "yes, start R0")

`checks/diag_reach_aborts.py` (instrumented mission episodes: lidar fixes and the tracked grasp point's jump at each,
the arm tracking error per descent step, contacts and wrist force at the descent's start, the true seat at its end;
truth for the record only). The 4 arm-phase aborts of section 20 were not learning problems:
- **Seeds 1, 2, 5 (wrist-force trip / arm tracking 14 and 23 mm), one bug:** the pre-close move goes to the point
  tracked when the move starts; the tracked point moved on by 4-23 mm meanwhile (client motion, station and nav
  updates; lidar fixes moved it only 0.2-2 mm each), and the first descent command jumped the arm there in one 10 ms
  step (0.15-0.29 rad/s joint jerk, 2-2.5 N inertial wrist force, no contact). Fix (`reach_bench.reach_close`): the
  commanded grip point closes on the tracked one at <= 30 mm/s (`V_CATCH`), the descent waits until within 1 mm
  (`CATCH_MM`), safe abort if not caught up in 3 s; the tracking monitor measures against the commanded point. (The
  bench: a no-op, its target does not move.)
- **Seed 6 (no contact):** the true seat at the descent's end was 1.9 mm above the ring: the axial estimate error
  (11.6 mm at the reach, ~17 mm by the end) exceeded the 15 mm give-up depth. `OVER` 15 -> 25 mm. Then a lost grip in
  the drift (inboard pad 5.1 -> 3.4 N): a 21 mm tangential error turns the section's radial 3.9 deg and skews the
  jaws. The ring net's tangential output matched the true tangential error to ~2 mm on all 24 tune seeds that
  reached the reach, so the mission now applies it with the radial one (`ring_mission.py`; the shift search's
  tangential, section 17, had been the bad one). Its axial output is the true error with a -7 +- 3 mm offset in the
  mission (the bench-trained geometry differs; unused).

**Result (`runs/ring_mission25_r0.json`, the same tune seeds): 24 / 25 captured, 1 safe abort (seed 21, the station
attitude in flight, before the arm), 0 collisions** (section 20: 20 / 25). Peak contact median / max 168 / 327 N
(was 159 / 486), client push after the close median / max 2.5 / 6.2 mm/s (2.8 / 6.3), drift seat change <= 0.09
mm. Wall time per episode median 120 s (101 s: deeper descent, the catch-up).

**For the RL plan (section 21):** none of the tune-seed aborts is left in the arm phase, so the residual policy's
only target is the extended envelope (R3 first). The base got right before learning, as P2 taught.

**10 Hz lidar, one smoke (`--lidar-hz 10`, seed 4; user: sensors up to 10 Hz for experiment):** captured, rate error
at the reach 0.024 deg/s (1 Hz: 0.034), radial error after the net 2.2 mm (4.8); but 823 s wall per episode against
120 s: the RelNav window fit (20 s, now 200 fixes, refitted at every fix) dominates. A 25-seed run would take ~70 min
on 5 workers. Bounding it (a window of at most ~40 fixes spread over 20 s, or a fit every 1 s with the newest fix
for the prediction) is open; 1 Hz stays the default.

## 23. Multi-rate sensing: 10 Hz fixes, a slower navigation refit, a 10 Hz anomaly monitor (plan, 22 Nov; user: "could the capturing run at 2 Hz or 5 Hz and the anomaly detection at 10 Hz", "properly plan and make it")

**Rates (all options; the defaults keep the section 22 behaviour: 1 Hz fixes, a refit at every fix, no monitor):**

| Loop | Rate | Input |
|---|---|---|
| Arm joint servo, contact guards (touch pads, wrist force) | every physics step, 500 Hz (unchanged: contact forces build in ms) | encoders, pads, wrist F/T |
| Descent command (catch-up, section 22) | 100 Hz (unchanged), on the navigation prediction | prediction |
| Bus control law | 10 Hz (unchanged) | prediction |
| Lidar fix (one ring fit per frame) | `--lidar-hz`, 10 | lidar |
| **Innovation monitor** | every fix (10 Hz) | the fix against the prediction |
| **Navigation refit** (the 20 s window, Euler propagation) | `--fit-hz`, 2 | at most `--fit-max` 40 fixes spread over the window, the newest always in |

The prediction (propagation from the last refit) runs wherever it is needed, as now; only the refit gets slower.

**The monitor (`--innov off | record | on`).** For every fix once the filter has a solution: the innovation =
the fitted ring centre minus the predicted one (mm) and the angle between the fitted and predicted ring axes (deg);
the roll is not observed by a ring fix and not monitored.
- *Gate:* a fix whose innovation exceeds `G_MM` or `G_DEG` is not given to the filter (outlier rejection).
- *Anomaly:* `ANOM_N` rejected fixes in a row (0.5 s at 10 Hz) = the client does not move as predicted, or the
  lidar is wrong; the response: in flight, the phase aborts (safe, as the station-error abort); in the reach (pre-close
  move, descent), the contact-guard trip mechanism stops the arm, "abort: ... (nav anomaly)"; after the palm touch
  (the close is under way) and after the lock it is recorded only.
- Gates set from the innovation statistics of normal episodes (`record` mode), not tuned on the result.

**Checks.** (1) Timing and innovation statistics on 2 seeds (target: wall time near the 1 Hz case, 120 s).
(2) Fault cases, monitor on and off, same seeds: a lidar fault (the fitted ring centre biased 40 mm for 2 s in the
descent) and a client kick (an impulse giving ~10 mm/s during the descent); wanted: monitor on = a safe abort
before contact, no collision. (3) The 25 tune seeds (compare 24 / 25), the truth audit (the monitor reads no truth).
**Not in it:** the wrist camera stays one image at the pre-reach pose (the net is trained there only; a 10 Hz camera
monitor in the descent would need its own training data).

**Built and checked (22 Nov).** `ring_mission.py`: `--lidar-hz`, `--fit-hz`, `--fit-max`, `--innov off|record|on`,
`--g-mm`, `--g-deg`; RelNav `add` stores every fix and refits at `FIT_DT` over `_sel()` (at most `FIT_MAX`, evenly
spread, the newest in); `Mission.innovation` (gate, anomaly, responses as planned; the arm stop goes through the
contact-guard trip, "abort: pre-close / descent (nav anomaly)"). Speed-ups with identical results (checked on 12
lidar frames: max difference 0; `_exp` 2e-16): the ring fit's matched-filter search vectorised with squared radii
and a band pre-filter (57 -> 20 ms a frame), a Rodrigues `_exp` and a plain 3-vector cross in the propagation
(the refit 0.36 -> 0.15 s). 10 s of 10 Hz flight: 17.5 -> 9.1 s wall (profiled); left: about a third each refit,
ring fit, lidar render (28 ms a frame).

- **Gates** from `record` mode (2 seeds, 10 Hz, normal episodes: innovation max 13.3 mm, 1.83 deg): **25 mm, 3 deg**
  (the planned 2 deg sat on the normal maximum), `ANOM_N` 5 (0.5 s).
- **Faults (`checks/mr_faults.py`, seed 4, `runs/mr_faults.json`):**

  | Fault (in the descent) | monitor off | monitor on |
  |---|---|---|
  | lidar: ring centre biased 40 mm for 2 s | captured (the 2 Hz refit over 40 fixes diluted the 20 biased fixes) | abort (nav anomaly) 0.5 s in, before contact |
  | client kick ~10 mm/s | abort (wrist force) after the tool touched the ring off the pads | abort (nav anomaly) before contact |

  0 collisions in all four. The lidar row is the cost of the conservative response: a transient sensor fault the
  filter would have ridden through becomes an abort. Lead, not built: hold the descent on an anomaly, resume when
  fixes agree again, abort after a timeout (the kick would still abort: the gated filter never re-converges).
- **25 tune seeds (`runs/ring_mission25_mr.json`; 10 Hz fixes, 2 Hz refit over <= 40, monitor on): 24 / 25
  captured, 1 safe abort (seed 21, station attitude, as at 1 Hz), 0 collisions**; no anomaly and no rejected fix on
  any seed. Rate error at the reach median 0.038 deg/s (1 Hz: 0.053), radial error after the net median 2.2 mm (2.4),
  peak contact max 291 N (327), client push max 6.4 mm/s (6.2). Normal innovations reached 20.5 mm / 1.49 deg in the
  reach and 17.8 mm / 2.20 deg in flight: the 25 mm gate's margin is thin (1.2x), recorded. Wall time median 565 s
  an episode on 5 workers (1 Hz: 120 s): 10 Hz is for experiments, 1 Hz stays the default for batch work.
- **Truth audit (`ring_truth_audit.py --multirate`, seeds 0, 1, both captured):** no truth read in the monitor or the
  refit; two reads outside the function classes: the hand-over prior in `fix` (the first fix, hold3, once) and the
  client push written to the row in `reach_close` (a record); both now classified by line (`LINE_OK`), not rerun.

## 24. P7.8 plan: GPU residual RL on the reach and close, harder clients, a learned close moment (proposal, 21 Nov; user: "you can use the gpu for rl, I'll allow it", "harder clients, and 1" = the policy also picks when to close; "plan first")

Replaces the CPU / TD3 budget of section 21 (R1-R5 stay as the outline; R0 is done). Not started.

**Probe (21 Nov, `scratchpad/warp_probe.py`, the `rl` pixi env: mujoco-warp 3.7.0.1, RTX 4060 8 GB):** the rev-2 scene
(nq 22, nv 20, 343 colliding geoms: 327 boxes, 14 cylinders, 2 capsules; elliptic cone, Newton, implicitfast, dt 2 ms)
loads in MuJoCo Warp only with **noslip off** (Warp has no noslip solver; the CPU scene uses 20 noslip iterations).
Free stepping (no arm contact): 119k env-steps/s at 256 worlds, 769k at 1024, 889k at 4096 (contact buffers 64
contacts / 400 rows per world). CPU in contact: ~1.3k/s per core. Contact steps on the GPU are not measured yet.

**Shape (what changes from section 21).**
- **Sim on the GPU** (MuJoCo Warp, thousands of worlds), **PPO with rsl-rl** (installed, used in P1-P3), ~1e7
  decisions; the off-policy TD3 option is dropped (kept as a fallback if PPO stalls). No CPU pool runs at the same
  time; GPU temperature logged; runs in checkpointed chunks (<= ~1.5 h).
- **Harder clients** (the training and evaluation distribution, beyond the mission's): spin about the ring axis
  0-4 deg/s, transverse rate (nutation) 0-1 deg/s, ring tilt along the section 0-4 deg, client mass / inertia +-30%,
  friction 0.25-0.6, estimate errors up to 2x the mission's E1 spread; a curriculum from the mission range outwards.
- **Action:** a bounded residual on the grip-site Cartesian target (+-5 mm/s, +-1 deg/s, zero-initialised) **plus a
  close command** (option 1): the scripted close fires at palm touch; the policy may hold it (up to a time limit) or
  fire it once inside the shield's close window (palm within 3 mm of the estimated top, wrist force under limit).
  The close mechanics (lead screw, lock) stay scripted.
- **Perception is not run on the GPU:** the lidar ring fit / RelNav and the camera net are replaced in training by
  an error model fitted to the mission's own records (the camera net's radial / tangential residual at pre-reach, the
  innovation statistics of section 23, rate error ~0.04 deg/s, plus a slow drift), randomised wider than measured,
  including near zero (P2 r2's lesson). The final evaluation runs the real pipeline on the CPU.
- Observation, reward, terminations (CaT), shield: as section 21 (own-sensor observation; sparse lock reward on the
  truth in training only; a safe abort scores 0; guards outside the policy). Policy at 10 Hz (50 physics steps).

**Steps.**
- **G0, port check (gate for the whole plan):**
  (a) collision trim for the RL scene: only the ring section near the grasp, the tool and the arm links collide
  (the 343-geom broadphase dominates);
  (b) Warp contact speed at a palm-touch / closing state (target >= 100k env-steps/s);
  (c) **noslip off on the CPU:** the scripted reach on the bench over the P7.5 cases with noslip 0 vs 20; if the
  grip or the lock changes, the GPU physics is not the mission's and the plan stops here for a decision (U);
  (d) Warp vs CPU agreement on a few identical reach trajectories (contact forces, seat, timing).
- **G1, batched scripted base on the GPU:** the reach-and-close of `reach_bench.reach_close` (the pre-close catch-up,
  slow zone, palm follow, close, lock check, guards) as batched torch / Warp: a damped-least-squares Cartesian
  controller replaces the per-step IK (`ik_lim`), the arm PD and the guards per physics step. **Check (R2):** a = 0
  reproduces the CPU base's outcomes on the same cases (captured / abort class per case, seat within 1 mm).
- **G2, envelope (R3):** the scripted base over the harder-client grid on the GPU (25+ per cell is now cheap, so
  ~100 per cell); the gate and the training distribution are fixed from this, before training.
- **G3, train (R4):** PPO residual + close command, curriculum, a few seeds of training; checkpointed and resumable.
- **G4, evaluate (R5):** GPU validation cells (policy vs a = 0, same draws); then the CPU real pipeline (noslip on,
  lidar, ring fit, net) on harder-client bench cases and the 25 mission tune seeds (no worse than the scripted 24 / 25,
  0 collisions); the test split once, at the end.

**Gate (draft, fixed at G2):** in the harder-client cells, captured clearly above the scripted base at 0 collisions,
no rise in peak pad / wrist force or client push; the CPU real-pipeline check agrees with the GPU result within the
cell noise; the nominal mission not worse.

**Risks:** the GPU physics without noslip differs from the mission's (G0c decides); the policy exploits Warp contact
artefacts (check on the CPU with noslip, at dt 1 ms); the error model is not the real perception (the CPU check);
porting the scripted base is the biggest piece of work and the place for bugs (G1's check); thermal (chunks, logged
temperature, no CPU pool alongside).

### 24.1 G0 results (21 Nov): the port check passes

Tool: `checks/g0_port.py` (record the scripted bench reach on the CPU from the descent start, then replay its commands
with the arm PD and the bus pin per step on each sim; one CPU evaluator for all variants) and `reach_bench.py
--noslip N`. Runs: `runs/g0c_ns{20,0}.json`, `runs/g0_rec_{0..3}.pkl`, `runs/g0_port.json`.
- **G0c, noslip off (closed loop, reach bench cases 0-11):** the same outcome in all 12 (8 grip, 2 wrist check "no
  verdict", 1 descent hook, 1 descent wrist F/T); seat within 0.5 mm, slip under the 3 N m loads alike (max 1.1 mm
  both), contact force peaks +-50% (contact noise). Noslip is not needed for the grip or the lock at this level.
  (Aside: the bench gets 8 / 12 now against 25 / 25 in the P7.4 run, the classical wrist search path; not chased.)
- **G0a, trim:** 224 of 288 ring segments off (+-40 deg sector kept): the CPU replay is identical to the untrimmed
  one (bit for bit, 4 / 4 seeds).
- **G0d, GPU vs CPU (seeds 0-3, all grip on the CPU):** the GPU (Warp, noslip 0, float32, trimmed) grips in 4 / 4;
  seat after the hold within 0.7 mm of the CPU (1.50-2.06 vs 1.39-1.79 mm radial), grip-site path within 0.5-1.65 mm,
  slip under load 0.24-0.90 mm (CPU 0.32-0.66). Differences: contact forces chatter more during the close (single
  samples to ~1 kN, as the CPU's own peaks), and the free client is pushed more after the close (up to 6.6 vs 4.3
  mm/s; seed 2 6.2 vs 3.5): matters for the 5 mm/s push termination, set it on the GPU's own numbers. Bug found and
  fixed in the replay: the lock must hold the jaws where the sim has them (the CPU's lock values on the GPU jaws
  unloaded the clamp, seed 2); the env does it per world, so the lock values are per-world data, not the model's.
- **G0b, speed in contact:** 170-260k env-steps/s at 1024 worlds, 187-196k at 4096 (including the per-step torch PD
  and pin in Python; a fused step would be faster), no NaN worlds; contact buffers 160 contacts / 800 rows per world
  (the clamp makes up to 85 box-box contacts). GPU 87 C at 99 %, 38-40 W, steady. Budget: 1e7 decisions x 50 steps =
  5e8 steps, ~45 min of stepping.
- **Limits of G0:** 4 seeds, a client at rest on the bench (no spin), commands replayed (not closed loop on the GPU).
  G1's a = 0 check is the closed-loop test.

### 24.2 G1 results (21-22 Nov): the batched scripted base on the GPU matches the CPU bench

Code: `rl/arm_torch.py` (batched arm kinematics, Jacobian and the damped least-squares IK of `reach_bench._ik1`;
`rl/check_arm_torch.py`: FK / Jacobian / IK equal MuJoCo and `_ik1` to 1e-15), `rl/gpu_reach.py` (`GpuReach`:
reach_close for N worlds: the pre-close cart with the bench's waypoint timing, settle, the guarded descent with the
catch-up, the palm-follow close, the per-world lock, the hold and the 3 N m hold check; control ticks every 5 steps,
asynchronous per world; guards and the collision check every step; policy hooks `residual`, `close_cmd`),
`rl/g1_check.py` (record the CPU bench at the entry of reach_close, run the same starts on the GPU, compare; `speed`).
- **Parity (bench cases 0-24, 22 reach the close):** **22 / 22 the same class** (20 grip, 1 descent hook, 1 descent
  wrist F/T); descent stop height within 0.1 mm; seat after the close 0.43 mm further out on the GPU (median; -0.19
  to +1.16), slip under load alike.
- **Fixed on the way:** (1) the touch zones: the ring lip lands on the palm zone's edge (exactly its half height);
  float32 on the GPU dropped those contacts (palm 0 N while the wrist saw 9-23 N: 6 / 22 wrist F/T aborts), so the GPU
  model's touch zones are 1 mm larger (`TOUCH_PAD`); (2) the hold-end pad loads are a mean over the last 0.2 s of the
  hold (`HOLD_AVG`): one sample, as the CPU bench, made 16 / 1024 copies of grips "no grip" (the bouncing client
  unloads palm and lip for an instant; the clamp held 58-92 N, slip <= 1.1 mm).
- **Copies of one start (1024 worlds, the 22 starts repeated):** 1017 / 1024 agree with the start's own result; the 7
  others are light axial seats (palm / lip 1-5 N mean, under the 5 N rule; clamp 63-74 N, slip <= 0.7 mm): float32
  contact noise near the threshold, ~0.7 %, kept as is. Earlier (one-sample scoring) 4096 worlds: 4045 / 4096.
- **Speed (the whole reach-to-load episode, 15.5k physics steps):** 71-78k env-steps/s at 1024 worlds (13-14 ms a
  step), 123k at 4096 (33 ms a step); GPU 87 C, 89 %, 34 W. The control and checks run as many small torch ops per
  step; batching them (contacts and guards per tick, a graph-captured PD + pin) is the speed lever if G3 needs it.
- **Not covered:** a client at rest only (the bench); a moving client needs `track` (the estimate carried with the
  client's estimated motion) and the error model: part of G2.

### 24.3 G2 first pass (22 Nov): the scripted base on harder clients, GPU

Code: `rl/gpu_reach_moving.py` (`GpuReachMoving`: the free client started with w_ax about the ring axis + w_tr across
it, torque-free after; the bus held at its station on the client's FREE motion (a torque-free ghost from the start:
a bus on the pushed truth dragged the client to 60 mm/s) + a smooth station error (3 sinusoids per axis, 0.02-0.2 Hz:
per-step white noise shook the arm, 11 / 22 "grip, slips"); the estimate: the ring in the estimated body frame with
the camera residual (radial / tangential 2.5 mm, axial -10 +- 5 mm, axis 0.5 deg + the cell's tilt_r), the pose and
rate re-anchored every 0.5 s to the truth + AR(1) errors (1.5 mm, 0.15 deg, 0.04 deg/s, 3 s); all spreads x `scale`),
`rl/g2_envelope.py` (cells, classes: captured / safe abort / failed close / collision).
- **Smoke:** no motion, no error 22 / 22; the mission's range (1 deg/s axial, 0.5 transverse, scale 1) 17-18 / 22,
  the rest arm-tracking aborts at ~5.0 mm.
- **Sweep, 3996 worlds (108 cells x 37: w_ax 0 / 2 / 4, w_tr 0 / 0.5 / 1, tilt_r 0-3 deg, scale 1-3), 24 min:**
  **0 collisions**. Captured by scale: **1: 796 / 1332 (60 %), 2: 41 / 1332 (3 %), 3: 0**. By tilt_r: 23 / 26 / 21 / 14 %
  (failed closes rise 3 -> 103 at 3 deg: "grip, slips", "no grip"). Spin matters less: w_ax 4 with w_tr 1 drops to
  24-43 % at scale 1. End-of-hold client speed p90 6-35 mm/s by cell (CPU bench 1.7-9.6 at rest).
- **Why the base fails (the abort reasons):** arm tracking 1812 (1185 of them waiting at the top: the estimate
  steps a few mm at each 0.5 s re-anchor, the descent waits for the 1 mm catch-up, the catch-up runs at V_CATCH 30
  mm/s, and the joint PD has no velocity feed-forward: a ~0.16 s lag = ~5 mm = TRACK_MM; captures reach 4.1 mm median,
  5.0 max); wrist F/T trips without contact (304 at scale 2, 697 at 3: the catch-up bursts' inertial load on the 2 N
  guard); "not caught up" 134; failed closes on tilt.
- **Reading:** the scale-1 model is harsher than the mission (the mission's 24 / 25 at up to 2 deg/s; here 60 %); the
  mission's estimate probably moves more smoothly than 0.5 s re-anchors with these AR errors. And the scripted base
  has a classical weakness (catch-up speed x PD lag ~ the tracking limit; an inertial wrist guard), which a policy
  would learn around trivially (P2's lesson: fix the base first). Next (U): calibrate the error model to the mission
  (log the grasp-point estimate's steps and drift in a few mission reaches), and the base fix (joint velocity
  feed-forward or a slower catch-up), then the sweep again; the gate after that.

### 24.4 G2 calibration, the base fix, the envelope and the draft gate (22 Nov; user: "yes, calibrate and fix")

- **Calibration (`checks/calib_track.py`: Mission.track wrapped, the multi-rate mission on tune seeds 0-4, all
  captured):** the estimated grasp point against the true ring top at the same section, over the reach (14-22 s):
  steps at the 0.5 s refits median 0.57 / p90 0.98 / max 1.86 mm (the first-pass model: ~3x); slow drift std
  1.6-3.0 mm radial, 0.6-2.6 tangential, 1.8-2.9 axial; offsets radial -0.7..-2.3, tangential 0..+7, axial -5..+13 mm
  (the model had -10 +- 5: wrong sign), axis 0.35-1.14 deg. Scale 1 refitted: camera residual radial 2.5 /
  tangential 3 mm, axial +5 +- 7 mm, axis 0.6 deg; pose AR(1) 2 mm, 0.1 deg, 0.02 deg/s, 20 s. Result: the base at
  the mission's range (1 deg/s axial, 0.5 transverse, scale 1) 40 / 44 (the mission: 24 / 25).
- **The base fix: joint velocity feed-forward** (`reach_bench.Bench.step(qd_tgt)`, `FF`, now the default; `--no-ff`):
  the target's joint velocity (the cart's segment rate, the descent tick's IK step / 5 dt) on the PD's damping term,
  clipped to 0.5 rad/s, **low-passed (TAU_FF 0.15 s: a velocity step through kd is a torque step, the wrist F/T guard
  tripped on every world's first step)**, **free-space moves only (in the close it doubled the push: 6 -> 18 mm/s at the
  hold's end)**, the filter reset per episode. GPU smoke (no-ff / ff): mission range 40 / 44 -> 44 / 44, scale 2 20 /
  44 -> 28 / 44, end-of-hold push 7.3 / 7.9 mm/s. **CPU: the ring mission's 25 tune seeds (1 Hz, net) 23 / 25 both
  with and without it, 0 collisions, push and seat per seed within +-0.5 mm/s / mm**; the two aborts are the flight's
  station attitude (seeds 3 and 21, low spin, 7.3 / 7.0 deg; seed 3 captured in the R0 run before the section 23 code
  changed the numerics, at 6.02 deg on the way: marginal, the same lead as 21). Reach bench (classical wrist search,
  not the mission's path): 18 / 22 grips vs 20 (cases 1, 2: the search's E1 moved 17 -> 23 and 6.6 -> 21 mm, then hook).
- **Calibrated sweep with the fix (`runs/g2_sweep_cal.json`, 3996 worlds, 34 min, GPU 90 C):** captured by scale 1 / 2 /
  3: **79 / 45 / 18 %** (first pass 60 / 3 / 0); mission-like cells (tilt <= 1, w_ax <= 2) at scale 1: 95-100 %. Failure
  modes: scale 1: failed closes (no grip 104, slips 70; tilt 2-3 deg), not caught up 50; scale 2-3: wrist F/T trips in
  the descent, hook / no-contact aborts. **8 collisions, all at w_ax 4 deg/s**: 6 in the descent (an off-tool touch at
  < 0.5 mm/s client speed, stopped by the wrist F/T guard), 2 in the close (the client kicked to 60 and 144 mm/s).
- **Sets (base rates, scale 1 unless noted):**
  N nominal (w_ax <= 2, tilt <= 1): 438 / 444 (99 %), 0 collisions, push p90 13.2 mm/s;
  H1 hard (tilt 2-3 or w_ax 4): 614 / 888 (69 %), 181 failed closes, 3 collisions, push p90 20.0;
  of it tilt only: 304 / 444 (68 %, 135 failed closes); spin only (w_ax 4, tilt <= 1): 180 / 222 (81 %);
  H2 (scale 2, all): 603 / 1332 (45 %), push p90 23.1.
- **Draft gate (G3/G4, on fresh draws):** N >= 97 % and 0 collisions; **H1 >= 80 %** (base 69 %; +11 points, ~7 standard
  errors on 888) with collisions <= the base's and failed closes down; H2 not worse; push p90 and pad peaks not above
  the base (+10 %); then the CPU real pipeline: the 25 tune seeds >= 23 / 25 at 0 collisions.
- **What the policy is for:** H1's failed closes (the tool leaning on a tilted ring section; the close moment on a
  spinning client: option 1). Classical alternatives to weigh first (P2): the tilt is an estimate error, so better
  perception (the wrist stereo experiment, user 22 Nov) shrinks H1 directly; a compliant palm would too.

## 25. Wrist stereo, a narrow elbow and a side-stowed tool (22-23 Nov; user: "go ahead with stereo vision, you can make changes in the arm cam module in freecad")

**Camera layout (user-steered, in order):** a second camera 60 mm inboard of the first (z stack, a converging pair);
"like eyes" (side by side, 60 mm across the tool); "the ring between them, along the jaws' travel" (one camera
inboard, one on the housing's outboard end, 172 mm radial baseline; the outboard view was half hidden by the hook
jaw); "not that far apart, a single module parallel to the movement plane of the grippers"; placed by the user "on
the housing side face". **Final:** one stereo module (0.45 kg, 24 x 40 x 80 mm) on the back housing's -y face
(`build_ring_tool.py`: x 62-86, y -75..-35, z -60..+20), two parallel cameras 50 mm apart along z (the jaws'
travel), `wrist_cam` (tool mm (87, -55, -45)) and `wrist_cam2` ((87, -55, +5)), both pitched 10 deg radially in,
75 deg / 128 px, the lamp between them. The inboard camera unit (z -101..-65) is gone. Candidate views:
`checks/stereo_view.py` (trial positions listed in it), `runs/stereo_view.png`.

**Narrow elbow, tool stowed beside the forearm (user: "the gripper, instead of the middle, sits on the other side of
the arm, near the edge"; "redesign the elbow"):** `make_scene_rev2.py`: upper tube y +0.04 (was +0.08), forearm
y -0.035 (was -0.08), tubes 9 mm apart folded, elbow module y -0.075..+0.08 (155 mm, was 230); **kinematic change:
the wrist (wrist1) sits WRIST_OFF = 0.12 m along -y (the wrist pitch axis) off the forearm**, so the folded tool
lies beside the forearm toward the face edge (stow q unchanged, tool origin (0.363, -0.12, -0.15)). Shoulder pitch
bracket boss r 0.06 -> 0.05 (the folded wrist end passes it on the same y line now). Restraints: elbow cheek at
y -0.085 with its HDRM; a wrist plate at y -0.205 standing on the bus's launch-lock block, HDRM pin on the wrist
pitch axis; snubber pad follows the tool (y -0.094). **Stow: depth 0.727 m from the port face (was 0.823 m; ESPA
0.71), clearances >= 5.4 mm (module to the wrist plate), bus camera 0 % blocked (was 2.6 %); staged deployment
clear** (`stow_secure.py`, `deploy_check.py`). FreeCAD: `ring_tool.FCStd`, `servicer_rev2_arm.FCStd` rebuilt
(no arm-bus overlap), renders `models/servicer_rev_2/arm_v3_*.png`. Tool mass 2.17 kg (was 2.27).

**Nets (`checks/gen_stereo.py`, `nets/stereo_net.py`):** 15 000 bench samples (seeds 7810000+; centre +-30 mm
radial / tangential, +-25 mm axial, axis tilt <= 3 deg), three views each: `in` (wrist_cam), `out` (wrist_cam2),
`mot` (wrist_cam after a 40 mm radial arm move: motion stereo). One CNN per view set, outputs centre offset (radial,
tangential, axial) + axis tilt (along radial, tangential) with sigmas. Held-out bench, |err| p50 / p95 (mm, mm, mm,
deg, deg): mono 0.22/1.1, 0.31/1.6, 0.54/3.1, 0.09/0.58, 0.20/1.08; stereo 0.18/0.86, 0.27/1.41, 0.62/3.68,
0.08/0.51, 0.15/0.88; motion 0.19/0.91, 0.26/1.33, 0.64/3.53, 0.08/0.44, 0.14/0.79. Training on the laptop GPU: the
device shut down three times (sustained ~86-87 C at 70 % duty); finished at half duty with 5-epoch checkpoints.

**Mission (`ring_mission.py --cam stereo [--stereo-act in|in+out] [--stereo-tilt]`; 25 tune seeds, 3 workers):**
- **Finding: the ring net's -7 mm "axial offset" (s17-s22) was arm lag, not the camera.** The nets assume the camera
  sits at the pre-reach pose on the estimate; in the mission the arm lags the moving estimate (mean 1.6 mm radial,
  7 mm axial at the shot). Measured from the section the camera actually looks from (rebuilt from the grip site's
  pose: `--stereo-frame camera`, default), mono and stereo are unbiased.
- In-mission |err| p50 / p95 (23 seeds that reached the reach): **mono** 0.20/0.34 mm radial, 0.27/1.33 tangential,
  0.71/1.72 axial, 0.10/0.31 and 0.15/0.82 deg tilt; **stereo** 0.16/0.53, 0.22/0.89, 0.59/2.88 (one 9.8 mm axial
  outlier), 0.11/0.30, 0.16/0.59 deg; **motion stereo** 5.0/6.9 mm radial, 6.8/14.6 axial (the client and the arm
  move between the shots): unusable as built.
- The 40 mm motion-stereo move tripped the wrist F/T at the descent start on 13 / 25 seeds (no contact); without it
  (default now) 3 / 3 of those captured.
- **Result: stereo (acting on radial + tangential) 23 / 25 captured, mono 23 / 25, both 0 collisions**; the two
  aborts are the known in-flight station-attitude aborts (seeds 3, 21). Stereo tightens the tangential and tilt
  tails (p95 1.33 -> 0.89 mm, 0.82 -> 0.59 deg), not the axial; on the tune seeds it does not change the outcome.
- Open: the hard set (G2: tilt 2-3 deg, 4 deg/s spin) is where tilt matters: re-sweep it with the stereo estimate
  (`--stereo-tilt` applies the tilt; not yet run); the GPU env's calibrated error model (s24.4) assumed the old net's
  axial offset; the old ring net (`ringcam_net.pt`) no longer matches the camera (retired; `--cam net` invalid).

### 25.1 GPU env redone for the new arm (23 Nov; user: "yes, redo GPU env")
- **G0 (`checks/g0_port.py`, 4 seeds):** cpu20 / cpu0 / cpu20_trim / gpu all grip 4 / 4, every pad loaded; trim
  exact; GPU seat within 0.7 mm of the CPU, grip-site path within 1.3 mm; GPU peak 83 C. Fix: the recorder's `step()`
  override predated the feed-forward (s24.4) and crashed, so the first replay ran on the old arm's recordings (saved in
  `runs/old_arm/`); G0 now records and replays without the feed-forward (a physics port check).
- **G1 (`rl/g1_check.py`, 25 bench seeds):** CPU bench 25 / 25 grip on the new arm (old arm 20 / 22); GPU base the same
  class on 22 / 25 (old 22 / 22). The 3 differences are all GPU-pessimistic: seeds 11 and 23 (the most yawed seats,
  3.6-4.9 deg) slip 3-7 deg under the 3 N m load on the GPU (no noslip) vs 0.3-0.4 deg on the CPU; seed 10 seats
  like the CPU but reads "no grip" on the pad rule. Accepted (a sim that errs toward failure is the safe side).
- **Half GPU load (user):** `gpu_reach.DUTY = 0.5` (sleep in `run()`), pause at 90 C until 82 C.
- **Error model:** `gpu_reach_moving.PERC` / `g2_envelope.py --perc stereo`: radial / tangential residual N(0, 0.35 /
  0.55 mm), the cell's tilt measured and removed, tilt residual per axis 90 % N(0, 0.18 deg) + 10 % N(0, 0.7 deg)
  (held-out bench: stereo tilt error does not grow with tilt, p50 0.16-0.23 deg, p95 0.86-1.42 deg at 0-3 deg).
  `--perc net` keeps the s24.4 calibrated model. `--scales` limits the sweep's error scales.
- **G2 smoke (new arm):** no motion 22 / 22, mission range 44 / 44 (both models), scale 2: net 28 / 44, stereo 31 / 44,
  0 collisions.
- **Scale-1 sweeps (1332 worlds each, ~33 min each at half duty, GPU 85 C; `runs/g2_sweep_arm3_{net,stereo}.json`):**

  | set | net captured | net collisions | stereo captured | stereo collisions |
  |---|---|---|---|---|
  | N (444) | 98.2 % | 0 | 98.6 % | 0 |
  | H1 (888) | 66.0 % | 5 | 88.4 % | 2 |
  | H1 tilt >= 2, w_ax <= 2 (444) | 67.3 % | 0 | 99.1 % | 0 |
  | w_ax 4 (444) | 64.6 % | 5 | 77.7 % | 2 |

  Stereo with the tilt removes every failed close (net: 202 on H1, mostly no grip / slips at tilt) and passes the
  draft bar (N >= 97 % at 0 collisions, H1 >= 80 %). Its 2 collisions are both cell w_ax 4 / w_tr 1 / tilt 2: the tool
  touches the client 60-69 mm above the seat in the descent and the wrist F/T aborts (net: 5 in the w_ax 4 cells).
  Most w_ax 4 losses are safe "grasp point not caught up" aborts. Next: `--stereo-tilt` in the mission (25 tune
  seeds); the axial-wobble case is the target for G3.

### 25.2 G3: PPO residual on the descent (23 Nov; user: "go to G3 PPO")
- **Trainer `rl/g3_ppo.py`** (own PPO, synchronous episodes: each iteration a fresh batch of worlds, one episode, then
  5 epochs of PPO; checkpoint per iteration `runs/g3_ppo.ckpt`, resumable; log `runs/g3_ppo.log.jsonl`). Error model:
  stereo with the tilt (s25.1). Load test off in training (on in eval).
- **Policy hook changed:** the first hook (a velocity on the commanded grip point) was erased by the catch-up every tick
  once caught up. Now `GpuReach.offset`: an offset of the tracked grip point in the estimated section frame; the base
  tracks section(h) + offset (offset 0 = the scripted reach, code path otherwise unchanged).
- **Action (10 Hz, descent only):** offset rate <= 5 mm/s per axis, offset <= 6 mm. The close stays scripted (failed
  closes are 0 with the stereo tilt). **Obs (28, no truth):** gap commanded -> tracked, arm tracking error, tracked
  point velocity, estimated client rate (section frame), height, catch-up state, pads, wrist force step, offset, last
  action. **Reward (truth, training only):** captured +1, failed close -1, collision -3, safe abort 0, -0.002 |a|^2.
- **Cells:** w_ax U(0, 2 + 2.5 L), w_tr U(0, 1), tilt U(0, 1 + 2 L), scale U(0.5, 1.5); L 0 -> 1 over the first quarter.
- **Smoke (2 x 128 worlds):** runs end to end, ~7 min an iteration, 0 collisions, KL 0.001.
- **Run (started 23 Nov):** 20 iterations x 1024 worlds (~10 min each at half duty), then the eval on the G2 grid at
  scale 1 (`runs/g2_g3_eval.json`; base: `runs/g2_sweep_arm3_stereo.json`); log `runs/g3.log`, `runs/g3.done`.
- **Stopped after iteration 9 (user, on the recommendation):** at full difficulty (it 5-9) captured 952 / 962 / 946 /
  949 / 955 of 1024, collisions 3 / 2 / 1 / 1 / 1: flat within the draw noise; PPO steps tiny (KL <= 0.007), std
  0.37 -> 0.33: the policy stayed near a = 0. Iterations ~20 min (hard worlds ran to the 30000-step cap). Why, likely:
  ~95 % of worlds capture whatever the policy does (little signal per batch), and the main loss (not caught up at
  w_ax 4) may be outside a 6 mm offset's reach. Kept: `runs/g3_ppo.ckpt` (it 9, resumable), `g3_ppo_it{4,9}.pt`, log.
- **Next instead (classical first):** (1) the 2 stereo collisions (w_ax 4 / w_tr 1 / tilt 2; tool touches 60-69 mm up
  in the descent): replay, find the geom pair; (2) a feed-forward of the tracked point's velocity in the descent
  catch-up (the base chases the point's position only), re-sweep the w_ax 4 cells vs the stereo base; (3) then decide on
  RL (hard cells only, more authority, the close command).

### 25.3 The stereo base's descent collisions, diagnosed (23 Nov)
- **Run (`rl/diag_coll.py`, `runs/diag_coll.json`):** 400 worlds, w_ax 4, w_tr 0.5 / 1, tilt 0-3, scale 1, stereo,
  load test off; the first bad contact per world now logged (`GpuReach._contacts`: geom pairs, phase, height, true
  seat). 287 captured, 110 safe aborts, **3 collisions**.
- **All 3 the same:** the inboard jaw pad (`pad_l_0` / `pad_l_2`) on the **client bus** (not the ring), in the descent,
  **39-54 s after the descent started** (a normal descent: ~13 s). At the contact the tool is 68-153 mm inside the ring
  wall and yawed 137-171 deg from the grasp point (true seat): the client had turned under a tool that was no longer
  on the grasp point. The wrist F/T / inboard pad then aborts, after the touch.
- **Cause:** the descent's "not caught up" abort (`CATCH_S` 3 s) applies only before the descent starts (at H_CLOSE).
  Once caught up the first time, a descent that loses the tracked point (the point moving at ~V_CATCH: w_ax 4 deg/s
  at the ring radius plus the nutation) has no limit: h stays put, the commanded point chases the grasp point round the
  spinning client for tens of seconds, and the jaw meets the bus. The CPU bench (`reach_bench.reach_close`) has the
  same gap; the mission's spin (<= 2 deg/s) has not reached it.
- **Fix (proposed, not built):** (1) a lost-catch abort at any height: not caught up for CATCH_S -> abort, safe, and a
  descent time limit; (2) a feed-forward of the tracked point's velocity in the catch-up (V_CATCH then bounds the
  closing speed, not the absolute speed), which should also recover many of the w_ax 4 "not caught up" aborts. Both
  in the CPU bench and the GPU port, then a w_ax 4 re-sweep vs `runs/g2_sweep_arm3_stereo.json`.

### 25.4 Descent catch-up on the target's state, and lost-catch aborts (23 Nov; user: "build the fix"; "the important states are x and x dot: position, angle and velocity, angular velocity")
- **Feed-forward from the estimate's state:** the tracked grasp point's velocity v + w x (p - com) (the client's
  estimated COM velocity and angular rate) moves the commanded point each tick; V_CATCH (30 mm/s) now bounds only the
  closing speed; clip 60 mm/s (`V_FF_MAX`). The orientation target stays the estimated pose's, every tick. CPU:
  `Bench.point_vel` (0 on the static bench; the mission from `nav.predict`: w, v, com); GPU:
  `GpuReachMoving.point_vel` (the estimate's a_v, a_w). Flag `reach_bench.CATCH_FF`. (A first cut differenced the
  tracked position; replaced by the state, per the user: no filter lag, no refit spikes.)
- **Aborts (safe):** not caught up for CATCH_S (3 s) at any height ("lost the grasp point"); descent over T_DESC_MAX
  (30 s; normal ~13 s).
- **Re-sweep, spinning cells, scale 1, stereo (`runs/g2_spin_ff.json`, 888 worlds, 11 min) vs the same cells of
  `g2_sweep_arm3_stereo.json`:**

  | cells | before: captured | before: collisions | after: captured | after: collisions |
  |---|---|---|---|---|
  | w_ax 2 (444) | 98.6 % | 0 | 99.3 % | 0 |
  | w_ax 4, tilt <= 1 (222) | 74.3 % | 0 | 98.6 % | 0 |
  | w_ax 4, tilt >= 2 (222) | 81.1 % | 2 | 99.1 % | 0 |
  | w_ax 4, all (444) | 77.7 % | 2 | **98.9 %** | **0** |

  The 67 "not caught up" aborts and both collisions are gone; the new aborts never fired. Left at w_ax 4: 3 safe
  aborts, 2 failed closes (no grip). Episodes end sooner (the sweep ran ~3x faster).
- **Not yet checked:** the CPU bench (feed-forward 0 there: only the aborts change) and the mission's 25 tune seeds
  (the feed-forward is live there: re-run).

### 25.5 Measurement latency (23 Nov; user: "add a bit lag for realism")
- **Model:** a measurement taken at t reaches the estimator at t + L, L = 150 ms +- 20 ms (lidar scan + fit on flight
  hardware), time-stamped t; the estimate propagates from t to now (position, attitude, velocity, angular rate).
  CPU mission: each lidar fix is queued and handed to RelNav at t + L with its own stamp (the fit already works on
  stamps, `predict` propagates to now; innovation gate evaluated at the stamp): `ring_mission.LAT / LAT_J`. GPU:
  each refit anchors on the truth from L ago (a 10 ms truth history), stamped, propagated to now:
  `gpu_reach_moving.LAT / LAT_J / PROP_NOW`, `g2_envelope.py --lat --no-prop`. **Default now 150 ms in both.**
- **Check (`scratchpad/lat_check.py`, w_ax 4 / w_tr 1, error scale 0: the lag alone):** estimate vs truth, grasp
  point median / p90: no lag 0.05 / 0.08 mm; 150 ms stamped 0.06 / 0.10 mm; 150 ms unstamped 3.8 / 5.8 mm (0.63 deg);
  300 ms unstamped 7.6 / 11.4 mm (1.25 deg). The lag is in; stamping removes it.
- **Spin sweeps (888 worlds each, scale 1, stereo, the s25.4 catch-up):** captured at w_ax 2 / 4, collisions:
  no lag 99.3 / 98.9 %, 0; 150 ms 99.3 / 99.1 %, 0; 300 ms 100 / 98.9 %, 0; **150 ms unstamped 99.3 / 99.1 %, 0**.
  Even the unstamped lag costs nothing: spin about the ring axis moves the grasp point along the ring, and an error
  along a continuous ring section is harmless (the 4-6 mm are mostly tangential). It would matter for transverse
  motion (nutation, a tumbling client) and in the radial / axial directions.
- **Not yet run:** the mission's 25 tune seeds with the latency and the s25.4 catch-up.
- **Mission, 25 tune seeds, with the s25.4 catch-up and the 150 ms latency (`runs/ring_mission_ff_lat.json`; stereo
  in+out, 3 workers): 24 / 25 captured, 0 collisions** (before both: 23 / 25, `ring_mission_stereo_in_out.json`).
  Seed 3 now captured (before: the in-flight station-attitude abort, 5.2 deg; marginal, the nav's numerics changed);
  seed 21 still aborts in flight (8.5 deg), as on every run. On the 23 seeds captured in both, the closed seat and
  the client push are the same within run noise (seat radial 0.9-2.2 mm, push 0.8-4.1 mm/s both ways). The mission's
  spins (<= 2 deg/s) did not need the catch-up; it changes nothing there, as intended.

## 26. Plan: the flight, hold-out to fly-in (proposal, 23 Nov; user: "take the learning for the previous tasks (rl, delay, position-velocity inputs, etc) and plan it out")

**Problem.** The one failure left in the mission is in flight: the station attitude abort (bus attitude vs its target
> 5 deg for 5 s). Seed 21 on every run (6.5-10 deg), seed 3 marginal (5.2-7.3 deg, captured on some runs); both slow
spinners (0.01, 0.19 deg/s). Seed 21's error grows through the approach: 0.27 (hold 3.2 m), 2.95 (to 1.8 m), 4.14
(hold 1.8 m), 8.5 deg (to station); its position error stays ~7 mm.

**What the code does now.** `_law` is a full-state PD with feed-forward: target pose, velocity and rate from the nav
estimate (`station`: p_d, R_d = R_est R_rel, v_d = v + w x r, w_d = w, a_ff = w x (w x r)), at CTRL_DT 0.1 s, KP_A
0.25 / KD_A 0.9 (0.5 rad/s), translation 0.3 rad/s; wheels with momentum unloading by thrusters. The abort's error is
tracking (bus vs R_d), but R_d is the estimate. Fixes: box fits beyond 2.2 m (full pose), ring fits closer (ring
centre and axis only: **roll about the ring axis is not observed close in**, the estimate carries it on its rate).

**Lessons carried over (this block, s24-s25).**
1. Measure before fixing: the -7 mm axial bias was arm lag, the "descent collisions" were a lost catch-up 40 s long.
   Log the full state against the truth first, in the task's own frame.
2. Classical first; learning only on a measured gap with signal. G3 PPO stayed flat (the base captured ~93 %, little
   signal per batch); a feed-forward fixed the gap in one sweep (77.7 -> 98.9 %).
3. Controllers and feed-forward work on the state, x and x dot (position, attitude, velocity, angular rate) from the
   estimator, not on differenced positions (user, s25.4).
4. Latency: time-stamp, propagate to now, and check the effect really reaches the estimate (a null result can be a bug;
   s25.5's check: 0.06 mm stamped vs 3.8 mm unstamped).
5. Decompose errors by direction: along the ring they were harmless. Here: roll about the ring axis vs axis tilt vs
   position, and which of them each sensor observes.
6. No unbounded phase: every wait / chase has a progress test and a time limit, ending in a safe action.
7. A new hook or law must reproduce the old behaviour at its neutral setting (offset 0, feed-forward 0) before tuning.
8. Runs small (2-4 seeds diagnosis, 25 tune seeds, slow-spin set), one at a time, 3 CPU workers; GPU at half load only
   if a GPU flight env is ever built.

**Steps.**
- **F0, diagnose (seeds 21, 3, and captured 0, 12 for reference):** log every 0.1 s through hold 3.2 -> station: the
  estimate vs truth (attitude split into roll about the ring axis / axis tilt, rate, COM position / velocity), R_d vs
  the bus attitude, the commanded vs delivered torque (wheel momentum and fill, unloading on / off, thruster
  saturation), the fix kind (full / ring / rejected) and the refit residuals. Answer: estimate error or tracking error;
  which axis; which phase (the box-to-ring hand-over at 2.2 m is the suspect).
- **F1, observability:** a table per range band of what each fix observes. If the drift is the unobserved roll: make
  the target independent of it. The station needs the ring axis and centre; the bus roll about the axis is free, so
  hold it inertially (or slave it to the estimate with a slow time constant) once box fits end, so an unobserved state
  cannot steer the bus. Keep roll for what needs it (the grasp azimuth: chosen while box fits still run).
- **F2, control on the full state:** check the law with the 150 ms latency (target propagated to the control time),
  add the angular-acceleration feed-forward of a nutating client (Euler's equations on the estimate), check wheel
  momentum and unloading margins on the failing seeds. Neutral check first: F1/F2 changes off -> today's 24 / 25.
- **F3, safe responses:** a progress / time limit per flight phase; on a station error, back off to the previous hold
  and retry once before aborting (a retreat is safe, an abort ends the mission). Thresholds from F0's normal seeds.
- **F4, envelope (CPU):** slow spinners (spin < 0.2 deg/s, the failing class: ~8 extra seeds), the tune seeds, spin up
  to 4 deg/s with nutation, latency 0 / 150 / 300 ms, inertia error. A GPU flight env (error-model nav, as G2) only if
  the CPU runs leave a question the sweep needs.
- **F5, evaluate:** the 25 tune seeds >= 24 / 25 at 0 collisions (target 25 / 25), the slow-spin set without the
  station abort, then the test split once.

**Not planned:** RL for the flight. Revisit only if F4 leaves a measured gap with signal (e.g. tumbling clients) that
the classical steps cannot close.

### 26.1 F0: the station attitude abort, diagnosed (23 Nov)
- **Run (`runs/f0_flight.json`, `--flight-log --flight-only`, seeds 21, 3, 0, 12; 1 Hz fixes, 150 ms latency):** seed
  21 aborts as always (7 mm, 8.5 deg); 3, 0, 12 reach the station. Log at 2 Hz (`ring_mission._flog`, truth read for
  diagnostics only): the estimate vs truth split into roll about the true ring axis and axis tilt, the ring-centre
  error, the rate error; the bus vs its target split the same way; torque, force, wheels, fix kinds.
- **It is the estimate, in roll about the ring axis, and only there.** Tilt stays 0.4-1.3 deg, the ring centre 9-45 mm
  (shrinking with range), tracking tilt <= 0.3 deg, position tracking <= 25 mm. Actuators are not involved: torque
  <= 0.43 N m, wheels <= 6 % full, no unloading; force saturates only for a moment at each phase start.
- **When:** the last box fix is at 61.6 s (range 2.2 m); from then on ring fixes only (they measure the axis and the
  centre, not the roll). The ring-only refit carries the rate of the last box fit (frozen: its roll-rate error stays
  constant, 0.004-0.05 deg/s) and holds the roll weakly to the prediction. At 81.6 s the last box fix leaves the 20 s
  fit window, and on seed 21 the estimated roll starts to move **in steps at each ring fix** (1-2 deg a fix): +10 deg
  (91 s), -3.7 (105 s), +17 deg (119 s), a ~35 s oscillation while the client is still (true rate 0.006 deg/s). The bus
  follows the target (target = estimated attitude x R_rel) and lags it: tracking roll up to 8.9 deg -> the abort.
- **Two effects in the roll:** (a) the frozen rate error ramps the roll (seeds 0, 3: 0.05 deg/s -> 4-6 deg by the
  station; seed 12 at 2 deg/s spin: 0.013 deg/s, <= 2.3 deg); (b) on the slowest spinners (21: 0.01, 3: 0.19 deg/s) an
  oscillation of +-5-17 deg that moves at the ring fixes and runs together with the bus's own roll. (b)'s exact path
  through the ring-only refit is not pinned down yet (each ring fix should carry the predicted roll; something in the
  fix or the refit couples the bus attitude into it): first item of F1.
- **The roll does not matter to the reach** (it moves the grasp point along the ring: s25.5's lesson); it matters only
  because the station target R_d follows the full estimated attitude, roll included.
- **F1, now concrete:** (1) find (b)'s path (an offline replay of the nav on the recorded fixes of seed 21); (2) make
  the station's attitude target independent of the unobserved roll: the target needs the ring axis and centre; the bus
  roll about the axis is free, so hold it on the last box-fix estimate (inertial, propagated with the box-fit rate)
  once box fixes end; (3) neutral check (the change off -> today's results), then seeds 21, 3, 0, 12 and the 25.

### 26.2 F1: the roll fix (23 Nov)
- **Path found (`runs/f1_diag21.json`, roll at each step of every fix, seed 21):** a ring fix carries the predicted roll
  unchanged (prior vs fix within 0.005 deg); the roll moves in the refit. From 82 s (no box fix left in the 20 s
  window) the ring-only refit's free 3-axis correction, held to zero only by the 0.05 weight, took 1-2 deg of roll a
  refit from the axis noise; each refit starts from the last fix (which carries the last result), so it accumulated,
  and the bus rolling after the target changed the lidar's view and the noise: the +-17 deg oscillation.
- **Fix 1, lock always (`RING_ROLL_LOCK`, ring-only refits correct only across the ring axis):** seeds 21, 3, 0, 12
  flight: tracking roll <= 0.34 deg (was <= 8.9), torque 3-14x lower, seed 21 reaches the station. **But the 25 tune
  seeds: 22 / 25** (`ring_mission_f1.json`): 11, 14 not settled (position 36-49 mm), 22 no grip. Lesson: the roll is
  unobservable only when the ring axis does not move. A client spinning across its ring axis sweeps it, and the axis
  history over the window then carries the full attitude through the dynamics; the free fit used that (the old
  comment: a frozen rate left 9 / 25 unsettled).
- **Fix 2, the lock gated on observability:** locked only when the axis sweeps < `SWEEP_MIN` over the window (the
  estimated rate across the axis x the window). 10 deg: seed 22 (11.5 deg) sat at the edge and still failed; **4 deg**
  (tune-seed sweeps: 21 0.2, 3 1.4 locked; 0 6.2, 22 11.5, 11, 14 free).
- **Mission, 25 tune seeds, all current changes (stereo, the s25.4 catch-up, 150 ms latency, gated roll lock;
  `runs/ring_mission_f1d.json`): 25 / 25 captured, 0 collisions** (first clean sweep; was 24 / 25). 21 of the 25 are
  identical to the run before the lock (the lock did not engage); 2, 3, 5 changed (locked), 21 now captured. Client
  push after capture <= 4.45 mm/s (seed 2; before <= 4.12).
- **Open:** the frozen-rate roll ramp (4-8 deg by the station on slow seeds) is tracked smoothly and harmless to the
  reach; the roll-free station target (F1 step 2) is not needed for the abort and not built. F3-F5 remain: slow-spin
  set, envelope, test split.

### 26.3 F3-F4: phase limits, the envelope, the spin rating (23 Nov)
- **F3, phase limits (audit):** every flight phase is bounded: holds 20 / 15 s, ramps of fixed length, settle <= 30 s
  (abort if not settled), the station error abort (5 s over 150 mm / 5 deg), no fix 15 s, the nav anomaly; the reach's
  descent has the s25.4 limits. Retreat-and-retry not built (no station abort left inside the rating; beyond it the
  limit is thrust, the same on a retry).
- **F4, envelope (full mission, 10 seeds each; flags `--spin`, `--lat`, `--i-err`):** slow spin 0-0.2 deg/s (seeds
  7602000+, the class that failed): 10 / 10; latency 300 ms (tune 0-9): 10 / 10; nav inertia error +-15 % (tune 0-9):
  10 / 10; **fast spin 2-4 deg/s (7603000+): 2 / 10, 8 safe aborts** (4 drifted > 200 mm off the 3.2 m hold, 4 not
  settled), 0 collisions anywhere.
- **Why fast spin fails: thrust.** 1.5 N per axis on the 196 kg bus = 0.0077 m/s^2; the station ~3.5 m from the
  spin axis needs w^2 r = 0.0043 (2 deg/s), 0.0067 (2.5), 0.0096 (3), 0.017 m/s^2 (4 deg/s). A design limit, not a
  control fault; more thrust or a closer spin-matching hold would move it.
- **Rating (user, 23 Nov: "keep our rated spin at 2 degrees, with max at 2.5 for fast ones"):** rated 2 deg/s (the
  tune / test distribution, 0-2), maximum 2.5 deg/s. New gate: after the 3.2 m hold, if the nav's rate (box fixes)
  exceeds `SPIN_MAX` 2.5 deg/s, abort before the approach ("spin beyond rating"; `spin_est_deg_s` recorded).
  Queued after the test split: 2-2.5 deg/s (10 seeds, 7604000+) and 2.6-4 deg/s (5 seeds, 7605000+, the gate).
- **F5, test split (seeds 7601000-24, first use, frozen code: `runs/ring_mission_test.json`): 24 / 25 captured,
  1 safe abort (not settled), 0 collisions.** (The gate was added after the run started; it cannot
  fire inside 0-2 deg/s.)

### 26.4 The spin limit: the physics (user, 23 Nov: "note the physics you used ... for later, if the thrusters change")
`checks/spin_limit.py` computes it from the scene and the bus file; rerun it when the thrusters, the bus mass /
inertia, the wheels or the hold distances change.
- **Setting:** the bus holds a point p_d fixed to a rigid, torque-free client spinning at w; rr = p_d - client COM
  (`ring_mission.station`: p_d = client origin + R (C_B + N_B (H_TOP + s)), v_d = v + w x rr, a_ff = w x (w x rr)).
- **Translation (the binding limit):**
  - centripetal: a_c = |w x (w x rr)| = w^2 r_perp, r_perp = the distance of p_d from the spin axis through the COM;
    worst case r_perp = |rr| (spin axis across the station line);
  - Coriolis while moving along the station line at V_APP: a_cor = |2 w x v| <= 2 w V_APP;
  - nutation: w_dot x rr, w_dot = I^-1 (-w x I w) (torque-free; small at these rates);
  - available: a_max = F_axis / m_bus per axis (`F_MAX`, the law's clip; the canted 16 x 1 N nozzles give more on
    some axes); the control needs its own share, so hold only if a_c + a_cor <= eta a_max (eta 0.85 here);
  - limit (worst axis): w_max = sqrt(eta a_max / |rr|); with the approach: w^2 r + 2 w V = eta a_max ->
    w_max = -V/r + sqrt((V/r)^2 + eta a_max / r).
- **Attitude:** the bus turns with the client: wheel momentum H = I_bus w (+ the gyroscopic w x I w, w x H);
  limit w_max = h_max / I_max (thruster unloading beyond).
- **Numbers today (196.2 kg, 1.5 N per axis -> a_max 0.00765 m/s^2; I_bus 14.4 / 24.4 / 26.6 kg m^2; wheels 12 N m s;
  V_APP 0.02 m/s; eta 0.85):**

  | hold | s | \|rr\| | w_max (thrust) | w_max (thrust + Coriolis) |
  |---|---|---|---|---|
  | 3.2 m hold | 3.20 m | 3.98 m | 2.32 deg/s | 2.05 deg/s |
  | 1.8 m hold | 2.00 m | 2.78 m | 2.77 deg/s | 2.39 deg/s |
  | station | 1.00 m | 1.78 m | 3.46 deg/s | 2.88 deg/s |

  At the 3.2 m hold (worst axis): 2.0 deg/s needs 63 % of a_max, 2.5 deg/s 99 %, 3.0 deg/s 143 %, 4.0 deg/s 254 %.
  Wheels: 26 deg/s (not binding; 2.5 deg/s stores 1.2 of 12 N m s).
- **What it means for the rating:** the rated 2 deg/s holds on the worst axis, including the approach's Coriolis
  (2.05). The 2.5 deg/s maximum (user) is holdable only when the spin axis lies partly along the station line
  (r_perp < |rr|); on the worst axis it needs 99 % of a_max at the 3.2 m hold. (s26.3's "~3.5 m, 0.0067 m/s^2 at
  2.5 deg/s" used a rough radius; the real |rr| is 3.98 m.) A gate on the required acceleration (the estimate's
  |w x (w x rr)| + 2 |w| V_APP against eta a_max) instead of the rate alone would admit exactly the holdable cases.
- **Levers if the design changes:** a_max scales with F_axis / m (2 x the thrust -> w_max x 1.41); a closer first
  hold scales w_max with 1 / sqrt(|rr|); a slower approach shrinks the Coriolis term.

### 26.5 The acceleration gate (user, 23 Nov: "yes, use acceleration")
- **Gate (`Mission.spin_gate`, after the 3.2 m hold, from the nav's box-fix state):** the worst over the approach
  (s 3.2 -> 1.0 m) of |w x (w x rr)| + 2 V_APP |w x n| against `ETA_SPIN` 0.85 x F_MAX / m_bus (6.50 mm/s^2), plus
  the 2.5 deg/s cap; logged per episode (`spin_gate`: w, a_need, a_lim). Truth-side check: tune and test seeds need
  <= 76 / 79 % of the limit (none rejected).
- **Physics check (2-2.5 deg/s, seeds 7604000-09, the rate-only gate, all approached; `runs/f4_fast25.json`):** 7 / 10
  captured, 3 not settled (4004, 4006, 4008), 0 collisions. All 3 failures need 109-130 % of the limit (truth);
  the 2 others above it (4000, 4001 at 102-103 %) captured: the 15 % control reserve covers a few %.
- **With the gate (same seeds, `runs/f4_fast25_acc.json`):** 5 captured, 4 gated before the approach (4001, 4006,
  4008, 4009), 1 not settled (4004), 0 collisions. The estimate decides near the edge: 4004 read 0.99 (truth 1.09)
  and went in, 4009 read 1.02 (truth < 1, captured without the gate) and was held back; the box-fix rate is good to
  a few % at the hold, so the gate's edge is +-~10 %.
- **2.6-4 deg/s (`runs/f4_over.json`):** 1 gated (2.84 deg/s, cap), 4 aborted in the 3.2 m hold itself (station error
  169-415 mm) before the gate's check at its end; all safe, 0 collisions.
- **Reading:** the physics predicts the failures (every not-settled case is above the limit); the gate turns late
  safe aborts into early ones (no approach on a client the bus cannot hold) at the cost of some marginal captures.
  Not tuned further (explore, don't over-optimise). Leads if wanted: check the gate earlier in the hold (the fast
  spinners fail the hold itself), widen ETA_SPIN, or a hysteresis band on the estimate's uncertainty.
- **Note (23 Nov close):** the 22 Nov plan named 7700000-024 as the P7 test split; F5 used 7601000-024 instead (also
  never used before, so still a clean split). 7700000-024 remains unused: keep it for the next frozen-code test.

### 26.6 CPU reach bench check (23 Nov)
- **Run (`checks/reach_bench.py --n 25`, seeds 7400000-24, `runs/reach_bench_s26.json`):** smoke on 7400000 passed, then
  **25 / 25 grips, 0 collisions, 0 aborts** (was 23 / 25 in `reach_bench_c` on the inboard cam, 18 / 25 in
  `reach_bench_ff`).
- **Metrics:** wrist cam 25 / 25 `disagree` (0 `no verdict`); E0 in-plane p50 / p95 / max 14.0 / 25.5 / 28.8 mm -> E1
  7.2 / 19.6 / 25.4 mm; pre-close radial p50 / p95 / max 3.0 / 9.3 / 11.25 mm (all inside the ~12 mm capture window).
- **Contact / hold:** all 25 / 25 stopped cleanly on `palm` (no hook / wrist F/T trips, no catch-up / time aborts);
  worst seat error 0.84 mm radial, 0.60 mm axial, 0.87 deg tilt; 3 N m hold slip <= 1.58 mm / 0.58 deg; client push
  <= 5.03 mm/s; peak arm torque <= 37 %. The side-mounted `wrist_cam` geometry from s25 drives the jump over
  `reach_bench_ff`.

### 26.7 Full GPU re-sweep: scales 1 & 2, stereo + catch-up + 150 ms latency (23 Nov)
- **Fix before sweep (`rl/gpu_reach.py`):** `self.n_lost` was checked for the s25.4 `"lost the grasp point"` abort but
  never incremented in `_descent_tick` (1-line parity fix with `reach_bench.reach_close`).
- **Smoke (`runs/g2_smoke_s26.json`, `--perc stereo --lat 0.15`, half GPU load):** no motion 22 / 22, mission range
  44 / 44, scale 2: 37 / 44 (was 31 / 44 in s25.1), 0 failed closes, 0 collisions.
- **Full sweep (`runs/g2_sweep_s26_stereo.json`, 2664 worlds = 72 cells x 37, scales 1 & 2, `--perc stereo --lat 0.15`,
  21.5 min at `DUTY = 0.5`, GPU ~86 C, 18214 steps):** **0 collisions across all 2664 worlds.**

  | set | s24.4 cal (net) | s25.1 stereo (no catch-up) | **s26 stereo + catch-up + 150 ms** | collisions | push p50 / p90 (mm/s) |
  |---|---|---|---|---|---|
  | Scale 1 ALL (1332) | 79.0 % | 91.8 % (2 coll) | **99.4 %** (1324 / 1332; 7 safe, 1 fail) | **0** | 6.54 / 10.66 |
  | N (w_ax <= 2, tilt <= 1; 444) | 98.6 % | 98.6 % (0 coll) | **99.5 %** (442 / 444; 1 safe, 1 fail) | **0** | 6.43 / 10.67 |
  | H1 (tilt >= 2 or w_ax = 4; 888) | 69.1 % | 88.4 % (2 coll) | **99.3 %** (882 / 888; 6 safe, 0 fail) | **0** | 6.62 / 10.66 |
  | H1 tilt >= 2, w_ax <= 2 (444) | 68.5 % | 99.1 % (0 coll) | **99.3 %** (441 / 444; 3 safe, 0 fail) | **0** | 6.59 / 10.60 |
  | H1 w_ax = 4, all (444) | 81.1 % (tilt<=1) | 77.7 % (2 coll) | **99.3 %** (441 / 444; 3 safe, 0 fail) | **0** | 6.68 / 10.74 |
  | **H2 / Scale 2 ALL (1332)** | 45.3 % | — | **71.3 %** (950 / 1332; 373 safe, 9 fail) | **0** | 8.14 / 14.80 |

- **Failure modes:** Scale 1: 4 `wrist_ft`, 3 `no contact`, 1 `grip, slips` (0 catch-up / time aborts). Scale 2: 270
  `wrist_ft`, 96 `no contact`, 7 `hook`, 5 `no grip`, 3 `grip, slips`, 1 `grip, off seat`. Push p90 fell from 20.0 ->
  10.66 mm/s on H1 and 23.1 -> 14.80 mm/s on H2.
- **Closure:** every G2/s24.4 draft gate is exceeded with margin (N 99.5 % vs >= 97 %, H1 99.3 % vs >= 80 %, H2 71.3 %
  vs 45.3 %, 0 collisions everywhere, push well below the base). The s25/s26 base is closed out; no RL gap remains on
  this grid.

## 27. End-to-end P7 calibration: far end and release (23 Nov; user: "end to end integration, calibrate the far end and release first")

### 27.1 Far-end calibration: sensors + 30 m -> 8 m flight + staged unfold (`checks/far_bench.py`)
- **Sensors (`far_bench.py sensors --n 25`, 6 ranges 30..3.2 m, `LIDAR_MAX = 60 m`, `runs/far_bench_sensors.json`):**
  - **Finding:** `client_p7` has a single 3-panel solar wing on `+Y` (2.7 m^2 vs 1.0 m^2 bus face, dark blue). Raw
    `far_nav.lidar_fix` median lands on the wing (lateral error p50 / p95 **1.46 / 1.80 m** at 8 m, **1.61 / 1.97 m**
    at 12 m, **0.55 / 1.46 m** at 30 m). Meanwhile `bus_cam` ignores the dark wing and locks onto the bright bus
    (lateral p50 **0.49 m** at 30 m, **0.19 m** at 8 m, 25 / 25 ok).
  - **Fix (`lidar_fix_gated`, `r_gate = 1.2 m` around camera bearing ray, `BLOB_BACK = 0.48 m`):** 25 / 25 ok at all
    ranges; lateral p50 / p95 drops to **0.42 / 0.74 m** (30 m), **0.29 / 0.43 m** (20 m), **0.30 / 0.43 m** (12 m),
    **0.25 / 0.42 m** (8 m); LOS p50 / p95 **0.042 / 0.151 m** at 30 m, **0.192 / 0.223 m** at 8 m.
  - **Hand-over range:** 12 m has 0 / 25 `ring_fit`; **8 m** has **25 / 25 `box_fit`** (118 mm / 1.45 deg p50) and
    **25 / 25 `ring_fit`** (118 mm / 1.79 deg p50; total incl. +-1 deg mount error).
- **Closed-loop flight (`far_bench.py flight --n 10 --workers 3`, `runs/far_bench_flight.json`, seeds 7510000-09,
  `FarKF` with anisotropic covariance + camera-gated lidar + 150 ms latency, rev-2 16 x 1 N thrusters + 4 x 12 N m s
  wheels, `v_max = 0.10 m/s`, staged arm unfold at 8 m):**
  - **10 / 10 ok, 0 collisions, 0 s stale** (all 10 converged at 30 m in **16.0 s**).
  - **8 m arrival (vs truth):** est pos error p50 / p95 **0.169 / 0.246 m**, est vel error **2.21 / 4.35 mm/s**, true
    range error **0.089 / 0.169 m**, settled speed **0.77-3.66 mm/s**.
  - **Staged arm unfold at 8 m:** max attitude excursion p50 / p95 **0.60 / 0.63 deg**, wheel fill <= **6.3 %**,
    propellant p50 / p95 **97.2 / 109.3 g** (`30 -> 8 m`).
  - **Remaining far-end piece:** 8 m attitude acquisition (lidar-only on `client_p7`, no bar corner net) + transfer
    arc onto the ring axis at 3.2 m.

### 27.2 Release calibration: ring tool v3 unlock, retract, rev-2 retreat & staged re-stow (`checks/release_bench.py`)
- **Finding on first smoke:** `unlock_open` (`t_open = 4 s`, thrusters off, wheels holding attitude) and straight-back
  `retract` (`0.20 m` along latched `+n_est` at `25 mm/s`) were spotless (**0.115 mm/s** push, **0 contacts**), but
  starting the joint-space `q_cmd -> ready -> stow` fold immediately at `1.0 m` swung `pad_l_0` into the ring and bus
  (404 contact steps at 500 Hz, 47.2 N peak, 34.5 mm/s push).
- **Fix:** hold the retracted arm pose `q_cmd` while rev-2 thrusters back the bus off `arm_clear_m = 0.8 m` along
  `+n_est` (putting bus >= 1.8 m from the ring top), then fold `q_cmd -> ready -> qb -> qa -> stow` during the
  remainder of the `2.2 m` min-jerk retreat (`v_back = 0.05 m/s`).
- **25-seed sweep (`release_bench.py --n 25 --workers 3`, `runs/release_bench.json`, seeds 7520000-24, latched error
  +-5 mm / +-0.5 deg, residual stack vel +-2 mm/s / +-0.05 deg/s, 500 Hz contact audit, 60 s wall):**
  - **25 / 25 ok, 0 collisions, 0 / 25 post-open contacts at 500 Hz** (`post_open_steps_max = 0`).
  - **Client push / tip-off:** `dv_tot` p50 / p95 **0.060 / 0.110 mm/s**, `dw_tot` **0.003 / 0.009 deg/s** (vs P6's
    4-16 mm/s release push).
  - **Rev-2 retreat & re-stow:** **0 plume pulses** across all 25 seeds (50 deg canted `-X` nozzles stay outside the
    30 deg cone), propellant p50 / p95 **15.2 / 15.4 g**, wheel fill <= **1.8 %**, stow error **0.04 deg**.

### 27.3 Full end-to-end P7 mission integration (`mission/p7_e2e_mission.py`, `perception/lidar_acq.py`)
- **8 m lidar-only 6-DOF acquisition (`perception/lidar_acq.py`):** isolates `client_p7`'s asymmetric `+Y` solar wing
  (`1.15 < dist < 4.5 m` from `FarKF`'s bus estimate), extracts wing-PCA axes, runs 8-angle `lidar_nav.fit` with a
  `ring_fit` bonus when `+X` faces the sensor. 25-seed random-360-deg self-test at 8 m: 1.48 deg / 119.8 mm median
  (17 / 25 within 10 deg across all 360 deg; mislocks are back-face `-X` flips / occluded-wing views).
- **End-to-end runner (`mission/p7_e2e_mission.py`):** one continuous truth-free MuJoCo episode:
  `far_hold (30 m) -> far_app (30 -> 8 m) -> staged unfold (stow -> qa -> qb -> ready -> q_app) -> acq8 (lidar_acq +
  15 s RelNav) -> xfer_to_hold3 (<= 60 deg chords on 8 m sphere onto ring axis, then 8 -> 3.2 m) -> ring_mission chain
  (hold3 -> spin_gate -> to1.8 -> hold1.8 -> to_station -> settle -> stereo_check with --stereo-tilt -> reach_close) ->
  detumble (rev-2 wheels to < 0.05 deg/s, pad grip monitor) -> calibrated release (unlock_open 4 s -> retract 0.20 m ->
  0.8 m bus clear -> 2.2 m retreat + reverse staged stow, 500 Hz contact audit)`.
- **1-seed end-to-end smoke (`runs/p7_e2e_smoke.json`, seed 7710000, spin 0.43 deg/s, 89.0 deg off-axis start):**
  - **`complete`, 0 collisions, 0 plume pulses, 0 post-open contacts at 500 Hz** (1301.2 s sim, 276 s wall, 417.3 g
    total propellant, max wheel fill 11.9 %).
  - Far hold conv 16.0 s; 8 m lidar acq err **3.37 deg**; `xfer_to_hold3` 89.0 deg arc -> settled at station
    **12.9 mm / 0.22 deg**; stereo + tilt cut `E0` 19.4 mm / 1.30 deg -> `E1` **0.3 mm / 0.55 deg**; closed seat
    **1.37 mm rad, 0.00 mm ax, 0.00 deg tilt**, push 1.26 mm/s; detumble to 0.057 deg/s (grip held); release push
    **0.024 mm/s**, tip-off **0.0024 deg/s**, re-stow error **0.04 deg**, end dist 3.91 m.
  - Note: first smoke caught that unfolding only to `servicer_ready` (not `q_app`) clipped `client_bus` during
    `to_station`; adding `ready -> q_app` to the 8 m unfold fixed it cleanly.
- **10-seed end-to-end validation (`runs/p7_e2e_val10.json`, seeds 7710000-09, 3 workers, 20 min wall):**
  - **8 / 10 `complete`, 2 safe aborts (`abort: hold3: no fix`), 0 collisions, 0 plume pulses.**
  - **10 / 10** passed `far_hold` (16 s), `far_app`, `unfold`, and truth-free 8 m lidar acquisition (`acq8_init` median
    **1.39 deg**, up to 164 deg off-axis); **8 / 8** that reached `hold3` completed capture + detumble + release
    (7 / 8 had 0 post-open contacts at 500 Hz, 1 had a single 2.09 N step; `--stereo-tilt` cut `E0` 2.6-32.6 mm ->
    `E1` **0.3-3.3 mm**; release push p50 / p95 **0.053 / 0.637 mm/s**; total prop p50 / p95 **477 / 575 g**).
  - **Root cause of the 2 safe aborts (seeds 7710001 at 1.97 deg/s, 7710007 at 1.99 deg/s):** `fly_body_leg` flew the
    8 m transfer *co-rotating in the client body frame* at `r = 8 m`, which requires centripetal acceleration
    `w^2 r = 9.75 mm/s^2 > 7.65 mm/s^2` (`F_MAX / m`), flinging the bus 13-15 m off before `hold3`.
  - **Fix applied (`fly_inertial_leg` + 120 s detumble):** fly `<= 55 deg` inertial chords on the 8 m sphere, then a
    90 s inertial cubic-Hermite rendezvous leg (with closed-loop `M.nav.predict(t_arr)` retargeting) arriving at the
    3.2 m hold point with matching tangential velocity `w x r` (synchronizing only at 3.2 m, per D7); detumble cap
    raised `60 -> 120 s`. Re-test on the 2 failing seeds (`runs/p7_e2e_fix2.json`, `runs/p7_e2e_fix7.json`):
    - **`7710001` (1.97 deg/s):** now **`complete`**, 0 collisions, 0 plume pulses, 0 post-open contacts, `hold3`
      40.5 mm / 0.38 deg, `E1` 0.1 mm / 0.13 deg, detumble 0.084 deg/s, release push 0.050 mm/s, propellant cut **2.5x**
      (**306.2 g** vs 773.9 g).
    - **`7710007` (1.997 deg/s, `a_need = 95 %` of `a_lim`):** propellant cut to 302.2 g, safe abort at `hold3`
      (`2.0 m / 21.2 deg`, 0 collisions; near-worst-axis thrust edge, not tuned further).
    - **Combined 10-seed end-to-end rate: 9 / 10 `complete`, 1 safe abort, 0 collisions, 0 plume pulses.**

### 27.4 Frozen 25-seed end-to-end test split (`runs/p7_e2e_test25.json`, commit `f1a1af5`, seeds `7700000-24`)
- **Result (3 workers, 43.7 min wall):** **9 / 25 `complete`, 12 safe aborts, 4 collisions** (all 16 failures and all 4
  collisions occurred inside `xfer_to_hold3` / its hand-off to `hold3`).
- **What passed cleanly:**
  - **25 / 25** passed `far_hold` (16 s), `far_app` (`30 -> 8 m`), and staged `unfold` (`0` collisions).
  - **22 / 25** locked attitude at 8 m via truth-free `lidar_acq.acquire` (`0.48-22.8 deg`, median `1.2 deg`; 3 mislocks
    at `91-178 deg`).
  - **9 / 9 (100 %)** that passed `hold3` completed `to1.8 -> hold1.8 -> to_station -> settle -> stereo+tilt capture ->
    detumble -> release -> re-stow` with **0 collisions, 0 plume pulses**, prop p50 **296.9 g**, release push p50
    **0.060 mm/s** (7 / 9 had 0 post-open contact steps at 500 Hz).
- **Three exact bugs exposed in `xfer_to_hold3` (`fly_inertial_leg`, written right before freeze without a 10-seed
  re-run):**
  1. **Late roll slew (8 aborts):** `align_rel` turned on only at `u > 0.5` (45 s before `hold3`), jumping `R_d` up to
     180 deg in roll; rev-2's `0.2 N m` wheels (`0.43 deg/s^2`) arrived at `hold3` still mid-slew (`15-133 deg` off)
     while position was `8-163 mm`, tripping `hold3`'s 5 s abort immediately.
  2. **Hermite curve bow into client/wing (all 4 collisions):** `T_in = 90 s` x `v_arr = 0.14 m/s` gave a `12.6 m`
     tangent handle (`h11 * T * v1`), bowing the trajectory inside the `3.84 m` solar wing (`wing_r_panel1/3`,
     `client_bus`) with no keep-out radius enforcement during `xfer_to_hold3`.
  3. **8 m single-frame mislocks (3 aborts: `7700001`, `7700007`, `7700012`):** `lidar_acq.acquire` locked onto a
     `91-178 deg` flip without confirming `ring_fit.ok` on arrival at the `+X` face.
- **Next fix (before next clean split `7720000-24`):** replace the 90 s Hermite leg with a keep-out-guarded radial
  approach (`||r|| >= 5.0 m` until within `25 deg` of `+n`), start `R_rel` roll alignment at `u = 0` (at 8 m), add a
  `ring_fit` verification gate on the `+X` face at 8 m, and allow a 20 s pull-in settle at `3.2 m` before arming
  `hold3`'s 5 s abort timer.
- **v3 keep-out + lidar-pointing fix (`runs/p7_e2e_v3_val10.json`, seeds `7710000-09`, 3 workers, 18.5 min):**
  - Implemented pure min-jerk inertial transfer with `min_r = 6.0 m` outside the 25 deg `+X` cone (zeroing inward
    `vr`/`ar` on clamp), continuous client-centred lidar pointing `FB._point_R(bore_b, -rr, (Rw @ R_rel)[:, 2])`,
    8 m roll pre-slew + gated `ring_fit` check, and 35 s `pull_in3` settle.
  - **Result:** **6 / 10 `complete`, 4 safe aborts, 0 collisions, 0 / 6 post-open contacts at 500 Hz**, prop p50
    **312.8 g** (down from 477 g), release push p50 **0.056 mm/s**.
  - **Clean spin split:** **6 / 6 (100 %)** on spins `<= 1.11 deg/s`; all 4 safe aborts (`1.22-1.99 deg/s`) had
    attitude `0.21-0.79 deg` (roll fix worked!) but `min_r = 6.0 m` clamped the `65 s` descent until `u ~ 0.85`
    while the fast-spinning `+X` face swept into view. Next refinement if desired: stage at `r = 6.0 m` and run a
    shorter `30 s` (`2.0 m`) radial intercept into `3.2 m`.
