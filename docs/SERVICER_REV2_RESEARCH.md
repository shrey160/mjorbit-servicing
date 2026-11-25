# Rev-2 servicer: brake both ways, grip by geometry

The rev-2 servicer for Project 7 should stay an ESPA-class vehicle and change how it pushes, turns and holds. The recommended design point is **about 200 kg wet** on a standard 15 in ESPA port, with the arm stowed in a flat fold that a FreeCAD check puts at **645 mm deep against 710 mm allowed**. It flies **16 green-monopropellant 1 N thrusters in four recessed mid-depth edge pods**, with the X nozzles canted 30 deg outward, so it can push both ways on every axis and make pure couples about every axis. Attitude comes from **four 12 N m s / 0.2 N m reaction wheels in a pyramid**, modelled in every phase and unloaded by real thruster couples. It keeps the 1.18 m six-joint arm on the client-facing face, and the new ring end effector holds the 610 mm launch adapter ring **by form closure** (jaws behind the lip and a palm on the separation face, after US 10,414,053), simulated at **mu 0.4 with a 0.2 stress case** instead of today's 1.5. The project's own calculations show that the three numbers the researchers flagged as inconsistent are consistent. All three come from the grasp lever arm and from time spent synchronised: the ~20 N m s detumble, the 1.2-2 m/s fly-in cost and the 84 N m s stack-move momentum. So the cheapest gains are in the approach. Synchronise only from a new 3 m stop, pick the pre-grasp point near the spin axis, and make thrust pass through the stack CoM. The evidence is thin in places. Most web numbers came from search snippets or third-party catalogues, and no small-arm datasheet, ring-lip drawing or vacuum friction coefficient was found. Every value below carries its confidence label, and the verify-before-building list names the numbers that most need a primary datasheet.

## The rev-2 baseline in one table

Each value below is sized from a datasheet-class number or from a project calculation, and the basis column says which. "Project calculation" means first-principles arithmetic on the project's measured demands (the coordinator sizing checks of 20 Nov), not a sourced fact. "Project choice" means a design decision with no source behind the number.

| Item | Rev-2 baseline | Today | Basis (confidence) |
|---|---|---|---|
| Launch class | ESPA standard, 15 in port; stowed envelope 610 x 710 x 970 mm | stowed depth 0.81 m (breaks the envelope) | ESPA volume 24 x 28 x 38 in ([Wikipedia, citing AIAA 2001](https://en.wikipedia.org/wiki/EELV_Secondary_Payload_Adapter), secondary); stow fit from a FreeCAD check (project calculation) |
| Wet mass | **200 kg design point**: ~150 kg sum of parts + ~30% margin; mass properties from parts | 150.1 kg, uniform-density box | Port capacity 220 kg heritage / 257 kg MAC ([Moog](https://moog.com/products/spacecraft-payload-interfaces/espa.html), vendor page); older 180 kg secondary limit conflicts (verify) |
| Bus | 0.50 (x) x 0.60 x 0.90 m, **collision geometry on**, long edges chamfered/recessed for pods | box without contacts | Audit 6; stow check leaves only 5 mm a side |
| Propulsion | Green monoprop (LMP-103S class), 1 N nominal, 0.25-1 N over blowdown, Isp ~220 s; ~4 kg propellant for 40 m/s | ideal +-0.75 N per world axis | ECAPS 1N HPGP: 0.38 kg, 204-231 s ([SatNow](https://www.satnow.com/products/thrusters/ecaps/36-1262-1n-hpgp), third-party); ELSA-d flew 8 x 1 N HPGP ([eoPortal](https://eoportal.org/satellite-missions/elsa-d)) |
| Thruster layout | **16 nozzles, 4 pods** at the long side edges at mid-depth; each pod +X, -X (both canted 30 deg outward), inward Y, inward Z | 4 pods, all on the aft face | Linear-program check: rank 6, pure force ~3.4 N per axis, pure couples 1.0-1.7 N m (project calculation) |
| Wheels | **4 x 12 N m s, 0.2 N m, ~5 kg** in a pyramid (54.7 deg): 19.6 N m s / 0.33 N m about X and Y, 27.7 / 0.46 about Z | 1 N m, 30 N m s, ideal torque in the grasp phase | [Rocket Lab RW-12 datasheet](https://rocketlabcorp.com/assets/Uploads/RW-12-Datasheet-v3.5.pdf) (momentum, torque); mass third-party; pyramid envelope is a project calculation |
| Unloading | Thruster couples when any wheel passes 80%; magnetorquers as a design note only | pure 0.5 N m couple at a 1 m lever | Real allocated couples (project calculation) |
| Arm | 6 joints, 0.55 + 0.45 + 0.13 m, 1.18 m reach; shoulder on the +X face 0.25 m below centre; flat-fold stow; **10 deg/s enforced**; ~20 kg with ring tool | 16.55 kg; speed limit not enforced | 10 deg/s on CAESAR and e.Deorbit ([DLR CAESAR](https://www.dlr.de/en/rm/research/robotic-systems/arms/caesar)); stow check |
| End effector | Ring tool: outboard + inboard jaws, palm on the separation face, two-stage close, **non-backdrivable lead-screw drive**, pad depth <= 45 mm, jaw path matched to the 609.6 mm ring | parallel slide jaws, soft servo, 30 N cap | [US 10,414,053](https://patents.google.com/patent/US10414053); [NASA GSC-TOPS-190](https://technology.nasa.gov/patent/GSC-TOPS-190); clamp force set in the physics check (no sourced value) |
| Grip friction | **mu 0.4 nominal, 0.2 stress, 0.8 jam check** | 1.5 | Judgement band; no vacuum measurement found |
| Sensors | Keep the +X bench (512 px 40 deg camera + 128 x 128 45 deg flash lidar, 60 m); add a narrow camera and an LWIR option; wrist and palm cameras; 2 star trackers; tactical IMU; wrist F/T, joint torque, pad load cells | bench + wrist + palm cameras; state from the sim | Flown RPO suites pair lidar with visible and IR cameras (sources in the sensors section) |
| Power | +-Y body cells, 100-150 W in sun; energy ledger logged per phase, no gating | no model | Design note (see decision 6) |
| Thermal | Radiator on -Z; design note only | visual only | No sizing source found |

## Three "inconsistent" demands come from one lever arm

The researchers flagged three demands from earlier runs as inconsistent with simple checks. The project calculations show all three are consistent, and the explanation drives the whole redesign. These reconciliations are project calculations and take precedence over the flags in the research notes.

**The ~20 N m s detumble momentum comes from the grasp lever, not the client's spin.** A uniform 300 kg, 1.5 x 1 x 1 m box at 2 deg/s carries only **1.7-2.8 N m s** about its own centre. That is the figure that looked too small next to the measured ~20 N m s. After a rigid grasp, though, the stack turns about the combined CoM, and the relative motion adds a reduced-mass term mu d^2 w, with mu = m_s m_c / (m_s + m_c). With a 150 kg servicer and 2.0-2.5 m between the two centres of mass, that term alone is **14-22 N m s**. With a 200 kg servicer it is **17-26 N m s**, plus the client's own ~2.8. The wheel sizing driver is therefore the square of the grasp distance. A shorter grasp distance cuts the wheel requirement faster than any change in the client.

**The 1.2-2 m/s fly-in cost is acceleration held over time, not a velocity match.** The grasp point does move at only w r, about 0.06 m/s. But to stay at a body-fixed point r from a client CoM spinning at 2 deg/s, the servicer must accelerate continuously at w^2 r: **2.2 mm/s^2 at 1.8 m, 3.05 at 2.5 m, 3.7 at 3.0 m**. Delta-v is the integral of |a| dt, and 3 mm/s^2 for 400-700 s gives **1.2-2.1 m/s**, matching the measured median of 1.2 and maximum of 2.07 m/s. The cost scales with time spent synchronised and with r. Synchronising only for the last metres, or placing the pre-grasp point near the spin axis, cuts it directly.

**The 84 N m s stack-move figure is three terms added together.** It is detumble (up to ~30), identification pulses, and **holding attitude against off-CoM thrust (~0.7 N m x 40 s = 28)**. The last term disappears once thrust passes through the stack CoM. The first falls with a shorter grasp lever and with real unloading. Neither needs bigger wheels.

**Friction alone is the wrong holding principle, though the loads are smaller than the researchers assumed.** One note assumed 0.05-0.1 m/s^2 stack loads, which would need 15-30 N through the grip. In practice 2 N per axis on a 450 kg stack gives ~4.4 mm/s^2, so the client needs only **~1.3 N** through the grip. The larger grip loads come from detumble torques acting through a 1-2 m lever. The runs measured pad force dropping to **0.34-0.87 of the grasp value** under post-capture loads at mu 1.5. At a realistic mu of 0.3-0.6 that margin shrinks 2.5-5x. The conclusion holds in a weaker form: the gripper should hold by geometry and use friction only for the soft first stage.

## A 200 kg ESPA bus with a flat-folded arm fits the standard port

**The port mass limit is the most important unresolved number in the design.** Moog lists the standard ESPA 15 in port at **257 kg under the ESPA mass acceleration curve and 220 kg with heritage load factors**, with the CoG within 50.8 cm of the port ([Moog](https://moog.com/products/spacecraft-payload-interfaces/espa.html)). An older summary citing a 2001 AIAA paper gives **180 kg per secondary payload** and the **610 x 710 x 970 mm** volume the project already uses ([Wikipedia](https://en.wikipedia.org/wiki/EELV_Secondary_Payload_Adapter)). If the true limit is 180 kg, ESPA Heavy keeps the same 15 in ports with **322 kg heritage / 450 kg MAC capacity** ([Moog](https://moog.com/products/spacecraft-payload-interfaces/espa.html)), so the redesign does not depend on the answer. ESPA Grande, with 24 in ports, is quoted at both 700 kg (Moog) and 320 kg (archived Moog page and Momentus, via Wikipedia), with a volume of 1100 x 1200 x 970 mm. That conflict is unresolved. SpaceX plate tiers (a full plate with 200 kg included) were seen only in a search snippet and are UNVERIFIED.

**Flown and planned vehicles put a capture-capable small servicer near 150-200 kg, though only ELSA-d is well sourced.** Astroscale's ELSA-d servicer was **about 175-180 kg** with **eight 1 N HPGP green-monoprop thrusters** and magnetic capture, no arm ([eoPortal](https://eoportal.org/satellite-missions/elsa-d)). ADRAS-J carried **12 x 1 N ADN monopropellant thrusters** ([Gunter's Space Page](https://space.skyrocket.de/doc_sdat/adras-j.htm)). Its ~150 kg wet / 120 kg dry mass appears only on a tracking site and is UNVERIFIED. The ClearSpace-1 servicer was quoted at **580 kg wet in a 1600 x 1300 mm envelope** for a ~95 kg target (UNVERIFIED, slide snippet; [ESA Clean Space Days](https://indico.esa.int/event/644/contributions/12346/attachments/7636/14732/CleanSpaceDays_ADRIOS_ClearSpace1_Woicke.pdf)). That is the strongest argument for going bigger, but it carries a full deorbit budget (a 350 m/s servicer delta-v in another ESA deck) that P7 does not need. MEV, at 2.3-2.9 t, is a scale reference only. No arm-equipped microsat with a published mass was found, so the arm mass rests on the project's own joint-module model.

**A rough parts budget lands at about 150 kg before margin.** No sourced budget fractions were found, because the NASA Small Spacecraft State of the Art 2026 PDF was too large to read ([NASA SST-SOA](https://www.nasa.gov/smallsat-institute/sst-soa/)). The split below is a project estimate, labelled as such:

| Subsystem | Estimate | Basis |
|---|---|---|
| Arm + ring tool | ~20 kg | today's 16.55 kg arm; a two-stage ring tool is heavier than parallel jaws |
| ADCS | ~23 kg | 4 x ~5 kg wheels (third-party RW-12 mass) + 2 star trackers 0.2-0.35 kg + IMU + electronics |
| Propulsion, dry | ~14 kg | 16 x 0.38 kg thrusters = 6.1 kg, plus tank, valves and lines (estimate) |
| Propellant | ~4 kg | 40 m/s at 230 s from 220 kg = 3.9 kg (project calculation) |
| RPO sensors | ~10 kg | flash lidar class 6.7 kg ([ASC GSFL-16KS](https://asc3d.com/gsfl_16KS)) + cameras |
| Power, avionics | ~21 kg | body cells, battery, Q8-class computers (estimate) |
| Structure, thermal, harness | ~55 kg | ~35% of dry (estimate) |
| **Sum / with margin** | **~147 kg / ~200 kg** | ~30% margin for an experimental concept |

The bus should get its mass properties from parts, as the clients do. A uniform 200 kg box of today's size has inertias of about 19.5 / 17.7 / 10.2 kg m^2 (project calculation). A real bus is lower, because propellant, battery and wheels sit near the centre.

**The arm can stow inside the envelope. Today's 0.81 m stow depth comes from the stow pose, not the link lengths.** A FreeCAD concept check (project calculation, figure below) put the shoulder module 250 mm below centre on the +X face. In the check, the upper arm folds up the face, the elbow folds 180 deg, and the forearm comes back down 140 mm to the side, with the wrist and a 140 x 200 x 160 mm gripper at the bottom. Result: **645 mm stowed depth from the aft port face against 710 allowed**, width 600 against 610, height 900 against 965, and no part intersecting the bus. That leaves **65 mm of depth** for launch locks and the sensor bench, and **only 5 mm a side in width**. The thruster pods on the long edges must therefore sit in chamfered or recessed corners. The check did not cover whether the joint ranges reach the 180 deg elbow fold, cable routing, or the bench's view with the arm stowed.

![Arm folded flat on the +X face inside the ESPA envelope (FreeCAD concept check, 20 Nov)](<../research_notes/Servicer rev 2 research for P7/stow_check.png>)

*Figure: concept stow check. Grey: 500 x 600 x 900 mm bus. Orange: arm in a flat fold. Wireframe: ESPA envelope. File: ../research_notes/Servicer rev 2 research for P7/stow_check.png*

## Sixteen canted 1 N thrusters give every force and every couple

**Thruster magnitude was never the problem; layout was.** A 1 N thruster gives 5 mm/s^2 on 200 kg. Holding a body-fixed point near a client spinning at 2 deg/s needs 0.45 N at 150 kg, 0.60 N at 200 kg and 0.75 N at 250 kg (project calculation). **Two 1 N thrusters per direction (2 N per axis) give about 3x margin at 200 kg.** Today's +-0.75 N per world axis is the right order but physically wrong: every rev-1 nozzle sits on the aft face, so the bus cannot push toward the client's side of itself, which braking, back-off and retreat all require, and it cannot make a pitch or yaw couple without a net force.

**Green monopropellant is the plausible default; cold gas is the clean-plume alternative.** The 1 N HPGP class (LMP-103S) is listed at **0.25-1 N, 204-231 s Isp and 0.38 kg** across third-party catalogues ([SatNow](https://www.satnow.com/products/thrusters/ecaps/36-1262-1n-hpgp); [satsearch](https://satsearch.co/products/ecaps-1n-hpgp-thruster)), and it flew on ELSA-d. The 0.25-1 N range is consistent with blowdown feed pressure (inference), so the simulation should include a low-pressure case. Hydrazine 1 N units are equivalent for the simulation: the Nammo MHT-1N is quoted at 228 s and under 0.45 kg ([Nammo](https://www.nammo.com/products/space/space-propulsion-thrusters-and-engines/mht-1n/), via search summary). Cold gas is credible at this thrust: a Moog unit gives 0.9 N on GN2 at 70 s Isp ([Moog datasheet](https://www.moog.com/content/dam/moog/literature/sdg/space/propulsion/moog-coldgasthrusters-datasheet.pdf), snippet). It has fast valves and no plume chemistry on the client or the cameras, but needs roughly **25-40 L of tank at 300 bar** for 15-25 m/s (project calculation). Electric and water propulsion are excluded: Hall thrusters give ~40-60 mN/kW ([NASA SOA propulsion 2022](https://www.nasa.gov/wp-content/uploads/2023/05/4.-soa-in-space-propulsion-chapter-2022.pdf)) and are far too weak for proximity operations.

**Propellant mass is not a driver for monoprop.** From a 220 kg start, 15 / 25 / 40 m/s costs 1.5 / 2.4-2.5 / 3.9-4.0 kg of monoprop, against 5.1 / 8.5 / 13.4 kg of GN2 (project calculation). At 1.2-2 m/s per synchronised fly-in, ten attempts cost about 20 m/s, so a **40 m/s proximity budget (~4 kg)** covers ten attempts, close-range trims, the stack move and retreat with margin. The 40 m/s figure is a project choice, and orbit transfer is outside it.

**The layout is four mid-depth edge pods with four nozzles each.** The pods sit on the four long side edges of the bus at x = 0, y = +-0.30, z = +-0.45 m. Each pod has a +X-force nozzle, a -X-force nozzle, an inward-Y nozzle and an inward-Z nozzle. A linear program with every thruster bounded 0..1 gives the maximum pure wrench in units of one thruster (project calculation):

| Cant of X nozzles | pure Fx | pure Fy, Fz | pure Mx | pure My | pure Mz |
|---|---|---|---|---|---|
| 0 deg | 4.0 | 2.0 | 1.5 | 1.8 | 1.2 |
| 15 deg | 3.86 | 2.73 | 1.61 | 1.74 | 1.16 |
| **30 deg** | **3.46** | **3.41** | 1.71 | 1.56 | 1.04 |

The allocation matrix has full rank 6: pure force on every axis and pure couples about every axis, both signs. With 1 N thrusters that means **>= 2 N per axis and >= 1 N m couples**, against the 0.5 N m unloading couple assumed today. **A 30 deg outward cant is recommended over 15 deg.** It nearly equalises pure force across the three axes (3.46 / 3.41 / 3.41 N), and it points the plume of the -X-force nozzles, which fire toward the client, about 30 deg off the approach axis. That fits NASA's rule of thumb that a surface **30-40 deg or more off a plume centreline is nearly free of plume-particle damage** ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20100019171/downloads/20100019171.pdf), snippet). The cost is 13% of axial thrust. Mid-depth pods also keep nozzles off the +X face, where the sensors and arm sit, and off the aft face.

**Redundancy is a real requirement, not polish.** ELSA-d lost four of its eight thrusters and had to replan on the remaining four ([Satellite Today](https://www.satellitetoday.com/technology/2022/05/04/astroscale-elsa-d-mission-makes-close-approach-with-client-spacecraft)). With 16 nozzles, the physics check should confirm that the allocation matrix stays full rank after any single thruster failure, at reduced authority. The ADRAS-J arrangement of 8 diagonal fine-control and 4 straight thrusters (UNVERIFIED, press-kit snippet) is a 12-thruster precedent if mass gets tight.

**After capture, re-allocate about the estimated stack CoM instead of gimballing.** With a 200 kg servicer and the 300 kg client's CoM about 2.3 m ahead (0.25 m bus half-depth + 1.3 m grasp + 0.75 m client half-length), the stack CoM sits **~1.38 m ahead of the servicer centre** (project calculation). Symmetric pods put an X force through it when the client CoM lies on the servicer X axis. Lateral forces need a matching couple over the 1.38 m lever, which this layout can supply. Re-running the allocation on the estimated stack CoM (`cm_nav`) removes the ~28 N m s of the 84 N m s that came from holding attitude against off-CoM thrust. No public detail was found on how MEV keeps thrust through the stack CoM ([Wikipedia MEV](https://en.wikipedia.org/wiki/Mission_Extension_Vehicle)), so this method is the project's own.

## Four 12 N m s wheels suffice once thrusters make real couples

**The 1 N m wheel torque in today's model is about 5x what datasheet-class wheels in this size give; its 30 N m s capacity is plausible.** The 12 N m s class gives **0.2 N m at ~5 kg** (Rocket Lab RW-12: 12 N m s and 0.2 N m from the [vendor datasheet](https://rocketlabcorp.com/assets/Uploads/RW-12-Datasheet-v3.5.pdf), mass from [SatNow](https://satnow.com/products/reaction-wheels/rocket-lab/38-1192-rw4-12-0)). Blue Canyon's RW8 gives 8 N m s / 0.25 N m / 4.4 kg ([SatCatalog](https://www.satcatalog.com/component/rw8/), third-party), and Bradford's W18 gives 18 N m s / 0.265 N m ([Bradford datasheet 2019](https://static1.squarespace.com/static/603ed12be884730013401d7a/t/6054f630baf06f76bbabe02a/1616180789682/be_datasheet_rwu_2019dec.pdf)). The Honeywell HR14 offers 25 / 50 / 75 N m s at 0.2 N m, **7.5 kg and 22 W average** for the 25 N m s unit ([Basilisk datasheet model](https://hanspeterschaub.info/basilisk/Documentation/utilities/datasheet_RW.html); [SatCatalog HR14-25](https://satcatalog.com/component/hr14-25)), but that rests on an old Honeywell PDF. Small CMGs answer a torque problem P7 does not have. The Airbus CMG 15-45S gives 45 N m but weighs **18.4 kg per unit** and targets ~1000 kg satellites ([SatNow](https://www.satnow.com/products/control-moment-gyroscopes/airbus/43-1213-cmg-15-45-s)).

**Four 12 N m s wheels in a pyramid cover the nominal detumble; thrusters top up the worst cases.** In a standard pyramid (wheels 54.7 deg from Z), the per-axis capacity is 2h sin(beta) = 1.63h about X and Y, and 4h cos(beta) = 2.31h about Z. With 12 N m s / 0.2 N m wheels that gives **19.6 N m s and 0.33 N m about X and Y, and 27.7 N m s and 0.46 N m about Z**, with single-failure tolerance (project calculation). The measured momentum entering the move is a median of 14 and a maximum of 26 N m s. A 200 kg servicer raises the lever term to 17-26 + 2.8 N m s, and a rough stack inertia of ~730 kg m^2 at 2 deg/s gives ~25 N m s (project calculation). So the wheels absorb a typical detumble, and in the worst cases they reach the 80% threshold and hand over to thruster couples. The thruster couples now exist (>= 1 N m). Unloading by RCS in a separate mode is standard practice ([NASA ISSFD 2007](https://www.issfd.org/ISSFD_2007/2-4.pdf)), and it is cheap: 20 N m s at a 0.4 m effective lever is ~50 N s, about **23 g** at 220 s, matching the 25 g measured in earlier runs. At 0.33 N m, a 2 deg/s stack detumbles in roughly 80 s on wheels alone (project calculation).

**The alternative is wheels-only detumble with 25 N m s wheels.** Four HR14-25-class wheels give ~41 N m s about X and Y and never need thrusters while the client is held. That keeps plumes away from the captured client during the most delicate phase, at a cost of ~10 kg more and 22 W per wheel. **This is the switch to make if, in the physics check, the captured-state allocation cannot form couples without firing the -X nozzles at the client.**

**Magnetorquers are a between-mission tool here.** A 2 A m^2 rod gives about 40-120 uN m in a 20-60 uT field ([Space Inventor](https://supersecret.space-inventor.com/modules/magnetorquer) for the rod rating; torque is a project calculation). At even 1e-3 N m, dumping 84 N m s takes about 23 h. The stack's dipole is also unknown while a client is attached. Record magnetorquers as a design note and do not model them in the capture chain.

## The ring gripper must hold by geometry, not by friction

**Two published ring tools share one topology: jaws on both sides of the flange and a datum against the separation face.** The NRL / Northrop gripper of US 10,414,053 grasps a section of the Marman ring with **outboard and inboard jaws and a palm on the ring's separation surface**. It closes in two stages: the jaws close parallel to the palm, then draw the ring down against it ([Google Patents](https://patents.google.com/patent/US10414053)). NASA's Goddard gripper for OSAM-1, licensed to Northrop, **constrains all six degrees of freedom** when closed ([NASA GSC-TOPS-190](https://technology.nasa.gov/patent/GSC-TOPS-190)). ESA's e.Deorbit planned arm capture and arm detumble, then a SENER rotary clamp locking onto Envisat's ring ([Satellite Today](https://www.satellitetoday.com/innovation/2018/04/27/esa-selects-seners-clamping-mechanism-to-help-combat-space-debris/)). None of these sources publishes clamp force, jaw size or misalignment tolerance. The ring geometry is only partly known: the 24 in interface has **36 x 1/4 in bolts on a 609.6 mm circle**, and the PAS 610S system is 73 mm high and 5.8 kg with 15 kN bolt preload (UNVERIFIED, search text of a [Beyond Gravity datasheet](https://www.beyondgravity.com/sites/default/files/media_document/2024-01/Beyond-Gravity-PAS-610S-Separation-System.pdf)). **No lip height, lip thickness or allowable flange load was found.**

**So the rev-2 end effector replaces the parallel pinch with a form-closure ring tool.** It closes in a soft stage (jaws contact the lip, friction and the compliant wrist absorb the residual motion) and then a hard stage (jaws behind the lip draw the ring onto the palm datum, after which geometry carries the load). The drive should be a **non-backdrivable lead screw**: the jaw holds its position unpowered, and the soft-servo force drop seen in the runs (to 0.34 of grasp) cannot happen. This is a buildable part in the "would need building" sense: the topology is patented and flown on MRV-class hardware, but no small-scale version has a datasheet. Keep the pad depth into the flange at <= 45 mm (project constraint) and match the jaw path to the 609.6 mm circle.

**The grip friction should be simulated at 0.4, with 0.2 as the stress case.** No measured vacuum friction for any candidate pad material against anodised aluminium was found. What exists points away from 1.5. Vespel SP-3 (MoS2-filled) is specified for **low-friction** vacuum slide pads ([Curbell](https://www.curbellplastics.com/materials/plastics/dupont-vespel-sp-3/)). Graphite-filled grades are poor in vacuum ([Curbell slides](https://www.curbellplastics.com/wp-content/uploads/2023/03/Solving-Friction-and-Wear-Challenges-Slides.pdf)). Gecko-type dry adhesives reach ~75-80 kPa shear, but on glass and without vacuum data ([Stanford BDML](https://bdml.stanford.edu/pmwiki/pmwiki.php/Main/AdhesionOuterSpace)). The audit's 0.3-0.6 for metal on anodised aluminium is a judgement band; elastomer at ~1 is a judgement ceiling. Since the design holds by geometry, friction matters in two places only. The first is the soft stage, where low mu lets the ring slide before the lip catches. The second is the draw-in, where high mu could jam the ring short of the palm. Hence three values: **0.4 nominal, 0.2 stress, 0.8 jam check**.

**The arm stays one arm, on the client-facing face, with the shoulder 0.25 m below centre.** The stow check confirms that position fits the envelope. Keep the camera and lidar bench on the opposite upper corner of the +X face from the folded arm, and check its view of the ring at the 1.8 m pre-grasp. Real space arms move slowly: **10 deg/s** is the common safety limit (CAESAR, e.Deorbit). Measured ROKVISS joint stiffness was **~4900 N m/rad**, with joint friction up ~50% in orbit ([Wedler et al. 2012](https://esmats.eu/amspapers/pastpapers/pdfs/2012/wedler.pdf)). Enforce the speed limit now; leave stiffness as Level 3. A second arm, a stinger or a clamp adds a load path the loads do not need: through-grip stack loads are ~1.3 N, and the 6-DOF ring tool already constrains the stack. MEV-style stanchions (a probe draws the client onto three stanchions against its separation ring, per the earlier unprepared-client research) or an e.Deorbit-style clamp remain the upgrade path if the stack move shows arm compliance problems.

## Sensors carry over; power and thermal stay design notes

**The single +X optical bench matches flown practice in kind, and adding sensor types adds more than duplicating the bench.** Flown and planned RPO suites pair lidar with visible and infrared cameras. MEV-1 tuned visible, IR and lidar sensors on its approach ([Northrop MEV fact sheet](https://cdn.northropgrumman.com/-/media/wp-content/uploads/Mission-Extension-Vehicle-MEV-fact-sheet.pdf)). OSAM-1 planned a Kodiak lidar with two wide-FOV and two narrow-FOV cameras ([NASA](https://www.nasa.gov/isam/relative-navigation-system/)). ADRAS-J handed over from a visible camera to an IR camera with model-matching navigation within a few km ([Astroscale](https://www.astroscale.com/en/news/astroscales-adras-j-completes-successful-rendezvous-and-initiates-proximity-approach)). The flown lidar class for unprepared clients is Jena-Optronik's RVS 3000, a scanning time-of-flight lidar with ICP pose estimation, flown on MEV-1, MEV-2 and MRV ([Jena-Optronik](https://jena-optronik.de/en/news/reader/joint-commercial-and-government-robotic-mission-for-space-maintenance-jena-optronik-sensors-onboard-northrop-grummans-mrv.html)). ASC's flash lidars use a 128 x 128 array ([GoldenEye](https://asc3d.com/wp-content/uploads/2019/07/GoldenEyeProductMay2013PreFinal.pdf)) with selectable 15/30/45 deg FOV, and the newer GSFL-16KS is quoted at 6.7 kg and 5-15 Hz ([ASC](https://asc3d.com/gsfl_16KS)). The project's 128 x 128, 45 deg, 60 m lidar is therefore class-representative. Its point density sets the hand-over ranges. At ~6.1 mrad per pixel, a 1 m face spans **~20 px at 8 m but only ~3 px at 60 m** (project calculation), which is why lidar-led pose belongs inside ~20-30 m and a farther hold would be a camera phase.

**The recommended additions are a narrow-FOV camera on the bench and an LWIR camera option for eclipse.** An ADR conference paper argues IR gives stable images in sun or eclipse ([ESA SDC9](https://conference.sdo.esoc.esa.int/proceedings/sdc9/paper/180/SDC9-paper180.pdf), attribution from a snippet). The palm light stays for the last centimetres.

Contact and state sensors should be modelled at datasheet classes, mostly terrestrial or catalogue values with a space derating. The wrist F/T sensor gets ATI-class resolution of **~0.3 N and 0.005 N m** (snippet of an ATI product page; the [ATI Mini40](https://www.ati-ia.com/products/ft/ft_models.aspx?id=mini40) page gives finer values); no space F/T noise or drift data was found. Strain-wave joints get a **0.4-0.7 N m static torque error and +-0.5 N m ripple** ([Frontiers 2024](https://frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1416360/full); [DLR elib](https://elib.dlr.de/105945/)). Star trackers weigh ~0.2-0.35 kg and draw ~0.8 W, and the Blue Canyon NST needs a **45 deg sun keep-out half-cone** (17.5 deg with an extended baffle) ([Sodern Auriga](https://api.sodern.com/uploads/AURIGA_8_PAGE_a11f9a3724.pdf); [BCT NST](https://bluecanyontech.com/wp-content/uploads/NST-1_2025.pdf)), which constrains the attitude profile during capture and not just the layout. A STIM300-class tactical IMU at 0.3 deg/h bias instability and 0.15 deg/sqrt(h) ARW ([Sensonor](https://sensonor.com/products/inertial-measurement-units/stim300)) is adequate for a ~10 min capture between star-tracker updates.

**Power does not limit collision safety, so it should be logged, not modelled.** The +-Y body cells (0.79 m^2) at 100-150 W in sun work out to ~125-190 W/m^2. That is plausible against ~307 W/m^2 module output for 25% cells ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20150016418/downloads/20150016418.pdf)), but it is a sun-facing figure, not an orbit average. Datasheet-class loads are 10-22 W average per wheel, ~10 W average per HPGP thruster ([SatCatalog](https://satcatalog.com/component/1n-hpgp)) and **4-25 W** for Xiphos Q8S/Q8JS-class rad-tolerant computers ([satsearch](https://satsearch.co/products/xiphos-q8s-processor)). No arm, lidar or illumination power was found, and no radiation-tolerant Jetson-class GPU was found either. Treat on-board vision networks on a COTS GPU as an assumption to flag. With four wheels, an arm, a lidar and compute, capture-phase load plausibly exceeds the body cells' output, so a battery carries the peaks. A per-phase energy ledger tells the project whether deployable arrays would be needed, and if they would, they belong off the +X face, off the -Z radiator and out of the approach corridor. **Thermal research found nothing usable.** The -Z radiator stays a design note.

## The approach should spend synchronised time only in the last three metres

**The P7 chain already has the shape of flown approaches. What it lacks is passive safety and a cap on synchronised time.** MEV-1 held autonomously at ~80 m and ~20 m with a ground go before each move, then held at 1 m before a joint go to dock ([Spaceflight Now](https://spaceflightnow.com/2020/02/26/two-commercial-satellites-link-up-in-space-for-first-time/)). ADRAS-J closed from 50 m to ~15 m, and its collision-avoidance logic **aborted autonomously on a relative attitude anomaly** before the capture initiation point ([Astroscale](https://astroscale.com/en/news/astroscales-adras-j-achieves-historic-15-meter-approach-to-space-debris)). Dragon checks out its abort at ~250 m and holds at 30 m and 10 m ([Spaceflight Now](https://spaceflightnow.com/falcon9/003/rendezvous.html)). P7's 30 m / 8 m / 1.8 m structure fits this pattern, but its 8 m hold is closer than the 15-20 m near holds of these missions. The hold is justified by lidar point density (~20 px across the face), not by precedent.

**Passive safety: every hold and leg should pass a free-drift check that uses no thrusters.** Passive safety means avoiding collision after complete loss of control ([arXiv 2405.00867](https://arxiv.org/pdf/2405.00867)). Separation in the radial and cross-track plane is the main geometric driver ([ISSFD24 Schmidt](https://www.issfd.org/ISSFD_2014/ISSFD24_Paper_S16-5_Schmidt.pdf)). For P7 this means three things. First, propagate each hold and each leg in Clohessy-Wiltshire free drift (n ~1.1e-3 rad/s at 400 km) against a keep-out sphere of **at least 2 m around the client CoM** (half-diagonal of the box ~1.03 m), enlarged by the wing and dish spans. Second, reject a hold that fails the check. Third, test a failed-thruster abort in simulation, passing only when free drift alone stays clear. The drift horizon is a project choice: a "24 h" figure appeared only in a search summary. The CW terms are small inside the capture zone (3n^2 x ~0.03 mm/s^2 at 8 m against ~3 mm/s^2 of demand; project calculation), so adding them as an applied force in the scene is cheap. They matter for the drift horizon.

**Synchronising only from a new 3 m stop roughly halves the fly-in cost.** Because the fly-in cost is acceleration multiplied by time, the approach should translate unsynchronised from 8 m to a **3 m station-keeping stop on the estimated spin axis**, where the arm is still stowed. From there it synchronises only for the final fly-in, reach and grasp. If that takes ~200-250 s at ~3 mm/s^2, it costs **~0.6-0.75 m/s instead of 1.2-2.1** (project estimate, to confirm in runs). Choosing the pre-grasp point near the spin axis lowers r and the cost further. A client spinning about its X axis puts the ring centre on the axis, while spin about Y or Z sweeps the ring at w x 0.75 m. For a torque-free spin about a principal axis, waiting for a favourable phase does not help, though it can for a true tumble (inference). Contactless detumble by plume impingement is studied ([Borelli et al., PoliMi](https://re.public.polimi.it/bitstream/11311/1146010/1/BOREG01-20.pdf)) but stays out of scope. P7's 0-2 deg/s is inside the range real programmes designed for: ClearSpace-1 GNC for up to 3 deg/s, and the e.Deorbit arm studies up to 5 deg/s ([ESA SDC](https://conference.sdo.esoc.esa.int/node/17595), snippets).

**A farther 50-80 m inspection hold belongs upstream, not in the simulation.** MEV's 80 m and ADRAS-J's 50 m holds are inspection points for camera-led phases. At 60 m the project lidar has ~3 px across the face. Recommend recording a 50-80 m inspection hold as the upstream phase that delivers the 30 m hand-over. Start the simulation at 30 m with hand-over errors of ~1% of range and ~1 deg, from a NASA CEV relative-navigation expectation ([NTRS 20070025134](https://ntrs.nasa.gov/api/citations/20070025134/downloads/20070025134.pdf)): about 0.3 m and 1 deg at 30 m. One degree of attitude error at 8 m is ~0.14 m at the client, large against a 0.61 m ring, so it must carry into the spin-axis estimate.

**Corridor and plume keep-out should be written as forbidden firing directions per phase.** No closed-form keep-out cone for a tumbling-client approach was found. ISS practice combines corridors and keep-out spheres with automatic abort on exit ([newspaceeconomy](https://newspaceeconomy.ca/2024/09/05/how-do-spacecraft-dock-with-the-international-space-station/), snippet). Plume mass flux is highest at docking range ([Glasgow](https://eprints.gla.ac.uk/268663/3/268633.pdf), snippet). Recommend a **10-15 deg half-angle corridor** about the pre-grasp line (project choice). Inside ~3 m, any nozzle whose plume centreline falls within 30 deg of the client is disabled in the allocation. The 30 deg cant keeps -X braking available under that rule.

**Observation cap, retries and gates.** Observation currently takes a median of 208 s and a maximum of 1458 s. Cap it at about **1.5x the 95th percentile**, with a deterministic back-out to the 30 m hold on cap. Allow 2-3 attempts with a propellant floor below which no approach starts, and count every safe abort as a good result. Gate each hold (30 / 8 / 3 / 1.8 m) automatically on range, rate, estimate covariance and propellant, and fire aborts on the covariance as well as the mean, as ADRAS-J's attitude-anomaly abort did. Record the ground go at 8 m and before the grasp commit as a real-mission addition that the simulation replaces with automatic gates.

## Simulation fidelity changes close all eight audit findings

The redesign is mostly data (a new `servicer_rev_2` scene through the `robot_spec` layer), plus five model changes. Rank the work by what can cause or hide a collision.

| # | Audit finding | Rev-2 fix | Check in the physics stage |
|---|---|---|---|
| 1 | No -X thrust | 16 nozzles in 4 mid-depth edge pods, +X and -X in every pod, 30 deg outward cant: 3.46 N pure Fx per N of thrust, both signs | Braking and back-off authority per axis; low-pressure (0.5 N) case |
| 2 | No pure pitch/yaw couple | Rank-6 allocation, pure couples 1.04-1.71 N m per N; unloading uses allocated couples at real levers, not 0.5 N m at 1 m | Rank after any single thruster failure; couples in the captured state with plume-restricted nozzles |
| 3 | Wheels oversized; no grasp-phase momentum limit | 4 x 12 N m s / 0.2 N m pyramid in **every** phase, with momentum and torque clamps, gyroscopic term, 80% unload threshold | Detumble at 2 deg/s, 400 kg client; count of unload events |
| 4 | mu 1.5 optimistic | Form-closure ring tool; mu 0.4 nominal, 0.2 stress, 0.8 jam check | No release or lip slip at mu 0.2; full draw-in at 0.8 |
| 5 | Stowed arm breaks ESPA | Flat fold: 645 / 710 mm depth, 600 / 610 width, 900 / 965 height; pods recessed into chamfered edges | Joint ranges reach the 180 deg elbow fold; launch locks in 65 mm |
| 6 | Bus has no collision | Bus, pods, bench and panels as collision geoms; any bus-client contact counts as a collision | Contact pairs active in all phases |
| 7 | Idealised thrust | Per-nozzle point force at its body-frame site; on/off PWPF with a minimum on-time; propellant mass depletes; allocation about the **estimated** CoM / stack CoM | Thrust saturation and delta-v against P6 numbers |
| 8 | Joint speed, soft jaw, no power | Enforce 10 deg/s per joint; jaws on a non-backdrivable lead screw (position held unpowered, finite stroke speed); per-phase energy ledger logged | Arm timing at 10 deg/s; ledger peaks |

Some error models for the thrusters have to be assumed, because no vendor tolerance was found. Use **+-5% thrust scale and 1-2 deg misalignment** as a starting Monte Carlo range. One thesis measured up to **13.2% force variation** across eight binary thrusters ([University of Kentucky](https://uknowledge.uky.edu/me_etds/250), excerpt), so a wider stress case is justified. For the minimum impulse bit, a 20 ms minimum on-time at 1 N (20 mN s) sits between Dawn's published B1 values of 1.4 mN s in cold gas and 74 mN s in bipropellant mode ([Dawn](https://www.dawnaerospace.com/thrusters)). VACCO's cold-gas valves respond in under 20 ms ([VACCO MiPS](https://www.cubesat-propulsion.com/wp-content/uploads/2015/11/JPL-MarCO-Micro-CubeSat-Propulsion-System-datasheet.pdf)). PWPF is the standard way to get near-linear behaviour from on-off thrusters ([Krøvel and Gravdahl](https://tomgra.folk.ntnu.no/papers/Krovel_STEC_2005.pdf)). Three further changes are worth making now: mass properties from parts, the arm's CoM shift between stowed and deployed (4 to 8 cm today) fed to the attitude controller, and the ROKVISS ~50% in-orbit joint friction rise as a stress case. Joint stiffness and slosh stay Level 3.

## Seven decisions, options and recommendations

**Decision 1: launch envelope and mass class.**

| Option | For | Against |
|---|---|---|
| **A. ESPA standard 15 in port, ~200 kg wet** | Matches ELSA-d scale; arm fits (645 mm); keeps the "small servicer" story; Moog heritage 220 kg | 180 kg older limit unresolved; only 5 mm width margin |
| B. Same port on ESPA Heavy (322 kg heritage) | Same envelope, removes mass risk | Ring-level choice only; volume not confirmed |
| C. ESPA Grande 24 in port | 1100 x 1200 x 970 mm volume, bigger wheels and tank, longer arm | 320 vs 700 kg conflict; larger servicer raises reduced mass and wheel demand; departs from the D6 scale decision |
| D. SpaceX full plate | 200 kg included (UNVERIFIED) | Plate dimensions not retrieved |

**Recommendation: A**, with B as the fallback if the standard-port limit proves to be 180 kg. Nothing in P7's demands needs C.

**Decision 2: propulsion type and thruster layout.**

| Option | For | Against |
|---|---|---|
| **A. Green monoprop 1 N x 16, four mid-depth edge pods, X nozzles canted 30 deg** | Flown class (ELSA-d HPGP); ~4 kg for 40 m/s; rank-6 with near-equal 3.4 N per axis; plume 30 deg off the client | Catalyst-bed and blowdown behaviour; numbers third-party |
| B. Hydrazine 1 N, same layout | 228 s, mature | Same simulation as A; toxic handling (irrelevant in the simulation) |
| C. Cold gas GN2 ~1 N, same layout | Clean plume, fast valves, small MIB | 70 s Isp, 25-40 L tank at 300 bar; mass |
| D. 12 thrusters (ADRAS-J-style 8 + 4) | Lighter | Less single-failure margin (ELSA-d lost half) |
| E. Cant 15 deg instead of 30 | 3.86 N axial | Lateral force 2.73 N; plume closer to the client |

**Recommendation: A**, with C kept as a one-parameter variant (Isp, MIB, tank mass) if contamination of the client or optics becomes a theme.

**Decision 3: wheels and unloading.**

| Option | For | Against |
|---|---|---|
| **A. 4 x 12 N m s / 0.2 N m pyramid + thruster-couple unloading at 80%** | Datasheet-class (RW-12), ~20 kg, single-failure tolerant; covers typical detumble; unloading now physically real (~25 g per detumble) | Worst cases (26-29 N m s) need thruster top-up near the captured client |
| B. 4 x 25 N m s (HR14 class) wheels-only detumble | ~41 N m s about X and Y; no plume during detumble | +10 kg, 22 W each; HR14 sourcing via Basilisk |
| C. 3 orthogonal 12 N m s | Lightest | No redundancy; 12 N m s per axis |
| D. Small CMGs | 45 N m | 18.4 kg each, built for ~1000 kg class |
| Unloading: magnetorquers | No propellant | ~23 h for 84 N m s; stack dipole unknown: design note only |

**Recommendation: A**, switching to B if the captured-state allocation cannot form couples without firing at the client, or if more than a few runs in 25 need unloading during detumble.

**Decision 4: arm placement and any second contact.**

| Option | For | Against |
|---|---|---|
| **A. One arm, shoulder on the +X face 0.25 m below centre, flat-fold stow, 6-DOF ring tool** | CAD-checked stow; ring tool constrains 6 DOF; through-grip loads ~1.3 N | Bench view with the arm stowed unchecked; elbow fold range unchecked |
| B. Shoulder on a side face or corner | Clears the sensor cone | Asymmetric reach and CoM; fold geometry not checked |
| C. A + MEV-style stanchions (draw ring onto bus) | Stiff stack for moves | Would need building at this scale; bus must reach the ring's load path |
| D. Second arm (GITAI S2 2 x 1.5 m; ClearSpace multi-arm) | Redundant grasp | Mass, envelope, a second failure point |
| E. SENER-style clamp after detumble | e.Deorbit precedent | Would need building; adds a second close-range contact phase |

**Recommendation: A**, with C as the upgrade path if the stack move shows arm-compliance problems.

**Decision 5: grip pad material, friction value and stress case.**

| Option | Simulated mu | For | Against |
|---|---|---|---|
| **A. Hard jaw faces (hard-anodised Al or Ti) on a form-closure tool** | **0.4 nominal** | Geometry holds; friction only matters in the soft stage | No vacuum mu measured |
| B. Elastomer pads | ~1 (ceiling) | More soft-stage grip | Outgassing, creep, unsourced value; hides a weak geometric hold |
| C. Vespel SP-3 on the palm datum | low | Lets the ring slide onto the palm during draw-in | Low friction in the soft stage |
| D. Gecko dry adhesive | 75-80 kPa shear (glass) | Novel | Would need building; vacuum performance unquantified |

**Recommendation: A**, with three cases. The stress case is **mu 0.2** with a 400 kg client, 2 deg/s about the worst axis, maximum CoM offset, and detumble at the wheel-plus-thruster torque limit; it passes with no release and no lip slip. The jam check runs **mu 0.8** and passes when the draw-in reaches the palm. Optionally, give the palm datum a low-mu face (C).

**Decision 6: what is modelled now and what is a design note.**

| Item | Model now | Design note |
|---|---|---|
| Mass properties from parts, bus collision, per-nozzle thrust + MIB + errors + propellant depletion, wheel limits in all phases, joint speed limit, non-backdrivable jaws, realistic mu, D5 contact-sensor noise, CW terms and free-drift monitor | Yes | |
| Power | Per-phase energy ledger (wheels 10-22 W, compute 4-25 W, thrusters ~10 W, arm and lidar estimated), logged, not gating | Deployable arrays, battery sizing |
| Thermal | | Radiator on -Z, eclipse cycles, plume heating |
| Magnetorquers, joint stiffness, slosh | | Yes (Level 3) |

**Recommendation:** model everything that changes authority, contact or the safety monitors; log power; note thermal. This follows the realism bar: power and thermal cannot cause a collision in P7, but a missing -X thrust or a 1 N m wheel can hide one.

**Decision 7: approach layout.**

| Option | For | Against |
|---|---|---|
| **A. Holds at 30 / 8 / 3 / 1.8 m; synchronise only from the 3 m stop; pre-grasp near the spin axis; CW free-drift check per hold and leg against a >= 2 m keep-out; 10-15 deg corridor; no firing within 30 deg of the client inside 3 m; observation cap 1.5x p95 with back-out to 30 m, 2-3 attempts, propellant floor; automatic gates on covariance** | Matches the MEV / ADRAS-J pattern; ~halves fly-in delta-v (estimate); aborts no longer depend on working thrusters | New stop and monitors to build and tune |
| B. A plus a simulated 50-80 m inspection hold | Closer to MEV / ADRAS-J | Camera-led phase (~3 px of lidar); long runs on a thermally limited laptop |
| C. Full safety-ellipse / walking-ellipse inspection | Strongest passive safety | Much more delta-v and time; beyond P7's ring-capture focus |
| D. Keep today's synchronised fly-in from 8 m | No change | 1.2-2.1 m/s per attempt; long synchronised exposure |

**Recommendation: A**, with the 50-80 m hold recorded as the upstream phase that delivers the 30 m hand-over (~0.3 m, ~1 deg).

## Verify before building

These are the numbers that most need a primary datasheet, ranked by how much of the design rests on them.

| Number | Value used | Why it matters | Where to check |
|---|---|---|---|
| ESPA standard port mass limit and payload depth | 220 kg heritage vs 180 kg; 710 mm depth | Sets decision 1 and the 65 mm stow margin | Moog ESPA User's Guide (PDF not readable in this pass) |
| ESPA Grande capacity | 320 vs 700 kg | Only matters if decision 1 goes to C | Moog / Momentus primaries |
| 1 N HPGP thrust range, Isp, MIB, valve response | 0.25-1 N, 204-231 s, MIB not found | Thruster model, low-pressure case | ECAPS / Bradford datasheet (brochure too large to fetch) |
| RW-12 mass and power; HR14 numbers | ~5 kg (third-party); 0.2 N m, 7.5 kg, 22 W (Basilisk) | Wheel model and mass budget | Rocket Lab and Honeywell datasheets |
| 610 mm ring lip height, thickness, allowable flange load | not found (PAS 610S 73 mm, 5.8 kg UNVERIFIED) | Jaw geometry, pad depth, clamp force | Beyond Gravity PAS 610S drawing; Eaton/Marman V-band catalogue |
| Vacuum friction of jaw materials on anodised Al | 0.2 / 0.4 / 0.8 judgement band | Soft-stage and draw-in behaviour | DTIC ADA040471 (PTFE-bronze on hard anodise in vacuum); ESR tribology data |
| Small-arm mass, torque, stiffness (1-1.5 m class) | project joint-module model | Arm mass, CoM shift, speed | Motiv / Rocket Lab Robotics, GITAI, OEDMS and ETS-VII papers |
| Thruster misalignment and scale tolerance | +-5%, 1-2 deg (assumption; 13.2% precedent) | Monte Carlo ranges | Vendor acceptance data; Fehse Appendix C |
| Flash lidar array, mass, power | 128 x 128, 6.7 kg, power unknown | Sensor mass and point density | ASC current GSFL datasheet; RVS 3000 data |
| Star tracker and IMU values | NST 0.75 vs 1.5 W; STIM300 0.3 vs 0.5 deg/h | Minor; conflicting catalogues | Vendor datasheets |
| ADRAS-J mass and thruster layout | ~150 kg wet; 8 + 4 thrusters (UNVERIFIED) | Reference-class check | Astroscale ADRAS-J press kit |

## Conclusion

Every hard demand on the rev-1 servicer turns out to be a geometry problem. The detumble momentum depends on how far the grasp sits from the stack CoM, the fly-in cost on how long and how far off the spin axis the servicer stays synchronised, the stack-move momentum on whether thrust passes through the CoM, and the grip margin on whether the jaws close behind the lip. None of these is fixed by more torque, more friction or more propellant, which is why today's idealisations (1 N m wheels, mu 1.5, a force box at the bus centre) hid the problems instead of solving them. Fixing the geometry lets rev-2 use smaller, datasheet-class parts and still have more real authority than rev-1 had in the model.

What the project will learn from rev-2 is mostly about its own margins. Once the wheels saturate realistically, thrusters fire through real levers, and the bus can collide, the P6 safety monitors face failure modes they have never seen: a stuck nozzle at the 3 m stop, wheel saturation mid-detumble, a jaw that slips at mu 0.2. The useful result is a map of which of these end in a safe abort, not a higher capture count. That map is the evidence that an ESPA-class servicer can attempt a 300 kg tumbling, unprepared client at all.
