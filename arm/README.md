# Tendon Arms: two 7-DOF arms + grippers on a printed torso

Two human-like arms with two-finger grippers on a torso. All seven joints and the gripper of each arm are driven by **1 mm Dyneema** cords in **PTFE Bowden tubes** from **16 motor drive units** in the torso: a column between the shoulders, built from 2020/2040 aluminium extrusion and printed parts. The left arm is the mirror image of the right one.

![arm](../docs/img/arm_assembly.png)

Interactive 3D model: open `docs/viewer/` (tab *Arms · torso*), see the repo [README](../README.md). The torso, both arms, the drive column and every fixed tube run are in the viewer; the left arm mirrors the right arm's joint values, *Reach mode* drives the right arm.

| | Joint | Type | Range | Moment arm | Spool turns, full range |
|---|---|---|---|---|---|
| S1 | Shoulder flexion (forward) | hinge L | −30…+120° | 42 mm | 2.2 |
| S2 | Shoulder abduction (sideways) | hinge L | −10…+100° | 42 mm | 1.6 |
| S3 | Upper-arm rotation | roll (2× 6806) | ±80° | 28 mm | 1.6 |
| E | Elbow | hinge M | 0…+115° | 36 mm | 1.4 |
| F | Forearm twist | roll (2× 6805) | ±85° | 25.5 mm | 1.5 |
| W1 | Wrist flexion | hinge S | ±60° | 20 mm | 0.8 |
| W2 | Wrist tilt | hinge S | ±30° | 20 mm | 0.4 |
| G | Gripper (one cord + spring) | 2 geared fingers | 0…62 mm | 20 mm | 0.3 |

Upper arm is 300 mm (S2 axis to elbow), forearm 250 mm (elbow to wrist). The shoulders (S1 axes) sit 650 mm above the work surface and 350 mm apart.

## Controlling the hand (kinematics)

Give a hand position (and optionally an orientation) and get the 7 joint angles plus the turns of each drive spool:

```bash
.venv/bin/python -m arm.ik -40 164 -695
```

The equations, solver and accuracy are in [docs/kinematics.md](../docs/kinematics.md). The viewer's **Reach mode** runs the same solver live in the browser.

## How it's built

```
drive unit ×8 per arm ──PTFE tubes──► shoulder comb ──► S1 ─ S2 ─ link ─ S3 ─ upper link ─ E ─ link ─ F ─ forearm link ─ W1 ─ W2 ─ gripper
(torso column)                                         hinge hinge      roll               hinge     roll                 hinge hinge
```

* **Hinge module** (`hinge.py`): base + two rotor halves.
  * **Base:**
    * a plate with heat-set inserts;
    * two lugs with captive nuts for the stub axles: M8×20 into 608 bearings on the shoulder (S1, S2), M5×16 into 625 bearings on the elbow and wrist (E, W1, W2);
    * a boss with two angled Bowden sockets aimed exactly along the drum tangents;
    * a slot that lets the tubes of the joints further out pass through.
  * **Rotor:** printed as two halves split on the motion plane, each flat on that face.
    * The drum is hollow, with the bearings at its ends.
    * Each half has one cord groove with a knotted chord anchor.
    * A top plate with inserts holds the next link.
* **Roll module** (`roll.py`): stator + rotor + retainer + 2 idlers. Check the race shoulder sizes of the 6806/6805 you buy against `roll.py`.
  * The stator tube carries two thin-section bearings.
  * The rotor ring holds their outer races and has two grooves.
  * Each cord comes up from a socket in the stator plate and turns over a 623 idler pulley onto the rotor. The sockets lean 45° inwards: the cord leaves the idler at its lower-outer side and runs under it, so the socket opens under the plate ~10 mm closer to the axis. That keeps the link below narrow (S3 sits on a plain L tube, F on a 76 mm "ML" tube) and the stator plates at Ø 104 / 100 mm.
  * The bore stays open for the tubes going further down.
* **Links** (`link.py`):
  * tubes with outward bolting lips, bolt heads on the lips;
  * a full 48° cone carries the top lip, with a counterbore up through it for each bolt head;
  * on the S→ML forearm link the bottom bolts are reached with a hex key from inside the wide bore.
* **Drive unit** (`drive.py`, one per joint, 16 in all), grown out of the old hand crank (`crank.py`, kept as the library for the spool and blocks):
  * the crank's **helical spool** (r 8 mm, cord A on the −X helix, cord B on the +X helix) on a 608 (wall A, M8 shaft + nut cup) and a 6805 around a Ø25 journal (wall B), so the cord load stays in the frame;
  * the crank's two **socket blocks** (A and B), one per cord, each pulled up by its own M4 screw (10 mm travel), and its **tube clip**;
  * a **spool hub**, the same for every motor: a flange with two M3 drive pins (r 12) outside wall B;
  * per motor type a **horn adapter** (takes the pins in two radial slots: torque only, no side load on the motor) and a **motor cup** (end plate + 4 hollow pillars, 4× M3 into inserts in wall B at y ±22, z ±25);
  * an **encoder-cable guide** on the frame top, so each joint's magnetic-encoder cable runs with its two tubes.
* **Gripper** (`gripper.py`): palm + two geared fingers + two slide-on pads, one pull cord and one opening spring; see below.
* **Shoulder balance** (`balance.py`): two springs on cords from a yoke on `link_s1_s2` over sheaves on the shoulder beam; see below.
* **Torso** (`torso.py`, see *Two-arm torso* below): 2040/2020 extrusion frame, a drive column of 7 printed saddles, two S1 hangers under the clavicle, two shoulder beams with tube combs, an electronics tray and an E-stop bracket.
* **Fittings** (`fittings.py`): a cord thimble to join Dyneema to a spring hook or an eye.

## Two-arm torso (`torso.py`, `drive.py`)

```
            S1 hanger ┌──────── clavicle 2040 (z 1295..1335) ────────┐ S1 hanger
       S1 unit (hangs, ▼ tubes)    ┌──┴── drive column ──┴──┐    S1 unit (hangs)
                 │               R units ◄ saddle ×7 ► L units               │
                 │   tubes out+down 12°  \│  spine  │/  tubes out+down 12°   │
   right shoulder ▼  ─── comb ◄──────────  │  2×2040 │  ──────────► comb ───  ▼ left shoulder
   beam, S1 at x = -175, z = 650  ═══ deck rails / cross rails (z 679..719) ═══  beam, S1 at x = +175
                 │ arm                     │         │                     arm │
   ─────────────────────── work surface (z = 0) ─────┴ base rails ┴────────────────────
                       (nothing of the torso in front of y = 100 below the shoulders)
```

**Frames.** TORSO frame: origin on the work surface on the mid-plane, +X = the robot's left, −Y = forwards, +Z = up. The right arm is the original arm: its chain, `kinematics.py`, `balance.py` and `check_ik.py` still work in its BEAM frame (the old single-arm world: underside of the shoulder-beam floor, S1 axis at (0, −95, −69)), which sits at `assembly.MOUNT` = (−175, 95, 719) in the TORSO frame. The left arm is the mirror image in x = 0: `assembly.arm_world("L", q)` mirrors the right arm posed with the same q, so equal joint values give a mirror-symmetric pose and the SHOULDER frame (X outwards, Y forwards, Z up) means the same for both arms. Helpers in `assembly.py`: `arm_base`, `to_torso`, `point_torso`, `arm_world`, `balance_world`, `shoulder_frame_torso`, `drive_frames`, `drive_solids`, `torso_solids`, `tube_routes`.

**Layout** (TORSO frame, mm):

| | x | y | z | |
|---|---|---|---|---|
| S1 axes | ±175 | 0 | 650 | shoulder spacing 350 |
| Shoulder beams (printed) | ±175 ±95 (wings) | −30 … 110 | 719 … 771 | S1 bolts under the floor, masts hook over the walls |
| Spine posts, 2× 2040 | ±(20 … 60) | 110 … 130 | 20 … 1335 | on 2× 2040 base rails (y 110 … 460) screwed to the table |
| Deck: cross rails 2× 2020 (front, rear) | ±270 | 90 … 110, 400 … 420 | 679 … 699 | front one bolted to the spine, stringers 2× 2020 at x ±40 |
| Deck rails 4× 2020 | ±90, ±260 (= arm ± 85) | 50 … 455 | 699 … 719 | carry the beam wings and the spring anchor posts |
| Drive column: 7 saddles | ±64 | −32 … 110 | 809 … 1255 | level pitch 65, unit axes z = 830 + 65 k |
| Column drive units (12) | ±(14 … 125) | −36 … 96 | 790 … 1255 | lean 12° outwards-down, motors to the rear |
| Clavicle 2040 | ±262 | 90 … 110 | 1295 … 1335 | across the spine top |
| S1 drive units (2), hanging | ±(176 … 234) | −36 … 96 | 1164 … 1270 | tubes straight down, from printed hangers |
| Electronics tray | ±75 | 130 … 142 | 300 … 520 | back of the spine |
| E-stop bracket | ±28 | 380 … 440 | 699 … 759 | on the rear cross rail, between the deck rails |

Nothing of the torso is in front of y = 100 below z = 679, so the space in front of the shoulders is free down to the table. Torso top: 1335 mm above the work surface.

**Where each drive unit sits** (right arm; the left one is the same unit turned, not mirrored, at −x):

| Joint | Motor | Position | Spool axis (x, y, z) | Tube exits |
|---|---|---|---|---|
| S1 | STS3095 | hanging under the right clavicle end | (−205, 0, 1236) | z 1164, straight down |
| S2 | STS3095 | column level 0 | (−53, 0, 830) | x −122, z 807 / 823 |
| S3 | STS3250 | level 1 | (−53, 0, 895) | z 872 / 888 |
| E | STS3250 | level 2 | (−53, 0, 960) | z 937 / 953 |
| F | STS3215 | level 3 | (−53, 0, 1025) | z 1002 / 1018 |
| W1 | STS3215 | level 4 | (−53, 0, 1090) | z 1067 / 1083 |
| W2 | STS3215 | level 5 | (−53, 0, 1155) | z 1132 / 1148 |
| G | STS3215 | level 6 (top) | (−53, 0, 1220) | z 1197, one tube |

Heavy units low, the light wrist and gripper units at the top. The column starts at z 830 because the lowest unit's outer corner must clear the inner balance mast (top at z 781).

### Drive units and motors (`drive.py`)

Every unit is the same frame, spool, socket blocks, tube clip and nut cup; only the **horn adapter** and the **motor cup** depend on the motor. The cup bolts to the same 4 inserts in wall B for every motor, and the adapter couples through the same two drive pins, so a joint is upgraded by swapping two printed parts. `check_arm.py drives` puts a DM-J4310 on all 16 positions and checks that it fits.

| Motor | Joints (phase 1) | Body | Output / horn | Mounting | Source |
|---|---|---|---|---|---|
| Feetech STS3215 (C001 7.4 V / C018 12 V) | F, W1, W2, gripper | 45.2 × 24.7 × 35 mm, 55 g | 25T spline (OD 5.9); Ø20 × 2.5 horn, 4× M3 on a 9.8 mm square; output 12.5 mm off the case centre | 4× M2 tapped in the bottom cover at (4.2 / −20.3, ±10.2) from the case centre; 2 connectors in the bottom cover | Feetech STEP model in [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) `STEP/SO100/STS3215_03a.step` (measured); [servodatabase](https://servodatabase.com/servo/feetech/sts3215), [Core Electronics](https://core-electronics.com.au/feetech-sts3215-smart-servo.html) |
| Feetech STS3250 | S3, E | 45.22 × 24.72 × 35 mm | as STS3215 (same housing, assumed) | as STS3215 (assumed) | [OpenELAB](https://openelab.io/products/feetech-sts3250-c001-ttl-servo) |
| Feetech STS3095 (C001 / C002) | S1, S2 | 65 × 30 × 48 mm, 194.5 g | 15T spline Ø7.6 | **not published** in any source I could read | [OpenELAB](https://openelab.io/products/feetech-sts3095-c001-ttl-servo), rcdrone.top listing |
| Damiao DM-J4310-2EC V1.1 (upgrade) | S1, S2 (S3, E if needed) | Ø56–57 × 46 mm, ~300 g | front 6× M3 on Ø50 | rear 4× M3 on Ø38 | [DM-J4310-2EC V1.1 manual (aifitlab wiki)](https://aifitlab-wiki.super.site/damiao-docs/dm-j4310-2ec-v11-motor-instruction-manual); hole patterns from a reseller listing (rcdrone.top), not the official drawing |

What is verified and what is not:

* **STS3215 / STS3250:** body, horn and the bottom M2 pattern come from Feetech's own STEP model; the STS3250 is assumed to share the housing (same published size). The cup holds the servo by its 4 bottom M2 screws, with windows for the idler horn and the two connectors.
* **STS3095:** only the envelope and spline are published. Its cup locates the case in a 2 mm pocket and straps it with two cable ties; its adapter has four radial rows of M3 holes (patterns on Ø12 … 26). The output offset (17.5 mm from the case centre) and the 4 mm horn height are **estimates: measure your servo** and adjust `MOTORS["STS3095"]` before printing.
* **DM-J4310:** the hole patterns are from a reseller; **check them against the drawing of your motor revision.**

Axial stack (unit X): spool between the walls (±16.5), 6805 in wall B (16.5 … 23.5), hub flange 26 … 33 with the pins 4 mm proud, adapter 33.5 … 39.5, motor output face at x = 39.5, cup end plate at 74 (STS32xx), 86 (DM-J4310) or 92 (STS3095).

**Joint encoders:** each unit has a Ø6 cable hole and a tie slot on its frame top, and the cables are tied to the shoulder comb's arms, so each joint's encoder cable (AS5048A / MT6701 chain, `docs/ai_control.md` §3.3) runs with that joint's tubes.

### Tube routing and bend budget

The old bench had a fixed ~180° U-turn in every tube (up out of the crank, along the beam, down into S1). Now every unit's tubes leave towards their shoulder:

* **Column units** lean 12° outwards-down, so each tube leaves heading for the shoulder. It bends down once (~70°) and drops a few degrees more into the **shoulder comb**: a printed plate in the beam channel, 36 mm above the beam floor, that holds the 13 pass-through tubes in a 5 × 3 grid (pitch 5.8) right above the S1 opening. From the comb they drop straight through the floor into S1's tube slot. Lower units take the inner comb columns, so their runs stay inside the higher ones.
* **S1's own two sockets** lean 41° forwards and backwards (`hinge.py`, aimed at the drum tangents). A tube coming in sideways from the column would need 100–118°. So the two **S1 units hang above their shoulders** (under the clavicle, outboard of the column's tube curtain) with the tubes leaving straight down: 45–50°. The S1 tubes drop almost straight down past the comb's narrow arms (y ±12 at comb height) and bend the last ~41° into their sockets just above the S1 base; a notch in the beam's front wall leaves room for adjusting them.
* Each joint cord may go to either spool helix; `tube_routes()` picks the mapping with the smaller bend (the firmware sets each motor's sign). On the left arm most units are flipped, because the left units are the same parts turned 180°.

`check_arm.py bends` computes every run as straight – arc – straight – arc – straight with arcs of **R 25 mm** (PTFE 4×2 minimum) and the corner points free along the start and end lines, and checks the runs against the torso parts and the other units:

| Tube | Bend (right arm) | Left arm | Fixed run | Friction factor e^(μθ), μ 0.25 |
|---|---|---|---|---|
| S1 A / B | **47.5° / 44.6°** | 49.4° / 47.0° | 445 mm | 1.23 |
| S2 … W1 (A and B) | **78.0°** (61–74° + 4–41°) | 78.0° | 70 … 340 mm | 1.41 |
| W2 A / B | 78.1° | 78.1° | 390 … 405 mm | 1.41 |
| Gripper | 78.1° | 78.1° | 450 mm | 1.41 |
| *old bench, every tube* | *~180°* | | *~0.5–1.6 m* | *2.19* |

Every fixed bend is ≤ 78°, inside the 90° target, at R ≥ 25 mm. (μ = 0.25 is an assumed Dyneema-on-PTFE value for the comparison only.) The joint bends inside the arm (S1 + S2 + E + W1 for the wrist cords) are unchanged.

### Gravity balance on the torso

`balance.py` is unchanged in physics and geometry: the masts still hook over the shoulder beam's walls (a = 113 mm, b = 81 mm, X_S = ±85, k = 0.185 N/mm), and `tools/check_balance.py` still passes with the same numbers. What moved: the spring anchor posts now bolt (2× M5 + T-nuts) onto the torso's deck rails, which run exactly under the two cord planes (x = arm ± 85) back to y = 455; the posts sit at the same place relative to the shoulder as before. The left arm uses the mirrored set (`balance_mast_l` ↔ `_r`, posts and sheaves are symmetric).

### Controller, power, E-stop

* **Electronics tray** (back of the spine, z 300 … 520): 4× M3 inserts on 92 × 52 for a controller carrier (Teensy 4.1 class, `docs/ai_control.md` §3.4; one per arm fits side by side on a 100 × 60 carrier), 4× M3 on 40 × 30 for a Feetech bus adapter, a Ø12 hole for an M12 cable gland (12 V supply leads) and a Ø16 hole for a panel fuse holder, and tie slots along both edges.
* **Servo bus wiring** runs down the spine posts' slots; each column level's two units are 65 mm apart, so the daisy-chain cables stay short.
* **12 V, 30 A** supply (the four STS3095 can stall at 9.8 A each): an external PSU into the tray's gland.
* **E-stop:** a 22 mm mushroom (NC) in the printed bracket on the rear cross rail, reachable from behind and beside the robot. Wire it to drop a contactor / relay in the 12 V motor supply, not just a controller input.

### Checked (`tools/check_arm.py torso bends drives balance`)

* **Arm vs arm and arm vs torso**, both arms at every joint limit (mirror-symmetric, 14 poses), two combined poses and five poses with the hands in the shared middle zone (IK, low-lever posture): see *Checked in CAD* below.
* **Drive units:** every motor's unit fits together, the 16 units clear each other, the saddles, hangers, extrusions and shoulder beams, and a DM-J4310 fits on all 16 positions.
* **Tube runs:** the bend table above; the runs clear the torso parts and the other units.

### Compromises

* **Tall torso:** 7 column levels at 65 mm + the clavicle put the top at 1335 mm (685 mm above the shoulders). One unit per arm per level keeps every tube exit in the shoulder's plane (y ≈ 0); two units per level would halve the height but add 15–30° of sideways bend per tube.
* **S1 units outside the column** (under the clavicle, above the shoulders): needed because S1's sockets lean 41° fore/aft. Fixing that would mean S1 sockets aimed sideways (a `hinge.py` change).
* **Front-heavy:** the arms and drive column are in front of the spine posts; the base rails must be screwed (or clamped) to the table.
* **STS3095 and DM-J4310 mounting patterns are unverified** (see the motor table); their cups and adapters are the parts to re-check.
* **Over-constrained spool:** the spool runs in its own two bearings and the motor shaft in its own; the two-pin dog coupling (slots 3.8 wide, 7 long) absorbs small misalignment, nothing more.
* **Both hands at their inner edge at once** (x = −188 in each shoulder frame) cross the mid-plane: the shared zone is for one hand at a time there (`check_arm.py torso` reports it as KNOWN).
* **The hand cranks are retired**: `crank.py` remains the library for the spool and tension blocks, the crank-only parts (frame, arm, knob, lock pin, racks, combs) are no longer exported.

## Gripper (`gripper.py`)

Two angular fingers on the W2 rotor plate, closed by **one cord in one tube** and opened by a spring. One tube keeps the arm's tube count and friction low. A second (antagonistic) cord isn't needed: the fingers only have to be *pulled* closed, and the spring brings them back (see the pull-back numbers below).

```
            pad    pad             finger A: gear sector + cord drum (R 20) + tail
             |      |              finger B: gear sector + tail
          A   \    /   B           the gear sectors (m 1.25, r 20) mesh on the axis,
        (o)===gears===(o)          so both fingers always open by the same angle
        /  ~~~spring~~~  \         <- spring between the tails pulls them together = opens
     [=======palm========]         <- 4x M3x10 into W2's inserts; Bowden socket in the centre boss
              ||                   cord + tube come up the axis through W2's open centre
```

* **Palm:** a Ø 62 mm disc that bolts on the W2 rotor plate like a link lip (flange class S, 4× M3×10, heads on top). Two cheeks carry the finger pivots (M3×25 + nut), 40 mm apart. The centre boss is the Bowden socket: the gripper tube comes up through W2's open centre and stops in it; the cord continues straight up the axis.
* **Fingers:** three layers each (gear / drum / spring). The cord meets finger A's drum at its tangent point on the gripper axis, wraps it and is knotted in a pocket. Pulling the cord turns finger A closed; the gear sectors close finger B by the same angle. The pads touch at 0°. Open stops on the palm (under the finger tails) end the travel at 41°, before the gears run out of teeth.
* **Spring:** one small extension spring between the two finger tails, below the pivots, hooked over two printed posts. Pulling the tails together opens both fingers.
* **Pads:** slide-on sleeves over the straight finger ends, held by one M3×6 screw each (self-tapping into the finger). Print them in **TPU 95A** for grip, or in PETG.

| | |
|---|---|
| Opening between the pad centres | 0 … 62 mm (fingers 0 … 41°, angular jaws) |
| Grasp centre (= kinematic tool point) | 72 mm past the W2 plate (`kinematics.TOOL_OFFSET` = `gripper.GRASP_Z`) |
| Pad force per pad | F = (T·20 mm − 2·τ_spring) / (2·40 mm) |
| Cord for **10 N on each pad** | **42–44 N** over the whole opening (12 N per pad at 50 N of cord) |
| Holding the 0.3 kg object | 4.9 N per pad with PETG pads (μ 0.3), 1.8 N with TPU (μ 0.8): 23 / 11 N of cord |
| Cord stroke | 14.3 mm = 0.28 spool turns |
| Spring pull-back on the cord | 4.2 N closed … 1.9 N fully open (drags the cord back through the tube) |
| Gear tooth load at 10 N per pad | ~20 N (module 1.25, 4.8 mm face, ~10 MPa) |
| Mass | **51 g** (41 g printed: palm 18, fingers 7 + 6, pads 2 × 4; 10 g hardware) |

**Opening spring:** music-wire extension spring, wire 0.45–0.5 mm, OD 4 mm, ~34 mm inside the hooks, rate 0.15–0.3 N/mm (an assortment-box spring). It runs from 39 mm (open) to 55 mm (closed), so it stretches at most 21 mm. Any rate in that band keeps the cord for 10 N per pad under 47 N. A 1.5 mm O-ring or a small rubber band around the two posts also works.

**The gripper drive unit** (top of each arm's column, STS3215) is a standard drive unit with its cord on one helix only (socket block A; block B is left out). Its ratio is 20 / 8 = **2.5 : 1**, like the wrist: 44 N of cord is **0.35 N·m on the spool**. Winding the cord on closes the gripper; unwinding lets the spring open it. The gripper tube has its own hole (middle row) in the shoulder comb.

## Gravity balance springs (`balance.py`)

Two springs take most of the arm's weight off S1, so the S1 motor works much less.

**How it works (zero-free-length balancing).** Each side has a Dyneema cord. It starts at a pin **B** on the arm, b = 81 mm below the S1 axis, and runs over a sheave **A**, a = 113 mm straight above the axis. From there it goes back over the torso's deck rail to an extension spring. The spring pulls with F = k·|AB| (it behaves as a zero-length spring; see below). Then its moment about S1 is k·a·b·sin(S1), and the arm's weight moment is m·g·d·sin(S1). Both follow the same sine, so one setting balances the arm at every flexion angle. The two cords run in the side planes x = ±85 mm, outside the beam and the S1 base. They are symmetric, so the S1 bearings see no twist.

* **Yoke:** `link_s1_s2` has two flat arms out to x = ±85 mm. Its flanges and its 22 mm length are unchanged, so the kinematics stay the same. Each arm ends in a fork with an M3 pin, and the cord is tied in a loop around the pin. The loop pivots, so the cord line always passes through the pin centre. The arm tips slope down towards S2, which leaves room for the upper arm at full abduction (S2 = 100°).
* **Masts (`balance_mast_l/_r`):**
  * Each mast hooks over its beam side wall: a top cap, an inner lip and a front lip around the beam's front end.
  * Fix it with 2× M4 through the wall. Drill Ø4.5 through the beam wall, using the mast as the jig. The nuts go inside the channel.
  * It carries the sheave on two 623s and an M3 axle. The sheave centre sits 10 mm behind the S1 axis, so the hanging cord passes exactly over the axis.
  * A fairlead at the back sends the cord backwards at z = 54 mm.
* **Springs + anchor posts (`balance_post` ×2):**
  * The springs run 1:1 (no tackle) along x = ±85 mm (relative to the arm), right above the torso's two deck rails for that arm.
  * Each spring hooks onto an **M5 eye bolt** through a post that bolts onto the deck rail (2× M5 + T-nuts), 290–340 mm behind the beam's S1 position (y ≈ 385–435 in the TORSO frame).
  * The two nuts on the eye bolt tune the spring offset by ±20 mm.

**Balance level.** The mass model is shared with `statics.py`. It takes the CAD volume and surface of every printed part: the skin (walls + top/bottom layers) prints solid, the rest at the part's infill (links 3 walls + 15%, modules 4 walls + 25%; see Print list). Bearings, axles, inserts, bolts and the PTFE tubes are added as point masses.

| | |
|---|---|
| Mass moving with S1 | 1.48 kg (0.98 kg PETG + 0.50 kg hardware, gripper included) |
| COM below S1, arm straight | 379 mm, so m·g·d = **5.5 N·m** (10.1 N·m with 0.5 kg in the hand) |
| Effective lever, elbow bent 0/60/115° | 379 / 340 / 268 mm |

A spring can only follow sin(S1). At full abduction (S2 = 100°) the arm's weight moment about S1 falls to well under half of that. So the spring level is the **minimax** over all S1 × S2 × E poses: k·a·b = 3.39 N·m for both sides together.

With the gripper added, the masts stay as they are (a = 113 mm) and only k was retuned. A scan of k = 0.16 … 0.23 N/mm gives 61–63 % everywhere (the anchor screw takes up the rest). **k = 0.185 N/mm** puts the anchor screw at its nominal position.

**Result** (`tools/check_balance.py`, real sheave geometry):

| | Without springs | With springs |
|---|---|---|
| Peak S1 holding torque (S1 × S2 × E ranges) | 5.5 N·m | **2.08 N·m (−62%)** |
| Same, as force on a 62 mm crank (old bench) | 16.9 N | **6.4 N** |
| With 0.5 kg payload | 10.1 N·m / 31.0 N | 6.7 N·m / 20.5 N |

The springs don't act on S2: its peak stays 3.4 N·m.

**Spring (one per side).**

| Property | Value |
|---|---|
| Material | Music-wire extension spring |
| Wire | Ø 1.2 mm |
| OD | 15 mm |
| Free length | 105 mm over the hooks |
| Rate | k = 0.185 N/mm |
| Active coils | about 42 |
| Initial tension | ≤ 10 N |
| Working load | 18–36 N |
| Safe load | 40 N (45% of Rm in shear) |
| Extension in use | 84–178 mm (max length ≥ 290 mm) |

* **Where to buy:** any catalogue extension spring close to 0.185 N/mm, OD ~15 mm, ~105 mm free length. Before buying, check that initial tension is ≤ 10 N and maximum extended length is ≥ 290 mm. A rate within ±0.02 N/mm still keeps ≥ 61% balance (re-run `tools/check_balance.py` with your value in `balance.K_SPRING`).
* **Initial tension does not spoil the balance.** The spring pulls with Fi + k·e. Moving the anchor by Fi/k makes that equal k·|AB|, as long as the spring stays open over the whole range. That needs Fi < k·|AB|min ≈ 18 N.
* **No 2:1 tackle.** With a 1:1 run the cord pull stays at ≤ 35 N and the parts stay simple. The 175 mm extension fits along the deck rail. A 2:1 tackle would halve the stroke but double the spring load and add a moving pulley.

**Fitting and tuning.**
1. Tie each cord to its yoke pin with a bowline loop.
2. Run the cord up over the sheave, through the fairlead and back to the spring hook. The cord is about 0.41 m per side, including the knots.
3. With the arm hanging, set the eye-bolt nuts so the spring is stretched by about 178 mm (spring length about 283 mm).
4. Then fine-tune: the arm should hold still at S1 = 30° with the S1 motor unpowered.

## Printability (`tools/printability.py`)

**All 57 part files print without supports** in the orientation they are exported in. The check:
* no overhang steeper than 45°;
* no flat bridge wider than about 8 mm;
* no wall under 1.2 mm;
* one watertight body per part, fitting a 250 mm bed.

Notable orientations:
* **Hinge rotor halves** print **top plate down**. Their tube window opens upward, the bearing pockets are teardrops, and the drum top is covered by the neck. Near the top of the drum the cord runs through a short closed channel in the neck: thread it with a CA-stiffened tip after tying the knot.
* **Roll rotors** print top plate down.
* **The drive spool** prints standing on its −X boss. Its grooves (the crank's) are 0.5 mm deep at 1.9 mm pitch, about 3.4 turns per cord; the 6805 journal shoulder is a 43° cone.
* **Drive frames** print foot down. The top beam has a 48° roof and the bearing pockets (608, 6805) are teardrops.
* **Motor cups** print end plate down (pillars vertical), **horn adapters** motor face down, **saddles** bottom down (the unit faces lean 12°, the bolt holes are teardrops).
* **Gripper fingers** print gear side down (finger A's drum layer sits entirely on its gear sector). The palm prints plate down, the pads stand on their open end.

```bash
.venv/bin/python -m arm.export && .venv/bin/python tools/printability.py arm/stl
```

## Checked in CAD (`tools/check_arm.py`)

* All parts are valid, watertight single solids, and **every part fits a 250 mm bed**.
* **Every joint is collision-free at both range limits.** So are 7 combined poses (elbow fully bent with forearm twist and wrist, shoulder raised and rotated, and so on).
* Cords from each tube exit to its drum tangent and along the groove touch nothing, in every hinge and roll joint.
* **Two arms on the torso** (`torso`, `bends`, `drives`): see *Two-arm torso* above and the RESULT lines below.
* Every joint's center passage is big enough for the tubes that pass through it (13 at S1, down to 1 at W2, counting the gripper tube). The hinge slots grew for it: `slot_y` = 6.5 mm on S1, S2, E and W1.
* **Gripper:** fingers, pads and palm never touch over the whole opening (0–41°, 5° steps). The cord runs free from the socket to the drum, the spring runs free between the posts, the open stops end the travel, and the tool point equals the grasp centre. The arm's range and pose checks run with the fingers fully open (the widest pose).
* The balance hardware (yoke, masts, sheaves, cords, springs, posts) touches nothing at the S1/S2 limits and in combined poses. That includes the shoulder beam, the S1 base, the deck rails and the drive units. The yoke keeps clear of the upper arm at full abduction. **Reach limit added by the balance:** with the hand folded back over the shoulder (S2 > 90°, S3 < −70° and E > 105° all together), the wrist touches the −X mast. Every other combination clears it.

**Not verified (needs the printed arm):**
* friction through the long tube runs;
* real stiffness and backlash;
* print tolerances (bearing fits, heat-set insert holes);
* how the tubes actually lie inside the links.

## Weight optimisation

Same mass model for both columns (CAD volume + surface, skin solid, per-part infill, hardware as point masses):

| | Before | After |
|---|---|---|
| Printed PETG (4 walls + 40% everywhere → link/module profiles) | 1468 g | 1000 g |
| Hardware | 704 g | 563 g |
| **Arm total** | **2.17 kg** | **1.56 kg (−28%)** |
| S1 worst-case torque (0.3 kg payload) | 10.4 N·m | 7.7 N·m (−26%) |
| S1 peak with the balance springs (no payload) | 2.9 N·m | 1.83 N·m |

What changed, by how much it helps the shoulder:

* **Elbow and wrist hinges on 625 bearings + M5 axles** instead of 608 + M8 (−141 g of steel at the far end of the arm). They see at most 120 N; a 625 is rated 670 N static. Lugs shrink with them.
* **Print profiles:** links 3 walls + 15%, modules 4 walls + 25%. Loads are tiny (link stress ≤ 1.3 MPa, insert pull ≤ 52 N), so 40% infill bought nothing.
* **Link walls 3 → 2 mm.**
* **Roll stators slimmed by inclining the Bowden sockets 45° inwards** (plates Ø 126 → 104 mm on S3, Ø 122 → 100 mm on F). The link above S3 is now a straight L tube instead of a cone; the forearm cone flares to Ø 76 instead of Ø 98. `tools/check_arm.py tubes` checks that every socket exit lies inside the link bore.
* Not done, on purpose: hollowing the solid-looking cones. At 15% infill their inside is mostly air already; a cavity would only add two more solid skins.

## Loads (worst case over the workspace, 1.63 kg arm incl. the gripper + 0.3 kg object, no springs)

From `python -m arm.statics` (full table in `docs/loads_report.txt`); cord = torque / drum radius + 20 N pretension.

| | Joint torque | Cord tension | Force on a 62 mm crank (old bench) |
|---|---|---|---|
| S1 | 8.3 N·m | 217 N | 28 N |
| S2 | 5.7 N·m | 156 N | 20 N |
| S3 | 2.5 N·m | 109 N | 14 N |
| E | 2.5 N·m | 90 N | 12 N |
| W1 | 0.9 N·m | 63 N | 8 N |
| G (gripper) | 10 N per pad | 44 N | 6 N |

* **Cord margin:** 1 mm Dyneema breaks around 1000 N, so the shoulder has about a 4× margin. Use 1.2–1.5 mm cord if you have it: change `CORD_D` in `params.py` and re-export.
* **Holding:** the Feetech servos hold position while powered; the arm drops (slowly, balanced at S1) when the 12 V supply or the E-stop cuts them. Rest the hands on the table before switching off.

## Print list

`stl/PRINT_LIST.md` lists every file with its quantity and orientation; all STLs are already oriented for printing. Re-export with:

```bash
.venv/bin/python -m arm.export
```

**Material and settings:** PETG (or PLA+ for a first try), 0.2 mm layers, gyroid infill. Two profiles (the mass model uses the same ones):

* **Links** (`link_*`): 3 walls, 4 top/bottom layers, 15% infill. Their 2 mm tube walls print solid anyway; the worst link bending stress is 1.3 MPa.
* **Modules and everything else:** 4 walls, 25% infill. The shoulder beam gets 6 walls (see PRINT_LIST).

## Hardware

Both arms + torso. The arm lines are per arm × 2.

**Torso frame (aluminium extrusion, cut lengths):**

| Qty | Part | Where |
|---|---|---|
| 2 | 2040, 1315 mm | spine posts (x ±20 … 60, y 110 … 130) |
| 2 | 2040, 350 mm | base rails, screwed / clamped to the table (y 110 … 460) |
| 1 | 2040, 524 mm | clavicle (front of the spine top, carries the S1 hangers) |
| 2 | 2020, 540 mm | front and rear deck cross rails |
| 2 | 2020, 270 mm | deck stringers (spine → rear cross rail) |
| 4 | 2020, 405 mm | deck rails under the balance-cord planes (x ±90, ±260) |
| ~20 | 2020/2040 corner bracket + M5×10 + T-nut | frame joints |
| ~60 | M5×10–12 bolt + M5 T-nut (slot 6) | saddles to the spine (28), beam wings (8), S1 hangers (8), spring posts (8), electronics tray (4) |
| 2 | M5×70 bolt + T-nut | E-stop bracket |
| ~8 | 4×30 wood screw or a G-clamp | base rails to the table (the torso is front-heavy) |

**Drive units (16):**

| Qty | Part | Where |
|---|---|---|
| 4 | Feetech STS3095 (12 V) | S1, S2 (both arms) |
| 4 | Feetech STS3250 (12 V) | S3, E |
| 8 | Feetech STS3215-C018 (12 V) | F, W1, W2, gripper |
| 16 | 6805-2RS (25×37×7) | spool journal in wall B |
| 16 | 608ZZ (8×22×7) | spool, wall A |
| 16 | M8×70 bolt + M8 nut | spool shaft (head in the hub flange, nut in the nut cup) |
| 96 | M3 heat-set insert | wall B (4 per unit, motor cup) + hub flange (2 per unit, drive pins) |
| 32 | M3×12 bolt | drive pins (4 mm proud of the flange) |
| 48 | M3×55 bolt | STS3215/3250 cups (4 per unit) |
| 16 | M3×75 bolt or M3 threaded rod + nut | STS3095 cups (4 per unit) |
| 48 | M3×8 bolt | adapters to the STS3215/3250 horns (4 per unit) |
| 48 | M2×8 bolt | STS3215/3250 bottom cover to the cup (4 per unit) |
| 8 | cable tie 3.6 mm | STS3095 straps (2 per unit) |
| 30 | M4×45 bolt + nut | tension screws (one per socket block; the gripper units have one) |
| 56 | M4×25 bolt + nut | column unit feet to the saddles (nuts in the saddle's centre cavity) |
| 8 | M4×30 bolt + nut | S1 unit feet to the hangers |
| (upgrade) | Damiao DM-J4310-2EC + 6× M3×10 + 4× M3×70 per motor | any position; print its adapter and cup |

**Arms (× 2):**

| Qty (both arms) | Part | Where |
|---|---|---|
| 8 | 608ZZ bearing (8×22×7) | shoulder hinges S1, S2 |
| 12 | 625ZZ bearing (5×16×5) | elbow / wrist hinges E, W1, W2 |
| 4 | 6806-2RS (30×42×7) | upper-arm rotation |
| 4 | 6805-2RS (25×37×7) | forearm twist |
| 8 | 623ZZ (3×10×4) + M3×12 bolt (self-taps into the inner cheek, no nut) | roll idlers |
| 8 | M8×20 bolt, M8 nut, M8 washer | shoulder hinge stub axles |
| 12 | M5×16 bolt, M5 nut, M5 washer | elbow + wrist hinge stub axles |
| 164 | M3 heat-set insert (4.0 mm hole, 5.5 deep) | module plates (160), comb screws in the beam floors (4) |
| 160 | M3×10 bolt | links to modules (152), gripper palms to W2 (8) |
| 4 | M3×25 bolt + M3 nut | gripper finger pivots |
| 4 | M3×6 screw (self-tapping) | gripper pads |
| 2 | Extension spring, music wire 0.45–0.5 × OD 4 × ~34 mm inside the hooks, 0.15–0.3 N/mm | gripper opening springs |
| 8 | M3×12 bolt | S1 bases to the shoulder beams |
| 4 | M3×12 bolt | shoulder combs to the beam floors |
| 12 | M3×10 self-tapping | roll retainers |
| 4 | Extension spring, music wire 1.2 × OD 15 × 105 mm, 0.185 N/mm | shoulder balance |
| 8 | 623ZZ (3×10×4) | balance sheaves (2 each) |
| 4 | M3×20 bolt (axle, self-taps into the outer cheek) + 2 M3 washers each | balance sheaves |
| 4 | M3×20 bolt + nut | balance cord pins in the yokes |
| 8 | M4×20 bolt + nut + washer | balance masts through the beam walls (drill Ø4.5) |
| 4 | M5×60 eye bolt + 2 M5 nuts + washers | balance spring anchors (±20 mm tuning) |
| ~2 m | 1 mm Dyneema (from the spool) | balance cords, ~0.41 m each |
| **~32 m** | **PTFE tube 2 mm ID × 4 mm OD** | 2 per joint, ~0.5 m (S1) to ~1.3 m (W2), + 1 to each gripper (~1.4 m) *(est.)* |
| **~50 m** | **1 mm Dyneema** (buy a 100 m spool) | 2 cords per joint, + 1 per gripper *(est.)* |

**Electronics:**

| Qty | Part | Where |
|---|---|---|
| 1–2 | Teensy 4.1 (one per arm, `docs/ai_control.md` §3.4) on a 100 × 60 carrier | electronics tray, 4× M3 on 92 × 52 |
| 1–2 | Feetech TTL bus adapter (half-duplex buffer) | tray, 4× M3 on 40 × 30 |
| 1 | 12 V, ≥ 30 A power supply (360 W) | external; leads through an M12 cable gland in the tray |
| 1 | panel fuse holder Ø16 + 30 A fuse | tray |
| 1 | 22 mm mushroom E-stop, NC | E-stop bracket on the rear cross rail |
| 1 | 12 V DC contactor / relay, ≥ 40 A | cuts the motor supply when the E-stop opens |
| 16 | AS5048A (or MT6701) magnetic encoder + diametric magnet | joints (`docs/ai_control.md` §3.3), cables along the tubes |
| — | Feetech 3-pin bus cables, 2.5 mm² supply wire | bus daisy-chain down the spine slots |

## Assembly order

1. **Melt in all heat-set inserts** first.
2. **Build each hinge.** Press the bearings (608 on S1/S2, 625 on E/W1/W2) into both rotor halves.
   1. For each cord: tie a double overhand knot, pull the cord from the knot pocket through the chord hole into the groove, and lay it over the drum. Cord A goes on the −X half, cord B on the +X half; they wrap opposite ways.
   2. Drop the nuts into the lug slots, set the rotor halves between the lugs, add washers and screw in the stub axles (M8×20 or M5×16).
   3. Thread each cord down through its angled socket in the base.
3. **Build each roll joint.**
   1. Press both thin-section bearings into the rotor, one from each side.
   2. Slide the rotor over the stator tube and screw the retainer on top (3× M3×10, reached through the rotor's open top before the next link goes on). The rotor is held **only** by these bearings: the retainer clamps the upper inner race and preloads the pair, and the rotor never touches the stator plate (the visible gap is running clearance).
   3. Fit the idlers (623 + printed tyre on an M3×16).
   4. Knot each cord into its rotor anchor, lay it in its groove to the tangent point, over the idler and down through the socket.
4. **Build the gripper.**
   1. Push the gripper tube (~1.7 m) up into the palm's centre socket from below and thread the cord up through it and out of the boss.
   2. Lay the cord in finger A's drum groove and pass it through the anchor hole into the knot pocket (open on the spring side). Tie a double overhand knot and pull it back into the pocket.
   3. Set both fingers between the cheeks with the gears meshed and the **pads touching** (fingers closed). Push the M3×25 pivots through cheek, finger and cheek, nut on the outside, snug but free.
   4. Hook the opening spring over the two tail posts. Slide the pads on and fix each with an M3×6.
   5. Bolt the palm onto the W2 rotor plate (4× M3×10), with the tube going down through W2's open centre.
5. **Thread the tubes and bolt the chain together from the wrist upwards.**
   1. Start at W2. Push a PTFE tube over each cord and into its socket. The gripper tube already runs through W2.
   2. Each time you add the next module or link above, feed **all** tubes that already exist through its center.
   3. Bolt each link with M3×10 through its lips into the inserts.
   4. Mark every tube at both ends with its joint and cord (A/B), for example with colored tape.
6. **Build the torso frame.** Base rails on the table behind the work area (screw or clamp them: the torso is front-heavy), the two spine posts on them, the front cross rail against the posts' front faces at z 679 … 699, the stringers and the rear cross rail, then the four deck rails on top (x = ±90, ±260). Bolt the clavicle across the post tops (front face). Bolt the electronics tray to the posts' back and the E-stop bracket onto the rear cross rail.
7. **Build the 16 drive units.** Press the 608 into wall A and the 6805 into wall B; set 4 M3 inserts in wall B and 2 in the spool flange; screw the two M3×12 drive pins in (4 mm proud). Slide the spool's journal into the 6805, push the M8×70 in from the flange side (head in its hex pocket) through the spool and the 608, and tighten the nut in the nut cup on the wall A side. Drop the socket blocks in (A on the +y side), fit the tube clip with the two M4×45 tension screws. Bolt the horn adapter to the motor's horn, the motor into its cup, and the cup onto wall B (4× M3 through the pillars) so the pins enter the adapter's slots.
8. **Mount the units.** Column: one saddle per level on the spine posts (2 M5 per post), one right and one left unit on each saddle (4× M4 each, nuts in the saddle's centre cavity, reached from the front), S2 at the bottom up to G at the top. S1: a hanger under each end of the clavicle, the S1 unit hanging under it (4× M4, nuts in the hanger's pockets).
9. **Mount each shoulder.** Bolt the shoulder beam onto its deck rails (4× M5 through the wings), the S1 base under its floor (4× M3×12 from inside the channel), the shoulder comb over the floor opening (2× M3×12 into inserts). Hook the masts and fit the balance hardware (see above), the spring posts on the deck rails.
10. **Route the tubes.** Push each of the 13 pass-through tubes (S2 … W2 A/B + gripper) up through its comb hole (see `assembly.comb_assignment()`: lower units on the inner columns) and on to its unit; the two S1 tubes come straight down from the S1 unit past the comb's arms into their sockets. Keep every free bend at R ≥ 25 mm. Zip-tie each joint's encoder cable to its tube pair at the unit's cable guide and to the comb's arms.
11. **Fit each cord.** Feed the two tubes into the socket blocks through the tube clip; put an M4 nut in each block's nut trap. With the joint at 0°, wind each cord **about 2 turns** onto the spool (helix as listed by `check_arm.py bends`: on the left arm most units take the cords the other way round), pass it through the flange hole and knot it. **Gripper:** one tube, socket block A only; fingers fully open on their stops, cord wound on, slack taken up with the block A screw.
12. **Tension:** turn each unit's two M4 screws (one per cord) to lift its socket block until the joint has no play; use them separately to trim the joint's zero position. Re-tension after the first hours, because Dyneema and PETG both settle a little. Set each servo's zero with the joint encoder (firmware), and its current limit below the cord break load.

Tip: pre-stretch the Dyneema by hanging 5 kg on it overnight before cutting it to length.

## Later

* **Motor upgrade:** swap S1/S2 (then S3/E) to the Damiao DM-J4310 (print its adapter and cup; 24 V + CAN, `docs/ai_control.md`).
* **Lower torso:** S1 sockets aimed sideways (a `hinge.py` change) would let the S1 units join the column; two units per level would halve the column height at the cost of sideways tube bends.
