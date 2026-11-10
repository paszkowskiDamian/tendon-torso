# Tendon Arm: 7 DOF, hand-cranked, fully printable

A human-like arm without a hand. All seven joints are driven by **1 mm Dyneema** cords in **PTFE Bowden tubes** from seven **hand cranks** next to the shoulder. Every crank sits where a motor (NEMA17 + spool) will go later.

![arm](../docs/img/arm_assembly.png)

Interactive 3D model: open `docs/viewer/` (tab *Arm · 7 DOF*), see the repo [README](../README.md).

| | Joint | Type | Range | Moment arm | Crank turns, full range |
|---|---|---|---|---|---|
| S1 | Shoulder flexion (forward) | hinge L | −30…+120° | 42 mm | 2.2 |
| S2 | Shoulder abduction (sideways) | hinge L | −10…+100° | 42 mm | 1.6 |
| S3 | Upper-arm rotation | roll (2× 6806) | ±80° | 28 mm | 1.6 |
| E | Elbow | hinge M | 0…+115° | 36 mm | 1.4 |
| F | Forearm twist | roll (2× 6805) | ±85° | 25.5 mm | 1.5 |
| W1 | Wrist flexion | hinge S | ±60° | 20 mm | 0.8 |
| W2 | Wrist tilt | hinge S | ±30° | 20 mm | 0.4 |

Upper arm is 300 mm (S2 axis to elbow), forearm 250 mm (elbow to wrist). The arm hangs about 0.9 m below the shoulder beam, so mount the beam on a high table, shelf or bench edge.

## Controlling the hand (kinematics)

Give a hand position (and optionally an orientation) and get the 7 joint angles plus the turns for each crank:

```bash
.venv/bin/python -m arm.ik -40 164 -695
```

The equations, solver and accuracy are in [docs/kinematics.md](../docs/kinematics.md). The viewer's **Reach mode** runs the same solver live in the browser.

## How it's built

```
crank ×7 ──PTFE tubes──► shoulder beam ──► S1 ─ S2 ─ link ─ S3 ─ upper link ─ E ─ link ─ F ─ forearm link ─ W1 ─ W2 ─► (gripper plate)
                                           hinge hinge      roll               hinge     roll                 hinge hinge
```

* **Hinge module** (`hinge.py`): base + two rotor halves.
  * **Base:**
    * a plate with heat-set inserts;
    * two lugs with captive M8 nuts (the M8×20 stub axles go into 608 bearings);
    * a boss with two angled Bowden sockets aimed exactly along the drum tangents;
    * a slot that lets the tubes of the joints further out pass through.
  * **Rotor:** printed as two halves split on the motion plane, each flat on that face.
    * The drum is hollow, with the bearings at its ends.
    * Each half has one cord groove with a knotted chord anchor.
    * A top plate with inserts holds the next link.
* **Roll module** (`roll.py`): stator + rotor + retainer + 2 idlers. Check the race shoulder sizes of the 6806/6805 you buy against `roll.py`.
  * The stator tube carries two thin-section bearings.
  * The rotor ring holds their outer races and has two grooves.
  * Each cord comes up from a socket in the stator plate and turns 90° over a 623 idler pulley onto the rotor.
  * The bore stays open for the tubes going further down.
* **Links** (`link.py`):
  * tubes with outward bolting lips, bolt heads on the lips;
  * a full 48° cone carries the top lip, with a counterbore up through it for each bolt head;
  * on the S→XM link the bottom bolts are reached with a hex key from inside the wide bore.
* **Crank module** (`crank.py`):
  * an M8 shaft on two 608s, with a helical spool clamped between them;
  * a crank arm with a **lock pin** that indexes every 10° on a ring of holes;
  * two **socket blocks** (A and B), one per cord, each pulled up by its own M4 screw (10 mm travel), so each cord's length and pretension is set on its own;
  * a **tube clip** on top, held by the two tension screw heads, that guides both tubes out vertically.
* **Torso** (`torso.py`):
  * a U-channel shoulder beam that screws onto the table and overhangs its edge, with S1 bolted underneath;
  * crank racks for 3 cranks each;
  * two snap-in **tube combs** that hold the 14 tubes in a labelled grid (S1 … W2 × A/B) inside the channel.
* **Fittings** (`fittings.py`): a cord thimble to join Dyneema to a spring hook or an eye.

## Printability (`tools/printability.py`)

**All 41 parts print without supports** in the orientation they are exported in. The check:
* no overhang steeper than 45°;
* no flat bridge wider than about 8 mm;
* no wall under 1.2 mm;
* one watertight body per part, fitting a 250 mm bed.

Notable orientations:
* **Hinge rotor halves** print **top plate down**. Their tube window opens upward, the bearing pockets are teardrops, and the drum top is covered by the neck. Near the top of the drum the cord runs through a short closed channel in the neck: thread it with a CA-stiffened tip after tying the knot.
* **Roll rotors** print top plate down.
* **The crank spool** prints standing on its boss. Its grooves are 0.5 mm deep at 1.9 mm pitch, about 3.4 turns per cord.
* **Crank frames** print foot down. The top beam has a 48° roof and the bearing holes are teardrops.

```bash
.venv/bin/python -m arm.export && .venv/bin/python tools/printability.py arm/stl
```

## Checked in CAD (`tools/check_arm.py`)

* All parts are valid, watertight single solids, and **every part fits a 250 mm bed**.
* **Every joint is collision-free at both range limits.** So are 7 combined poses (elbow fully bent with forearm twist and wrist, shoulder raised and rotated, and so on).
* Cords from each tube exit to its drum tangent and along the groove touch nothing, in every hinge and roll joint.
* Crank arms of neighbouring modules never touch, and the knob clears the table.
* Every joint's center passage is big enough for the tubes that pass through it (12 at S1, down to 2 at W1).

**Not verified (needs the printed arm):**
* friction through the long tube runs;
* real stiffness and backlash;
* print tolerances (bearing fits, heat-set insert holes);
* how the tubes actually lie inside the links.

## Loads (arm horizontal, 2 kg arm + 0.5 kg payload, static)

| | Joint torque | Cord tension | Force on the crank knob |
|---|---|---|---|
| S1 | ~10.6 N·m | ~250 N | ~32 N |
| S2 | ~9.3 N·m | ~220 N | ~29 N |
| E | ~3.6 N·m | ~100 N | ~13 N |
| W1 | ~1.0 N·m | ~50 N | ~6 N |

* **Cord margin:** 1 mm Dyneema breaks around 1000 N, so the shoulder has about a 4× margin. Use 1.2–1.5 mm cord if you have it: change `CORD_D` in `params.py` and re-export.
* **Lock pins:** the arm won't hold itself up without them. Keep the pins in whenever you let go of a crank.

## Print list

`stl/PRINT_LIST.md` lists every file with its quantity and orientation; all STLs are already oriented for printing. Re-export with:

```bash
.venv/bin/python -m arm.export
```

**Material and settings:** PETG (or PLA+ for a first try), 0.2 mm layers, 4–6 walls, 40% gyroid. Use 100% infill or 6+ walls around the lugs, drums and the shoulder beam.

## Hardware

| Qty | Part | Where |
|---|---|---|
| 24 | 608ZZ bearing (8×22×7) | 2 per hinge (10), 2 per crank (14) |
| 2 | 6806-2RS (30×42×7) | upper-arm rotation |
| 2 | 6805-2RS (25×37×7) | forearm twist |
| 4 | 623ZZ (3×10×4) + M3×12 bolt (self-taps into the inner cheek, no nut) | roll idlers |
| 10 | M8×20 bolt, M8 nut, M8 washer | hinge stub axles |
| 7 | M8×70 (or M8×65) bolt + M8 nut | crank shafts |
| 80 | M3 heat-set insert (4.0 mm hole, 5.5 deep) | module plates |
| 76 | M3×10 bolt | links to modules |
| 4 | M3×12 bolt | S1 base to beam |
| 6 | M3×10 self-tapping | roll retainers |
| 14 | M4×45 bolt + nut | crank tension screws (one per socket block) |
| 28 | M4×12 bolt + nut | crank feet to racks |
| 7 | M4×30 bolt + nut | crank knobs |
| ~20 | 4×30 wood screws | beam and racks to the table |
| **~17 m** | **PTFE tube 2 mm ID × 4 mm OD** | 2 per joint, ~0.55 m (S1) to ~1.6 m (W2) |
| **~25 m** | **1 mm Dyneema** (buy a 50 m spool) | 2 cords per joint, ~1.25 m (S1) to ~2.2 m (W2) |

## Assembly order

1. **Melt in all heat-set inserts** first.
2. **Build each hinge.** Press the 608s into both rotor halves.
   1. For each cord: tie a double overhand knot, pull the cord from the knot pocket through the chord hole into the groove, and lay it over the drum. Cord A goes on the −X half, cord B on the +X half; they wrap opposite ways.
   2. Drop the M8 nuts into the lug slots, set the rotor halves between the lugs, add washers and screw in the M8×20 stub axles.
   3. Thread each cord down through its angled socket in the base.
3. **Build each roll joint.**
   1. Press both thin-section bearings into the rotor, one from each side.
   2. Slide the rotor over the stator tube and screw the retainer on top.
   3. Fit the idlers (623 + printed tyre on an M3×16).
   4. Knot each cord into its rotor anchor, lay it in its groove to the tangent point, over the idler and down through the socket.
4. **Thread the tubes and bolt the chain together from the wrist upwards.**
   1. Start at W2. Push a PTFE tube over each cord and into its socket.
   2. Each time you add the next module or link above, feed **all** tubes that already exist through its center.
   3. Bolt each link with M3×10 through its lips into the inserts.
   4. Mark every tube at both ends with its joint and cord (A/B), for example with colored tape.
5. **Mount the shoulder beam** on the table edge (wood screws) and bolt the S1 base underneath it. Thread all 14 tubes through the two tube combs (each tube in its labelled hole), snap the combs into the side windows at y = 33 and y = 60, and lead the tubes along the channel to the cranks. Keep every free bend at R ≥ 25 mm.
6. **Fit each crank.**
   1. Feed its two tubes down through the tube clip and the beam and push them into the socket blocks (A on the +y block, B on the −y block). Put an M4 nut in each block's nut trap and screw an M4×45 through the clip and the beam into it.
   2. Put the joint at 0° and the crank lock pin in hole 0.
   3. Wind each cord **about 2 turns** onto the spool: cord A on the −X helix, cord B on the +X helix, both going *under* the spool. Pass the cord through the flange hole and knot it.
7. **Tension:** turn each crank's two M4 screws (one per cord) to lift its socket block until the joint has no play; use them separately to trim the joint's zero position. Re-tension after the first hours, because Dyneema and PETG both settle a little.

Tip: pre-stretch the Dyneema by hanging 5 kg on it overnight before cutting it to length.

## Later

* **Motors:** swap each crank for a NEMA17 + the J1-S spool. The tubes and joints stay as they are.
* **Gripper:** bolts onto the W2 rotor plate (4× M3). Its cable passes through W2's open center.
* **Second arm:** mirror the shoulder beam and add 7 more cranks.
