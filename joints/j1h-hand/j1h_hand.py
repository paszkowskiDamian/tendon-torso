"""
J1-H: hand-cranked tendon hinge with 3:1 force multiplication (bench rig)
========================================================================

Parametric CadQuery model. Run:

    .venv/bin/python joints/j1h-hand/j1h_hand.py

and STL/STEP files land in joints/j1h-hand/stl/.

Frame
-----
* Joint axis = X, Z up, joint origin = (0, 0, 0). The base plate top is at
  z = -HD. The crank spool axis is parallel to the joint axis, at (y = -D, z = 0).
* Joint angle 0 deg = lever pointing straight up (+Z). Positive angle turns
  +Y towards +Z: the lever tips TOWARDS the crank. Usable range -90..+90 deg;
  past about +91 deg the lever tip meets the lower cord.

Concept
-------
* The output link is a LEVER on a hollow DRUM (cord radius R_DRUM = 30 mm)
  that turns on two 608 bearings between two pylons of the base plate.
* A hand CRANK turns a SPOOL (cord radius R_SPOOL = 10 mm) in its own frame.
  Two cords (A over the top, B under the bottom) run as straight external
  tangents from the spool to the drum: a cut-open belt. One cord winds on while
  the other pays out, so there is no slack in either direction.
* Force multiplication = R_DRUM / R_SPOOL = 3. The cord tension is the same
  everywhere, so the torque on the drum is 3x the torque on the spool, and the
  crank turns 3 times the joint angle (+-90 deg joint = +-270 deg crank). Lever
  and crank arm have holes at the same radius (50 mm): a weight on the lever
  is held by one third of it on the crank handle.
* Tensioning: the crank frame sits in slots and is pushed away from the drum
  by one M4 screw in the base plate. Moving the spool away tightens both cords
  at once. Coarse length is set with the knots on the spool flanges.
* The drum centre stays open (bearings at its ends, slot in the bottom, channel
  up the lever) so a pass-through cord for a more distal joint can be tested,
  as on J1.

Parts
-----
  base.stl        x1   base plate with hinge pylons and tension-screw block
  drum_a.stl      x1   -X half of drum + lever (print hub face down)
  drum_b.stl      x1   +X half (same, mirrored cord anchor)
  frame.stl       x1   crank frame (two walls, slotted foot)
  spool.stl       x1   helical spool, r = 10 mm
  crank_arm.stl   x1   60 mm crank with lock-pin hole
  nut_cup.stl     x1   holds the M8 nut of the crank shaft
  knob.stl        x1   turns on an M4 bolt
  lock_pin.stl    x1   locks the crank every 30 deg (10 deg of joint)
"""
from pathlib import Path
import math
import sys

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from cadlib import box, cyl_x, hex_prism_x, hex_prism_z, hole_along, groove_ring, polar_yz  # noqa: E402

OUT = Path(__file__).parent / "stl"

# ---------------------------------------------------------------- parameters
# cords and ratio
CORD_D = 2.5                  # braided cord diameter (2.5 mm paracord-type braid)
R_DRUM = 30.0                 # cord centre radius on the drum  = moment arm
R_SPOOL = 10.0                # cord centre radius on the spool
RATIO = R_DRUM / R_SPOOL      # 3.0
R_GROOVE = R_DRUM - CORD_D / 2
R_HUB = R_DRUM + 2.0
GROOVE_W = CORD_D + 0.9
GROOVE_X = 14.5               # |x| of the two drum grooves (cord A at -x, B at +x)
WRAP0 = 135.0                 # wrap of each cord on the drum at 0 deg
RANGE = 90.0                  # +- joint range (deg); the crank turns RATIO * RANGE
PASS_HALF = 5.0               # pass-through slot half width (x)
SLOT_RANGE = 100.0            # half angle of the bottom slot in the drum
CHAN_Y = 2.5                  # pass-through channel half width (y) in the lever

# drum bearings 608 (8 x 22 x 7)
BRG_D, BRG_B, BRG_BORE = 22.0, 7.0, 8.0
FIT = 0.15
CAV_HALF = 11.0
HUB_HALF = CAV_HALF + BRG_B   # 18
R_CAV = 9.5

# layout
HD = 72.0                     # joint axis above the base plate top
D = 138.0                     # spool axis behind the joint axis (same height)
SPOOL_C = (-D, 0.0)           # spool axis (y, z) in the joint frame
CRANK_R = 50.0                # crank handle radius (clears the frame foot)
LEVER_HOLES = (50.0, 100.0)   # weight holes on the lever; the first one = crank radius
LEVER_L = 106.0               # lever length to the tip centre
LEVER_Y = 9.0                 # lever half width (y)
LEVER_X = HUB_HALF            # lever as wide as the drum; the grooves tunnel through its root
TAB_Z = (48.0, 72.0)          # M3 tabs joining the halves (kept low: at +90 deg they pass under cord A)
TAB_Y = LEVER_Y + 2.5
HUB_BOLT_R, HUB_BOLT_ANG = 25.0, (20.0, 160.0)   # M3 through-bolts in the hub (head, nut)

# hinge pylons
LUG_GAP = 1.8                 # hub face -> lug (M8 washer, 1.6 mm, on the inner race)
LUG_T = 12.0
LUG_IN = HUB_HALF + LUG_GAP
LUG_OUT = LUG_IN + LUG_T
LUG_R = 16.0

# base plate
BASE_T = 6.0
BASE_X = 48.0
BASE_Y = (-197.0, 50.0)
Z_BASE = -HD                  # top face of the base plate

# crank frame (local frame: spool axis = X through the origin)
FOOT_T, FOOT_X, FOOT_Y = 10.0, 44.0, 36.0
WALL_B_R, WALL_A_R = 40.0, 22.0    # radius of the round wall tops
SLOT_X, SLOT_Y, SLOT_LEN = 40.0, 28.0, 10.0   # foot slots (tension travel = SLOT_LEN)
INDEX_R, INDEX_N, INDEX_DEPTH = 30.0, 12, 6.0
PUSH_Y = (-D - FOOT_Y - SLOT_LEN - 9.0, -D - FOOT_Y - SLOT_LEN - 3.0)   # tension block (y range)
PUSH_Z = Z_BASE + 5.0         # tension screw axis

# spool: one helical groove per cord, the cord centre at R_SPOOL
PITCH = CORD_D + 2.0          # 1.7 mm land between turns (printable ridge)
TURNS = 2.5
HELIX_C = R_SPOOL + 0.15      # centre radius of the swept groove circle
GROOVE_R = CORD_D / 2 + 0.15  # swept circle radius: groove bottom at R_SPOOL - CORD_D/2
HELIX_END = GROOVE_X + 1.25 * PITCH    # outer end of each helix: 1.25 turns from the 0-deg fleet point
HELIX_X = (HELIX_END - TURNS * PITCH, HELIX_END)   # cord B helix (x range); cord A mirrored
_F0 = HELIX_END + GROOVE_R + 0.1       # inner flange face, right after the groove run-out
SP_FLANGE = (_F0, _F0 + 6.0)  # |x| range of the outer flanges (6 mm thick: room for the knot notch)
SP_FLANGE_R = 15.0
CORE_R = R_SPOOL + 0.5        # cord 70 % buried, opening 2.7 mm > CORD_D
KNOT_NOTCH = 4.0              # depth of the knot notch in the flange outer face
WALL_IN = SP_FLANGE[1] + 1.6  # M8 washer between each flange and its 608
WALL_T = 9.0
WALL_OUT = WALL_IN + WALL_T
ARM_X = (WALL_OUT + 0.5, WALL_OUT + 8.5)

# fasteners
M3_CLR, M3_HEAD_D, M3_NUT_AF = 3.4, 6.2, 5.8
M4_CLR, M4_NUT_AF, M4_NUT_T = 4.5, 7.3, 3.4
M8_CLR, M8_NUT_AF, M8_NUT_T, M8_HEAD_AF, M8_HEAD_T = 8.4, 13.2, 6.8, 13.2, 5.5


# ---------------------------------------------------------------- geometry
def tangents():
    """External tangent lines spool -> drum in the YZ plane.

    Returns {"A": ..., "B": ...} with the drum tangent point `td` (y, z), the spool
    tangent point `ts` (y, z) and the common normal angle `phi` (deg, from +Y
    towards +Z). Cord A is the upper line, cord B the lower one.
    """
    cy, cz = SPOOL_C
    d = math.hypot(cy, cz)
    u = math.atan2(cz, cy)
    g = math.acos((R_DRUM - R_SPOOL) / d)
    out = {}
    for name, s in (("A", -1), ("B", 1)):
        p = u + s * g
        n = (math.cos(p), math.sin(p))
        out[name] = dict(td=(R_DRUM * n[0], R_DRUM * n[1]),
                         ts=(cy + R_SPOOL * n[0], cz + R_SPOOL * n[1]),
                         phi=math.degrees(p) % 360.0)
    return out


def drum_anchor_angle(cord):
    """Drum-frame angle (deg) of the cord anchor at joint angle 0.

    Cord A leaves its tangent point and wraps towards +Y (decreasing phi), cord B
    wraps towards +Y the other way round (increasing phi). Both wrap the far side
    of the drum, away from the spool, so the lever on top never meets them.
    """
    t = tangents()[cord]["phi"]
    return t - WRAP0 if cord == "A" else t + WRAP0


def spool_wrap_angle(cord, x):
    """World angle (deg) of the spool groove at axial position x (joint at 0 deg).

    The cord enters the helix at the tangent angle at the fleet point
    x = -+GROOVE_X and the groove walks outwards towards the anchor flange.
    Both helices are left-handed (phi decreases with x).
    """
    t = tangents()[cord]["phi"]
    xf = -GROOVE_X if cord == "A" else GROOVE_X
    return t - 360.0 / PITCH * (x - xf)


def cord_free_length():
    t = tangents()["A"]
    return math.dist(t["td"], t["ts"])


# ---------------------------------------------------------------- helpers
def teardrop_x(r, x0, x1, y=0.0, z=0.0, phi=50.0, alpha=52.0):
    """Horizontal hole along X that prints without support (point up, +Z)."""
    c = cyl_x(r, x0, x1)
    s, co = math.sin(math.radians(phi)), math.cos(math.radians(phi))
    h = r * co + r * s * math.tan(math.radians(alpha))
    tri = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(-r * s, r * co), (0, h), (r * s, r * co), (0, 0)]).close()
           .extrude(x1 - x0))
    return c.union(tri).translate((0, y, z))


def cut_all(body, cuts):
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- drum + lever
def drum_half(anchor_deg):
    """The -X half (x in [-HUB_HALF, 0]) of drum + lever. Prints HUB FACE DOWN (x = -HUB_HALF
    on the bed): the bottom slot, the lever channel and the open centre then face up, the
    bearing pocket is a plain bore from the bed, and the lever spans the full drum width
    so it stands on the bed too. The cord grooves tunnel through the lever root.

    anchor_deg: drum-frame angle where this half's cord dives into its anchor hole.
    The cord arrives along the groove with DEcreasing phi.
    """
    H = HUB_HALF
    hub = cyl_x(R_HUB, -H, 0)
    lever = (box(-LEVER_X, 0, -LEVER_Y, LEVER_Y, 0, LEVER_L)
             .union(cyl_x(LEVER_Y, -LEVER_X, 0).translate((0, 0, LEVER_L))))
    body = hub.union(lever)
    for z in TAB_Z:
        for s in (1, -1):
            y0, y1 = ((LEVER_Y - 1.0, TAB_Y + 4.5) if s > 0 else (-TAB_Y - 4.5, -(LEVER_Y - 1.0)))
            body = body.union(box(-H, 0, y0, y1, z - 5, z + 5))

    cuts = [
        cyl_x(R_CAV, -CAV_HALF, 0.1),                                  # open centre
        cyl_x(BRG_D / 2 + FIT, -H - 0.1, -CAV_HALF),                    # 608 pocket
        box(-PASS_HALF, 0.1, -CHAN_Y, CHAN_Y, 0, LEVER_L + LEVER_Y + 1),   # channel up the lever
        groove_ring(-GROOVE_X, R_GROOVE, R_HUB, GROOVE_W),
    ]
    # bottom slot: pass-through cords cross the axis over +-SLOT_RANGE
    cuts.append(cq.Workplane("YZ").workplane(offset=-PASS_HALF).moveTo(0, 0)
                .lineTo(*polar_yz(R_HUB + 3, -90 - SLOT_RANGE))
                .threePointArc(polar_yz(R_HUB + 3, -90), polar_yz(R_HUB + 3, -90 + SLOT_RANGE))
                .close().extrude(PASS_HALF + 0.1))
    # cord anchor: from the groove bottom, slanting inwards into the open centre.
    # The stopper knot sits inside the cavity (reach it through the bottom slot).
    y0, z0 = polar_yz(R_GROOVE + 0.3, anchor_deg)
    y1, z1 = polar_yz(6.0, anchor_deg)
    cuts.append(hole_along((-GROOVE_X, y0, z0), (-7.0, y1, z1), CORD_D + 1.0))
    # M3 through-bolts in the hub: head counterbore at the first angle, nut at the second
    for k, a in enumerate(HUB_BOLT_ANG):
        y, z = polar_yz(HUB_BOLT_R, a)
        cuts.append(hole_along((-H - 1, y, z), (1, y, z), M3_CLR))
        if k == 0:
            cuts.append(hole_along((-H - 1, y, z), (-H + 3.5, y, z), M3_HEAD_D))
        else:
            cuts.append(hex_prism_x(-H - 1, -H + 3.0, y, z, M3_NUT_AF))
    # lever tabs: +y head counterbore, -y nut trap, both 11 mm deep (M3x20)
    for z in TAB_Z:
        for s in (1, -1):
            y = s * TAB_Y
            cuts.append(hole_along((-H - 1, y, z), (1, y, z), M3_CLR))
            if s > 0:
                cuts.append(hole_along((-H - 1, y, z), (-7.0, y, z), M3_HEAD_D))
            else:
                cuts.append(hex_prism_x(-H - 1, -7.4, y, z, M3_NUT_AF))
    # weight holes through the lever (M4 bolt or a hook)
    for z in LEVER_HOLES:
        cuts.append(hole_along((-LEVER_X - 1, 0, z), (1, 0, z), M4_CLR))
    return cut_all(body, cuts)


def drum_a():
    return drum_half(drum_anchor_angle("A"))


def drum_b():
    """+X half: the mirrored model turned 180 deg about Z, so its anchor lands at
    cord B's angle and its heads meet A's nuts."""
    return drum_half(180.0 - drum_anchor_angle("B")).rotate((0, 0, 0), (0, 0, 1), 180)


def drum_pair(angle=0.0):
    comp = cq.Compound.makeCompound([drum_a().val(), drum_b().val()])
    return cq.Workplane().add(comp).rotate((0, 0, 0), (1, 0, 0), angle)


# ---------------------------------------------------------------- base plate
def base():
    """Base plate with the two hinge pylons and the tension-screw block. Prints as is."""
    plate = box(-BASE_X, BASE_X, BASE_Y[0], BASE_Y[1], Z_BASE - BASE_T, Z_BASE).edges("|Z").fillet(8)
    body = plate
    for s in (1, -1):
        x0, x1 = sorted((s * LUG_IN, s * LUG_OUT))
        pylon = box(x0, x1, -LUG_R, LUG_R, Z_BASE - 0.1, 0).union(cyl_x(LUG_R, x0, x1))
        gx = x1 if s > 0 else x0
        gusset = (cq.Workplane("XZ")
                  .polyline([(gx, -24.0), (gx, Z_BASE - 0.1), (s * (BASE_X - 3.0), Z_BASE - 0.1)])
                  .close().extrude(3.0, both=True))
        body = body.union(pylon).union(gusset)
    # tension-screw block behind the crank frame
    body = body.union(box(-12.0, 12.0, PUSH_Y[0], PUSH_Y[1], Z_BASE - 0.1, Z_BASE + 16.0))

    cuts = []
    for s in (1, -1):
        x0, x1 = sorted((s * LUG_IN, s * LUG_OUT))
        cuts.append(teardrop_x(M8_CLR / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - M8_NUT_T / 2, xm + M8_NUT_T / 2, -M8_NUT_AF / 2, M8_NUT_AF / 2,
                        -M8_NUT_AF / 2 / math.cos(math.pi / 6), LUG_R + 1))
    # pass-through opening under the drum
    cuts.append(box(-PASS_HALF, PASS_HALF, -12, 12, Z_BASE - BASE_T - 1, Z_BASE + 1).edges("|Z").fillet(3))
    # crank frame screws: M4 from the top, nut traps underneath
    for x in (-SLOT_X, SLOT_X):
        for y in (-D - SLOT_Y, -D + SLOT_Y):
            cuts.append(cq.Workplane("XY").workplane(offset=Z_BASE - BASE_T - 1).center(x, y)
                        .circle(M4_CLR / 2).extrude(BASE_T + 2))
            cuts.append(hex_prism_z(Z_BASE - BASE_T - 1, Z_BASE - BASE_T + M4_NUT_T + 0.3, x, y, M4_NUT_AF))
    # tension screw (M4) through the block, nut dropped in from the top
    yc = (PUSH_Y[0] + PUSH_Y[1]) / 2
    cuts.append(hole_along((0, PUSH_Y[0] - 1, PUSH_Z), (0, PUSH_Y[1] + 1, PUSH_Z), M4_CLR))
    cuts.append(box(-M4_NUT_AF / 2, M4_NUT_AF / 2, yc - (M4_NUT_T + 0.3) / 2, yc + (M4_NUT_T + 0.3) / 2,
                    PUSH_Z - M4_NUT_AF / 2 / math.cos(math.pi / 6), Z_BASE + 17))
    return cut_all(body, cuts)


# ---------------------------------------------------------------- crank frame (local frame)
def _wall(x0, x1, r_top, half_bot):
    """Wall standing on the foot: trapezoid from +-half_bot at the foot to +-r_top at
    the axis height, with a round top of radius r_top."""
    prof = (cq.Workplane("YZ").workplane(offset=x0)
            .polyline([(-half_bot, -HD), (half_bot, -HD), (r_top, 0), (-r_top, 0)]).close()
            .extrude(x1 - x0))
    return prof.union(cyl_x(r_top, x0, x1))


def frame():
    """Crank frame: foot + wall A (-X, small) + wall B (+X, with the index ring).
    Local frame: spool axis = X through the origin, foot bottom at z = -HD. Prints foot down."""
    foot = box(-FOOT_X, FOOT_X, -FOOT_Y, FOOT_Y, -HD, -HD + FOOT_T)
    body = foot.union(_wall(WALL_IN, WALL_OUT, WALL_B_R, FOOT_Y)).union(_wall(-WALL_OUT, -WALL_IN, WALL_A_R, FOOT_Y))
    cuts = []
    for s in (1, -1):
        xi, xo = s * WALL_IN, s * WALL_OUT
        a, b = sorted((xi, xi + s * BRG_B))
        cuts.append(teardrop_x(BRG_D / 2 + FIT, a - 0.1 * (s < 0), b + 0.1 * (s > 0)))   # 608 pocket from the inner face
        a, b = sorted((xi, xo))
        cuts.append(teardrop_x(9.5, a - 1, b + 1))                                        # clearance for the bosses
    # index holes for the lock pin (blind, from the outer face of wall B)
    for k in range(INDEX_N):
        y, z = polar_yz(INDEX_R, 360.0 / INDEX_N * k)
        cuts.append(cyl_x(2.2, WALL_OUT - INDEX_DEPTH, WALL_OUT + 1).translate((0, y, z)))
    # foot slots: at zero travel the screw sits at the +Y end of its slot
    for x in (-SLOT_X, SLOT_X):
        for y in (-SLOT_Y, SLOT_Y):
            cuts.append(cq.Workplane("XY").workplane(offset=-HD - 1).center(x, y - SLOT_LEN / 2)
                        .slot2D(SLOT_LEN + M4_CLR, M4_CLR, angle=90).extrude(FOOT_T + 2))
    return cut_all(body, cuts)


# ---------------------------------------------------------------- spool
def _grooved_ring(x0, x1, phi0):
    """Ring of the spool core carrying one left-handed helical groove from x0 to x1
    (plus GROOVE_R run-out at both ends), at world angle phi0 (deg, from +Y towards +Z)
    at x0. Built along Z like the helix and turned into place: cutting a helix straight
    out of the revolved spool makes OCC skip parts of the cut."""
    h = x1 - x0
    e = GROOVE_R + 0.1
    path = cq.Wire.makeHelix(PITCH, h, HELIX_C, lefthand=True)
    prof = cq.Workplane("XZ").center(HELIX_C, 0).circle(GROOVE_R)
    g = prof.sweep(cq.Workplane().add(path), isFrenet=True)
    ring = cq.Workplane("XY").workplane(offset=-e).circle(CORE_R).circle(CORE_R - 3.0).extrude(h + 2 * e)
    core = ring.cut(g)
    assert ring.val().Volume() - core.val().Volume() > 50, "helix groove cut failed"
    # built along +Z starting at +X; after the Y turn the start sits at -90 deg in YZ
    core = core.rotate((0, 0, 0), (0, 1, 0), 90)
    return core.rotate((0, 0, 0), (1, 0, 0), phi0 + 90.0).translate((x0, 0, 0))


def spool():
    """Crank spool, cord radius R_SPOOL on both helices. Built along X; prints standing.
    Each cord enters its helix at |x| = GROOVE_X (joint at 0 deg) and walks outwards to an
    axial anchor hole through the flange; the stopper knot sits in a notch in the flange's
    outer face."""
    f0, f1 = SP_FLANGE
    xa, xb = HELIX_X
    e = GROOVE_R + 0.1
    body = (cyl_x(CORE_R, -f0 - 0.1, f0 + 0.1)
            .union(cyl_x(SP_FLANGE_R, -f1, -f0)).union(cyl_x(SP_FLANGE_R, f0, f1)))
    # the outer shell of the core is replaced by the grooved rings in the helix sections
    for x0, x1 in ((xa - e, xb + e), (-xb - e, -xa + e)):
        body = body.cut(cyl_x(CORE_R + 1.0, x0, x1).cut(cyl_x(CORE_R - 2.5, x0, x1)))
    body = body.union(_grooved_ring(xa, xb, spool_wrap_angle("B", xa)))
    body = body.union(_grooved_ring(-xb, -xa, spool_wrap_angle("A", -xb)))
    cuts = [cyl_x(M8_CLR / 2, -f1 - 1, f1 + 1)]
    for cord, xe, sgn in (("B", xb, 1), ("A", -xb, -1)):
        a = spool_wrap_angle(cord, xe)
        y, z = polar_yz(HELIX_C, a)
        cuts.append(hole_along((sgn * (xe - 1.0), y, z), (sgn * (f1 + 1), y, z), CORD_D + 1.0))
        notch = box(min(sgn * (f1 - KNOT_NOTCH), sgn * (f1 + 1)), max(sgn * (f1 - KNOT_NOTCH), sgn * (f1 + 1)),
                    6.5, SP_FLANGE_R + 2, -4.0, 4.0).rotate((0, 0, 0), (1, 0, 0), a)
        cuts.append(notch)
    return cut_all(body, cuts)


# ---------------------------------------------------------------- crank arm, nut cup, knob, pin
def crank_arm():
    x0, x1 = ARM_X
    hub = cyl_x(13.0, x0, x1)
    boss = cyl_x(6.0, WALL_IN + BRG_B, x0 + 0.1)        # presses the 608-B inner race
    arm = box(x0, x1, -7, 7, 0, CRANK_R).union(cyl_x(7.0, x0, x1).translate((0, 0, CRANK_R)))
    body = hub.union(boss).union(arm)
    cuts = [cyl_x(M8_CLR / 2, WALL_IN, x1 + 1),
            hex_prism_x(x1 - M8_HEAD_T - 0.3, x1 + 1, 0, 0, M8_HEAD_AF),
            cyl_x(2.25, x0 - 1, x1 + 1).translate((0, 0, INDEX_R)),           # lock pin
            cyl_x(M4_CLR / 2, x0 - 1, x1 + 1).translate((0, 0, CRANK_R)),     # knob bolt
            hex_prism_x(x0 - 1, x0 + M4_NUT_T + 0.3, 0, CRANK_R, M4_NUT_AF)]
    return cut_all(body, cuts)


NUT_CUP_T = 10.0


def nut_cup():
    x1 = -(WALL_IN + BRG_B)
    x0 = -WALL_OUT - 0.5 - NUT_CUP_T
    body = cyl_x(6.0, -WALL_OUT - 0.5, x1).union(cyl_x(12.0, x0, -WALL_OUT - 0.5))
    cuts = [cyl_x(M8_CLR / 2, x0 - 1, x1 + 1), hex_prism_x(x0 - 1, x0 + M8_NUT_T + 0.3, 0, 0, M8_NUT_AF)]
    return cut_all(body, cuts)


LOCK_PIN_L = (ARM_X[1] - ARM_X[0]) + 0.5 + INDEX_DEPTH - 0.5


def lock_pin():
    return (cq.Workplane("XY").circle(2.0).extrude(LOCK_PIN_L)
            .union(cq.Workplane("XY").workplane(offset=LOCK_PIN_L).circle(6.0).extrude(4.0))
            .edges(">Z").fillet(1.0))


def knob():
    k = cq.Workplane("XY").circle(7.0).extrude(26.0).cut(cq.Workplane("XY").workplane(offset=-1).circle(M4_CLR / 2 + 0.2).extrude(28))
    return k.edges().fillet(1.0)


# ---------------------------------------------------------------- assembly
def assembly(angle=35.0):
    """All parts in the joint frame, joint at `angle` deg, crank at RATIO * angle."""
    cy, cz = SPOOL_C
    ca = RATIO * angle
    rot_spool = lambda w: w.rotate((0, 0, 0), (1, 0, 0), ca).translate((0, cy, cz))   # noqa: E731
    kn = (knob().rotate((0, 0, 0), (0, 1, 0), 90).translate((ARM_X[1] + 0.5, 0, CRANK_R)))
    pin = (lock_pin().rotate((0, 0, 0), (0, 1, 0), -90).translate((WALL_OUT - INDEX_DEPTH + 0.5, 0, INDEX_R)))
    return {
        "base": base(),
        "drum": drum_pair(angle),
        "frame": frame().translate((0, cy, cz)),
        "spool": rot_spool(spool()),
        "crank_arm": rot_spool(crank_arm()),
        "nut_cup": nut_cup().translate((0, cy, cz)),
        "knob": rot_spool(kn),
        "lock_pin": rot_spool(pin),
    }


def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


if __name__ == "__main__":
    cy, cz = SPOOL_C
    export(base().translate((0, 0, HD + BASE_T)), "base")
    export(drum_a().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, HUB_HALF)), "drum_a")   # hub face down
    export(drum_b().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, HUB_HALF)), "drum_b")
    export(frame().translate((0, 0, HD)), "frame")                        # foot down
    export(spool().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, SP_FLANGE[1])), "spool")   # standing
    export(crank_arm().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, ARM_X[1])), "crank_arm")  # outer face down
    export(nut_cup().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, WALL_OUT + 0.5 + NUT_CUP_T)), "nut_cup")
    export(knob(), "knob")
    export(lock_pin(), "lock_pin")
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]),
                        str(OUT / "assembly_preview.step"))
    t = tangents()
    print(f"ratio {RATIO:.2f}  cord free span {cord_free_length():.1f} mm  "
          f"tangent A drum {t['A']['phi']:.1f} deg  B {t['B']['phi']:.1f} deg")
    print("done ->", OUT)
