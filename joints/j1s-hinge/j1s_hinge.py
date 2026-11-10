"""
J1-S - 1-DOF tendon hinge, stepper version with 6:1 tendon reduction
=====================================================================

    python joints/j1s-hinge/j1s_hinge.py      # -> joints/j1s-hinge/stl/

What changed vs J1
------------------
* REDUCTION IN THE TENDON. Joint drum r = 36 mm, motor spool r = 6 mm -> 6:1.
  A NEMA17 (~0.4 N*m) gives ~2 N*m at the joint (elbow class). The joint
  turns 6x slower than the motor, so +/-90 deg joint = +/-1.5 motor turns.
* HELICAL MOTOR SPOOL. Each tendon has its own helical groove (4.5 turns),
  so the cord never stacks on itself.
* WIDE OPEN CENTRE FOR BOWDEN TUBES. The drum is a big hollow wheel: two
  side walls carry the 608 bearings, the 14 mm wide x r=28 mm cavity in the
  middle is open over +/-100 deg at the bottom. Up to 3 PTFE tubes (4 mm OD)
  pass through and can bend at >= 25 mm radius without kinking.
* BOWDEN SOCKETS: 3 tube sockets in the proximal cap and 3 in the distal end
  plate, to measure friction and coupling of tube-routed tendons.
* The proximal link tapers (tendons converge from r=36 at the drum to r=6
  at the spool), roughly upper-arm sized.

Frame: joint axis = X, links along Z, joint at origin (same as J1).
"""
import math
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from cadlib import (box, chord_anchor, cyl_x, external_tangent, groove_ring,  # noqa: E402
                    hex_prism_x, hex_prism_z, hole_along, polar_yz)

OUT = Path(__file__).parent / "stl"

# ---------------------------------------------------------------- parameters
CORD_D = 1.0
# drum (joint)
R_CORD_DRUM = 36.0                 # tendon centreline radius on the drum
R_GROOVE = R_CORD_DRUM - CORD_D / 2
R_HUB = R_CORD_DRUM + 2.0
GROOVE_W = 2.4
GROOVE_X = 11.5
ANCHOR_WRAP = 135.0
RANGE = 100.0
# spool (motor)
R_CORD_SPOOL = 6.0
RATIO = R_CORD_DRUM / R_CORD_SPOOL      # 6.0
HELIX_PITCH = 1.6
SPOOL_TURNS = 4.5                       # per tendon; +/-1.5 needed for +/-90 deg

# bearings 608
BRG_D, BRG_B = 22.0, 7.0
FIT = 0.15
CAV_HALF = 7.0          # cavity half width (x) -> 14 mm for tubes
WALL_IN = 1.0           # thin wall between cavity and bearing (outer-race ledge)
HUB_HALF = CAV_HALF + WALL_IN + BRG_B   # 15
R_CAV = 28.0
R_LEDGE = 9.5

# links
LUG_GAP, LUG_T, LUG_R = 1.0, 12.0, 16.0
CAP_TOP = -(R_HUB + 8.0)          # -46
CAP_T = 6.0
PROX_LEN = 100.0                  # shell below the cap
PROX_TOP_RX, PROX_TOP_RY = 30.0, 37.0
PROX_BOT_R = 23.0
PROX_WALL = 3.0
FLANGE, FLANGE_T, FLANGE_HOLE = 62.0, 5.0, 23.0

DIST_OD, DIST_ID = 44.0, 37.0
DIST_START = R_HUB + 4.0          # 42
DIST_LEN = 116.0
END_T = 6.0
TAB_Z = (72.0, 112.0, 148.0)

TUBE_OD, TUBE_ID = 4.0, 2.0       # PTFE Bowden liner
TUBE_X = (-5.0, 0.0, 5.0)

M3_CLR, M3_HEAD_D, M3_NUT_AF = 3.4, 6.2, 5.6
M8_CLR, M8_NUT_AF, M8_NUT_T = 8.4, 13.2, 6.8

# base / NEMA17
BASE_TOP_T = 6.0
BASE_H = 82.0
SHAFT_Z = -36.0                    # motor shaft below the base top
NEMA = 42.3
NEMA_HOLE = 31.0
NEMA_PILOT = 22.0
SHAFT_D = 5.0
MOTOR_WALL_X = -21.0               # +x face of the motor wall (motor face at -26)
MOTOR_WALL_T = 5.0
SLOT = 8.0


# ---------------------------------------------------------------- derived
def tube_bot():
    return CAP_TOP - CAP_T - PROX_LEN


def spool_z():
    return tube_bot() + SHAFT_Z


def tendon_line():
    """Flexor tangent points (drum, spool) in (y, z); extensor is the mirror y -> -y."""
    return external_tangent(R_CORD_DRUM, R_CORD_SPOOL, spool_z())


def tendon_y_at(z):
    (y1, z1), (y2, z2) = tendon_line()
    t = (z - z1) / (z2 - z1)
    return y1 + t * (y2 - y1)


# ---------------------------------------------------------------- distal half
def distal_half():
    hub = cyl_x(R_HUB, -HUB_HALF, 0)
    neck_x = -(GROOVE_X - GROOVE_W / 2 - 0.8)          # 9.5: tendon clears the neck
    neck = box(neck_x, 0, -20, 20, 20, DIST_START + 5)
    top = DIST_START + DIST_LEN
    tube = (cq.Workplane("XY").workplane(offset=DIST_START).circle(DIST_OD / 2).extrude(DIST_LEN)
            .intersect(box(-DIST_OD, 0, -DIST_OD, DIST_OD, DIST_START, top)))
    body = hub.union(neck).union(tube)
    tab_y = DIST_OD / 2 + 4.5
    for z in TAB_Z:
        body = body.union(box(-6, 0, DIST_OD / 2 - 3, tab_y + 4.5, z - 5, z + 5))
        body = body.union(box(-6, 0, -tab_y - 4.5, -(DIST_OD / 2 - 3), z - 5, z + 5))

    cuts = [
        cq.Workplane("XY").workplane(offset=DIST_START + 4).circle(DIST_ID / 2).extrude(top - END_T - DIST_START - 4),
        cyl_x(R_CAV, -CAV_HALF, 0.1),                                  # tube cavity
        cyl_x(R_LEDGE, -CAV_HALF - WALL_IN - 0.1, -CAV_HALF + 0.1),     # hole in the thin wall
        cyl_x(BRG_D / 2 + FIT, -HUB_HALF - 0.1, -CAV_HALF - WALL_IN),  # bearing pocket
        box(-CAV_HALF + 0.5, 0.1, -13, 13, 10, DIST_START + 6),        # cavity -> distal tube
    ]
    sector = (cq.Workplane("YZ").workplane(offset=-CAV_HALF)
              .moveTo(0, 0)
              .lineTo(*polar_yz(R_HUB + 3, -90 - RANGE))
              .threePointArc(polar_yz(R_HUB + 3, -90), polar_yz(R_HUB + 3, -90 + RANGE))
              .close().extrude(CAV_HALF + 0.1))
    cuts.append(sector)
    cuts.append(groove_ring(-GROOVE_X, R_GROOVE, R_HUB, GROOVE_W))
    cuts.append(chord_anchor(-GROOVE_X, R_GROOVE, R_HUB, ANCHOR_WRAP, +1, CORD_D))
    # lighten the side wall between bearing and groove
    for a in range(0, 360, 60):
        y, z = polar_yz(20.5, a + 30)
        cuts.append(hole_along((-HUB_HALF - 1, y, z), (-CAV_HALF, y, z), 9.0))
    # tab bolts (+y head, -y nut -> both halves identical)
    for z in TAB_Z:
        for s in (1, -1):
            y = s * tab_y
            cuts.append(hole_along((-7, y, z), (1, y, z), M3_CLR))
            cuts.append(hole_along((-7, y, z), (-3.5, y, z), M3_HEAD_D) if s > 0
                        else hex_prism_x(-7, -3.5, y, z, M3_NUT_AF))
    # Bowden sockets in the end plate (tube pushed in from below, cord exits on top)
    for x in TUBE_X:
        if x > 0.1:
            continue
        cuts.append(cq.Workplane("XY").workplane(offset=top - END_T - 0.1).center(x, 0)
                    .circle(TUBE_OD / 2 + 0.15).extrude(4.1))
        cuts.append(cq.Workplane("XY").workplane(offset=top - END_T).center(x, 0)
                    .circle(TUBE_ID / 2 + 0.2).extrude(END_T + 1))
    # end-plate holes for a test weight / next segment
    for y in (-13.0, 13.0):
        cuts.append(cq.Workplane("XY").workplane(offset=top - END_T - 1).center(-7, y)
                    .circle(M3_CLR / 2).extrude(END_T + 2))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- proximal
def proximal():
    cap_bot = CAP_TOP - CAP_T
    tb = tube_bot()
    lug_in = HUB_HALF + LUG_GAP
    lug_out = lug_in + LUG_T

    outer = (cq.Workplane("XY").workplane(offset=tb).circle(PROX_BOT_R)
             .workplane(offset=PROX_LEN).ellipse(PROX_TOP_RX, PROX_TOP_RY).loft())
    cap = (cq.Workplane("XY").workplane(offset=cap_bot).ellipse(PROX_TOP_RX, PROX_TOP_RY).extrude(CAP_T)
           .union(box(-lug_out, lug_out, -LUG_R - 4, LUG_R + 4, cap_bot, CAP_TOP)))
    flange = box(-FLANGE / 2, FLANGE / 2, -FLANGE / 2, FLANGE / 2, tb, tb + FLANGE_T).edges("|Z").fillet(6)
    body = outer.union(cap).union(flange)
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        body = body.union(box(x0, x1, -LUG_R, LUG_R, CAP_TOP - 1, 0).union(cyl_x(LUG_R, x0, x1)))

    w = PROX_WALL
    inner = (cq.Workplane("XY").workplane(offset=tb - 0.01).circle(PROX_BOT_R - w)
             .workplane(offset=PROX_LEN + 0.01).ellipse(PROX_TOP_RX - w, PROX_TOP_RY - w).loft())
    cuts = [inner]
    # tendon holes, placed on the converging tendon line
    zc = CAP_TOP - CAP_T / 2
    for x, s in ((-GROOVE_X, 1), (GROOVE_X, -1)):
        y = s * tendon_y_at(zc)
        cuts.append(cq.Workplane("XY").workplane(offset=cap_bot - 1).center(x, y).slot2D(5.0, 3.0, 90).extrude(CAP_T + 2))
    # Bowden sockets in the cap (tube pushed in from above)
    for x in TUBE_X:
        cuts.append(cq.Workplane("XY").workplane(offset=CAP_TOP - 4).center(x, 0).circle(TUBE_OD / 2 + 0.15).extrude(5))
        cuts.append(cq.Workplane("XY").workplane(offset=cap_bot - 1).center(x, 0).circle(TUBE_ID / 2 + 0.2).extrude(CAP_T + 2))
    # M8 stub axle + captive nut
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        cuts.append(cyl_x(M8_CLR / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - M8_NUT_T / 2, xm + M8_NUT_T / 2, -M8_NUT_AF / 2, M8_NUT_AF / 2,
                        -M8_NUT_AF / 2 / math.cos(math.pi / 6), LUG_R + 1))
    # flange: bore + bolt holes
    cuts.append(cq.Workplane("XY").workplane(offset=tb - 1).circle(PROX_BOT_R - w).extrude(FLANGE_T + 2))
    for x in (-FLANGE_HOLE, FLANGE_HOLE):
        for y in (-FLANGE_HOLE, FLANGE_HOLE):
            cuts.append(cq.Workplane("XY").workplane(offset=tb - 1).center(x, y).circle(M3_CLR / 2).extrude(FLANGE_T + 2))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- spool
SP_COLLAR = (-18.0, -13.0)     # motor side, set screw
SP_FLEX = (-13.0, -5.0)        # flexor helix
SP_MID = (-5.0, 5.0)           # middle flange
SP_EXT = (5.0, 13.0)           # extensor helix
SP_END = (13.0, 17.0)          # outer flange


def _helix_cut(x0, x1):
    """Helical groove for the cord, cord centre at R_CORD_SPOOL, along +X from x0 to x1."""
    h = x1 - x0
    path = cq.Wire.makeHelix(HELIX_PITCH, h, R_CORD_SPOOL)
    prof = cq.Workplane("XZ").center(R_CORD_SPOOL, 0).circle(CORD_D / 2 + 0.15)
    g = prof.sweep(cq.Workplane().add(path), isFrenet=True)
    # helix is built along +Z from z=0 -> rotate so it runs along +X starting at x0
    return g.rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, 0, 0))


def spool():
    core_r = R_CORD_SPOOL + CORD_D / 2 + 0.3       # 6.8
    body = (cyl_x(11.0, *SP_COLLAR)
            .union(cyl_x(core_r, SP_FLEX[0], SP_EXT[1]))
            .union(cyl_x(10.0, *SP_MID))
            .union(cyl_x(11.0, *SP_END)))
    cuts = []
    for sec in (SP_FLEX, SP_EXT):
        cuts.append(_helix_cut(sec[0] + 0.6, sec[1] - 0.6))
    # D-bore for the 5 mm motor shaft (flat 0.5 mm), shaft reaches ~x=-2
    bore = cyl_x(SHAFT_D / 2 + 0.1, SP_COLLAR[0] - 1, -1.0)
    flat = box(SP_COLLAR[0] - 1, -1.0, -5, 5, SHAFT_D / 2 - 0.5, 5)
    cuts.append(bore.cut(flat))
    # M3 set screw onto the flat + nut slot
    xm = sum(SP_COLLAR) / 2
    cuts.append(hole_along((xm, 0, 0), (xm, 0, 12), 3.2))
    cuts.append(box(xm - 1.5, xm + 1.5, -2.9, 2.9, 4.0, 12))
    # tendon end holes (axial) through the collar and the outer flange + knot recess
    for (x0, x1) in (SP_COLLAR, SP_END):
        y, z = polar_yz(8.0, -45)
        cuts.append(hole_along((x0 - 1, y, z), (x1 + 1, y, z), 1.8))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- base
def base():
    """Base frame: top surface z = 0, motor shaft at (0, 0, SHAFT_Z). Print upside down."""
    yw = 45.0
    xl, xr = -78.0, 31.0
    sw = 5.0
    top = box(MOTOR_WALL_X - MOTOR_WALL_T, xr, -yw - sw, yw + sw, -BASE_TOP_T, 0)
    wall_m = box(MOTOR_WALL_X - MOTOR_WALL_T, MOTOR_WALL_X, -yw, yw, -BASE_H, 0)
    wall_r = box(xr - 5, xr, -yw, yw, -BASE_H, 0)
    side_a = box(xl, xr, yw, yw + sw, -BASE_H, 0)
    side_b = box(xl, xr, -yw - sw, -yw, -BASE_H, 0)
    body = top.union(wall_m).union(wall_r).union(side_a).union(side_b)

    cuts = []
    # opening for tendons + spool under the proximal link
    cuts.append(box(-20, 20, -19, 19, -BASE_TOP_T - 1, 1).edges("|Z").fillet(5))
    for x in (-FLANGE_HOLE, FLANGE_HOLE):
        for y in (-FLANGE_HOLE, FLANGE_HOLE):
            cuts.append(cq.Workplane("XY").workplane(offset=-BASE_TOP_T - 1).center(x, y).circle(M3_CLR / 2).extrude(BASE_TOP_T + 2))
            cuts.append(hex_prism_z(-BASE_TOP_T - 1, -BASE_TOP_T + 2.5, x, y, M3_NUT_AF))
    # motor wall: pilot slot + 4 slotted screw holes -> slide the motor DOWN to tension
    x0 = MOTOR_WALL_X - MOTOR_WALL_T - 1
    cuts.append(cq.Workplane("YZ").workplane(offset=x0).center(0, SHAFT_Z - SLOT / 2)
                .slot2D(NEMA_PILOT + 1 + SLOT, NEMA_PILOT + 1, 90).extrude(MOTOR_WALL_T + 2))
    for dy in (-NEMA_HOLE / 2, NEMA_HOLE / 2):
        for dz in (-NEMA_HOLE / 2, NEMA_HOLE / 2):
            cuts.append(cq.Workplane("YZ").workplane(offset=x0).center(dy, SHAFT_Z + dz - SLOT / 2)
                        .slot2D(SLOT + 3.4, 3.4, 90).extrude(MOTOR_WALL_T + 2))
    # lighten far wall and side walls
    cuts.append(box(xr - 6, xr + 1, -yw + 14, yw - 14, -BASE_H + 14, -BASE_TOP_T - 10))
    for y0, y1 in ((yw - 1, yw + sw + 1), (-yw - sw - 1, -yw + 1)):
        cuts.append(box(xl + 12, MOTOR_WALL_X - MOTOR_WALL_T - 12, y0, y1, -BASE_H + 14, -12))
        cuts.append(box(MOTOR_WALL_X + 8, xr - 13, y0, y1, -BASE_H + 14, -BASE_TOP_T - 8))
    # floor screw holes (self-tapping, into the side-wall edges)
    for x in (xl + 8, xr - 10):
        for y in (yw + sw / 2, -yw - sw / 2):
            cuts.append(cq.Workplane("XY").workplane(offset=-BASE_H - 1).center(x, y).circle(1.4).extrude(15))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- main
def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


def assembly(angle=0.0):
    d = distal_half()
    pair = cq.Compound.makeCompound([d.val(), d.rotate((0, 0, 0), (0, 0, 1), 180).val()])
    return {
        "distal": cq.Workplane().add(pair).rotate((0, 0, 0), (1, 0, 0), angle),
        "proximal": proximal(),
        "spool": spool().rotate((0, 0, 0), (1, 0, 0), angle * RATIO).translate((0, 0, spool_z())),
        "base": base().translate((0, 0, tube_bot())),
    }


if __name__ == "__main__":
    export(distal_half().rotate((0, 0, 0), (0, 1, 0), 90), "distal_half")   # split face down
    export(proximal(), "proximal")                                          # flange down
    export(spool().rotate((0, 0, 0), (0, 1, 0), -90), "spool")              # motor-side collar down
    export(base().rotate((0, 0, 0), (1, 0, 0), 180), "base")                # top plate down
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]),
                        str(OUT / "assembly_preview.step"))
    print("ratio", RATIO, "tendon at cap y=%.1f" % tendon_y_at(CAP_TOP - CAP_T / 2),
          "spool z", spool_z(), "->", OUT)
