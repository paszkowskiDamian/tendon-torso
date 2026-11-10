"""
J1 - single-DOF tendon-driven hinge (test rig)
================================================

Parametric CadQuery model. Run:

    python joints/j1-hinge/j1_hinge.py

and STL/STEP files land in joints/j1-hinge/stl/.

Coordinate frame (joint frame)
------------------------------
* Joint axis      = X
* Link long axis  = Z (distal link points +Z at 0 deg)
* Joint origin    = (0, 0, 0)

Concept
-------
* The distal link carries a grooved DRUM (radius R) that rotates on two 608
  bearings. An antagonistic tendon pair (flexor / extensor) wraps the drum in
  opposite directions; each one is knotted into a chord hole in the drum.
* The joint's own tendons drop STRAIGHT down the hollow proximal link to a
  spool of the same radius on the servo (1:1, closed loop - one tendon winds
  while the other unwinds, so a single motor drives both directions).
* The centre of the drum is EMPTY: the two bearings sit at the ends, the
  M8 stub shafts only reach a few mm in, and a slot through the drum lets
  tendons for more distal joints pass straight through the joint axis.
  A cord crossing the axis does not change length when the joint bends, so
  distal joints are (to first order) decoupled from this one. This is the
  feature that lets every motor eventually live in the torso.
* The distal link is printed as TWO IDENTICAL HALVES split on the motion
  plane (X = 0), each printed flat on the split face. Print it twice.

Parts
-----
  proximal.stl     x1   hollow link with lugs, bolts onto the base
  distal_half.stl  x2   identical halves of the distal link + drum
  spool.stl        x1   servo spool (same radius as the drum)
  base.stl         x1   stand that holds the servo ("torso" stand-in)
"""

from pathlib import Path
import math
import cadquery as cq

OUT = Path(__file__).parent / "stl"

# ---------------------------------------------------------------- parameters
# tendon / drum
CORD_D = 1.0          # Dyneema / braided PE line diameter
R_GROOVE = 15.0       # drum groove bottom radius -> moment arm ~ R_GROOVE + CORD_D/2
R_HUB = 17.5          # drum outer radius (groove flanges)
GROOVE_W = 2.4        # groove width
GROOVE_X = 14.5       # |x| of the flexor/extensor grooves
R_CORD = R_GROOVE + CORD_D / 2  # tendon centreline radius
ANCHOR_WRAP = 135.0   # deg of wrap at 0 deg joint angle (allows ~ +/-120 deg)
RANGE = 100.0         # deg, half-angle of the pass-through slot opening

# bearings 608 (8 x 22 x 7)
BRG_D, BRG_B, BRG_BORE = 22.0, 7.0, 8.0
FIT = 0.15            # press-fit clearance on radius for bearing pockets
CAV_HALF = 11.0       # half width of the central cavity (|x| < CAV_HALF)
HUB_HALF = CAV_HALF + BRG_B   # = 18, half width of the hub
R_CAV = 9.5           # cavity radius (leaves a ledge for the outer race)
PASS_HALF = 5.0       # half width (x) of the pass-through slot

# links
LUG_GAP = 1.0         # gap between drum face and lug (fill with M8 washer)
LUG_T = 12.0          # lug thickness (holds a captive M8 nut)
AXIS_H = 27.0         # height of joint axis above the proximal cap
CAP_T = 6.0
PROX_OD, PROX_ID = 54.0, 48.0
PROX_LEN = 90.0       # tube length below the cap
FLANGE = 70.0         # square flange
FLANGE_T = 5.0
FLANGE_HOLE = 28.0    # +/- M3 hole position on the flange

DIST_OD, DIST_ID = 44.0, 37.0
DIST_START = 21.0     # z where the distal tube starts (must clear the lugs)
DIST_LEN = 110.0
TAB_Z = (52.0, 90.0, 126.0)   # z of the M3 tabs that join the halves

M3_CLR, M3_HEAD_D, M3_NUT_AF = 3.4, 6.2, 5.6
M8_CLR, M8_NUT_AF, M8_NUT_T = 8.4, 13.2, 6.8

# servo (MG996R / standard-size). CHECK AGAINST YOUR SERVO.
SERVO_L, SERVO_W = 40.7, 19.9         # body footprint
SERVO_HOLE_L, SERVO_HOLE_W = 49.5, 10.0   # mounting-hole pattern
SERVO_SHAFT_OFF = 10.2                # shaft offset from body centre along L
SERVO_TAB_TO_HORN = 17.0              # tab underside -> outer face of round horn
HORN_HOLE_R = 8.0                     # radius of the small screw holes in the disc horn
SLOT = 8.0                            # vertical slot length for tensioning

BASE_TOP_T = 6.0
BASE_H = 70.0          # base height (top plate to floor)
SPOOL_Z = -32.0        # spool axis below the top of the base


# ---------------------------------------------------------------- helpers
def cyl_x(r, x0, x1):
    """Cylinder along X from x0 to x1."""
    return cq.Workplane("YZ").workplane(offset=x0).circle(r).extrude(x1 - x0)


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def hole_along(p0, p1, d):
    """Cylinder of diameter d from point p0 to p1 (both 3-tuples)."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane(cq.Plane(origin=p0, xDir=_perp(v), normal=v)).circle(d / 2).extrude(v.Length)


def _perp(v):
    a = cq.Vector(1, 0, 0) if abs(v.x) < 0.9 * v.Length else cq.Vector(0, 1, 0)
    return v.cross(a).normalized()


def polar_yz(r, phi_deg):
    """Point in the YZ plane; phi measured from +Y towards +Z."""
    p = math.radians(phi_deg)
    return r * math.cos(p), r * math.sin(p)


def groove_ring(x, r_in, r_out, w):
    """Annular groove centred at x with a 45deg chamfered outer edge (printable)."""
    ring = cyl_x(r_out + 2, x - w / 2, x + w / 2).cut(cyl_x(r_in, x - w / 2, x + w / 2))
    return ring


def drum_anchor(x, sign=1):
    """Chord hole + knot pocket for one tendon (flexor side, phi from +Y over +Z).

    The tendon leaves the groove at ANCHOR_WRAP and dives through the drum wall
    on a chord, exiting further round where a knot sits in a counterbore.
    """
    y0, z0 = polar_yz(R_GROOVE + 0.3, ANCHOR_WRAP)
    exit_phi = ANCHOR_WRAP + 65
    y1, z1 = polar_yz(R_HUB + 1, exit_phi)
    h = hole_along((x, y0, z0), (x, y1, z1), CORD_D + 1.0)
    # knot pocket at the exit
    d = cq.Vector(y1 - y0, z1 - z0, 0).normalized()
    ky, kz = y1 - d.x * 3.5, z1 - d.y * 3.5
    k = hole_along((x, ky, kz), (x, y1 + d.x * 2, z1 + d.y * 2), 4.5)
    return h.union(k)


# ---------------------------------------------------------------- distal half
def distal_half():
    """Half of the distal link, x in [-HUB_HALF.., 0]. Print on the x=0 face."""
    x_hub = -HUB_HALF
    hub = cyl_x(R_HUB, x_hub, 0)
    # neck: narrow enough (|x| < groove) that tendons can wrap over the top
    neck_x = -(GROOVE_X - GROOVE_W / 2 - 1.0)
    neck = box(neck_x, 0, -R_HUB + 2, R_HUB - 2, 0, DIST_START + 3)
    tube = cq.Workplane("XY").workplane(offset=DIST_START).circle(DIST_OD / 2).extrude(DIST_LEN)
    tube = tube.intersect(box(-DIST_OD, 0, -DIST_OD, DIST_OD, DIST_START, DIST_START + DIST_LEN))
    # chamfered transition neck->tube so the tendon over the drum top is clear
    body = hub.union(neck).union(tube)

    # tabs joining the two halves (y = +/-)
    tab_y = DIST_OD / 2 + 4.5
    for z in TAB_Z:
        for s in (1, -1):
            body = body.union(box(-6, 0, s * (DIST_OD / 2 - 3) if s > 0 else -tab_y - 4.5,
                                  tab_y + 4.5 if s > 0 else -(DIST_OD / 2 - 3), z - 5, z + 5))

    # end plate at the far end with mounting holes for the next segment
    end_z = DIST_START + DIST_LEN
    # ---- cuts
    cuts = []
    cuts.append(cq.Workplane("XY").workplane(offset=DIST_START + 4).circle(DIST_ID / 2).extrude(DIST_LEN))
    cuts.append(cyl_x(R_CAV, -CAV_HALF, 0.1))                                  # central cavity
    cuts.append(cyl_x(BRG_D / 2 + FIT, -HUB_HALF - 0.1, -CAV_HALF))            # bearing pocket
    cuts.append(box(-PASS_HALF, 0.1, -8, 8, 0, DIST_START + 5))                # cavity -> tube
    # pass-through slot on the bottom, open over +/-RANGE around -Z
    sector = (cq.Workplane("YZ").workplane(offset=-PASS_HALF)
              .moveTo(0, 0)
              .lineTo(*polar_yz(R_HUB + 3, -90 - RANGE))
              .threePointArc(polar_yz(R_HUB + 3, -90), polar_yz(R_HUB + 3, -90 + RANGE))
              .close().extrude(PASS_HALF + 0.1))
    cuts.append(sector)
    # flexor groove on this half (x = -GROOVE_X)
    cuts.append(groove_ring(-GROOVE_X, R_GROOVE, R_HUB, GROOVE_W))
    cuts.append(drum_anchor(-GROOVE_X))
    # tab bolts: +y head counterbore, -y nut trap -> the two halves are identical
    for z in TAB_Z:
        for s in (1, -1):
            y = s * tab_y
            cuts.append(hole_along((-7, y, z), (1, y, z), M3_CLR))
            if s > 0:
                cuts.append(hole_along((-7, y, z), (-3.5, y, z), M3_HEAD_D))
            else:
                nut = (cq.Workplane("YZ").workplane(offset=-7)
                       .center(y, z).polygon(6, M3_NUT_AF / math.cos(math.pi / 6)).extrude(3.5))
                cuts.append(nut)
    # end-plate holes (4x M3 on +/-12 for the next joint / a test weight)
    for y, z in ((12, end_z - 6), (-12, end_z - 6)):
        cuts.append(hole_along((-DIST_OD, y, z), (1, y, z), M3_CLR))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- proximal
def proximal():
    cap_top = -AXIS_H
    cap_bot = cap_top - CAP_T
    tube_bot = cap_bot - PROX_LEN
    lug_in = HUB_HALF + LUG_GAP
    lug_out = lug_in + LUG_T
    lug_r = 13.0

    tube = cq.Workplane("XY").workplane(offset=tube_bot).circle(PROX_OD / 2).extrude(PROX_LEN + CAP_T)
    cap = box(-lug_out, lug_out, -PROX_OD / 2, PROX_OD / 2, cap_bot, cap_top)
    cap = cap.edges("|Z").fillet(8)
    flange = box(-FLANGE / 2, FLANGE / 2, -FLANGE / 2, FLANGE / 2, tube_bot, tube_bot + FLANGE_T)
    flange = flange.edges("|Z").fillet(6)
    body = tube.union(cap).union(flange)
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        lug = box(x0, x1, -lug_r, lug_r, cap_top - 1, 0).union(cyl_x(lug_r, x0, x1))
        # gusset ramps so the lug isn't a cantilevered plate
        body = body.union(lug)

    cuts = [cq.Workplane("XY").workplane(offset=tube_bot - 1).circle(PROX_ID / 2).extrude(PROX_LEN + 1)]
    # tendon holes: flexor (x=-GX, y=+R), extensor (x=+GX, y=-R)
    for x, y in ((-GROOVE_X, R_CORD), (GROOVE_X, -R_CORD)):
        cuts.append(cq.Workplane("XY").workplane(offset=cap_bot - 1).center(x, y).circle(1.4).extrude(CAP_T + 2))
        # small funnel on top to keep the cord from abrading
        cuts.append(cq.Workplane("XY").workplane(offset=cap_top - 1.0).center(x, y)
                    .circle(1.4).workplane(offset=1.01).circle(2.4).loft())
    # central pass-through slot for distal tendons
    cuts.append(box(-PASS_HALF, PASS_HALF, -12, 12, cap_bot - 1, cap_top + 1).edges("|Z").fillet(3))
    # M8 axle bolt + captive nut slot (nut dropped in from the top)
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        cuts.append(cyl_x(M8_CLR / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - M8_NUT_T / 2, xm + M8_NUT_T / 2, -M8_NUT_AF / 2, M8_NUT_AF / 2,
                        -M8_NUT_AF / 2 / math.cos(math.pi / 6), lug_r + 1))
    # flange holes
    for x in (-FLANGE_HOLE, FLANGE_HOLE):
        for y in (-FLANGE_HOLE, FLANGE_HOLE):
            cuts.append(cq.Workplane("XY").workplane(offset=tube_bot - 1).center(x, y).circle(M3_CLR / 2).extrude(FLANGE_T + 2))
    for c in cuts:
        body = body.cut(c)
    return body, tube_bot


# ---------------------------------------------------------------- spool
def spool():
    """Servo spool, same radius as the drum. Built along X, horn face at x = -SPOOL_HALF."""
    half = HUB_HALF
    body = cyl_x(R_HUB, -half, half)
    cuts = [cyl_x(3.6, -half - 1, half + 1)]          # screwdriver access to the horn screw
    cuts.append(cyl_x(R_CAV, -CAV_HALF + 3, CAV_HALF))  # lighten
    for x, rot in ((-GROOVE_X, 0), (GROOVE_X, 180)):
        cuts.append(groove_ring(x, R_GROOVE, R_HUB, GROOVE_W))
    cuts.append(drum_anchor(-GROOVE_X))
    cuts.append(drum_anchor(GROOVE_X).rotate((0, 0, 0), (0, 0, 1), 180))
    # 4 horn screw holes (M2 self-tappers that came with the servo)
    for a in range(0, 360, 90):
        y, z = polar_yz(HORN_HOLE_R, a + 45)
        cuts.append(hole_along((-half - 1, y, z), (-CAV_HALF + 3, y, z), 2.2))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- base
def base():
    """Stand / 'torso' stand-in. Frame: top surface of the base at z=0, spool axis at (0,0,SPOOL_Z).

    Print UPSIDE DOWN (top plate on the bed).
    """
    W = 7.0                          # wall thickness
    wall_x = -(HUB_HALF + SERVO_TAB_TO_HORN)   # +x face of the servo wall (tabs rest here)
    xin, xout = wall_x, wall_x - W
    ylen = 50.0
    top = box(xout, -xout, -ylen, ylen, -BASE_TOP_T, 0)
    wall_a = box(xout, xin, -ylen, ylen, -BASE_H, 0)
    wall_b = box(-xin, -xout, -ylen, ylen, -BASE_H, 0)
    # feet strips along the bottom edge for screws into a board
    body = top.union(wall_a).union(wall_b)

    cuts = []
    # opening below the proximal tube for the tendons (+ the spool)
    cuts.append(box(-HUB_HALF - 3, HUB_HALF + 3, -R_HUB - 6, R_HUB + 6, -BASE_TOP_T - 1, 1).edges("|Z").fillet(4))
    # proximal flange holes
    for x in (-FLANGE_HOLE, FLANGE_HOLE):
        for y in (-FLANGE_HOLE, FLANGE_HOLE):
            cuts.append(cq.Workplane("XY").workplane(offset=-BASE_TOP_T - 1).center(x, y).circle(M3_CLR / 2).extrude(BASE_TOP_T + 2))
            cuts.append(cq.Workplane("XY").workplane(offset=-BASE_TOP_T - 1).center(x, y)
                        .polygon(6, M3_NUT_AF / math.cos(math.pi / 6)).extrude(3.5))
    # servo cut-out in the wall: body L along Y, shaft offset so the spool is at y=0
    body_cy = -SERVO_SHAFT_OFF
    sz = SPOOL_Z
    cuts.append(box(xout - 1, xin + 1, body_cy - SERVO_L / 2 - 0.5, body_cy + SERVO_L / 2 + 0.5,
                    sz - SERVO_W / 2 - 0.5 - SLOT, sz + SERVO_W / 2 + 0.5))
    # slotted mounting holes -> slide the servo DOWN to tension both tendons
    for dy in (-SERVO_HOLE_L / 2, SERVO_HOLE_L / 2):
        for dz in (-SERVO_HOLE_W / 2, SERVO_HOLE_W / 2):
            y = body_cy + dy
            slot = (cq.Workplane("YZ").workplane(offset=xout - 1)
                    .center(y, sz + dz - SLOT / 2).slot2D(SLOT + 3.4, 3.4, angle=90).extrude(W + 2))
            cuts.append(slot)
    # floor screw holes in the wall bottoms
    for x in ((xin + xout) / 2, -(xin + xout) / 2):
        for y in (-ylen + 10, ylen - 10):
            cuts.append(cq.Workplane("XY").workplane(offset=-BASE_H - 1).center(x, y).circle(1.4).extrude(15))
    # lighten the far wall
    cuts.append(box(-xin - 1, -xout + 1, -ylen + 14, ylen - 14, -BASE_H + 14, -BASE_TOP_T - 10))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- main
def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


def assembly(angle=35.0):
    """Assembled model for previews. Base top sits under the proximal flange."""
    d = distal_half()
    pair = cq.Compound.makeCompound([d.val(), d.rotate((0, 0, 0), (0, 0, 1), 180).val()])
    dist = cq.Workplane().add(pair).rotate((0, 0, 0), (1, 0, 0), angle)
    prox, tube_bot = proximal()
    sp = spool().translate((0, 0, tube_bot + SPOOL_Z))
    bs = base().translate((0, 0, tube_bot))
    return {"distal": dist, "proximal": prox, "spool": sp, "base": bs}


if __name__ == "__main__":
    dh = distal_half()
    # print orientation: split face (x=0) down on the bed
    export(dh.rotate((0, 0, 0), (0, 1, 0), 90), "distal_half")
    p, _ = proximal()
    export(p, "proximal")
    export(spool().rotate((0, 0, 0), (0, 1, 0), 90), "spool")
    export(base().rotate((0, 0, 0), (1, 0, 0), 180), "base")
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]),
                        str(OUT / "assembly_preview.step"))
    print("done ->", OUT)
