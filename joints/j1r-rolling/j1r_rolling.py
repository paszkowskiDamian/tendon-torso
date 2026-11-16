"""
J1-R: cable-constrained ROLLING elbow (LIMS style), hand-cranked, 3:1
======================================================================

Parametric CadQuery model. Run:

    .venv/bin/python joints/j1r-rolling/j1r_rolling.py

and STL/STEP files land in joints/j1r-rolling/stl/.

Frame
-----
* Elbow axis of the upper arm (centre C1 of pulley 1) = X through the origin,
  Z up, the forearm points +Y at 0 deg, the crank spool sits behind (-Y).
* The base plate top is at z = -HC1. The crank frame of J1-H is reused as is.
* Joint angle theta: the forearm's rotation relative to the upper arm.
  Positive = flexion (forearm tip rises). The centre link turns theta/2.

The joint (what LIMS does)
--------------------------
* Two equal pulleys, cord radius R. Pulley 1 (two thin discs, the "tines")
  is part of the upper arm, pulley 2 is the hub of the forearm. Two SIDE
  PLATES on 608 bearings hold the centres 2R + GAP apart. The pulleys do
  not touch: the cords are the contact.
* Two RESTRAINT cords X1, X2 cross at the contact point in a figure 8, each
  anchored on both pulleys. They force pure rolling without slip and carry
  the joint load like ligaments. The joint angle is twice the rotation of
  each pulley against the side plates.
* DRIVE: cord A (flexor) and cord B (extensor) come from the crank spool
  (cord radius R/3), each turns 90 deg up over a small roller under the
  elbow, climbs tangentially onto a free IDLER (cord radius R, coaxial with
  pulley 1), runs along the common tangent to pulley 2 and is anchored
  there. Because the idler turns freely and the tangent stays parallel to
  the centre line, each cord changes length by exactly R * theta: the
  moment arm is R over the whole range and the crank turns 3 x theta.
* Tension of the drive loop: the crank frame slides on its slots (J1-H).
  Restraint cords: knots, tied tight (LIMS uses set screws).

Parts
-----
  base.stl        x1   base plate + upper-arm tines/web + tension block
  forearm.stl     x1   pulley 2 + lever (print hub face down)
  idler.stl       x1   free drive idler (608 inside)
  side_plate.stl  x2   centre link, 2x 608 each
  roller.stl      x2   90-deg guide rollers (623 inside)
  spacer_*.stl         spacer tubes for the idler and roller axles
  spool.stl       x1   crank spool, helices at the drive stations
  frame, crank_arm, nut_cup, knob, lock_pin: from J1-H
"""
from pathlib import Path
import math
import sys

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "common"))
sys.path.insert(0, str(HERE.parents[0] / "j1h-hand"))
from cadlib import box, cyl_x, hex_prism_x, hex_prism_z, hole_along, groove_ring, polar_yz  # noqa: E402
import j1h_hand as H  # noqa: E402  (crank frame family, spool helix, cord constants)

OUT = HERE / "stl"

# ---------------------------------------------------------------- parameters
CORD_D = H.CORD_D             # 2.5 mm braid
R = 30.0                      # cord centre radius of pulleys and idler (= moment arm)
R_SPOOL = H.R_SPOOL           # 10 -> RATIO 3
RATIO = R / R_SPOOL
GAP = 1.0                     # gap between the pulley bodies (the cords are the contact)
LC = 2 * R + GAP              # centre distance C1 -> C2 (side plate length)
R_GROOVE = R - CORD_D / 2     # groove bottom
R_BODY = R + 0.2              # pulley body: the cord stands proud of it by ~1 mm
GW = CORD_D + 0.9             # groove width
ST = {"X1": -14.0, "A": -7.0, "B": 7.0, "X2": 14.0}   # cord stations along X

# ranges and wraps (deg)
TH_MIN, TH_MAX = -20.0, 140.0  # extension: cord B's vertical run would graze pulley 2's groove bottom past -20
WRAP_P2 = 110.0               # drive cords on pulley 2 at 0 deg; A unwinds by theta/2 in flexion, B in extension
ANCHOR_P1 = {"X1": 110.0, "X2": 250.0}          # restraint anchors on pulley 1 (upper-arm frame)
ANCHOR_P2 = {"X1": 280.0, "X2": 80.0, "A": 90.0 - WRAP_P2, "B": 270.0 + WRAP_P2 - 360.0}   # on pulley 2 (forearm frame)
CHORD = 65.0                  # chord anchor: hole re-emerges this far round (knot pocket)

# layout
HC1 = 110.0                   # elbow axis above the base plate top
Z_BASE = -HC1
D = H.D                       # 138: spool axis behind C1 (crank frame of J1-H)
SPOOL_C = (-D, H.HD - HC1)    # spool axis (y, z): 72 above the base
ROLLER_R = 9.0                # cord radius of the guide rollers
ROLLER_Z = -55.0              # roller axes
ROLLER_C = {"A": (-(R + ROLLER_R), ROLLER_Z), "B": (R - ROLLER_R, ROLLER_Z)}   # (y, z)
ROLLER_W = GW + 3.2           # roller width (1.6 mm flanges)
B623 = (3.0, 10.0, 4.0)
B608 = (8.0, 22.0, 7.0)
FIT = 0.15

# upper arm: tines (pulley 1 discs) + web, part of the base plate
IDLER_HALF = abs(ST["A"]) + GW / 2 + 1.5          # 10.2: idler half width
TINE_IN = IDLER_HALF + 0.5                         # 10.7 inner face of the discs
TINE_OUT = abs(ST["X1"]) + GW / 2 + 1.6            # 17.3 outer face (= pulley 2 half width)
STEM_IN = 9.5                                      # tine stem is thicker below the disc
WEB_TOP = Z_BASE + 40.0
WEB_Y = (-50.0, 21.5)         # front edge under the 45-deg point of the discs (printable)
SIDE_IN = TINE_OUT + 1.6                           # washer between tine / hub and side plate bearing
SIDE_T = 7.0
SIDE_W = 34.0

# forearm
FORE_L = 110.0                # bar length from C2
FORE_HOLES = (50.0, 100.0)
FORE_Z = 9.0                  # bar half height

# base plate
BASE_T = H.BASE_T
BASE_X = H.BASE_X
BASE_Y = (H.BASE_Y[0], 25.0)
PUSH_Z = Z_BASE + 5.0

# spool (same envelope as J1-H, helices at the drive stations)
PITCH, TURNS = H.PITCH, H.TURNS
HELIX = {"A": (ST["A"] - 1.25 * PITCH, ST["A"] - 1.25 * PITCH + TURNS * PITCH),
         "B": (ST["B"] + 1.25 * PITCH - TURNS * PITCH, ST["B"] + 1.25 * PITCH)}
HELIX_END = {"A": HELIX["A"][0], "B": HELIX["B"][1]}      # anchor ends (outer)

M3_CLR, M3_HEAD_D, M3_NUT_AF = H.M3_CLR, H.M3_HEAD_D, H.M3_NUT_AF
M4_CLR, M4_NUT_AF, M4_NUT_T = H.M4_CLR, H.M4_NUT_AF, H.M4_NUT_T
M8_CLR = H.M8_CLR


# ---------------------------------------------------------------- 2-D cord geometry (y, z)
def tangent_line(c1, r1, s1, c2, r2, s2):
    """Tangent line of a cord travelling from circle 1 to circle 2.

    s = +1: the cord goes round that circle counter-clockwise (seen from +X),
    s = -1: clockwise. Same senses -> external tangent, opposite -> internal.
    Returns (P1, P2, phi1, phi2): tangent points and their normal angles (deg).
    """
    dy, dz = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dy, dz)
    u = math.atan2(dz, dy)
    k = (r1 - s1 * s2 * r2) / d
    for sgn in (1, -1):
        p1 = u + sgn * math.acos(max(-1.0, min(1.0, k)))
        n1 = (math.cos(p1), math.sin(p1))
        n2 = (s1 * s2 * n1[0], s1 * s2 * n1[1])
        P1 = (c1[0] + r1 * n1[0], c1[1] + r1 * n1[1])
        P2 = (c2[0] + r2 * n2[0], c2[1] + r2 * n2[1])
        t = (-s1 * n1[1], s1 * n1[0])                      # travel direction at P1
        if (P2[0] - P1[0]) * t[0] + (P2[1] - P1[1]) * t[1] > 0:
            return P1, P2, math.degrees(p1) % 360, math.degrees(math.atan2(n2[1], n2[0])) % 360
    raise ValueError("no tangent")


def c2_centre(theta):
    a = math.radians(theta / 2)
    return (LC * math.cos(a), LC * math.sin(a))


def spool_departure():
    """Spool tangent (normal) angles of cords A (top, CW) and B (bottom, CCW)."""
    out = {}
    for cord, s_spool, s_roll in (("A", -1, 1), ("B", 1, 1)):
        _, _, phi_s, _ = tangent_line(SPOOL_C, R_SPOOL, s_spool, ROLLER_C[cord], ROLLER_R, s_roll)
        out[cord] = phi_s
    return out


def spool_wrap_angle(cord, x):
    """World angle (deg) of the spool groove at axial position x, joint at 0 deg
    (fleet point at the cord station). Both helices are left-handed."""
    t = spool_departure()[cord]
    return t - 360.0 / PITCH * (x - ST[cord])


def routing(theta=0.0, n_arc=16):
    """All cord centre-line pieces at joint angle theta, as polylines of (x, y, z)
    points, keyed by cord. Used by the checks and the renders."""
    th = math.radians(theta)
    c2 = c2_centre(theta)

    def arc(c, r, a0, a1, x):
        pts = []
        for k in range(n_arc + 1):
            a = math.radians(a0 + (a1 - a0) * k / n_arc)
            pts.append((x, c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
        return pts

    paths = {}
    for cord, s_spool in (("A", -1), ("B", 1)):
        x = ST[cord]
        rc = ROLLER_C[cord]
        P1, P2, phi_s, phi_r = tangent_line(SPOOL_C, R_SPOOL, s_spool, rc, ROLLER_R, 1)
        pts = [(x, *P1), (x, *P2)]
        # roller: CCW from the arrival to the front point (0 deg), then straight up
        pts += arc(rc, ROLLER_R, phi_r - 360 if phi_r > 180 else phi_r, 0.0, x)[1:]
        if cord == "A":
            # up the back of the idler (tangent at 180 deg), CW over the top to 90 + th/2
            pts.append((x, -R, 0.0))
            pts += arc((0.0, 0.0), R, 180.0, 90.0 + theta / 2, x)[1:]
            # top common tangent to pulley 2, then CW on pulley 2 to the anchor
            a_top = 90.0 + theta / 2
            pts.append((x, c2[0] + R * math.cos(math.radians(a_top)), c2[1] + R * math.sin(math.radians(a_top))))
            pts += arc(c2, R, a_top, a_top - (WRAP_P2 - theta / 2), x)[1:]      # anchor at 90-WRAP_P2+theta
        else:
            # up the front of the idler (tangent at 0 deg), CCW round to 270 + th/2
            pts.append((x, R, 0.0))
            pts += arc((0.0, 0.0), R, 0.0, 270.0 + theta / 2, x)[1:]
            a_bot = 270.0 + theta / 2
            pts.append((x, c2[0] + R * math.cos(math.radians(a_bot)), c2[1] + R * math.sin(math.radians(a_bot))))
            pts += arc(c2, R, a_bot, a_bot + (WRAP_P2 + theta / 2), x)[1:]      # anchor at 270+WRAP_P2+theta
        paths[cord] = pts
    # restraint cords: anchor on pulley 1 -> contact point -> pulley 2 -> anchor
    contact = theta / 2                              # world angle of the contact point on pulley 1
    for cord, sgn in (("X1", -1), ("X2", 1)):
        x = ST[cord]
        a1 = ANCHOR_P1[cord]
        c1 = contact if cord == "X1" else contact + 360.0              # X1 runs CW to the contact, X2 CCW
        pts = arc((0.0, 0.0), R, a1, c1, x)
        a2 = 180.0 + c1                                                 # same point seen from C2 (world)
        anchor2 = ANCHOR_P2[cord] + theta                               # pulley-2 anchor in world angles
        pts += arc(c2, R, a2, anchor2 if abs(anchor2 - a2) < 180 else anchor2 - 360 * round((anchor2 - a2) / 360), x)[1:]
        paths[cord] = pts
    return paths


# ---------------------------------------------------------------- helpers
def cut_all(body, cuts):
    for c in cuts:
        body = body.cut(c)
    return body


def chord_anchor(x, phi, direction, r_groove=R_GROOVE, r_out=R_BODY):
    """Chord hole from the groove bottom at phi, re-emerging CHORD deg further round
    in `direction` (+1 CCW / -1 CW), with a knot pocket at the exit."""
    y0, z0 = polar_yz(r_groove + 0.3, phi)
    y1, z1 = polar_yz(r_out + 1.5, phi + direction * CHORD)
    h = hole_along((x, y0, z0), (x, y1, z1), CORD_D + 1.0)
    d = cq.Vector(y1 - y0, z1 - z0, 0).normalized()
    k = hole_along((x, y1 - d.x * 6.0, z1 - d.y * 6.0), (x, y1 + d.x * 2, z1 + d.y * 2), 7.0)
    return h.union(k)


def pulley_grooves(stations):
    return [groove_ring(x, R_GROOVE, R_BODY, GW) for x in stations]


# ---------------------------------------------------------------- base + upper arm
def base():
    """Base plate with the upper-arm tines (pulley 1 discs), their web and the
    tension block of the crank frame. Prints flat, tines up."""
    plate = box(-BASE_X, BASE_X, BASE_Y[0], BASE_Y[1], Z_BASE - BASE_T, Z_BASE).edges("|Z").fillet(8)
    body = plate.union(box(-TINE_OUT, TINE_OUT, WEB_Y[0], WEB_Y[1], Z_BASE - 0.1, WEB_TOP))
    for s in (1, -1):
        xi, xo = sorted((s * TINE_IN, s * TINE_OUT))
        si, so = sorted((s * STEM_IN, s * TINE_OUT))
        disc = cyl_x(R_BODY, xi, xo)
        stem = box(si, so, WEB_Y[0], WEB_Y[1], WEB_TOP - 0.1, -R_BODY + 8.0)
        gusset = (cq.Workplane("XZ")
                  .polyline([(s * TINE_OUT, Z_BASE + 30.0), (s * TINE_OUT, Z_BASE - 0.1), (s * (BASE_X - 6.0), Z_BASE - 0.1)])
                  .close().extrude(4.0, both=True))
        body = body.union(disc).union(stem).union(gusset)
    # tension block (J1-H)
    body = body.union(box(-12.0, 12.0, H.PUSH_Y[0], H.PUSH_Y[1], Z_BASE - 0.1, Z_BASE + 16.0))

    cuts = [H.teardrop_x(M8_CLR / 2, -TINE_OUT - 1, TINE_OUT + 1)]                 # elbow axle
    cuts += pulley_grooves([ST["X1"], ST["X2"]])                                  # restraint grooves
    cuts.append(chord_anchor(ST["X1"], ANCHOR_P1["X1"], +1))                      # X1 arrives CCW
    cuts.append(chord_anchor(ST["X2"], ANCHOR_P1["X2"], -1))                      # X2 arrives CW
    for cord in ("A", "B"):                                                        # roller axles, M3
        y, z = ROLLER_C[cord]
        cuts.append(H.teardrop_x(M3_CLR / 2, -TINE_OUT - 1, TINE_OUT + 1, y, z))
        cuts.append(hex_prism_x(-TINE_OUT - 1, -TINE_OUT + 2.8, y, z, M3_NUT_AF))       # nut recessed in the -X tine
        cuts.append(hole_along((TINE_OUT - 3.2, y, z), (TINE_OUT + 1, y, z), M3_HEAD_D))  # head recessed in the +X tine
    # keep the inside of the tines clear above the web: cords and idler live there
    cuts.append(box(-STEM_IN, STEM_IN, WEB_Y[0] - 1, WEB_Y[1] + 1, WEB_TOP - 0.2, 1.0))
    # crank frame screws + nut traps, tension screw (J1-H layout)
    for x in (-H.SLOT_X, H.SLOT_X):
        for y in (-D - H.SLOT_Y, -D + H.SLOT_Y):
            cuts.append(cq.Workplane("XY").workplane(offset=Z_BASE - BASE_T - 1).center(x, y)
                        .circle(M4_CLR / 2).extrude(BASE_T + 2))
            cuts.append(hex_prism_z(Z_BASE - BASE_T - 1, Z_BASE - BASE_T + M4_NUT_T + 0.3, x, y, M4_NUT_AF))
    yc = (H.PUSH_Y[0] + H.PUSH_Y[1]) / 2
    cuts.append(hole_along((0, H.PUSH_Y[0] - 1, PUSH_Z), (0, H.PUSH_Y[1] + 1, PUSH_Z), M4_CLR))
    cuts.append(box(-M4_NUT_AF / 2, M4_NUT_AF / 2, yc - (M4_NUT_T + 0.3) / 2, yc + (M4_NUT_T + 0.3) / 2,
                    PUSH_Z - M4_NUT_AF / 2 / math.cos(math.pi / 6), Z_BASE + 17))
    return cut_all(body, cuts)


# ---------------------------------------------------------------- forearm (pulley 2 + lever)
def forearm():
    """Pulley 2 with the forearm bar, in its own frame (axis X at the origin, bar
    along +Y = joint angle 0). Prints hub face down."""
    hub = cyl_x(R_BODY, -TINE_OUT, TINE_OUT)
    bar = (box(-TINE_OUT, TINE_OUT, 0, FORE_L, -FORE_Z, FORE_Z)
           .union(cyl_x(FORE_Z, -TINE_OUT, TINE_OUT).translate((0, FORE_L, 0))))
    body = hub.union(bar)
    cuts = [cyl_x(M8_CLR / 2, -TINE_OUT - 1, TINE_OUT + 1)]
    cuts += pulley_grooves(list(ST.values()))
    cuts.append(chord_anchor(ST["A"], ANCHOR_P2["A"], -1))      # A arrives CW
    cuts.append(chord_anchor(ST["B"], ANCHOR_P2["B"], +1))      # B arrives CCW
    cuts.append(chord_anchor(ST["X1"], ANCHOR_P2["X1"], +1))    # X1 arrives CCW
    cuts.append(chord_anchor(ST["X2"], ANCHOR_P2["X2"], -1))    # X2 arrives CW
    for y in FORE_HOLES:                                         # weight holes through the bar
        cuts.append(hole_along((-TINE_OUT - 1, y, 0), (TINE_OUT + 1, y, 0), M4_CLR))
    for k in range(5):                                           # lightening, clear of the chords (r >= 25)
        a = 130.0 + 25.0 * k                                     # back half of the hub only
        y, z = polar_yz(18.0, a)
        cuts.append(cyl_x(4.0, -TINE_OUT - 1, TINE_OUT + 1).translate((0, y, z)))
    return cut_all(body, cuts)


def forearm_at(theta):
    c2 = c2_centre(theta)
    return forearm().rotate((0, 0, 0), (1, 0, 0), theta).translate((0, c2[0], c2[1]))


# ---------------------------------------------------------------- idler, rollers, spacers, side plates
def idler():
    """Free drive idler: grooves at the A/B stations, one 608 in the middle."""
    body = cyl_x(R_BODY, -IDLER_HALF, IDLER_HALF)
    cuts = pulley_grooves([ST["A"], ST["B"]])
    cuts.append(cyl_x(B608[1] / 2 + FIT, -B608[2] / 2, B608[2] / 2))
    cuts.append(cyl_x(6.6, -IDLER_HALF - 1, IDLER_HALF + 1))        # clearance round the spacer tubes
    for k in range(6):                                              # lightening
        y, z = polar_yz(19.0, 60 * k + 30)
        cuts.append(cyl_x(4.5, -IDLER_HALF - 1, IDLER_HALF + 1).translate((0, y, z)))
    return cut_all(body, cuts)


def roller():
    """Guide roller, cord radius ROLLER_R, 623 bearing from one face."""
    w = ROLLER_W
    body = cyl_x(ROLLER_R + 1.75, -w / 2, w / 2)
    cuts = [groove_ring(0.0, ROLLER_R - CORD_D / 2, ROLLER_R + 1.75, GW),
            cyl_x(B623[1] / 2 + FIT, -w / 2 - 1, -w / 2 + B623[2]),
            cyl_x(B623[0] / 2 + 0.6, -w / 2 - 1, w / 2 + 1)]
    return cut_all(body, cuts)


def spacer(length, bore=M8_CLR / 2, od=6.0):
    return cq.Workplane("XY").circle(od).circle(bore).extrude(length)


IDLER_SPACER_L = TINE_IN - B608[2] / 2            # 7.2: tine inner face -> 608 inner race
ROLLER_SPACERS = {cord: (ST[cord] - ROLLER_W / 2 + TINE_IN, TINE_IN - (ST[cord] + ROLLER_W / 2)) for cord in ("A", "B")}


def side_plate():
    """Centre link: C1 at the origin, C2 at +Y (LC), 608 at each end, built as a flat
    plate in the XY plane (thickness along Z) for printing; in the assembly it is
    turned so its thickness runs along X."""
    r = SIDE_W / 2
    body = (cq.Workplane("XY").slot2D(LC + SIDE_W, SIDE_W, angle=90).extrude(SIDE_T)
            .translate((0, LC / 2, 0)))
    cuts = []
    for yc in (0.0, LC):
        cuts.append(cq.Workplane("XY").workplane(offset=-1).center(0, yc).circle(B608[1] / 2 + FIT).extrude(SIDE_T + 2))
    cuts.append(cq.Workplane("XY").workplane(offset=-1).center(0, LC / 2).slot2D(LC - 2 * r - 6, 10.0, angle=90).extrude(SIDE_T + 2))
    return cut_all(body, cuts)


def side_plate_at(theta, side):
    """Side plate in the joint frame on side +1 (+X) or -1 (-X)."""
    x0 = SIDE_IN if side > 0 else -SIDE_IN - SIDE_T
    p = side_plate().rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, 0, 0))   # thickness now along +X from x0
    return p.rotate((0, 0, 0), (1, 0, 0), theta / 2)


# ---------------------------------------------------------------- spool (J1-H envelope, new helices)
def spool():
    f0, f1 = H.SP_FLANGE
    e = H.GROOVE_R + 0.1
    body = (cyl_x(H.CORE_R, -f0 - 0.1, f0 + 0.1)
            .union(cyl_x(H.SP_FLANGE_R, -f1, -f0)).union(cyl_x(H.SP_FLANGE_R, f0, f1)))
    for cord in ("A", "B"):
        x0, x1 = HELIX[cord]
        body = body.cut(cyl_x(H.CORE_R + 1.0, x0 - e, x1 + e).cut(cyl_x(H.CORE_R - 2.5, x0 - e, x1 + e)))
        body = body.union(H._grooved_ring(x0, x1, spool_wrap_angle(cord, x0)))
    cuts = [cyl_x(M8_CLR / 2, -f1 - 1, f1 + 1)]
    for cord, sgn in (("B", 1), ("A", -1)):
        xe = HELIX_END[cord]
        a = spool_wrap_angle(cord, xe)
        y, z = polar_yz(H.HELIX_C, a)
        cuts.append(hole_along((xe - sgn * 1.0, y, z), (sgn * (f1 + 1), y, z), CORD_D + 1.0))
        notch = box(min(sgn * (f1 - H.KNOT_NOTCH), sgn * (f1 + 1)), max(sgn * (f1 - H.KNOT_NOTCH), sgn * (f1 + 1)),
                    6.5, H.SP_FLANGE_R + 2, -4.0, 4.0).rotate((0, 0, 0), (1, 0, 0), a)
        cuts.append(notch)
    return cut_all(body, cuts)


# ---------------------------------------------------------------- assembly
def assembly(theta=30.0):
    cy, cz = SPOOL_C
    ca = RATIO * theta
    at_spool = lambda w: w.rotate((0, 0, 0), (1, 0, 0), ca).translate((0, cy, cz))   # noqa: E731
    kn = H.knob().rotate((0, 0, 0), (0, 1, 0), 90).translate((H.ARM_X[1] + 0.5, 0, H.CRANK_R))
    pin = H.lock_pin().rotate((0, 0, 0), (0, 1, 0), -90).translate((H.WALL_OUT - H.INDEX_DEPTH + 0.5, 0, H.INDEX_R))
    parts = {
        "base": base(),
        "forearm": forearm_at(theta),
        "idler": idler().rotate((0, 0, 0), (1, 0, 0), -theta * 1.0),
        "side_plate_a": side_plate_at(theta, -1),
        "side_plate_b": side_plate_at(theta, 1),
        "frame": H.frame().translate((0, cy, cz)),
        "spool": at_spool(spool()),
        "crank_arm": at_spool(H.crank_arm()),
        "nut_cup": H.nut_cup().translate((0, cy, cz)),
        "knob": at_spool(kn),
        "lock_pin": at_spool(pin),
    }
    for cord in ("A", "B"):
        y, z = ROLLER_C[cord]
        parts["roller_" + cord] = roller().translate((ST[cord], y, z))
    return parts


def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


if __name__ == "__main__":
    export(base().translate((0, 0, HC1 + BASE_T)), "base")
    export(forearm().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, TINE_OUT)), "forearm")   # hub face down
    export(idler().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, IDLER_HALF)), "idler")
    export(roller().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, ROLLER_W / 2)), "roller")  # bearing pocket up
    export(side_plate(), "side_plate")
    export(spacer(IDLER_SPACER_L), "spacer_idler")
    for cord in ("A", "B"):
        for i, L in enumerate(ROLLER_SPACERS[cord]):
            if L > 2.0:
                export(spacer(L, bore=M3_CLR / 2, od=3.5), f"spacer_roller_{cord}{i + 1}")
    export(spool().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, H.SP_FLANGE[1])), "spool")
    cy, cz = SPOOL_C
    export(H.frame().translate((0, 0, H.HD)), "frame")
    export(H.crank_arm().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, H.ARM_X[1])), "crank_arm")
    export(H.nut_cup().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, H.WALL_OUT + 0.5 + H.NUT_CUP_T)), "nut_cup")
    export(H.knob(), "knob")
    export(H.lock_pin(), "lock_pin")
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]), str(OUT / "assembly_preview.step"))
    print(f"ratio {RATIO:.2f}, centre distance {LC}, spool departure angles {spool_departure()}")
    print("done ->", OUT)
