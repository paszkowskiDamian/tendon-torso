"""
J1-T: hand-held rolling elbow with a block-and-tackle drive, all-printed fasteners
==================================================================================

Parametric CadQuery model. Run:

    .venv/bin/python joints/j1t-tackle/j1t_tackle.py

and STL/STEP files land in joints/j1t-tackle/stl/.

Frame
-----
* Pin of the upper arm (centre C1 of rolling pulley 1) = X axis through the
  origin, Z up, the forearm points +Y at 0 deg, the handle points -Y.
* Joint angle theta = rotation of the forearm relative to the upper arm,
  positive = flexion (forearm tip rises). The centre links turn theta/2.

What it is
----------
Each link is a SANDWICH of two flat plates on printed bolts, as in the Rhino
sketch: between the plates sit two rolling DISCS (radius R, restraint and
pass-through grooves), and above and below the pin a stack of small SHEAVES
on a printed axle bolt (the "ribbed cylinders" of the sketch). The two links
roll on each other; two printed CENTRE LINKS on 608 bearings keep the pins
2R + GAP apart and two crossed restraint cords force pure rolling (J1-R).

Reduction = block and tackle (LIMS). One cord: anchored on the upper arm's
top block F1, reeved N_PASS = 4 times between F1 and the forearm's top block
F2, down to the knob SPOOL (RH helix, ~3 turns, capstan), then N_PASS times
between the bottom blocks E1 / E2 and anchored on E1. Blocks at height H_BLK
above / below the pins. With link 2 rotated theta the block distances are
    d_F = LC - 2 H sin(theta/2),   d_E = LC + 2 H sin(theta/2),
so d_F + d_E is constant: one cord, one spool, no slack in either direction.
Torque on the elbow = T * N_PASS * H_BLK * cos(theta/2), torque on the knob
= T * R_SPOOL:  ratio = N_PASS * H_BLK / R_SPOOL * cos(theta/2) = 5.3 cos(theta/2).

The KNOB sits on the spool axle outside the +X plate of the handle, a pistol
GRIP hangs under the handle: knob in the hand, no table stand. All screws are
printed (8 mm, 2 mm pitch, hex 13 AF); the only bought parts are 6 x 608.

Pass-through: grooves T1 / T2 on the discs. A cord that wraps pulley 1 on
one side, crosses at the contact point and wraps pulley 2 on the OTHER side
(a figure 8, like the restraint cords) keeps a constant length for any
joint angle; an uncrossed cord changes by R * theta.

Parts (all printed except the bearings)
--------------------------------------
  plate_handle x2, plate_forearm x2, disc x4, sheave x8, anchor_ring x2,
  centre_link x2, spool x1, knob x1, grip x1,
  bolt x10, bolt_spool x1, nut x11, washer x16,
  spacer_block x8 (disc -> plate), spacer_sheave x4, spacer_bar x2
"""
from pathlib import Path
import math
import sys

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "common"))
from cadlib import box, cyl_x, hex_prism_x, hex_prism_z, hole_along, groove_ring, polar_yz  # noqa: E402

OUT = HERE / "stl"

# ---------------------------------------------------------------- parameters
CORD_D = 2.5                  # braided cord
R = 30.0                      # rolling radius = cord centre radius of the discs
GAP = 1.0
LC = 2 * R + GAP              # pin distance C1 -> C2
R_GROOVE = R - CORD_D / 2
R_BODY = R + 0.2
R_PLATE = R                   # plates roll edge to edge with 1 mm gap
GW = CORD_D + 0.9
ST = {"X1": -18.5, "T1": -24.0, "X2": 18.5, "T2": 24.0}   # restraint X, pass-through T (|x| stations)
TINE_IN = 15.7                # disc inner face (sheave stacks live inside)
TINE_OUT = 24.0 + GW / 2 + 1.6    # 27.3 disc outer face
ANCHOR_P1 = {"X1": 230.0, "X2": 130.0}    # X1 arrives CCW (over the top), X2 CW (under the bottom); exits 285 / 75
CHORD = 55.0
TH_MIN, TH_MAX = -20.0, 130.0  # beyond ~135 the tackle runs graze the fillers of the opposite block axle

# block and tackle: 3 passes per side, flexor F entirely at x > 0, extensor E at x < 0
N_PASS = 3
H_BLK = 20.0                  # block axles at (y=0, z=+-H_BLK) from each pin
R_SH = 8.0                    # sheave cord radius
R_SHG = R_SH - CORD_D / 2     # 6.75 groove bottom
R_SHF = 9.5                   # sheave flange
SH_W = 5.4                    # sheave width incl. 1 mm flanges
SH_PITCH = 6.2                # sheave pitch in a block (hub bosses of 0.4 each face act as spacers)
R_AF = 12.0                   # anchor ring flange radius (room for the axial knot hole)
SH_BOSS = (SH_PITCH - SH_W) / 2

# spool, guide, knob. The F cord leaves the last upper-arm sheave straight DOWN (y = -R_SH)
# onto the front of the spool; the E cord leaves the back of the spool straight UP over a
# free GUIDE sheave and comes straight down onto the last lower sheave. Both last sheaves
# then keep 90 -+ theta/2 deg of wrap.
R_SPOOL = 12.0
RATIO0 = (N_PASS * H_BLK + R_SH / 2) / R_SPOOL   # 5.33 at 0 deg
PITCH = CORD_D + 2.0                     # 4.5
HELIX_C = R_SPOOL + 0.15
GROOVE_R = CORD_D / 2 + 0.15
CORE_R = R_SPOOL + 0.5
SP_CORE_X = 21.0                         # helix from -21 to +21 (fleet points at +-12.4 migrate 7.5 mm)
SP_FLANGE_R, SP_FLANGE_T = R_SPOOL + 4.0, 2.0
SPOOL_C = (-(R_SH + R_SPOOL), -62.0)     # spool axis (y, z): its front tangent is the line y = -R_SH
R_G = R_SPOOL                            # guide sheave cord radius: its back lies on the spool's back line
R_GG, R_GF = R_G - CORD_D / 2, R_G + 2.0
GUIDE_W, GUIDE_GW = 8.0, 6.0                 # wide groove: the run from the spool fleets up to 6 deg
GUIDE_C = (-(R_SH + R_G), 12.0)          # guide axis (y, z): above the pin, out of the centre links' sweep
PLATE_BOLT = (-18.8, -13.2)              # second plate-to-disc bolt (the guide bolt is the first), behind the link sweep
KNOB_R, KNOB_T, KNOB_N = 32.0, 10.0, 12

# stack along X (from the joint plane outwards), both links identical
B608 = (8.0, 22.0, 7.0)
FIT = 0.15
WASHER_T, WASHER_OD = 1.6, 12.0
LINK_IN = TINE_OUT + WASHER_T            # 28.9 centre link inner face
LINK_T, LINK_W = 7.0, 30.0
LINK_OUT = LINK_IN + LINK_T              # 35.9
PL_IN = LINK_OUT + WASHER_T              # 37.5 plate inner face
PL_T = 5.0
PL_OUT = PL_IN + PL_T                    # 42.5
SPACER_BLOCK_L = PL_IN - TINE_OUT        # 10.2 (plate bolts: disc -> plate)
SPACER_MID_L = 2 * TINE_IN               # plate bolts between the discs
SPACER_OD = 12.0                         # plate-bolt spacers (1.8 mm wall)
FILLER_OD = 11.0                         # fillers on the block axles (the opposite tackle's runs pass 0.3 mm clear at 135 deg)

# printed thread "P8": 8 mm nominal, 2 mm pitch, 60 deg flanks (trapezoid)
P_PITCH = 2.0
EXT_MINOR, EXT_MAJOR = 2.65, 3.85        # radii of the printed bolt thread
INT_MINOR, INT_MAJOR = 3.25, 4.25        # radii of the printed nut thread (cutter)
SHANK_R = 3.95                           # smooth shank (7.9 mm)
BOLT_CLR = 8.4                           # clearance hole
HEAD_AF, HEAD_T = 13.0, 5.0
NUT_AF, NUT_T = 13.2, 6.5
BOLT_SHANK = 2 * (PL_OUT + WASHER_T) + NUT_T + 1.0    # 95.7: plate / grip / bar bolts (through everything)
POCKET_D = HEAD_T + 0.4                                  # hex pocket for the block bolt heads in the -X discs
BLOCK_SHANK = 2 * TINE_OUT - POCKET_D - 0.5              # block axles: head pocketed in the -X disc, thread in the +X disc
PIN_SHANK = PL_OUT + WASHER_T - TINE_OUT + 11.0          # pin stubs: through plate + centre-link bearing, 11 mm into the disc
DISC_THREAD_L = 12.0
THREAD_L = 16.0

# handle plate / grip / forearm bar
HANDLE_Y = (-84.0, -5.0)
HANDLE_Z = (-86.0, 30.0)
GRIP_Y = (-82.0, -42.0)
GRIP_BOLTS = ((-62.0, -44.0), (-62.0, -74.0))
GRIP_Z = (-180.0, -34.0)
GRIP_X = 16.0
FORE_L = 110.0
FORE_Z = 12.0
FORE_BOLTS = (50.0, 100.0)
SPACER_BAR_OD = 12.0


# ---------------------------------------------------------------- 2-D cord geometry (y, z)
def tangent_line(c1, r1, s1, c2, r2, s2):
    """Tangent of a cord travelling from circle 1 to circle 2; s = +1 CCW / -1 CW travel
    (seen from +X). Returns (P1, P2, phi1, phi2): tangent points, normal angles (deg)."""
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
        t = (-s1 * n1[1], s1 * n1[0])
        if (P2[0] - P1[0]) * t[0] + (P2[1] - P1[1]) * t[1] > 0:
            return P1, P2, math.degrees(p1) % 360, math.degrees(math.atan2(n2[1], n2[0])) % 360
    raise ValueError("no tangent")


def e(deg):
    a = math.radians(deg)
    return (math.cos(a), math.sin(a))


def c2_centre(theta):
    return (LC * math.cos(math.radians(theta / 2)), LC * math.sin(math.radians(theta / 2)))


def block_centres(theta):
    c2 = c2_centre(theta)
    up = e(90 + theta)
    return {"F1": (0.0, H_BLK), "E1": (0.0, -H_BLK),
            "F2": (c2[0] + H_BLK * up[0], c2[1] + H_BLK * up[1]),
            "E2": (c2[0] - H_BLK * up[0], c2[1] - H_BLK * up[1])}


def block_distance(theta, which):
    s = math.sin(math.radians(theta / 2))
    return LC - 2 * H_BLK * s if which == "F" else LC + 2 * H_BLK * s


def ratio(theta):
    """Elbow torque / knob torque: the tackle runs give N_PASS * H_BLK * cos(theta/2) of moment
    arm, the last sheave of each tackle adds R_SH / 2 (its wrap changes by theta/2)."""
    return (N_PASS * H_BLK * math.cos(math.radians(theta / 2)) + R_SH / 2) / R_SPOOL


def _free_end_angles():
    """Spool angles where cord F arrives (CW travel) and cord E departs (towards the guide)."""
    b = block_centres(0.0)
    _, _, _, a_f = tangent_line(b["F1"], R_SH, +1, SPOOL_C, R_SPOOL, -1)     # F1 s4 CCW -> spool CW (internal)
    _, _, a_e, _ = tangent_line(SPOOL_C, R_SPOOL, -1, GUIDE_C, R_G, -1)      # spool CW -> guide CW (external)
    return a_f, a_e


SPOOL_IN, SPOOL_OUT = _free_end_angles()


def _solve_stations():
    """The RH helix carries the cord from the F fleet point (+DX/2) to the E fleet point
    (-DX/2), turning by 360 DX / PITCH: that must equal the angle from the F entry to the
    E exit (mod 360). Take the DX that puts the outermost sheave just inside the discs."""
    d = (SPOOL_IN - SPOOL_OUT) % 360.0
    best = None
    for m in range(1, 8):
        dx = (d / 360.0 + m) * PITCH
        if dx / 2 + SH_PITCH / 2 <= TINE_IN - 0.2 and (best is None or dx > best):
            best = dx
    if best is None or best / 2 - 2 * SH_PITCH < -0.1:          # the anchor ring must stay on its own side of x = 0
        raise RuntimeError("no station spacing matches the helix phase; move the spool or the guide")
    return best


DX = _solve_stations()
X_F, X_E = DX / 2, -DX / 2               # free-end stations (last sheaves of F1 and E1, fleet points on the spool)
X_G = -(TINE_IN - 0.2 - GUIDE_W / 2)     # guide station (its +X face 0.2 inside the disc)
# tackle stations every half pitch, ending at X_F: F2 anchor, F1 s1, F2 s2, F1 s3 (E mirrored in x)
F1_ST = {"s1": X_F - SH_PITCH, "s3": X_F}
F2_ST = {"anchor": X_F - 1.5 * SH_PITCH, "s2": X_F - 0.5 * SH_PITCH}
E1_ST = {k: -v for k, v in F1_ST.items()}
E2_ST = {k: -v for k, v in F2_ST.items()}
assert F2_ST["anchor"] - SH_PITCH / 2 > -0.1, "the F tackle must stay at x >= 0"
# fillers on the block axles: (towards +X disc, towards -X disc)
SPACER_SHEAVE = {"F1": (TINE_IN - (F1_ST["s3"] + SH_PITCH / 2), F1_ST["s1"] - SH_PITCH / 2 + TINE_IN),
                 "F2": (TINE_IN - (F2_ST["s2"] + SH_PITCH / 2), F2_ST["anchor"] - SH_PITCH / 2 + TINE_IN)}
SPACER_SHEAVE["E1"] = SPACER_SHEAVE["F1"][::-1]
SPACER_SHEAVE["E2"] = SPACER_SHEAVE["F2"][::-1]
SPACER_GUIDE_L = TINE_IN - (X_G + GUIDE_W / 2)     # from the guide to the +X disc (the -X side has 0.2 play)


def spool_wrap_angle(x):
    """World angle (deg) of the spool groove at axial position x, joint at 0 deg (RH helix)."""
    return SPOOL_IN + 360.0 / PITCH * (x - X_F)


def spool_turns(theta):
    """Spool rotation (deg) from 0 deg joint angle; positive = flexion (cord F wound in)."""
    th = math.radians(theta)
    return math.degrees((N_PASS * 2 * H_BLK * math.sin(th / 2) + R_SH * th / 2) / R_SPOOL)


def routing(theta=0.0, n_arc=16):
    """Cord centre lines at joint angle theta as polylines of (x, y, z), keyed by cord:
    F (top tackle + free end to the spool), S (spool wraps), E (free end + bottom tackle),
    X1, X2 (restraint), T1 (a pass-through cord)."""
    th = math.radians(theta)
    c2 = c2_centre(theta)
    b = block_centres(theta)

    def arc(c, r, a0, a1, x, x1=None):
        pts = []
        for k in range(n_arc + 1):
            t = k / n_arc
            a = math.radians(a0 + (a1 - a0) * t)
            xx = x if x1 is None else x + (x1 - x) * t
            pts.append((xx, c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
        return pts

    def ang(p, c):
        return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))

    paths = {}
    # ---- F tackle: anchor at F2's top, runs top / bottom / top, all sheaves wrapped CCW
    B1, B2 = b["F1"], b["F2"]
    d = block_distance(theta, "F")
    u = ((B2[0] - B1[0]) / d, (B2[1] - B1[1]) / d)
    n = (-u[1], u[0])
    top = lambda B: (B[0] + R_SH * n[0], B[1] + R_SH * n[1])          # noqa: E731
    bot = lambda B: (B[0] - R_SH * n[0], B[1] - R_SH * n[1])          # noqa: E731
    pts = [(F2_ST["anchor"], *top(B2)), (F1_ST["s1"], *top(B1))]
    pts += arc(B1, R_SH, ang(top(B1), B1), ang(top(B1), B1) + 180, F1_ST["s1"])[1:]      # back of F1 s1 (top -> bottom)
    pts.append((F2_ST["s2"], *bot(B2)))
    pts += arc(B2, R_SH, ang(bot(B2), B2), ang(bot(B2), B2) + 180, F2_ST["s2"])[1:]      # front of F2 s2 (bottom -> top)
    pts.append((F1_ST["s3"], *top(B1)))
    P1, P2, phi1, phi2 = tangent_line(B1, R_SH, +1, SPOOL_C, R_SPOOL, -1)
    a0 = ang(top(B1), B1)
    a1 = phi1 if phi1 > a0 else phi1 + 360
    pts += arc(B1, R_SH, a0, a1, F1_ST["s3"])[1:]
    x_f = X_F - PITCH * spool_turns(theta) / 360.0       # fleet point migrates along the helix
    pts.append((x_f, *P2))
    paths["F"] = pts
    # ---- spool: CW from the F entry to the E exit, (X_F - X_E)/PITCH turns
    x_e = X_E - PITCH * spool_turns(theta) / 360.0
    turns = (X_F - X_E) / PITCH
    paths["S"] = arc(SPOOL_C, R_SPOOL, phi2, phi2 - 360 * turns, x_f, x_e)
    # ---- E free end: up the back of the spool, over the guide (CW), down onto the back of E1 s4
    B1, B2 = b["E1"], b["E2"]
    d = block_distance(theta, "E")
    u = ((B2[0] - B1[0]) / d, (B2[1] - B1[1]) / d)
    n = (-u[1], u[0])
    G1, G2, g1, g2 = tangent_line(SPOOL_C, R_SPOOL, -1, GUIDE_C, R_G, -1)        # spool -> guide, external
    Q1, Q2, q1, q2 = tangent_line(GUIDE_C, R_G, -1, B1, R_SH, +1)               # guide -> E1 s4, internal
    pts = [(x_e, *G1), (X_G, *G2)]
    a0, a1 = g2, q1
    while a1 > a0:
        a1 -= 360
    pts += arc(GUIDE_C, R_G, a0, a1, X_G)[1:]                                   # CW over the top of the guide
    pts.append((E1_ST["s3"], *Q2))
    a0 = q2
    a1 = ang(bot(B1), B1)
    while a1 < a0:
        a1 += 360
    pts += arc(B1, R_SH, a0, a1, E1_ST["s3"])[1:]                   # CCW down the back of E1 s3 to the bottom point
    pts.append((E2_ST["s2"], *bot(B2)))
    pts += arc(B2, R_SH, ang(bot(B2), B2), ang(bot(B2), B2) + 180, E2_ST["s2"])[1:]      # front of E2 s2 (bottom -> top)
    pts.append((E1_ST["s1"], *top(B1)))
    pts += arc(B1, R_SH, ang(top(B1), B1), ang(top(B1), B1) + 180, E1_ST["s1"])[1:]      # back of E1 s1 (top -> bottom)
    pts.append((E2_ST["anchor"], *bot(B2)))
    paths["E"] = pts
    # ---- restraint cords: anchor on pulley 1 -> contact -> pulley 2 -> anchor (figure 8)
    contact = theta / 2
    for cord, sgn in (("X1", -1), ("X2", 1)):
        x = ST[cord]
        a1 = ANCHOR_P1[cord]
        c1 = contact if cord == "X1" else contact + 360.0
        pts = arc((0.0, 0.0), R, a1, c1, x)
        a2 = 180.0 + c1
        anchor2 = (ANCHOR_P1[cord] + 180.0) + theta                 # same disc turned 180 deg, on link 2
        d_a = anchor2 - a2
        d_a = d_a - 360 * round(d_a / 360) if cord == "X2" else d_a  # X1 wraps CCW 200+ deg, X2 CW
        if cord == "X1":
            while d_a < 0:
                d_a += 360
            while d_a > 360:
                d_a -= 360
        else:
            while d_a > 0:
                d_a -= 360
            while d_a < -360:
                d_a += 360
        pts += arc(c2, R, a2, a2 + d_a, x)[1:]
        paths[cord] = pts
    # ---- pass-through T1: from the handle along the top of pulley 1 (enters at 180), over the top
    # to the contact, crosses, under pulley 2 and out along the bottom of the forearm (exits at 270 + theta)
    x = ST["T1"]
    pts = [(x, -R, 60.0)] + arc((0.0, 0.0), R, 180.0, contact, x)
    pts += arc(c2, R, 180.0 + contact, 270.0 + theta, x)[1:]
    ex = e(theta)
    pts.append((x, pts[-1][1] + 70 * ex[0], pts[-1][2] + 70 * ex[1]))
    paths["T1"] = pts
    return paths


# ---------------------------------------------------------------- printed threads
def _thread_ridge(r_minor, r_major, pitch, length):
    path = cq.Wire.makeHelix(pitch, length, r_minor)
    w = 0.25 * pitch / 2
    bb = 0.75 * pitch / 2
    prof = cq.Workplane("XZ").polyline([(r_minor - 0.3, -bb), (r_major, -w), (r_major, w), (r_minor - 0.3, bb)]).close()
    return prof.sweep(cq.Workplane().add(path), isFrenet=True)


def external_thread(length, chamfer=1.0):
    """Printed bolt thread along +Z from z = 0, chamfered at both ends."""
    core = cq.Workplane("XY").circle(EXT_MINOR).extrude(length)
    ridge = _thread_ridge(EXT_MINOR, EXT_MAJOR, P_PITCH, length + 2 * P_PITCH).translate((0, 0, -P_PITCH))
    ro = EXT_MAJOR + 0.5
    pts = [(0, 0), (EXT_MAJOR - chamfer, 0), (ro, chamfer + 0.5), (ro, length - chamfer - 0.5), (EXT_MAJOR - chamfer, length), (0, length)]
    env = cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
    return core.union(ridge).intersect(env)


def thread_cutter(length):
    """Oversize bolt shape (z from -pitch to length + pitch) that cuts a printed nut thread."""
    core = cq.Workplane("XY").workplane(offset=-P_PITCH).circle(INT_MINOR).extrude(length + 2 * P_PITCH)
    ridge = _thread_ridge(INT_MINOR, INT_MAJOR, P_PITCH, length + 2 * P_PITCH).translate((0, 0, -P_PITCH))
    return core.union(ridge)


def bolt(shank=BOLT_SHANK, thread=THREAD_L):
    """Printed hex bolt along +Z: head z in [0, HEAD_T], shank, thread at the end. Prints standing."""
    head = cq.Workplane("XY").polygon(6, HEAD_AF / math.cos(math.pi / 6)).extrude(HEAD_T).edges(">Z").chamfer(0.8)
    sh = cq.Workplane("XY").workplane(offset=HEAD_T).circle(SHANK_R).extrude(shank - thread + 0.01)
    th = external_thread(thread).translate((0, 0, HEAD_T + shank - thread))
    return head.union(sh).union(th)


def nut():
    body = cq.Workplane("XY").polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(NUT_T).edges(">Z or <Z").chamfer(0.8)
    return body.cut(thread_cutter(NUT_T))


def washer(t=WASHER_T):
    return cq.Workplane("XY").circle(WASHER_OD / 2).circle(BOLT_CLR / 2).extrude(t)


def spacer(length, od=SPACER_OD, bore=BOLT_CLR):
    return cq.Workplane("XY").circle(od / 2).circle(bore / 2).extrude(length)


# ---------------------------------------------------------------- helpers
def teardrop_x(r, x0, x1, phi=50.0, alpha=52.0):
    """Horizontal hole along X that prints without support (point up, +Z)."""
    c = cyl_x(r, x0, x1)
    sn, co = math.sin(math.radians(phi)), math.cos(math.radians(phi))
    h = r * co + r * sn * math.tan(math.radians(alpha))
    tri = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(-r * sn, r * co), (0, h), (r * sn, r * co), (0, 0)]).close().extrude(x1 - x0))
    return c.union(tri)


def cut_all(body, cuts):
    for c in cuts:
        body = body.cut(c)
    return body


def chord_anchor(x, phi, direction, r_groove=R_GROOVE, r_out=R_BODY, pocket=6.5):
    """Chord hole from the groove bottom at phi, re-emerging CHORD deg further round in
    `direction` (+1 CCW / -1 CW), with a knot pocket at the exit."""
    y0, z0 = polar_yz(r_groove + 0.3, phi)
    y1, z1 = polar_yz(r_out + 1.5, phi + direction * CHORD)
    h = hole_along((x, y0, z0), (x, y1, z1), CORD_D + 1.0)
    d = cq.Vector(y1 - y0, z1 - z0, 0).normalized()
    k = hole_along((x, y1 - d.x * 6.0, z1 - d.y * 6.0), (x, y1 + d.x * 2, z1 + d.y * 2), pocket)
    return h.union(k)


def disc(kind="thread"):
    """Rolling disc of the +X side of the UPPER ARM: x in [TINE_IN, TINE_OUT], grooves X2 and
    T2, X2's chord anchor (arrives CW at 130, exits at 75). The block axles (y = 0, z = +-H_BLK)
    and the pin stubs screw into printed threads ("thread", the +X discs); the -X discs ("pocket")
    carry the block bolt heads in hex pockets on their outer face and are this part turned 180 deg
    about Y. The forearm's discs are the same parts turned 180 deg about X. Prints flat, inner face down."""
    body = cyl_x(R_BODY, TINE_IN, TINE_OUT)
    cuts = [groove_ring(ST["X2"], R_GROOVE, R_BODY, GW), groove_ring(ST["T2"], R_GROOVE, R_BODY, GW),
            chord_anchor(ST["X2"], ANCHOR_P1["X2"], -1)]
    # pin stub thread (from the outer face) and the block axles
    cuts.append(thread_cutter(DISC_THREAD_L).rotate((0, 0, 0), (0, 1, 0), -90).translate((TINE_OUT + 0.5, 0, 0)))
    for z in (H_BLK, -H_BLK):
        if kind == "thread":
            cuts.append(thread_cutter(TINE_OUT - TINE_IN + 2).rotate((0, 0, 0), (0, 1, 0), 90).translate((TINE_IN - 1, 0, z)))
        else:
            cuts.append(cyl_x(BOLT_CLR / 2, TINE_IN - 1, TINE_OUT + 1).translate((0, 0, z)))
            cuts.append(hex_prism_x(TINE_OUT - POCKET_D, TINE_OUT + 1, 0, z, HEAD_AF + 0.4))
    # plate bolts (guide axle and PLATE_BOLT) in all four symmetric positions: one STL for every disc
    for sy in (1, -1):
        cuts.append(cyl_x(BOLT_CLR / 2, TINE_IN - 1, TINE_OUT + 1).translate((0, sy * GUIDE_C[0], GUIDE_C[1])))
        for sz in (1, -1):
            cuts.append(cyl_x(BOLT_CLR / 2, TINE_IN - 1, TINE_OUT + 1).translate((0, sy * PLATE_BOLT[0], sz * PLATE_BOLT[1])))
    return cut_all(body, cuts)


def disc_at(side, link):
    """side +-1 (x side), link 1 (upper arm) or 2 (forearm, in the forearm's own frame)."""
    d = disc("thread" if side > 0 else "pocket")
    if side < 0:
        d = d.rotate((0, 0, 0), (0, 1, 0), 180)
    if link == 2:
        d = d.rotate((0, 0, 0), (1, 0, 0), 180)
    return d


# ---------------------------------------------------------------- sheaves, anchor ring, blocks
def sheave():
    """Small tackle sheave, axis X, centred at x = 0, hub bosses on both faces act as spacers."""
    body = cyl_x(R_SHF, -SH_W / 2, SH_W / 2).union(cyl_x(6.0, -SH_PITCH / 2, SH_PITCH / 2))
    cuts = [groove_ring(0.0, R_SHG, R_SHF, GW), cyl_x(BOLT_CLR / 2, -SH_PITCH, SH_PITCH)]
    return cut_all(body, cuts)


def anchor_ring():
    """Cord anchor of a block: a flange on the -X side only (the cord fleets away towards +X),
    the cord lies on the r 6.75 drum and leaves through an axial hole in the flange, the stopper
    knot sits on the outer face. Free on the axle. The E2 ring is the same part turned about Y."""
    body = (cyl_x(R_AF, -SH_W / 2, -SH_W / 2 + 1.0).union(cyl_x(R_SHG, -SH_W / 2, SH_W / 2))
            .union(cyl_x(6.0, -SH_PITCH / 2, SH_PITCH / 2)))
    cuts = [cyl_x(BOLT_CLR / 2, -SH_PITCH, SH_PITCH),
            cyl_x((CORD_D + 0.7) / 2, -SH_PITCH, 0.0).translate((0, 0, R_SHG + 3.0))]
    return cut_all(body, cuts)


def guide():
    """Free guide sheave for the E cord, cord radius R_G, axis X, centred at x = 0."""
    body = cyl_x(R_GF, -GUIDE_W / 2, GUIDE_W / 2).union(cyl_x(8.0, -GUIDE_W / 2 - 0.4, GUIDE_W / 2 + 0.4))
    cuts = [groove_ring(0.0, R_GG, R_GF, GUIDE_GW), cyl_x(BOLT_CLR / 2, -GUIDE_W, GUIDE_W)]
    for k in range(4):
        y, z = polar_yz(10.0, 90 * k + 45)
        cuts.append(cyl_x(2.5, -SH_PITCH, SH_PITCH).translate((0, y, z)))
    return cut_all(body, cuts)


def block_stack(stations, anchor_station=None):
    """Sheaves (and the anchor ring) of one block at their x stations, axis X at the origin."""
    parts = {}
    for name, x in stations.items():
        p = anchor_ring() if name == "anchor" else sheave()
        if name == "anchor" and x < 0:
            p = p.rotate((0, 0, 0), (0, 1, 0), 180)
        parts[name] = p.translate((x, 0, 0))
    return parts


# ---------------------------------------------------------------- centre link
def centre_link():
    """C1 at the origin, C2 at +Y (LC), 608 at each end; built flat in XY (thickness Z)."""
    body = cq.Workplane("XY").slot2D(LC + LINK_W, LINK_W, angle=90).extrude(LINK_T).translate((0, LC / 2, 0))
    cuts = [cq.Workplane("XY").workplane(offset=-1).center(0, yc).circle(B608[1] / 2 + FIT).extrude(LINK_T + 2) for yc in (0.0, LC)]
    cuts.append(cq.Workplane("XY").workplane(offset=-1).center(0, LC / 2).slot2D(LC - LINK_W - 6, 10.0, angle=90).extrude(LINK_T + 2))
    return cut_all(body, cuts)


def centre_link_at(theta, side):
    x0 = LINK_IN if side > 0 else -LINK_OUT
    p = centre_link().rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, 0, 0))
    return p.rotate((0, 0, 0), (1, 0, 0), theta / 2)


# ---------------------------------------------------------------- plates
def _plate_base(outline_wp):
    return outline_wp


def plate_handle():
    """Upper-arm / handle plate of the +X side, inner face at x = PL_IN. Prints flat, inner face
    down (bearing boss and bolt heads up). The -X plate is the same part mirrored."""
    y0, y1 = HANDLE_Y
    z0, z1 = HANDLE_Z
    prof = (cq.Workplane("YZ").workplane(offset=PL_IN)
            .polyline([(y0, z0), (y1, z0), (y1, z1), (y0, z1)]).close().extrude(PL_T).edges("|X").fillet(8.0))
    body = prof.union(cyl_x(R_PLATE, PL_IN, PL_OUT))
    boss = cyl_x(15.0, PL_OUT - 0.1, PL_OUT + 2.0).translate((0, *SPOOL_C))      # spool bearing boss, outer face
    body = body.union(boss)
    cuts = [cyl_x(BOLT_CLR / 2, PL_IN - 1, PL_OUT + 1).translate((0, y, z)) for y, z in ((0.0, 0.0), GUIDE_C, PLATE_BOLT)]
    cuts.append(cyl_x(B608[1] / 2 + FIT, PL_IN - 1, PL_OUT + 3).translate((0, *SPOOL_C)))
    for y, z in GRIP_BOLTS:
        cuts.append(cyl_x(BOLT_CLR / 2, PL_IN - 1, PL_OUT + 1).translate((0, y, z)))
    # lightening window behind the guide, above the spool
    cuts.append(cq.Workplane("YZ").workplane(offset=PL_IN - 1).center(-58, 2).slot2D(36, 14, angle=0).extrude(PL_T + 2))
    return cut_all(body, cuts)


def plate_forearm():
    """Forearm plate of the +X side in the forearm frame (pin at the origin, bar along +Y)."""
    bar = (box(PL_IN, PL_OUT, 0, FORE_L, -FORE_Z, FORE_Z)
           .union(cyl_x(FORE_Z, PL_IN, PL_OUT).translate((0, FORE_L, 0))))
    body = bar.union(cyl_x(R_PLATE, PL_IN, PL_OUT))
    cuts = [cyl_x(BOLT_CLR / 2, PL_IN - 1, PL_OUT + 1).translate((0, y, z)) for y, z in ((0.0, 0.0), (-GUIDE_C[0], -GUIDE_C[1]), (-PLATE_BOLT[0], -PLATE_BOLT[1]))]
    for y in FORE_BOLTS:
        cuts.append(cyl_x(BOLT_CLR / 2, PL_IN - 1, PL_OUT + 1).translate((0, y, 0)))
    cuts.append(cq.Workplane("YZ").workplane(offset=PL_IN - 1).center(FORE_L / 2 + 10, 0).slot2D(40, 8, angle=0).extrude(PL_T + 2))
    return cut_all(body, cuts)


def mirror_x(wp):
    return wp.mirror("YZ")


# ---------------------------------------------------------------- spool, knob, grip
def _grooved_ring(x0, x1, phi0):
    """Spool core ring with one RIGHT-handed helical groove from x0 to x1 (plus run-out),
    at world angle phi0 at x0. Built along Z and turned into place (see J1-H)."""
    h = x1 - x0
    ee = GROOVE_R + 0.1
    path = cq.Wire.makeHelix(PITCH, h, HELIX_C, lefthand=False)
    prof = cq.Workplane("XZ").center(HELIX_C, 0).circle(GROOVE_R)
    g = prof.sweep(cq.Workplane().add(path), isFrenet=True)
    ring = cq.Workplane("XY").workplane(offset=-ee).circle(CORE_R).circle(CORE_R - 3.0).extrude(h + 2 * ee)
    core = ring.cut(g)
    assert ring.val().Volume() - core.val().Volume() > 50, "helix groove cut failed"
    core = core.rotate((0, 0, 0), (0, 1, 0), 90)             # helix axis -> +X, start point at -Z side
    return core.rotate((0, 0, 0), (1, 0, 0), phi0 + 90.0).translate((x0, 0, 0))


SP_HUB_R = 10.0
SP_SHOULDER = LINK_OUT                 # 31.4: hub end faces (washer, then the plate bearing)
STUB_END = PL_OUT + 2.0 + WASHER_T + KNOB_T + NUT_T + 1.0    # 58.1: end of the +X stub
STUB_ROUND_END = PL_OUT + 2.0 + WASHER_T + 0.5               # 42.1: round part through the bearing
FLAT_AF = 6.0                           # knob drive flats on the stub


def spool():
    """Knob spool: cord radius R_SPOOL, one RH helix from -SP_CORE_X to +SP_CORE_X, thin flanges,
    conical hubs to the plate bearings. -X hub: threaded bore for the spool bolt. +X: integral
    stub with two flats and a printed thread for the knob and its nut. Prints standing on the
    -X hub face."""
    xc = SP_CORE_X
    f0, f1 = xc, xc + SP_FLANGE_T
    body = cyl_x(CORE_R, -f0 - 0.1, f0 + 0.1).union(cyl_x(SP_FLANGE_R, -f1, -f0)).union(cyl_x(SP_FLANGE_R, f0, f1))
    ee = GROOVE_R + 0.1
    body = body.cut(cyl_x(CORE_R + 1.0, -xc - ee, xc + ee).cut(cyl_x(CORE_R - 2.5, -xc - ee, xc + ee)))
    body = body.union(_grooved_ring(-xc, xc, spool_wrap_angle(-xc)))
    for s in (1, -1):
        cone = (cq.Workplane("YZ").workplane(offset=s * f1 - (0.1 if s > 0 else -0.1)).circle(SP_FLANGE_R - 0.01)
                .workplane(offset=s * 9.0).circle(SP_HUB_R).loft())
        hub = cyl_x(SP_HUB_R, *sorted((s * f1, s * SP_SHOULDER)))
        body = body.union(cone).union(hub)
    # +X stub: round through the bearing, then flats + thread
    stub = cyl_x(SHANK_R, SP_SHOULDER - 0.1, STUB_ROUND_END)
    thr = external_thread(STUB_END - STUB_ROUND_END).rotate((0, 0, 0), (0, 1, 0), 90).translate((STUB_ROUND_END, 0, 0))
    body = body.union(stub).union(thr)
    cuts = [box(STUB_ROUND_END, STUB_END + 1, -10, 10, FLAT_AF / 2, 10), box(STUB_ROUND_END, STUB_END + 1, -10, 10, -10, -FLAT_AF / 2)]
    # -X hub: printed internal thread for the spool bolt (14 deep)
    cuts.append(thread_cutter(14.0).rotate((0, 0, 0), (0, 1, 0), -90).translate((-SP_SHOULDER, 0, 0)))
    return cut_all(body, cuts)


SPOOL_BOLT_SHANK = SP_SHOULDER + 2.0 + WASHER_T + PL_T + WASHER_T - PL_IN + 12.0   # through washer, bearing, washer, 12 into the hub


def spool_bolt():
    return bolt(shank=SPOOL_BOLT_SHANK, thread=13.0)


def block_bolt():
    return bolt(shank=BLOCK_SHANK, thread=11.0)


def pin_bolt():
    return bolt(shank=PIN_SHANK, thread=11.0)


def knob():
    """Scalloped thumb knob, built along +Z (prints flat): D-D hole on the spool stub."""
    k = cq.Workplane("XY").circle(KNOB_R).extrude(KNOB_T)
    for i in range(KNOB_N):
        a = math.radians(360.0 * i / KNOB_N)
        k = k.cut(cq.Workplane("XY").workplane(offset=-1).center(KNOB_R * math.cos(a), KNOB_R * math.sin(a)).circle(6.0).extrude(KNOB_T + 2))
    k = k.edges("|Z").fillet(2.0).edges(">Z or <Z").chamfer(1.0)
    hole = cq.Workplane("XY").workplane(offset=-1).circle(BOLT_CLR / 2).extrude(KNOB_T + 2)
    hole = hole.intersect(cq.Workplane("XY").workplane(offset=-1).rect(FLAT_AF + 0.3, 20).extrude(KNOB_T + 2))
    k = k.cut(hole)
    # pointer notch
    k = k.cut(cq.Workplane("XY").workplane(offset=KNOB_T - 1.0).center(KNOB_R - 9, 0).circle(2.0).extrude(2))
    return k


def knob_at(theta=0.0):
    x0 = PL_OUT + 2.0 + WASHER_T
    k = knob().rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, 0, 0))
    return k.rotate((0, 0, 0), (1, 0, 0), spool_turns(theta)).translate((0, *SPOOL_C))


def grip():
    """Pistol grip: a tang between the handle plates (two bolts) and a handle hanging below them."""
    tang = box(-PL_IN + 0.3, PL_IN - 0.3, GRIP_Y[0], GRIP_Y[1], HANDLE_Z[0] + 2.0, GRIP_Z[1]).edges("|X").fillet(4.0)
    hand = (box(-GRIP_X, GRIP_X, GRIP_Y[0], GRIP_Y[1], GRIP_Z[0], HANDLE_Z[0] + 3.0)
            .edges("|Z").fillet(9.0).edges("<Z").fillet(6.0))
    body = tang.union(hand)
    # teardrop holes (point towards -Z: the grip prints tang down, i.e. upside down)
    cuts = [teardrop_x(BOLT_CLR / 2, -PL_IN - 1, PL_IN + 1).rotate((0, 0, 0), (1, 0, 0), 180).translate((0, y, z)) for y, z in GRIP_BOLTS]
    return cut_all(body, cuts)


# ---------------------------------------------------------------- assembly
def bolt_x(x_head, up=True):
    """A bolt along X with its head's outer face at x_head (pointing +X if up else -X)."""
    b = bolt()
    if up:
        return b.rotate((0, 0, 0), (0, 1, 0), -90).translate((x_head, 0, 0))      # head at x_head, shank towards -X
    return b.rotate((0, 0, 0), (0, 1, 0), 90).translate((x_head, 0, 0))           # head at x_head, shank towards +X


def nut_x(x_inner, up=True):
    n = nut().rotate((0, 0, 0), (0, 1, 0), 90)
    return n.translate((x_inner if up else x_inner - NUT_T, 0, 0))


def washer_x(x_inner):
    return washer().rotate((0, 0, 0), (0, 1, 0), 90).translate((x_inner, 0, 0))


def link_parts(link):
    """All parts of one link in its own frame (pin = X axis at the origin). link 1: handle, 2: forearm."""
    P = {}
    plate = plate_handle if link == 1 else plate_forearm
    P["plate_p"] = plate()
    P["plate_n"] = mirror_x(plate())
    P["disc_p"] = disc_at(1, link)
    P["disc_n"] = disc_at(-1, link)
    for nm, z in (("F", H_BLK), ("E", -H_BLK)):
        st = {"F": (F1_ST, F2_ST), "E": (E1_ST, E2_ST)}[nm][link - 1]
        for k, v in block_stack(st).items():
            P[f"{nm}_{k}"] = v.translate((0, 0, z))
        lp, ln = SPACER_SHEAVE[f"{nm}{link}"]
        if lp > 1.0:
            P[f"{nm}_spp"] = spacer(lp, od=FILLER_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((TINE_IN - lp, 0, z))
        if ln > 1.0:
            P[f"{nm}_spn"] = spacer(ln, od=FILLER_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((-TINE_IN, 0, z))
        P[f"{nm}_bolt"] = block_bolt().rotate((0, 0, 0), (0, 1, 0), 90).translate((-TINE_OUT + POCKET_D - HEAD_T, 0, z))
    # plate-to-disc bolts: through plate, spacer, disc, (guide | spacer), disc, spacer, plate
    pb = [GUIDE_C, PLATE_BOLT] if link == 1 else [(-GUIDE_C[0], -GUIDE_C[1]), (-PLATE_BOLT[0], -PLATE_BOLT[1])]
    for i, (y, z) in enumerate(pb):
        P[f"pb{i}_bolt"] = bolt_x(-PL_OUT, up=False).translate((0, y, z))
        P[f"pb{i}_nut"] = nut_x(PL_OUT).translate((0, y, z))
        for sgn in (1, -1):
            x0 = TINE_OUT if sgn > 0 else -PL_IN
            P[f"pb{i}_blk{'p' if sgn > 0 else 'n'}"] = spacer(SPACER_BLOCK_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, y, z))
        if link == 1 and i == 0:
            P["guide"] = guide().translate((X_G, y, z))
            P["guide_sp"] = spacer(SPACER_GUIDE_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((X_G + GUIDE_W / 2 + 0.4, y, z))
        else:
            P[f"pb{i}_mid"] = spacer(SPACER_MID_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((-TINE_IN, y, z))
    # extra bolts: grip or bar
    pts = GRIP_BOLTS if link == 1 else [(y, 0.0) for y in FORE_BOLTS]
    for i, (y, z) in enumerate(pts):
        P[f"b{i}_bolt"] = bolt_x(-PL_OUT, up=False).translate((0, y, z))
        P[f"b{i}_nut"] = nut_x(PL_OUT).translate((0, y, z))
        if link == 2:
            P[f"b{i}_spacer"] = spacer(2 * PL_IN, od=SPACER_BAR_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((-PL_IN, y, z))
    # pin stubs: head - washer - plate - washer - 608 (centre link) - washer - disc thread
    for sgn in (1, -1):
        xh = sgn * (PL_OUT + WASHER_T)
        P[f"pin{'p' if sgn > 0 else 'n'}_bolt"] = pin_bolt().rotate((0, 0, 0), (0, 1, 0), -90 * sgn).translate((xh, 0, 0))
        for x in (PL_OUT, LINK_OUT, TINE_OUT):
            P[f"pin_w{sgn * x:+.0f}"] = washer_x(x if sgn > 0 else -x - WASHER_T)
    if link == 1:
        P["grip"] = grip()
    return P


def forearm_at(theta):
    c2 = c2_centre(theta)
    out = {}
    for k, v in link_parts(2).items():
        out["fore_" + k] = v.rotate((0, 0, 0), (1, 0, 0), theta).translate((0, c2[0], c2[1]))
    return out


def pin_parts(theta):
    """The two centre links."""
    return {"link_a": centre_link_at(theta, -1), "link_b": centre_link_at(theta, 1)}


def spool_parts(theta):
    cy, cz = SPOOL_C
    rot = spool_turns(theta)
    P = {"spool": spool().rotate((0, 0, 0), (1, 0, 0), rot).translate((0, cy, cz)),
         "knob": knob_at(theta),
         "knob_nut": nut_x(PL_OUT + 2.0 + WASHER_T + KNOB_T).translate((0, cy, cz)),
         "spool_bolt": spool_bolt().rotate((0, 0, 0), (0, 1, 0), 90).translate((-(PL_OUT + 2.0 + WASHER_T), cy, cz))}
    for x in (PL_OUT + 2.0, -(PL_OUT + 2.0 + WASHER_T), SP_SHOULDER, -SP_SHOULDER - WASHER_T):
        P[f"spool_w{x:+.0f}"] = washer_x(x).translate((0, cy, cz))
    return P


def assembly(theta=30.0):
    parts = {}
    for k, v in link_parts(1).items():
        parts["arm_" + k] = v
    parts.update(forearm_at(theta))
    parts.update(pin_parts(theta))
    parts.update(spool_parts(theta))
    return parts


def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


if __name__ == "__main__":
    export(plate_handle().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, PL_IN)), "plate_handle")     # inner face down
    export(plate_forearm().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, PL_IN)), "plate_forearm")
    export(disc("thread").rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, TINE_IN)), "disc_thread")     # inner face down
    export(disc("pocket").rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, TINE_IN)), "disc_pocket")
    export(sheave().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, SH_PITCH / 2)), "sheave")
    export(anchor_ring().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, SH_PITCH / 2)), "anchor_ring")
    export(centre_link(), "centre_link")
    export(spool().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, SP_SHOULDER)), "spool")               # -X hub down
    export(knob(), "knob")
    export(grip().rotate((0, 0, 0), (0, 1, 0), 180).translate((0, 0, GRIP_Z[1])), "grip")     # tang down, handle up
    export(bolt(), "bolt")
    export(block_bolt(), "bolt_block")
    export(pin_bolt(), "bolt_pin")
    export(spool_bolt(), "bolt_spool")
    export(nut(), "nut")
    export(washer(), "washer")
    export(spacer(SPACER_BLOCK_L), "spacer_block")
    export(guide().rotate((0, 0, 0), (0, 1, 0), 90).translate((0, 0, GUIDE_W / 2 + 0.4)), "guide")
    for nm in ("F1", "F2"):
        for i, L in enumerate(SPACER_SHEAVE[nm]):
            if L > 1.0:
                export(spacer(L, od=FILLER_OD), f"spacer_sheave_{nm}{'ab'[i]}")
    export(spacer(SPACER_GUIDE_L), "spacer_guide")
    export(spacer(SPACER_MID_L), "spacer_mid")
    export(spacer(2 * PL_IN, od=SPACER_BAR_OD), "spacer_bar")
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]), str(OUT / "assembly_preview.step"))
    print(f"ratio at 0 deg {RATIO0:.2f}, spool at {SPOOL_C}, helix phase in/out {SPOOL_IN:.1f}/{SPOOL_OUT:.1f}, "
          f"DX {DX:.2f}, stations F1 {F1_ST} F2 {F2_ST}, sheave spacers {SPACER_SHEAVE}")
    print("done ->", OUT)
