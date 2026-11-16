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
T1_IN = 172.0                 # the pass-through cord meets pulley 1 here, coming up at 8 deg off vertical
TH_MIN, TH_MAX = 0.0, 180.0    # straight to fully folded: the plate flats meet at both ends

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
SP_CORE_X = 22.0                         # helix from -22 to +22 (fleet points at +-12.3 migrate 8 mm)
SP_FLANGE_R, SP_FLANGE_T = R_SPOOL + 4.0, 2.0
SPOOL_C = (-(R_SH + R_SPOOL), -62.0)     # spool axis (y, z): its front tangent is the line y = -R_SH
R_G = R_SPOOL                            # guide sheave cord radius
R_GG, R_GF = R_G - CORD_D / 2, R_G + 2.0
GUIDE_W, GUIDE_GW = 8.0, 6.0                 # wide groove: the runs on and off the guide fleet up to 6 deg
GUIDE_C = (-59.0, -3.0)                  # guide axis (y, z): far behind the discs, its run to E1 clears the plate bolt
PLATE_BOLT = (-22.0, -1.0)               # plate-to-disc bolt behind the pin: outside the centre links' sweep, inside the groove wall
NECK_D = 5.0                             # block axles: thin neck on the far side of the joint plane (the other tackle's runs pass 4 mm clear)
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
SPACER_MID_L = 2 * TINE_IN               # plate bolt between the discs
SPACER_PP_L = 2 * PL_IN                  # plate-to-plate bolts outside the discs (guide bolt, forearm)
SPACER_OD = 11.0                         # plate-bolt spacers (1.3 mm wall; they pass 1.5 mm from the centre links)
FILLER_OD = 11.0                         # fillers on the block axles (the opposite tackle's runs pass 0.3 mm clear at 135 deg)

# printed thread "P8": 8 mm nominal, 2 mm pitch, 60 deg flanks (trapezoid)
P_PITCH = 2.0
THREAD_OFFSET = 0.22                     # internal thread = external profile offset by this (measured on printed test pieces)
EXT_MINOR, EXT_MAJOR = 2.8, 4.0          # radii of the printed bolt thread (8 mm nominal, 1.2 mm deep)
INT_MINOR, INT_MAJOR = EXT_MINOR + THREAD_OFFSET, EXT_MAJOR + THREAD_OFFSET
SHANK_R = 3.95                           # smooth shank (7.9 mm)
BOLT_CLR = 8.4                           # clearance hole
HEAD_AF, HEAD_T = 13.0, 5.0
NUT_AF, NUT_T = 13.2, 6.5
BOLT_SHANK = 2 * (PL_OUT + WASHER_T) + NUT_T + 1.0    # 95.7: plate / grip / bar bolts (through everything)
POCKET_D = HEAD_T + 0.4                                  # hex pocket for the block bolt heads (outer face of every block hole)
BLOCK_NECK_L = TINE_OUT - POCKET_D - 0.5                 # 21.4: 5 mm neck from the head to 0.5 mm short of the joint plane
BLOCK_SHANK = TINE_IN + 0.5                              # 16.2: 7.9 mm shank carrying the sheaves
PIN_SHANK = PL_OUT + WASHER_T - TINE_OUT + 11.0          # pin stubs: through plate + centre-link bearing, 11 mm into the disc
DISC_THREAD_L = 12.0
THREAD_L = 16.0

# handle plate / grip / forearm bar
HANDLE_Y = (-84.0, R_PLATE)              # plate body: square with the quarter disc at the pin corner
HANDLE_Z = (-86.0, R_PLATE)
GRIP_Y = (-82.0, -52.0)                  # leaves a corridor for the pass-through cord between spool and grip
GRIP_WALL = 3.0
GRIP_BOLTS = ((-75.0, -74.0), (-61.0, -74.0))    # 3 mm clear of the knob rim
GRIP_Z = (-180.0, -64.0)                 # handle bottom, tang top (solid 20 mm tang)
GRIP_X = 16.0
FORE_L = 110.0
FORE_Z = 12.0
FORE_BOLTS = ((55.0, -45.0), (74.0, -70.0))       # (y, z) in the forearm frame: weight hooks, clear of the pass-through exit (z = -30)
SPACER_BAR_OD = 12.0
RIM, STRUT_W, BOSS_R = 12.0, 10.0, 9.0   # truss plates: rim width, strut width, boss radius around the bolt holes


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
    if best is None or best / 2 - 1.5 * SH_PITCH - SH_W / 2 < 0.0:   # the anchor ring body must stay on its own side of x = 0
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
assert F2_ST["anchor"] - SH_W / 2 >= 0.0, "the F tackle must stay at x >= 0"
# fillers on the block axles: (towards +X disc, towards -X disc)
SPACER_SHEAVE = {"F1": (TINE_IN - (F1_ST["s3"] + SH_PITCH / 2), F1_ST["s1"] - SH_PITCH / 2 + 0.5),
                 "F2": (TINE_IN - (F2_ST["s2"] + SH_PITCH / 2), F2_ST["anchor"] - SH_PITCH / 2 + 0.5)}   # (to the disc, to the axle step)
SPACER_SHEAVE["E1"] = SPACER_SHEAVE["F1"][::-1]
SPACER_SHEAVE["E2"] = SPACER_SHEAVE["F2"][::-1]
SPACER_GUIDE = (PL_IN + X_G - GUIDE_W / 2 - 0.2, PL_IN - (X_G + GUIDE_W / 2) - 0.2)   # plate-to-plate bolt: (-X of the guide, +X of the guide)


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
    a1 = phi1
    while a1 < a0 - 1e-6:
        a1 += 360
    while a1 >= a0 + 360 - 1e-6:
        a1 -= 360
    pts += arc(B1, R_SH, a0, a1, F1_ST["s3"])[1:]              # CCW over the back of F1 s3, wrap 90 - theta/2
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
    while a1 < a0 - 1e-6:
        a1 += 360
    while a1 >= a0 + 360 - 1e-6:
        a1 -= 360
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
    # ---- pass-through T1: comes up the back of the handle (past the spool flange and the grip),
    # meets pulley 1 at T1_IN, runs over the top to the contact, crosses, under pulley 2 and out
    # along the bottom of the forearm (exits at 270 + theta)
    x = ST["T1"]
    p0 = polar_yz(R, T1_IN)
    dd = e(T1_IN + 90.0)
    pts = [(x, p0[0] + 110.0 * dd[0], p0[1] + 110.0 * dd[1])] + arc((0.0, 0.0), R, T1_IN, contact, x)
    pts += arc(c2, R, 180.0 + contact, 270.0 + theta, x)[1:]
    ex = e(theta)
    pts.append((x, pts[-1][1] + 70 * ex[0], pts[-1][2] + 70 * ex[1]))
    paths["T1"] = pts
    return paths


# ---------------------------------------------------------------- printed threads
def _thread_ridge(r_minor, r_major, pitch, length, grow=0.0):
    """One helical ridge: trapezoid (root 0.75 P, crest 0.25 P wide) from inside r_minor out to
    r_major; `grow` widens both flanks (the internal cutter is the bolt profile offset by it)."""
    path = cq.Wire.makeHelix(pitch, length, r_minor)
    w = 0.25 * pitch / 2 + grow
    bb = 0.75 * pitch / 2 + grow
    prof = cq.Workplane("XZ").polyline([(r_minor - 0.3, -bb), (r_major, -w), (r_major, w), (r_minor - 0.3, bb)]).close()
    return prof.sweep(cq.Workplane().add(path), isFrenet=True)


def prism(r, z0, z1, n=48):
    """48-sided prism with its flats at radius r. OCC refuses to fuse or cut a cylinder that is
    coaxial with a helical sweep (the boolean quietly returns one operand), so every thread core
    and every threaded bore is a prism instead."""
    return cq.Workplane("XY").workplane(offset=z0).polygon(n, 2 * r / math.cos(math.pi / n)).extrude(z1 - z0)


def external_thread(length, chamfer=1.0):
    """Printed bolt thread along +Z from z = 0, chamfered at both ends."""
    core = prism(EXT_MINOR, 0, length)
    ridge = _thread_ridge(EXT_MINOR, EXT_MAJOR, P_PITCH, length + 2 * P_PITCH).translate((0, 0, -P_PITCH))
    ro = EXT_MAJOR + 0.5
    pts = [(0, 0), (EXT_MAJOR - chamfer, 0), (ro, chamfer + 0.5), (ro, length - chamfer - 0.5), (EXT_MAJOR - chamfer, length), (0, length)]
    env = cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
    return core.union(ridge).intersect(env)


def thread_cutters(length):
    """Two cutters (bore prism, helical ridge) for a printed internal thread from z = -pitch to
    length + pitch; cut them one after the other (fusing them first fails, see prism())."""
    core = prism(INT_MINOR, -P_PITCH, length + P_PITCH)
    ridge = _thread_ridge(EXT_MINOR, INT_MAJOR, P_PITCH, length + 2 * P_PITCH, grow=THREAD_OFFSET).translate((0, 0, -P_PITCH))
    return [core, ridge]


def internal_thread_x(x0, length, y=0.0, z=0.0, into=1):
    """Cutters for a thread along X starting at x0 and running `length` in direction `into` (+1/-1)."""
    out = []
    for c in thread_cutters(length):
        out.append(c.rotate((0, 0, 0), (0, 1, 0), 90 * into).translate((x0, y, z)))
    return out


def bolt(shank=BOLT_SHANK, thread=THREAD_L, neck=0.0):
    """Printed hex bolt along +Z: head z in [0, HEAD_T], optional thin neck, shank, thread at the
    end. Prints standing on the head."""
    head = cq.Workplane("XY").polygon(6, HEAD_AF / math.cos(math.pi / 6)).extrude(HEAD_T).edges(">Z").chamfer(0.8)
    body = head
    z = HEAD_T
    if neck > 0:
        body = body.union(cq.Workplane("XY").workplane(offset=z).circle(NECK_D / 2).extrude(neck + 0.01))
        z += neck
    body = body.union(cq.Workplane("XY").workplane(offset=z).circle(SHANK_R).extrude(shank - thread + 0.01))
    th = external_thread(thread).translate((0, 0, z + shank - thread))
    return body.union(th)


def nut():
    body = cq.Workplane("XY").polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(NUT_T).edges(">Z or <Z").chamfer(0.8)
    return cut_all(body, thread_cutters(NUT_T))


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


def disc():
    """Rolling disc of the +X side of the UPPER ARM: x in [TINE_IN, TINE_OUT], grooves X2 and
    T2, X2's chord anchor (arrives CW at 130, exits at 75). Pin stub thread from the outer face.
    Each block hole has a hex pocket on the outer face and a printed thread inside, so the same
    part serves every position: the -X discs are this part turned 180 deg about Y, the forearm's
    discs are turned 180 deg about X. Prints flat, inner face down."""
    body = cyl_x(R_BODY, TINE_IN, TINE_OUT)
    cuts = [groove_ring(ST["X2"], R_GROOVE, R_BODY, GW), groove_ring(ST["T2"], R_GROOVE, R_BODY, GW),
            chord_anchor(ST["X2"], ANCHOR_P1["X2"], -1)]
    cuts += internal_thread_x(TINE_OUT + 0.5, DISC_THREAD_L, into=-1)                       # pin stub
    for z in (H_BLK, -H_BLK):
        cuts += internal_thread_x(TINE_IN - 1, TINE_OUT - TINE_IN + 2, z=z, into=1)        # block axle thread
        cuts.append(hex_prism_x(TINE_OUT - POCKET_D, TINE_OUT + 1, 0, z, HEAD_AF + 0.4))      # or its head pocket
    for sy in (1, -1):                                                                        # PLATE_BOLT, both mirror positions
        for sz in (1, -1):
            cuts.append(cyl_x(BOLT_CLR / 2, TINE_IN - 1, TINE_OUT + 1).translate((0, sy * PLATE_BOLT[0], sz * PLATE_BOLT[1])))
    for k in range(4):                                                                        # lightening, between the block pockets and the plate bolts
        y, z = polar_yz(20.0, 45.0 + 90.0 * k)
        cuts.append(cyl_x(4.5, TINE_IN - 1, TINE_OUT + 1).translate((0, y, z)))
    # recess from the outer face: a 4 mm web between the groove rim and the hub, bosses kept round
    # the block pockets and the plate-bolt holes (prints inner face down, so the recess faces up)
    recess = cyl_x(25.0, TINE_IN + 4.0, TINE_OUT + 1).cut(cyl_x(10.0, TINE_IN + 3.0, TINE_OUT + 2))
    for z in (H_BLK, -H_BLK):
        recess = recess.cut(cyl_x(9.4, TINE_IN + 3.0, TINE_OUT + 2).translate((0, 0, z)))
    for sy in (1, -1):
        for sz in (1, -1):
            recess = recess.cut(cyl_x(6.5, TINE_IN + 3.0, TINE_OUT + 2).translate((0, sy * PLATE_BOLT[0], sz * PLATE_BOLT[1])))
    cuts.append(recess)
    return cut_all(body, cuts)


def disc_at(side, link):
    """side +-1 (x side), link 1 (upper arm) or 2 (forearm, in the forearm's own frame)."""
    d = disc()
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


def _outline2d(mirror=False, x0=PL_IN):
    """Closed 2-D outline of a plate on a YZ workplane at x = x0: a square body with the quarter
    disc at the pin corner (the Rhino sketch). mirror=True gives the forearm plate."""
    y0, y1 = HANDLE_Y
    z0, z1 = HANDLE_Z
    sg = -1 if mirror else 1
    wp = cq.Workplane("YZ").workplane(offset=x0)
    a45 = R_PLATE * math.cos(math.pi / 4)
    return (wp.moveTo(sg * y0, z0).lineTo(sg * y1, z0).lineTo(sg * y1, 0.0)
            .threePointArc((sg * a45, a45), (0.0, z1)).lineTo(sg * y0, z1).close())


def _truss_plate(mirror, holes, big=(), struts=()):
    """Plate = outline rim (RIM wide) + bosses round the holes + struts between them, PL_T thick,
    inner face at PL_IN. `big` = (y, z, r) extra bosses (bearing seat), `struts` = ((y, z), (y, z))."""
    full = _outline2d(mirror).extrude(PL_T)
    inner = _outline2d(mirror, PL_IN - 1).offset2D(-RIM).extrude(PL_T + 2)
    body = full.cut(inner)
    for (y, z) in holes:
        body = body.union(cyl_x(BOSS_R, PL_IN, PL_OUT).translate((0, y, z)))
    for (y, z, r) in big:
        body = body.union(cyl_x(r, PL_IN, PL_OUT).translate((0, y, z)))
    for (p, q) in struts:
        L = math.dist(p, q)
        ang = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
        body = body.union(cq.Workplane("YZ").workplane(offset=PL_IN).center((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
                          .slot2D(L + STRUT_W, STRUT_W, angle=ang).extrude(PL_T))
    body = body.intersect(_outline2d(mirror, PL_IN - 1).extrude(PL_T + 2))      # nothing outside the outline
    cuts = [cyl_x(BOLT_CLR / 2, PL_IN - 1, PL_OUT + 3).translate((0, y, z)) for y, z in holes]
    return cut_all(body, cuts)


def plate_handle():
    """Upper-arm / handle plate of the +X side, inner face at x = PL_IN. Prints flat, inner face
    down (bearing boss and bolt heads up). The -X plate is the same part mirrored."""
    pin, pb, gd = (0.0, 0.0), PLATE_BOLT, GUIDE_C
    sp = SPOOL_C
    a45 = R_PLATE * math.cos(math.pi / 4)
    holes = [pin, pb, gd] + list(GRIP_BOLTS)
    g1, g2 = GRIP_BOLTS
    struts = [(pin, pb), (pb, sp), (pb, gd), (gd, (HANDLE_Y[0] + RIM / 2, gd[1])), (gd, (gd[0], HANDLE_Z[1] - RIM / 2)),
              (pin, (a45, a45)), (pin, (R_PLATE - RIM / 2, -8.0)), (pin, (-8.0, R_PLATE - RIM / 2)),
              (sp, (sp[0], HANDLE_Z[0] + RIM / 2)), (sp, (HANDLE_Y[1] - RIM / 2, sp[1])),
              (pin, sp), (gd, sp), (gd, g1), (sp, g2), (pb, (pb[0], HANDLE_Z[1] - RIM / 2))]     # diagonals brace every bay
    body = _truss_plate(False, holes, big=[(sp[0], sp[1], B608[1] / 2 + 4.5)], struts=struts)
    boss = cyl_x(15.0, PL_OUT - 0.1, PL_OUT + 2.0).translate((0, *sp))         # spool bearing boss on the outer face
    body = body.union(boss)
    return body.cut(cyl_x(B608[1] / 2 + FIT, PL_IN - 1, PL_OUT + 3).translate((0, *sp)))


def plate_forearm():
    """Forearm plate of the +X side in the forearm frame (pin at the origin, body along +Y and
    below the pin, quarter disc towards the upper arm)."""
    pin, pb, pp = (0.0, 0.0), (-PLATE_BOLT[0], -PLATE_BOLT[1]), (-GUIDE_C[0], -GUIDE_C[1])
    h1, h2 = FORE_BOLTS
    a45 = R_PLATE * math.cos(math.pi / 4)
    holes = [pin, pb, pp, h1, h2]
    struts = [(pin, pb), (pb, pp), (pp, (-HANDLE_Y[0] - RIM / 2, pp[1])), (pp, h1), (h1, h2), (h2, (h2[0], HANDLE_Z[0] + RIM / 2)),
              (pin, (-a45, a45)), (pin, (-(R_PLATE - RIM / 2), -8.0)), (pin, (8.0, R_PLATE - RIM / 2)),
              (pp, (pp[0], HANDLE_Z[1] - RIM / 2)), (pin, h1), (pb, h1), (h1, (-HANDLE_Y[0] - RIM / 2, h1[1])),
              (h1, (-HANDLE_Y[1] + RIM / 2, h1[1])), (pb, (pb[0], HANDLE_Z[1] - RIM / 2))]        # diagonals brace every bay
    return _truss_plate(True, holes, struts=struts)


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
    # -X hub: printed internal thread for the spool bolt (14 deep); hollow core and +X hub
    cuts += internal_thread_x(-SP_SHOULDER, 14.0, into=1)
    pocket = (cq.Workplane("XY").polyline([(-SP_SHOULDER + 14.0 - 2.0, 0), (-SP_SHOULDER + 14.0 - 2.0, 7.0), (SP_CORE_X - 6.0, 7.0),
                                           (SP_CORE_X, 1.0), (SP_CORE_X, 0)]).close().revolve(360, (0, 0, 0), (1, 0, 0)))
    cuts.append(pocket)                      # hollow core: opens into the threaded hub, 45 deg cone at the far end (prints standing)
    return cut_all(body, cuts)


SPOOL_BOLT_SHANK = SP_SHOULDER + 2.0 + WASHER_T + PL_T + WASHER_T - PL_IN + 12.0   # through washer, bearing, washer, 12 into the hub


def spool_bolt():
    return bolt(shank=SPOOL_BOLT_SHANK, thread=13.0)


def block_bolt():
    """Block axle: head in the pocket of one disc, 5 mm neck across the other tackle's side,
    7.9 mm shank under the sheaves, thread into the opposite disc."""
    return bolt(shank=BLOCK_SHANK + 11.0, thread=11.0, neck=BLOCK_NECK_L)


def pin_bolt():
    return bolt(shank=PIN_SHANK, thread=11.0)


def knob():
    """Scalloped thumb knob, built along +Z (prints flat): D-D hole on the spool stub."""
    k = cq.Workplane("XY").circle(KNOB_R).extrude(KNOB_T)
    for i in range(KNOB_N):
        a = math.radians(360.0 * i / KNOB_N)
        k = k.cut(cq.Workplane("XY").workplane(offset=-1).center(KNOB_R * math.cos(a), KNOB_R * math.sin(a)).circle(6.0).extrude(KNOB_T + 2))
    k = k.edges("|Z").fillet(2.0).edges(">Z or <Z").chamfer(1.0)
    for i in range(6):                                              # windows
        a = math.radians(60.0 * i + 30.0)
        k = k.cut(cq.Workplane("XY").workplane(offset=-1).center(16.5 * math.cos(a), 16.5 * math.sin(a)).circle(5.5).extrude(KNOB_T + 2))
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
    """Pistol grip: a solid 20 mm tang between the handle plates (two bolts) and a hollow handle
    (3 mm walls, open at the bottom) hanging below them. Prints tang down: nothing bridges."""
    zt0, zt1 = HANDLE_Z[0] + 2.0, GRIP_Z[1]
    tang = box(-PL_IN + 0.3, PL_IN - 0.3, GRIP_Y[0], GRIP_Y[1], zt0, zt1).edges("|X").fillet(4.0)
    hand = (box(-GRIP_X, GRIP_X, GRIP_Y[0], GRIP_Y[1], GRIP_Z[0], HANDLE_Z[0] + 3.0)
            .edges("|Z").fillet(9.0).edges("<Z").fillet(6.0))
    body = tang.union(hand)
    w = GRIP_WALL
    cuts = [box(-GRIP_X + w, GRIP_X - w, GRIP_Y[0] + w, GRIP_Y[1] - w, GRIP_Z[0] - 1, HANDLE_Z[0]).edges("|Z").fillet(6.0)]
    cuts += [teardrop_x(BOLT_CLR / 2, -PL_IN - 1, PL_IN + 1).rotate((0, 0, 0), (1, 0, 0), 180).translate((0, y, z)) for y, z in GRIP_BOLTS]
    return cut_all(body, cuts)


# ---------------------------------------------------------------- assembly
def bolt_x(x_face, up=True, b=None):
    """A bolt along X whose head bears on the face at x_face: head on the +X side pointing +X
    (shank towards -X) if up, else head on the -X side (shank towards +X)."""
    b = bolt() if b is None else b
    if up:
        return b.rotate((0, 0, 0), (0, 1, 0), -90).translate((x_face + HEAD_T, 0, 0))   # head in [x_face, x_face + HEAD_T]
    return b.rotate((0, 0, 0), (0, 1, 0), 90).translate((x_face - HEAD_T, 0, 0))        # head in [x_face - HEAD_T, x_face]


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
        sgn = 1 if nm == "F" else -1                        # the F stack sits at x > 0, the E stack at x < 0
        if lp > 1.0:
            P[f"{nm}_spp"] = spacer(lp, od=FILLER_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((min(sgn * TINE_IN, sgn * (TINE_IN - lp)), 0, z))
        if ln > 1.0:
            P[f"{nm}_spn"] = spacer(ln, od=FILLER_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((min(-sgn * 0.5, -sgn * 0.5 + sgn * ln), 0, z))
        bb = block_bolt().rotate((0, 0, 0), (0, 1, 0), 90 * sgn)      # head at the -X disc for F, at the +X disc for E
        P[f"{nm}_bolt"] = bb.translate((-sgn * (TINE_OUT - POCKET_D + HEAD_T), 0, z))
    sg = 1 if link == 1 else -1
    # plate-to-disc bolt (PLATE_BOLT): plate, spacer, disc, spacer between the discs, disc, spacer, plate
    y, z = sg * PLATE_BOLT[0], sg * PLATE_BOLT[1]
    P["pb_bolt"] = bolt_x(-PL_OUT, up=False).translate((0, y, z))
    P["pb_nut"] = nut_x(PL_OUT).translate((0, y, z))
    for sgn in (1, -1):
        x0 = TINE_OUT if sgn > 0 else -PL_IN
        P[f"pb_blk{'p' if sgn > 0 else 'n'}"] = spacer(SPACER_BLOCK_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, y, z))
    P["pb_mid"] = spacer(SPACER_MID_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((-TINE_IN, y, z))
    # plate-to-plate bolt behind the discs: carries the guide on the upper arm, a plain spacer on the forearm
    y, z = sg * GUIDE_C[0], sg * GUIDE_C[1]
    P["gb_bolt"] = bolt_x(-PL_OUT, up=False).translate((0, y, z))
    P["gb_nut"] = nut_x(PL_OUT).translate((0, y, z))
    if link == 1:
        P["guide"] = guide().translate((X_G, y, z))
        P["gb_spn"] = spacer(SPACER_GUIDE[0]).rotate((0, 0, 0), (0, 1, 0), 90).translate((-PL_IN, y, z))
        P["gb_spp"] = spacer(SPACER_GUIDE[1]).rotate((0, 0, 0), (0, 1, 0), 90).translate((X_G + GUIDE_W / 2 + 0.2, y, z))
    else:
        P["gb_sp"] = spacer(SPACER_PP_L).rotate((0, 0, 0), (0, 1, 0), 90).translate((-PL_IN, y, z))
    # extra bolts: grip or weight hooks
    pts = GRIP_BOLTS if link == 1 else FORE_BOLTS
    for i, (y, z) in enumerate(pts):
        P[f"b{i}_bolt"] = bolt_x(-PL_OUT, up=False).translate((0, y, z))
        P[f"b{i}_nut"] = nut_x(PL_OUT).translate((0, y, z))
        if link == 2:
            P[f"b{i}_spacer"] = spacer(SPACER_PP_L, od=SPACER_BAR_OD).rotate((0, 0, 0), (0, 1, 0), 90).translate((-PL_IN, y, z))
    # pin stubs: head - washer - plate - washer - 608 (centre link) - washer - disc thread
    for sgn in (1, -1):
        P[f"pin{'p' if sgn > 0 else 'n'}_bolt"] = bolt_x(sgn * (PL_OUT + WASHER_T), up=sgn > 0, b=pin_bolt())
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
         "spool_bolt": bolt_x(-(PL_OUT + 2.0 + WASHER_T), up=False, b=spool_bolt()).translate((0, cy, cz))}
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
    export(disc().rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, TINE_IN)), "disc")                   # inner face down
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
    export(spacer(SPACER_GUIDE[0]), "spacer_guide_a")
    export(spacer(SPACER_GUIDE[1]), "spacer_guide_b")
    export(spacer(SPACER_MID_L), "spacer_mid")
    export(spacer(SPACER_PP_L), "spacer_plate")
    export(spacer(SPACER_PP_L, od=SPACER_BAR_OD), "spacer_bar")
    asm = assembly()
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]), str(OUT / "assembly_preview.step"))
    print(f"ratio at 0 deg {RATIO0:.2f}, spool at {SPOOL_C}, helix phase in/out {SPOOL_IN:.1f}/{SPOOL_OUT:.1f}, "
          f"DX {DX:.2f}, stations F1 {F1_ST} F2 {F2_ST}, sheave spacers {SPACER_SHEAVE}")
    print("done ->", OUT)
