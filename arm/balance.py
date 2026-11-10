"""Gravity balance for the shoulder (S1): zero-free-length spring balancing.

    .venv/bin/python -m arm.balance          # mass model + design numbers

Principle
---------
A link pivots about S1 (axis X) with weight moment m*g*d*sin(th) (th = flexion
from hanging). A cord from a point B on the arm (distance b from the S1 axis, on
the arm axis line) runs over a pulley A fixed a above the S1 axis. If the cord
pulls with F = k*|AB| (a "zero-length" spring) its moment about S1 is
k*a*b*sin(th) - for EVERY angle. So k*a*b = m*g*d balances the arm everywhere.
Offsets of A and B along the axis (X) do not change this, so two cords run in
the side planes x = +-X_S (outside the beam and the S1 base), symmetric, and
the S1 bearings see no twisting moment.

The zero length is emulated with a long extension spring behind the pulley: the
spring force is Fi + k*e (e = elongation); the cord from the pulley to the spring
is made so long that e = |AB| - Fi/k. The adjustable anchor (+-20 mm M5 screw)
tunes that offset (initial tension, cord stretch, knot length).

Hardware (per side): one spring, one 623-on-M3 sheave pair on the mast, a cord
loop on an M3 pin in the yoke. Parts:

* `link_s1_s2` gets two yoke arms (same flanges and length -> kinematics unchanged)
  carrying the cord pins B at (+-X_S, z = B_Z) in the link frame.
* `balance_mast_l/_r`: hooks over the beam side wall (+ front-end lip), carries
  the sheave A right above the S1 axis and a fairlead that sends the cord back.
* `balance_sheave`: printed sheave on two 623 bearings.
* `balance_post`: screwed to the table behind, with the M5 tuning screw.
"""
import math
from functools import lru_cache

import cadquery as cq
import numpy as np

from . import assembly as A
from . import hinge as H
from . import params as P
from . import roll as R
from . import torso as T
from .cad import box, cut_all, cyl_x, cyl_y, cyl_z, rod

G = 9.81

# ------------------------------------------------------------------ mass model
PETG_RHO = 1.27e-3          # g/mm^3
SHELL_T = 1.8               # 4 perimeters x 0.45 mm (+ top/bottom skins) printed solid
INFILL = 0.40               # 40 % gyroid inside (README print settings)
TUBE_G_PER_MM = 0.0214      # PTFE 4x2 (2.2 g/cm^3 * 9.42 mm^2) + 1 mm Dyneema inside
M3_BOLT, M3_INSERT = 1.0, 0.6
B608_G, B623_G = 12.0, 1.6
M8_SET_G = 21.0             # M8x20 socket bolt 13.5 g + nut 5 g + washer 2.5 g
ROLL_BEARING_G = {"S3": 19.0, "F": 15.0}   # 61806-2RS / 61805-2RS catalogue masses


def fill_factor(shape):
    """Effective solid fraction of a printed part: a SHELL_T skin over the whole
    surface is solid, the rest is INFILL. Thin parts come out (nearly) solid."""
    v, a = shape.Volume(), shape.Area()
    shell = min(v, 0.5 * a * SHELL_T)       # 0.5: opposite faces share thin walls
    return (shell + INFILL * (v - shell)) / v


@lru_cache(maxsize=None)
def _printed_masses():
    """[(element, 'base'|'mov', part, mass g, com (3,) in element frame)]."""
    out = []
    for name, spec, _ in A.CHAIN:
        st, mv = A.element_parts(name, spec)
        for which, parts in (("base", st), ("mov", mv)):
            for k, w in parts.items():
                s = w.val()
                c = cq.Shape.centerOfMass(s)
                m = s.Volume() * PETG_RHO * fill_factor(s)
                out.append((name, which, k, m, np.array([c.x, c.y, c.z])))
    return out


def _hardware_masses():
    """Bearings, axles, inserts, bolts, tubes as point masses in element frames."""
    out = []
    specs = dict((n, s) for n, s, _ in A.CHAIN)
    for name, spec, _ in A.CHAIN:
        if isinstance(spec, H.HingeSpec):
            F = P.FLANGES[spec.cls]
            Fr = P.FLANGES[spec.rcls]
            out.append((name, "mov", "608 x2", 2 * B608_G, np.zeros(3)))
            if name != "S1":                     # S1 lugs are on the beam
                out.append((name, "base", "M8 axles", 2 * M8_SET_G, np.zeros(3)))
            n_base = 4 if spec.base_style == "torso" else F["n"]
            out.append((name, "base", "inserts", n_base * M3_INSERT, np.array([0, 0, spec.in_z + 3])))
            out.append((name, "mov", "inserts", Fr["n"] * M3_INSERT, np.array([0, 0, spec.out_z - 3])))
        elif isinstance(spec, R.RollSpec):
            zm = (spec.in_z + spec.out_z) / 2
            out.append((name, "base", "bearings", 2 * ROLL_BEARING_G[name], np.array([0, 0, zm])))
            out.append((name, "base", "idlers", 2 * (B623_G + 1.5), np.array([0, 0, zm])))
            out.append((name, "base", "inserts", 8 * M3_INSERT, np.array([0, 0, spec.in_z + 3])))
            out.append((name, "mov", "inserts", 8 * M3_INSERT, np.array([0, 0, spec.out_z - 3])))
        else:
            ca, cb, _ = spec
            L = A.link_length(name, spec)
            out.append((name, "base", "M3 bolts", P.FLANGES[ca]["n"] * M3_BOLT, np.array([0, 0, 2.5])))
            out.append((name, "base", "M3 bolts", P.FLANGES[cb]["n"] * M3_BOLT, np.array([0, 0, L - 2.5])))
    # Bowden tubes (+ cord) inside the arm: an element carries the tubes of every joint
    # further out (a link before joint j also the 2 of j itself)
    names = [n for n, _, _ in A.CHAIN]
    for i, (name, spec, _) in enumerate(A.CHAIN):
        z0, z1 = A.element_io(spec, name)
        if isinstance(spec, (H.HingeSpec, R.RollSpec)):
            n = spec.tubes_through
        else:
            nxt = specs[names[i + 1]]
            n = nxt.tubes_through + 2
        if n == 0:
            continue
        if isinstance(spec, H.HingeSpec):     # half on the base, half on the rotor
            if name != "S1":
                out.append((name, "base", f"tubes x{n}", n * TUBE_G_PER_MM * -z0, np.array([0, 0, z0 / 2])))
            out.append((name, "mov", f"tubes x{n}", n * TUBE_G_PER_MM * z1, np.array([0, 0, z1 / 2])))
        else:
            out.append((name, "base", f"tubes x{n}", n * TUBE_G_PER_MM * (z1 - z0), np.array([0, 0, (z0 + z1) / 2])))
    return out


def masses(include_payload=0.0):
    """All point masses (element, base|mov, label, grams, local com) moving with S1
    (= everything but the S1 base). include_payload: grams at the tool point."""
    m = [r for r in _printed_masses() + _hardware_masses() if not (r[0] == "S1" and r[1] == "base")]
    if include_payload:
        from .kinematics import TOOL_OFFSET
        m.append(("W2", "mov", "payload", include_payload, np.array([0, 0, A.W2.out_z + TOOL_OFFSET])))
    return m


def _world_points(q, ms):
    fr = {n: (b, mv) for n, _, b, mv in A.chain_frames(q)}
    pts, ws, els = [], [], []
    for el, which, _, g, c in ms:
        M = fr[el][0] if which == "base" else fr[el][1]
        pts.append((M @ np.r_[c, 1.0])[:3])
        ws.append(g / 1000.0)
        els.append(el)
    return np.array(pts), np.array(ws), els, fr


def joint_axis(fr, j):
    """(point, unit axis) of hinge j in world for frames fr."""
    b, _ = fr[j]
    return b[:3, 3], b[:3, 0]


def gravity_torque(q, j="S1", payload=0.0, ms=None):
    """Gravity torque (N*m) about hinge j from everything outboard of it. Positive =
    turning the joint towards +angle (the holding torque is minus this)."""
    ms = ms if ms is not None else masses(payload)
    pts, ws, els, fr = _world_points(q, ms)
    order = [n for n, _, _ in A.CHAIN]
    k = order.index(j)
    o, ax = joint_axis(fr, j)
    keep = np.array([order.index(e) > k or (e == j) for e in els])
    if j != "S1":   # the base of j itself does not load j
        keep &= ~np.array([(e == j and r[1] == "base") for e, r in zip(els, ms)])
    f = np.array([0, 0, -G])
    tau = np.cross(pts[keep] - o, ws[keep][:, None] * f) / 1000.0
    return float(tau.sum(0) @ ax)


def mass_properties(payload=0.0):
    """Total mass (kg) moving with S1, COM distance d (m) from the S1 axis at q=0
    (along the hanging arm), and the m*g*d balance target (N*m)."""
    ms = masses(payload)
    pts, ws, _, fr = _world_points({}, ms)
    o, _ = joint_axis(fr, "S1")
    m = ws.sum()
    com = (pts * ws[:, None]).sum(0) / m
    d = (o[2] - com[2]) / 1000.0
    return m, d, m * G * d


def d_vs_elbow(es=(0, 30, 60, 90, 115), payload=0.0):
    """Effective COM lever (m) for flexion, from the S1 torque at S1 = 90 deg."""
    ms = masses(payload)
    m = sum(r[3] for r in ms) / 1000.0
    return {e: -gravity_torque({"S1": 90, "E": e}, "S1", ms=ms) / (m * G) for e in es}


# ------------------------------------------------------------------ design numbers
# Balance level: the arm's S1 gravity moment at 90 deg is m*g*d = 7.6 N*m straight,
# but only ~2 N*m with the shoulder fully abducted (S2 = 100) and the elbow bent. A
# spring can only follow sin(S1), so K = k*a*b (both sides) is the MINIMAX value over
# the S1 x S2 x E ranges (tools/check_balance.py re-derives it): 4.75 N*m.
K_DESIGN = 4.75             # N*m, both sides together

# Spring (per side, 1:1, no tackle): RS PRO 751938 class - music wire 1.4 mm, OD 15 mm,
# L0 105 mm over the hooks, rate 0.26 N/mm (~60 active coils). See README for the spec.
K_SPRING = 0.26             # N/mm
L0_SPRING = 105.0           # mm, free length inside the hooks
FI_SPRING = 3.0             # N, nominal initial tension (tuned out with the anchor screw)
OD_SPRING = 15.0
SPRING_F_SAFE = 63.0        # N, ~45 % of Rm shear stress (d 1.4, D 13.6, Wahl 1.15): max working load

X_S = 85.0                  # cord planes x = +-X_S (beam +-50, S1 base +-49.6)
B_Z = 6.0                   # cord pin height in the spacer frame (link z = 0 on the S1 rotor plate)
YOKE_TIP_H = 12.0           # yoke tip height (S2 side sloped away for full abduction)
B_DIST = A.S1.out_z + B_Z                                   # b = 81 mm
A_DIST = round(K_DESIGN * 1000 / (2 * K_SPRING * B_DIST))   # a = 113 mm
R_SHEAVE = 10.0             # cord centre radius on the sheave
SHEAVE_F = 1.2              # sheave flange rim
SHEAVE_W = 2 * SHEAVE_F + 3.0 * (0.9 + 1.1) + 0.4   # 8.8 mm: two 623 (8 mm) side by side inside
SHEAVE_DY = R_SHEAVE        # sheave centre behind the S1 axis: the hanging cord passes over the axis
Z_S1 = A.S1.in_z            # world z of the S1 axis (beam floor underside at 0)
C_Y = T.S1_Y + SHEAVE_DY    # sheave centre (world)
C_Z = Z_S1 + A_DIST
Z_LINE = C_Z + R_SHEAVE     # height of the cord from the sheave top to the spring
SLAB = SHEAVE_W / 2 + 1.5   # half width of the cord / sheave slot in the mast
Y_FAIRLEAD = -55.0          # back face of the mast
Y_EYE = 285.0               # spring anchor eye (M5 eye bolt) nominal position, +-20 adjust
POST_Y = (290.0, 312.0)     # anchor post column


def side_x(sx):
    """World x of the cord plane on side sx (+1 / -1)."""
    return sx * X_S


# ------------------------------------------------------------------ cord / spring model
def pins_world(q):
    """World positions of the two cord pins B (sorted by world x: -X_S, +X_S)."""
    fr = {n: mv for n, _, _, mv in A.chain_frames(q)}
    M = fr[A.BALANCE_YOKE_ON]
    pts = [(M @ np.array([sx * X_S, 0, B_Z, 1.0]))[:3] for sx in (1, -1)]
    return sorted(pts, key=lambda p: p[0])


def sheave_centre(sx):
    return np.array([side_x(sx), C_Y, C_Z])


def cord_geometry(Bw, sx):
    """Tangent point P on the sheave, unit cord direction at B (towards P) and the
    cord length s from B over the sheave to its top (where it leaves backwards)."""
    c = sheave_centre(sx)
    u = np.array([Bw[1] - c[1], Bw[2] - c[2]])          # (y, z) of B relative to the centre
    dist = np.hypot(*u)
    r = R_SHEAVE
    base = math.atan2(u[1], u[0])
    half = math.acos(r / dist)
    best = None
    for ang in (base + half, base - half):
        p = np.array([r * math.cos(ang), r * math.sin(ang)])
        tan_cw = np.array([math.sin(ang), -math.cos(ang)])   # clockwise travel (over the top to +y)
        if (p - u) @ tan_cw > 0:
            best = (ang, p)
    ang, p = best
    wrap = (ang - math.pi / 2) % (2 * math.pi)          # clockwise from P to the top
    P_ = np.array([side_x(sx), c[1] + p[0], c[2] + p[1]])
    d = P_ - Bw
    L = np.linalg.norm(d)
    return P_, d / L, L + r * wrap


S0_DESIGN = R_SHEAVE * math.pi / 2   # s at the virtual zero-length point (cord over a quarter of the sheave)


def cord_tension(s, s0=S0_DESIGN, k=K_SPRING):
    """Spring pull (N) on the cord for cord length s (mm): zero-length law k*(s - s0)."""
    return max(0.0, k * (s - s0))


def spring_torque(q, s0=S0_DESIGN, k=K_SPRING):
    """Moment (N*m) of both balance cords about S1 (sign as gravity_torque)."""
    fr = {n: (b, mv) for n, _, b, mv in A.chain_frames(q)}
    o, ax = joint_axis(fr, "S1")
    tau = 0.0
    for Bw in pins_world(q):
        sx = 1 if Bw[0] > 0 else -1
        _, u, s = cord_geometry(Bw, sx)
        F = cord_tension(s, s0, k) * u
        tau += np.cross(Bw - o, F) @ ax / 1000.0
    return float(tau)


def ideal_torque(q):
    """-k*a*b*sin(S1), both sides: the textbook zero-length balance (sign as gravity_torque)."""
    return 2 * K_SPRING * A_DIST * B_DIST / 1000.0 * math.sin(math.radians(q.get("S1", 0.0)))


def spring_state(q, s0=S0_DESIGN):
    """Per side (sorted by world x): (cord tension N, spring extension mm, spring length mm, front hook y)."""
    out = []
    for Bw in pins_world(q):
        sx = 1 if Bw[0] > 0 else -1
        _, _, s = cord_geometry(Bw, sx)
        t = cord_tension(s, s0)
        e = max(0.0, (t - FI_SPRING) / K_SPRING)
        L = L0_SPRING + e
        out.append((t, e, L, Y_EYE - L))
    return out


def cord_length():
    """Dyneema length per side, pin loop to spring hook (+ 2 x 60 mm for the knots)."""
    Bw = pins_world({})[1]
    _, _, s = cord_geometry(Bw, 1)
    y_hook = spring_state({})[1][3]
    return s + (y_hook - C_Y) + 120.0


# ------------------------------------------------------------------ printed parts
def _teardrop_y(r, y0, y1, x, z):
    """Hole along Y with a 50 deg roof towards +Z (horizontal hole in a part printed z-up)."""
    c, s = math.cos(math.radians(50)), math.sin(math.radians(50))
    tri = (cq.Workplane("XZ").workplane(offset=-y1)
           .polyline([(x + r * s, z + r * c), (x, z + r / c), (x - r * s, z + r * c), (x, z)]).close()
           .extrude(y1 - y0))
    return cyl_y(r, y0, y1).translate((x, 0, z)).union(tri)


def mast(sx=1):
    """Pulley mast for side sx, in WORLD coordinates. Hooks over the beam side wall
    (top cap + inner lip + front lip around the beam's front end), 2 M4 bolts through
    the wall, carries the sheave on an M3 axle between two cheeks right above the S1
    axis and a fairlead at its back face. Prints FRONT FACE DOWN (y up)."""
    x_w = T.BEAM_X
    y_f = T.BEAM_Y[0] - 6.0                 # front lip
    z_lo = min(20.0, C_Z - 24.0)
    z_hi = max(T.BEAM_H + 10.0, Z_LINE + 8.0)
    x_out = X_S + SLAB + 5.0
    body = box(x_w - 10, x_w + 7, y_f, Y_FAIRLEAD, z_lo, T.BEAM_H + 6)
    body = body.union(box(x_w + 7, x_out, y_f, Y_FAIRLEAD, z_lo, z_hi))
    body = body.cut(box(-x_w - 1, x_w, T.BEAM_Y[0], Y_FAIRLEAD + 1, -1, T.BEAM_H))      # the beam
    body = body.union(box(x_w - 10, x_w - T.WALL, T.BEAM_Y[0] + T.WALL, Y_FAIRLEAD, T.BEAM_H - 12, T.BEAM_H))
    x0, x1 = X_S - SLAB, X_S + SLAB
    cuts = []
    # cord coming up from the yoke (in front of the sheave centre): tent roof (y up in print)
    roof = (cq.Workplane("XY").polyline([(x0, y_f - 1), (x1, y_f - 1), (x1, C_Y), (X_S, C_Y + SLAB * 1.1),
                                         (x0, C_Y)]).close().extrude(C_Z - z_lo + 1).translate((0, 0, z_lo - 1)))
    cuts.append(roof)
    # sheave + wrap: everything above the axis in the slot, teardrop below it
    cuts.append(box(x0, x1, y_f - 1, Y_FAIRLEAD + 1, C_Z, z_hi + 1))
    rc = R_SHEAVE + 5.0
    c50, s50 = math.cos(math.radians(50)), math.sin(math.radians(50))
    td = (cq.Workplane("YZ").workplane(offset=x0)
          .polyline([(C_Y, C_Z), (C_Y + rc * c50, C_Z - rc * s50), (C_Y + rc / c50, C_Z),
                     (C_Y + rc * c50, C_Z + rc * s50)]).close().extrude(2 * SLAB))
    cuts.append(cyl_x(rc, x0, x1).translate((0, C_Y, C_Z)).union(td))
    body = cut_all(body, cuts)
    # fairlead: bridges the cheeks at the back, cord hole along Y
    fl = box(x0 - 0.1, x1 + 0.1, Y_FAIRLEAD - 8, Y_FAIRLEAD, Z_LINE - 6, z_hi)
    fl = fl.cut(cyl_y(2.0, Y_FAIRLEAD - 9, Y_FAIRLEAD + 1).translate((X_S, 0, Z_LINE)))
    body = body.union(fl)
    # axle (M3, self-taps into the outer cheek), wall bolts (M4, nut inside the channel)
    body = body.cut(cyl_x(1.7, x0 - 6, x1 + 6).translate((0, C_Y, C_Z)))
    for y in (-108.0, -72.0):
        body = body.cut(cyl_x(P.M4_CLR / 2, x_w - 1, x_w + 8).translate((0, y, (z_lo + T.BEAM_H) / 2 - 2)))
    if sx < 0:
        body = body.mirror("YZ")
    return body


def sheave():
    """Sheave on two 623 bearings, axis Z, prints flat. Cord centre radius R_SHEAVE;
    the groove's upper flank (its ceiling when printed) is a 48 deg cone."""
    rb = P.B623[1] / 2 + P.FIT
    ro = R_SHEAVE + 2.5
    rg = R_SHEAVE - P.CORD_D / 2
    f = SHEAVE_F
    z1 = f + (ro - rg) * 0.9                  # lower flank faces up: any slope
    z2 = z1 + 0.4
    z3 = z2 + (ro - rg) * 1.1                 # upper flank (groove ceiling): 48 deg
    pts = [(rb, 0), (ro, 0), (ro, f), (rg, z1), (rg, z2), (ro, z3), (ro, z3 + f), (rb, z3 + f)]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))


def post():
    """Spring anchor post (WORLD, side +X; mirrored for -X, it is symmetric so one
    STL serves both): foot with 4 wood screws, column with a horizontal M5 hole at
    the cord height for the eye bolt (2 nuts: +-20 mm tuning), gusset behind.
    Prints foot down."""
    y0, y1 = POST_Y
    foot = box(X_S - 20, X_S + 20, y0 - 10, y1 + 30, 0, 6).edges("|Z").fillet(5)
    col = box(X_S - 10, X_S + 10, y0, y1, 0, Z_LINE + 12)
    rib = (cq.Workplane("YZ").workplane(offset=X_S - 4)
           .polyline([(y1 - 1, 5), (y1 + 28, 5), (y1 - 1, Z_LINE + 4)]).close().extrude(8))
    body = foot.union(col).union(rib)
    body = body.cut(_teardrop_y(2.8, y0 - 1, y1 + 1, X_S, Z_LINE))
    for dx in (-13.0, 13.0):
        for y in (y0 - 4, y1 + 24):
            body = body.cut(cyl_z(2.4, -1, 7).translate((X_S + dx, y, 0)))
    return body


def printed_parts():
    """name -> (workplane in PRINT orientation, qty, note) for export.py."""
    return {
        "balance_mast_r": (mast(1).rotate((0, 0, 0), (1, 0, 0), 90), 1,
                           "balance pulley mast, +X side - front face down (tent roof over the cord slot)"),
        "balance_mast_l": (mast(-1).rotate((0, 0, 0), (1, 0, 0), 90), 1,
                           "balance pulley mast, -X side (mirror) - front face down"),
        "balance_sheave": (sheave(), 2, "balance sheave on 2x 623 - flat (48 deg groove ceiling)"),
        "balance_post": (post(), 2, "balance spring anchor post - foot down (M5 hole is a teardrop)"),
    }


def posed_parts(q=None, cords=True):
    """{name: workplane in WORLD} of the balance hardware for joint angles q: masts,
    sheaves, posts, springs (as cylinders) and cords. (The yoke is part of the arm:
    link_s1_s2.)"""
    q = q or {}
    out = {}
    for sx, tag in ((1, "r"), (-1, "l")):
        out[f"balance_mast_{tag}"] = mast(sx)
        c = sheave_centre(sx)
        out[f"balance_sheave_{tag}"] = (sheave().translate((0, 0, -SHEAVE_W / 2))
                                        .rotate((0, 0, 0), (0, 1, 0), 90).translate(tuple(c)))
        out[f"balance_post_{tag}"] = post() if sx > 0 else post().mirror("YZ")
    for Bw, st in zip(pins_world(q), spring_state(q)):
        sx = 1 if Bw[0] > 0 else -1
        tag = "r" if sx > 0 else "l"
        y_hook = st[3]
        out[f"balance_spring_{tag}"] = cyl_y(OD_SPRING / 2, y_hook, Y_EYE).translate((side_x(sx), 0, Z_LINE))
        if cords:
            P_, _, _ = cord_geometry(Bw, sx)
            out[f"balance_cord_{tag}"] = rod(tuple(Bw), tuple(P_), P.CORD_D)
            out[f"balance_cordback_{tag}"] = rod((side_x(sx), Y_FAIRLEAD + 1, Z_LINE),
                                                 (side_x(sx), y_hook, Z_LINE), P.CORD_D)
    return out


# ------------------------------------------------------------------ report
def main():
    m, d, mgd = mass_properties()
    mp, dp, mgdp = mass_properties(500.0)
    pm = [r for r in _printed_masses() if not (r[0] == "S1" and r[1] == "base")]
    hw = [r for r in _hardware_masses() if not (r[0] == "S1" and r[1] == "base")]
    vol = 0.0
    for n, s, _ in A.CHAIN:
        st, mv = A.element_parts(n, s)
        for k, w in list(mv.items()) + ([] if n == "S1" else list(st.items())):
            vol += w.val().Volume() / 1000
    mp_g = sum(r[3] for r in pm)
    print("== mass model (everything that moves with S1) ==")
    print(f"  printed PETG: {vol:.0f} cm^3 solid -> {mp_g:.0f} g (effective fill {mp_g / (vol * 1000 * PETG_RHO):.2f}:"
          f" {SHELL_T} mm solid skin + {INFILL:.0%} infill, per part)")
    print(f"  hardware (bearings, axles, inserts, bolts, tubes): {sum(r[3] for r in hw):.0f} g")
    print(f"  total m = {m:.2f} kg, COM d = {d * 1000:.0f} mm below S1 (q = 0), m*g*d = {mgd:.2f} N*m")
    print(f"  with 0.5 kg at the tool: m = {mp:.2f} kg, d = {dp * 1000:.0f} mm, m*g*d = {mgdp:.2f} N*m")
    print("  effective d vs elbow (S1 = 90): " + ", ".join(f"E{e}: {v * 1000:.0f} mm" for e, v in d_vs_elbow().items()))
    print("== balance design ==")
    print(f"  K = k*a*b (both sides) = {2 * K_SPRING * A_DIST * B_DIST / 1000:.2f} N*m (target {K_DESIGN})")
    print(f"  a = {A_DIST} mm (sheave above S1), b = {B_DIST:.0f} mm (pin below S1), X_S = +-{X_S:.0f} mm")
    print(f"  spring per side: k = {K_SPRING} N/mm, L0 = {L0_SPRING:.0f} mm, OD {OD_SPRING:.0f}, 1:1 (no tackle)")
    for q in ({"S1": -30}, {}, {"S1": 60}, {"S1": 120}):
        t, e, L, yh = spring_state(q)[1]
        print(f"   S1 {q.get('S1', 0):+4.0f}: cord {t:5.1f} N, extension {e:5.1f} mm, spring length {L:5.1f} mm,"
              f" hook at y {yh:5.0f}")
    print(f"  Dyneema per side (pin loop -> spring hook, incl. knots): {cord_length():.0f} mm")


if __name__ == "__main__":
    main()


