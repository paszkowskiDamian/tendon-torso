"""Two-finger gripper on the W2 rotor plate: one pull cord closes, one spring opens.

Gripper frame G: origin on the top face of the W2 rotor plate (centre), +Z away from
the wrist; the fingers swing in the XZ plane about pivots along Y at x = -+PX, z = ZP.
Finger A is the -X finger, finger B the +X finger.

Mechanism
---------
* PALM: a disc that bolts on the W2 rotor plate like a link lip (4x M3x10, heads on
  top, into W2's heat-set inserts, flange class "S"). Two cheeks carry the finger
  pivots (M3x25 bolts + nyloc nuts). A boss in the centre is the Bowden socket: the
  gripper tube comes up through W2's open centre and ends in it.
* FINGERS: angular jaws, synchronised by two gear sectors (module 1.25, pitch radius
  PX = 20 mm, so the pitch circles touch on the gripper axis). Each finger has three
  layers in Y:
    y in GEAR   gear sector (both fingers)
    y in DRUM   finger A only: a cord drum, radius R_C = PX, so the cord coming
                straight up the axis from the socket meets it at its tangent point
    y in SPRING the opening spring between the two finger tails
  Pulling the cord turns finger A closed; the gears close finger B by the same angle.
* SPRING: one extension spring between two posts on the finger tails (below the
  pivots, spring layer, it passes under the hubs). It pulls the tails together, which
  opens both fingers. Open stops on the palm (under the tails) end the travel at
  THETA_MAX before the gears run out of teeth.
* PADS: slide-on sleeves over the straight finger ends, one M3x6 (self-tapping into the
  finger) each. Print them in TPU for grip, or in PETG.

Statics (per pad, normal force): the cord pulls finger A with T * R_C, the gears pass
half of it to finger B, the pad normal force acts at L_PAD (constant in the finger frame,
the pad face is a plane of the finger):

    F_pad = (T * R_C - 2 * tau_spring(theta)) / (2 * L_PAD)

Print orientations (export.py): palm plate down; both fingers gear layer down (finger
A's drum layer sits fully on its gear sector, finger B has nothing above its gear but
hub/arm/tail); pads standing on their open end.
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import cadquery as cq

from . import params as P
from .cad import box, cut_all, cyl_y, cyl_z, polar, rod

# ------------------------------------------------------------------ main dimensions
PX = 20.0                     # pivot half spacing = gear pitch radius = cord drum radius
ZP = 30.0                     # pivot height above the W2 plate
R_C = PX                      # cord centre radius on finger A's drum (cord on the axis)
MOD, Z_TEETH = 1.25, 32       # gear module, tooth count of the full gear (r_p = 20)
PRESSURE = 20.0
BACKLASH = 0.15               # total, at the pitch circle
THETA_MAX = 41.0              # opening angle per finger at full opening (deg)
ANCHOR = 22.0                 # cord anchor, local angle on the drum (wrap at closed)

# Y layers of the fingers (finger y in [-FY, FY]); cheeks start at +-CHEEK_IN
FY = 7.0
GEAR = (-FY, -2.2)
DRUM = (-2.2, 2.2)
SPRING = (2.2, FY)
CHEEK_IN, CHEEK_T = FY + 0.3, 3.5
GROOVE_W = 1.6

PALM_T = 5.0
PALM_R = 31.0                 # disc radius (bolt circle 27 + head)
BOSS_R, BOSS_TOP = 4.2, 10.5  # Bowden socket boss
SOCKET_DEPTH = 8.0            # tube socket from the palm underside

HUB_R = 5.5
BAR = (-9.5, -3.0)            # x range of finger A's straight end (pad mount) at closed
BAR_Z = (ZP + 20.0, ZP + 56.0)
PAD_T, PAD_WALL, PAD_CAP = 2.5, 1.6, 1.6
PAD_Z0 = ZP + 24.0            # lower end of the pad sleeve
L_PAD = (PAD_Z0 + BAR_Z[1]) / 2 - ZP   # pad centre above the pivot (38 mm)
PAD_SCREW_Z = ZP + 46.0

TAIL = (-9.0, -9.0)           # spring post of finger A, relative to its pivot (closed)
TAIL_R, POST_R, CAP_R = 3.5, 1.5, 2.3

# opening spring (catalogue extension spring), between the two posts
SPRING_K = 0.20               # N/mm
SPRING_L0 = 34.0              # mm inside the hooks
SPRING_FI = 0.5               # N initial tension
SPRING_OD, SPRING_WIRE = 4.0, 0.5

GRASP_Z = ZP + L_PAD + 2.0    # tool point (kinematics.TOOL_OFFSET): pad centre at mid opening


@dataclass
class GripperSpec:
    """Chain element at the end of the arm (in = out = 0: the tool frame is the W2 plate)."""
    name: str = "gripper"
    cls: str = "S"
    R: float = R_C              # cord drum radius (crank ratio = R / spool radius)
    lo: float = 0.0             # finger opening angle range (deg)
    hi: float = THETA_MAX
    tubes_through: int = 0
    in_z: float = 0.0
    out_z: float = 0.0


GRIPPER = GripperSpec()


# ------------------------------------------------------------------ helpers
def prism_y(pts, y0, y1):
    """Polygon (x, z) extruded along Y from y0 to y1."""
    return cq.Workplane("XZ").workplane(offset=-y1).polyline(pts).close().extrude(y1 - y0)


def disc_y(cx, cz, r, y0, y1):
    return cyl_y(r, y0, y1).translate((cx, 0, cz))


def arc_pts(cx, cz, r, a0, a1, n=None):
    n = n or max(4, int(abs(a1 - a0) / 3) + 1)
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / (n - 1))),
             cz + r * math.sin(math.radians(a0 + (a1 - a0) * i / (n - 1)))) for i in range(n)]


def sector(cx, cz, r, a0, a1):
    return [(cx, cz)] + arc_pts(cx, cz, r, a0, a1)


def _inv(a):
    return math.tan(a) - a


def gear_pts(cx, cz, centres, a0, a1):
    """Gear sector outline (x, z): hub centre -> root arc from a0, involute teeth centred
    at the angles in `centres` (deg), root arc to a1 -> back to the centre."""
    rp = MOD * Z_TEETH / 2
    rb = rp * math.cos(math.radians(PRESSURE))
    rf, ra = rp - 1.25 * MOD, rp + MOD
    half_p = math.pi / (2 * Z_TEETH) - BACKLASH / 2 / (2 * rp) + _inv(math.radians(PRESSURE))

    def psi(r):
        return half_p - _inv(math.acos(min(1.0, rb / r)))

    rs = [rb + (ra - rb) * i / 6 for i in range(7)]
    pts = [(cx, cz), (cx + rf * math.cos(math.radians(a0)), cz + rf * math.sin(math.radians(a0)))]
    for c in sorted(centres):
        c = math.radians(c)
        right = [(rf, c - psi(rb))] + [(r, c - psi(r)) for r in rs]
        left = [(r, c + psi(r)) for r in reversed(rs)] + [(rf, c + psi(rb))]
        for r, a in right + left:
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    pts.append((cx + rf * math.cos(math.radians(a1)), cz + rf * math.sin(math.radians(a1))))
    return pts


PITCH_DEG = 360.0 / Z_TEETH


# ------------------------------------------------------------------ finger geometry (closed, in G)
def _arm_pts():
    """Finger A's arm from the hub to the straight pad bar (all layers)."""
    cx = -PX
    return [(cx - 5.3, ZP + 1.2), (BAR[0], BAR_Z[0]), (BAR[0], BAR_Z[1] - 2.0), (BAR[0] + 2.0, BAR_Z[1]),
            (BAR[1], BAR_Z[1]), (BAR[1], BAR_Z[0]), (-7.0, ZP + 9.0), (cx + 5.5, ZP + 3.5)]


def _tail(sx):
    """Tail lug (gear + drum layers) and spring post (spring layer) of finger A (sx=-1) / B (+1)."""
    cx = sx * PX
    tx, tz = cx + (TAIL[0] if sx < 0 else -TAIL[0]), ZP + TAIL[1]   # B mirrors A
    d = math.hypot(tx - cx, tz - ZP)
    nx, nz = -(tz - ZP) / d, (tx - cx) / d                # unit normal of the hub -> post line
    w = TAIL_R
    quad = [(cx + nx * w, ZP + nz * w), (tx + nx * w, tz + nz * w), (tx - nx * w, tz - nz * w),
            (cx - nx * w, ZP - nz * w)]
    lug = prism_y(quad, GEAR[0], DRUM[1]).union(disc_y(tx, tz, TAIL_R, GEAR[0], DRUM[1]))
    post = disc_y(tx, tz, POST_R, DRUM[1] - 0.1, FY - 1.4).union(disc_y(tx, tz, CAP_R, FY - 1.4, FY))
    return lug.union(post), (tx, tz)


def _mirror_pts(pts):
    return [(-x, z) for x, z in pts]


def finger(which="A"):
    """Finger A (-X, with the cord drum) or B (+X), closed, in the gripper frame."""
    sx = -1 if which == "A" else 1
    cx = sx * PX
    arm = _arm_pts() if which == "A" else _mirror_pts(_arm_pts())
    body = disc_y(cx, ZP, HUB_R, -FY, FY).union(prism_y(arm, -FY, FY))
    lug, _ = _tail(sx)
    body = body.union(lug)
    if which == "A":
        teeth = [k * PITCH_DEG for k in range(-5, 3)]
        body = body.union(prism_y(gear_pts(cx, ZP, teeth, -58.0, 30.0), GEAR[0], GEAR[1]))
        body = body.union(prism_y(sector(cx, ZP, R_C + 1.5, -52.0, 30.0), DRUM[0], DRUM[1]))
    else:
        # B mirrors A, its teeth shifted by half a pitch so they fill A's gaps on the axis
        teeth = [180.0 + PITCH_DEG / 2 + k * PITCH_DEG for k in range(-2, 5)]
        body = body.union(prism_y(gear_pts(cx, ZP, teeth, 150.0, 238.0), GEAR[0], GEAR[1]))
    cuts = [disc_y(cx, ZP, P.M3_CLR / 2, -FY - 1, FY + 1)]
    if which == "A":
        # cord groove in the drum rim
        ring = (prism_y(sector(cx, ZP, R_C + 4, -60.0, 40.0), -GROOVE_W / 2, GROOVE_W / 2)
                .cut(disc_y(cx, ZP, R_C - P.CORD_D / 2, -2, 2)))
        cuts.append(ring)
        # cord anchor: radial hole from the groove bottom into a knot pocket open to +Y
        a = math.radians(ANCHOR)
        rk = 12.5                                     # knot pocket centre radius
        p0 = (cx + (R_C + 0.3) * math.cos(a), 0.0, ZP + (R_C + 0.3) * math.sin(a))
        p1 = (cx + (rk - 1.0) * math.cos(a), 0.0, ZP + (rk - 1.0) * math.sin(a))
        cuts.append(rod(p0, p1, P.CORD_D + 0.6))
        cuts.append(disc_y(cx + rk * math.cos(a), ZP + rk * math.sin(a), 2.3, -1.2, DRUM[1] + 1))
    # pilot hole for the pad screw (M3 self-tapping), from the outer face of the bar
    x0, x1 = BAR[0] - 1.0, BAR[1] - 1.5             # blind, 1.5 mm short of the pad face
    if which == "B":
        x0, x1 = -x1, -x0
    cuts.append(cq.Workplane("YZ").workplane(offset=x0).center(0, PAD_SCREW_Z).circle(1.3).extrude(x1 - x0))
    return cut_all(body, cuts)


def pad(which="A"):
    """Slide-on pad sleeve over the finger end (closed position, in G). Open end down."""
    sx = -1 if which == "A" else 1
    c = 0.15
    xi, xo = BAR[1] + c, BAR[0] - c                 # cavity (finger A side)
    x0, x1 = xo - PAD_WALL, xi + PAD_T
    y0, y1 = -FY - c - PAD_WALL, FY + c + PAD_WALL
    z0, z1 = PAD_Z0, BAR_Z[1] + c + PAD_CAP
    body = box(x0, x1, y0, y1, z0, z1)
    cav = box(xo, xi, -FY - c, FY + c, z0 - 1, BAR_Z[1] + c)
    cuts = [cav]
    # three 45 deg V grooves across the face (grip, and they print as tiny overhangs)
    for zg in (z0 + 8, (z0 + z1) / 2, z1 - 8):
        v = [(x1 + 0.01, zg - 0.7), (x1 - 0.6, zg), (x1 + 0.01, zg + 0.7)]
        cuts.append(prism_y(v, y0 - 1, y1 + 1))
    cuts.append(cq.Workplane("YZ").workplane(offset=x0 - 1).center(0, PAD_SCREW_Z).circle(P.M3_CLR / 2)
                .extrude(PAD_WALL + 2))
    body = cut_all(body, cuts)
    return body if sx < 0 else body.mirror("YZ")


def _tail_low(theta):
    """Lowest z of finger A's tail lug at opening theta, and its x there."""
    ang = math.radians(theta)
    tx, tz = TAIL
    x = tx * math.cos(ang) - tz * math.sin(ang)      # CCW in (x, z) = -theta about +Y (opening of A)
    z = tx * math.sin(ang) + tz * math.cos(ang)
    return -PX + x, ZP + z - TAIL_R


def palm():
    F = P.FLANGES["S"]
    plate = cyl_z(PALM_R, 0, PALM_T)
    boss = cyl_z(BOSS_R, PALM_T - 0.5, BOSS_TOP)
    body = plate.union(boss)
    # cheeks: a low wall + a pillar round each pivot
    for sy in (1, -1):
        y0, y1 = sorted((sy * CHEEK_IN, sy * (CHEEK_IN + CHEEK_T)))
        pts = ([(-PX - 7.0, PALM_T - 0.5), (PX + 7.0, PALM_T - 0.5), (PX + 7.0, ZP)] + arc_pts(PX, ZP, 7.0, 0, 180)[1:-1]
               + [(PX - 7.0, ZP - 3.0), (PX - 7.0, 17.0), (-PX + 7.0, 17.0), (-PX + 7.0, ZP - 3.0)]
               + arc_pts(-PX, ZP, 7.0, 0, 180)[1:-1] + [(-PX - 7.0, ZP)])
        body = body.union(prism_y(pts, y0, y1))
    # open stops under the finger tails (gear + drum layers, clear of the spring)
    xs, zs = _tail_low(THETA_MAX)
    for sx in (1, -1):
        x = sx * -xs
        body = body.union(box(x - 3.0, x + 3.0, GEAR[0], DRUM[1] - 0.5, PALM_T - 0.5, zs))
    cuts = [cyl_z(P.TUBE_OD / 2 + 0.15, -1, SOCKET_DEPTH),                       # tube socket
            cyl_z(P.TUBE_ID / 2 + 0.2, SOCKET_DEPTH - 0.5, BOSS_TOP + 1),          # cord
            cq.Workplane("XY").workplane(offset=-0.01).circle(P.TUBE_OD / 2 + 0.9)   # entry chamfer
            .workplane(offset=0.75).circle(P.TUBE_OD / 2 + 0.15).loft()]
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        cuts.append(cyl_z(P.M3_CLR / 2, -1, PALM_T + 1).translate((x, y, 0)))
    for sx in (1, -1):
        cuts.append(disc_y(sx * PX, ZP, P.M3_CLR / 2, -CHEEK_IN - CHEEK_T - 1, CHEEK_IN + CHEEK_T + 1))
    # lighten the plate outside the cheeks (between the bolt heads)
    for sy in (1, -1):
        y0 = CHEEK_IN + CHEEK_T + 2.0
        win = cyl_z(7.0, -1, PALM_T + 1).translate((0, sy * (y0 + 7.0), 0))
        cuts.append(win)
    return cut_all(body, cuts)


# ------------------------------------------------------------------ kinematics of the fingers
def pivot(which):
    return (-PX if which == "A" else PX, 0.0, ZP)


def finger_rot(which, theta):
    """Rotation (deg about +Y through the pivot) of a finger at opening theta."""
    return -theta if which == "A" else theta


def opening(theta):
    """Distance between the two pad centres (mm) at opening theta (deg)."""
    a = math.radians(theta)
    rx, rz = PX, L_PAD                       # pad-face centre relative to pivot A (closed)
    x = -PX + rx * math.cos(a) - rz * math.sin(a)
    return -2 * x


def theta_for(w):
    """Opening angle for a pad-centre distance w (mm)."""
    lo, hi = 0.0, THETA_MAX
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if opening(m) < w else (lo, m)
    return (lo + hi) / 2


def spring_state(theta):
    """(length between post centres, force N, torque N*mm on each finger)."""
    ang = math.radians(theta)
    tx, tz = TAIL
    x = -PX + tx * math.cos(ang) - tz * math.sin(ang)
    z = ZP + tx * math.sin(ang) + tz * math.cos(ang)
    L = -2 * x
    ext = max(0.0, L - 2 * POST_R - SPRING_L0)
    F = SPRING_FI + SPRING_K * ext
    return L, F, F * (ZP - z)


def pad_force(T, theta):
    """Normal force (N) on each pad for a cord tension T (N) at opening theta."""
    return max(0.0, (T * R_C - 2 * spring_state(theta)[2]) / (2 * L_PAD))


def cord_for_force(F, theta):
    return (2 * L_PAD * F + 2 * spring_state(theta)[2]) / R_C


def cord_stroke():
    return R_C * math.radians(THETA_MAX)


def _rot_y(wp, which, theta):
    px, _, pz = pivot(which)
    return wp.rotate((px, 0, pz), (px, 1, pz), finger_rot(which, theta))


@lru_cache(maxsize=None)
def _parts_closed():
    return {"gripper_palm": palm(), "gripper_finger_a": finger("A"), "gripper_finger_b": finger("B"),
            "gripper_pad_a": pad("A"), "gripper_pad_b": pad("B")}


def parts(theta=THETA_MAX):
    """{name: workplane in G} with both fingers (and pads) at opening theta."""
    p = _parts_closed()
    out = {"gripper_palm": p["gripper_palm"]}
    for w in ("a", "b"):
        W = w.upper()
        out[f"gripper_finger_{w}"] = _rot_y(p[f"gripper_finger_{w}"], W, theta)
        out[f"gripper_pad_{w}"] = _rot_y(p[f"gripper_pad_{w}"], W, theta)
    return out


def printed_parts():
    """name -> (workplane in PRINT orientation, qty, note) for export.py."""
    rx = lambda w: w.rotate((0, 0, 0), (1, 0, 0), 90)    # y -> z: the -Y (gear) face goes down
    return {
        "gripper_palm": (palm(), 1, "gripper palm (4x M3x10 into W2, 2x M3x25 finger pivots, Bowden socket) - plate down"),
        "gripper_finger_a": (rx(finger("A")), 1, "gripper finger A (cord drum + gear) - gear side down"),
        "gripper_finger_b": (rx(finger("B")), 1, "gripper finger B (gear) - gear side down"),
        "gripper_pad": (pad("A"), 2, "gripper pad sleeve (TPU 95A or PETG), 1x M3x6 each - open end down"),
    }


def report():
    print("== gripper ==")
    print(f"  pivots x = +-{PX:.0f}, z = {ZP:.0f}; gear m {MOD} (r_p {MOD * Z_TEETH / 2:.0f}); drum R {R_C:.0f}; pad centre {L_PAD:.0f} mm above the pivot")
    print(f"  opening (pad centres) 0 .. {opening(THETA_MAX):.0f} mm at {THETA_MAX:.0f} deg per finger; cord stroke {cord_stroke():.1f} mm")
    for th in (0, 15, 30, THETA_MAX):
        L, F, t = spring_state(th)
        print(f"  theta {th:4.0f}: opening {opening(th):5.1f} mm, spring {L:5.1f} mm / {F:4.2f} N (pull-back on the cord"
              f" {2 * t / R_C:4.2f} N), cord for 10 N per pad {cord_for_force(10, th):5.1f} N, pad force at 50 N cord {pad_force(50, th):5.1f} N")


if __name__ == "__main__":
    report()
