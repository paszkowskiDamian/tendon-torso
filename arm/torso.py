"""Two-arm torso: a drive column between two shoulders.

TORSO frame (the world of the two-arm robot): origin on the work surface (table
top) on the torso mid-plane, +X = the robot's left, -Y = forwards, +Z = up. The
right arm is the original arm: its chain lives in its own BEAM frame (the old
single-arm world: origin on the underside of the shoulder-beam floor, S1 axis at
(0, S1_Y, -69)), which sits at MOUNT_R in the torso frame. The left arm is its
mirror image in the plane x = 0.

    S1 axes: x = +-SHOULDER_X (350 mm apart), y = 0, z = S1_Z (650 above the table)

Structure (aluminium extrusion + printed parts):

* SPINE: two vertical 2040 posts behind the shoulder line (y 110..130), on two 2040
  base rails along Y that are screwed to the table behind the work area. Nothing of
  the torso is in front of y = 100 below the shoulders.
* SHOULDER DECK: a front and a rear 2020 cross rail (along X, bolted to the spine)
  carrying four 2020 deck rails (along Y) under the balance-cord planes x = arm +-85.
* SHOULDER BEAM (printed, one per arm): the old U-channel shortened to its front
  part, with two wings that bolt onto the deck rails. S1 still bolts under its
  floor, the balance masts still hook over its walls (balance.py unchanged), the
  spring anchor posts now stand on the deck rails.
* DRIVE COLUMN: 8 levels of printed SADDLES bolted to the front of the spine posts.
  Each saddle carries one drive unit per arm (drive.py), back to back, motors to
  the rear. The units lean DRIVE_TILT outwards-down, so every tube leaves its unit
  heading towards its shoulder and slightly down.
* SHOULDER COMB (printed, one per arm): holds the 13 pass-through tubes in a 5 x 3
  grid right above the S1 floor opening; from there they drop straight into S1.
* ELECTRONICS TRAY on the back of the spine (controller, servo bus board, 12 V
  input), E-STOP box on the rear cross rail.
"""
import math

import cadquery as cq
import numpy as np

from . import drive as DR
from . import params as P
from .cad import box, cut_all, cyl_x, cyl_y, cyl_z

# ------------------------------------------------------------------ layout
SHOULDER_X = 175.0          # S1 axes at x = +-175
S1_Z = 650.0                # S1 axes above the work surface
S1_IN_Z = 69.0              # S1 axis -> underside of the shoulder-beam floor (= -S1.in_z)
MOUNT_Z = S1_Z + S1_IN_Z    # 719: underside of the beam floor = top of the deck rails
S1_Y = -95.0                # S1 axis in the BEAM frame (unchanged from the single-arm beam)
MOUNT_Y = -S1_Y             # beam frame -> torso frame: S1 axes on y = 0


def mount_matrix():
    """4x4: right arm's BEAM frame in the TORSO frame (pure translation)."""
    m = np.eye(4)
    m[:3, 3] = (-SHOULDER_X, MOUNT_Y, MOUNT_Z)
    return m


# shoulder beam (BEAM frame, as before: floor underside z = 0, channel along Y, front -Y)
BEAM_X = 50.0               # half width
BEAM_H = 52.0
BEAM_Y = (-125.0, 15.0)     # front end unchanged (masts hook round it), rear end shortened
WALL = 6.0
WING_X = 95.0               # wings out to the deck rails at x = +-85
WING_Y = (-45.0, BEAM_Y[1])
DECK_X = 85.0               # deck rail centre lines (beam frame) = balance cord planes
WING_BOLTS = [(sx * DECK_X, y) for sx in (1, -1) for y in (-32.0, 2.0)]   # M5 into T-nuts

# extrusions (TORSO frame): name -> (x0, x1, y0, y1, z0, z1)
RAIL = 20.0
SPINE_X = (20.0, 60.0)      # 2040 posts, 40 along X, at x = +-(20..60)
SPINE_Y = (110.0, 130.0)
DECK_Y = (MOUNT_Y + WING_Y[0], 455.0)
CROSS_Z = (MOUNT_Z - 2 * RAIL, MOUNT_Z - RAIL)
CROSS_X = 270.0
TOP_Z = 1335.0


def extrusions():
    """{name: (x0, x1, y0, y1, z0, z1, profile)} of every aluminium extrusion (TORSO frame)."""
    e = {}
    for sx, t in ((-1, "r"), (1, "l")):
        x0, x1 = sorted((sx * SPINE_X[0], sx * SPINE_X[1]))
        e[f"base_rail_{t}"] = (x0, x1, SPINE_Y[0], 460.0, 0.0, RAIL, "2040")
        e[f"spine_post_{t}"] = (x0, x1, SPINE_Y[0], SPINE_Y[1], RAIL, TOP_Z, "2040")
        xs = sx * 40.0
        e[f"stringer_{t}"] = (xs - 10, xs + 10, SPINE_Y[1], 400.0, *CROSS_Z, "2020")
        for k, xd in (("in", SHOULDER_X - DECK_X), ("out", SHOULDER_X + DECK_X)):
            xc = sx * xd
            e[f"deck_rail_{t}_{k}"] = (xc - 10, xc + 10, DECK_Y[0], DECK_Y[1], MOUNT_Z - RAIL, MOUNT_Z, "2020")
    e["clavicle"] = (*CLAVICLE, "2040")
    e["cross_front"] = (-CROSS_X, CROSS_X, SPINE_Y[0] - RAIL, SPINE_Y[0], *CROSS_Z, "2020")
    e["cross_rear"] = (-CROSS_X, CROSS_X, 400.0, 420.0, *CROSS_Z, "2020")
    return e


def extrusion_solid(name):
    x0, x1, y0, y1, z0, z1, _ = extrusions()[name]
    return box(x0, x1, y0, y1, z0, z1)


# ------------------------------------------------------------------ drive column
DRIVE_TILT = 12.0           # deg: the units' tube exits point this much below horizontal
FOOT_X = 20.0               # saddle face (unit foot centre) at x = +-20
COLUMN_ORDER = ["S2", "S3", "E", "F", "W1", "W2", "G"]   # bottom -> top; S1 hangs from the clavicle
LEVELS = len(COLUMN_ORDER)
LEVEL_PITCH = 65.0          # vertical pitch (unit half height 31 along its tilted y: 62/cos 12 + gap)
LEVEL_Z0 = 830.0            # lowest unit axis: its outer-lower corner clears the inner balance mast
# S1 drive units: S1's own sockets lean 41 deg fore / aft (hinge.py), so its two tubes must
# come from straight above. Each S1 unit hangs (tubes down) from a printed hanger under the
# CLAVICLE, a 2040 across the top of the spine, outboard of the column's tube curtain.
S1_UNIT_X = 205.0           # S1 unit axis x (+-), outboard of the column units' tubes
HANGER_Z = (1270.0, 1295.0) # hanger plate; the unit foot hangs under it
CLAVICLE = (-262.0, 262.0, SPINE_Y[0] - RAIL, SPINE_Y[0], 1295.0, TOP_Z)


def unit_rotation(side):
    """3x3 rotation of a drive unit (columns = unit X, Y, Z in the TORSO frame).
    Unit X (spool axis, motor) -> +Y (rearwards); unit Z (tubes) -> outwards, DRIVE_TILT down.
    Right and left units are the same parts, the left one turned (not mirrored)."""
    c, s = math.cos(math.radians(DRIVE_TILT)), math.sin(math.radians(DRIVE_TILT))
    sx = -1.0 if side == "R" else 1.0
    lx = np.array([0.0, 1.0, 0.0])
    lz = np.array([sx * c, 0.0, -s])
    ly = np.cross(lz, lx)
    return np.column_stack([lx, ly, lz])


def unit_matrix(side, level):
    """4x4 of the drive unit on a level (0 = bottom) for side 'R' / 'L' (TORSO frame)."""
    R = unit_rotation(side)
    sx = -1.0 if side == "R" else 1.0
    m = np.eye(4)
    m[:3, :3] = R
    # spool axis on the level height, foot centre (local (0, 0, FOOT_Z0)) at x = +-FOOT_X
    m[:3, 3] = (sx * FOOT_X - DR.FOOT_Z[0] * R[0, 2], 0.0, LEVEL_Z0 + level * LEVEL_PITCH)
    return m


def s1_unit_matrix(side):
    """4x4 of a hanging S1 drive unit (TORSO frame): unit Z (tubes) straight down, unit X
    (motor) rearwards, unit Y towards +X; foot underside against the hanger plate."""
    sx = -1.0 if side == "R" else 1.0
    m = np.eye(4)
    m[:3, :3] = np.column_stack([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    m[:3, 3] = (sx * S1_UNIT_X, 0.0, HANGER_Z[0] + DR.FOOT_Z[0])
    return m


def s1_hanger():
    """Printed hanger for the right S1 drive unit (TORSO frame; the left one is its mirror,
    the part is symmetric): a plate under the clavicle that the unit's foot hangs from
    (4 M4, nuts in pockets from the top) + a web against the clavicle's front face
    (2 M5 + T-nuts) and 2 M5 up into its bottom slot. Prints plate down."""
    xc = -S1_UNIT_X
    x0, x1 = xc - 34.0, xc + 34.0
    z0, z1 = HANGER_Z
    yb = CLAVICLE[2]
    body = box(x0, x1, -32.0, CLAVICLE[3], z0, z1)
    body = body.union(box(x0, x1, yb - 8.0, yb, z1 - 0.1, CLAVICLE[5] - 2.0))
    cuts = []
    m = s1_unit_matrix("R")
    for x, y in DR.FOOT_BOLTS:
        p = (m @ np.array([x, y, DR.FOOT_Z[0], 1.0]))[:3]
        cuts.append(cyl_z(P.M4_CLR / 2, z0 - 1, z1 + 1).translate((p[0], p[1], 0)))
        cuts.append(cq.Workplane("XY").workplane(offset=z1 - 4.0).center(p[0], p[1])
                    .polygon(6, P.M4_NUT_AF / 0.866).extrude(5.0))
    for xx in (x0 + 14.0, x1 - 14.0):
        cuts.append(cyl_z(2.75, z0 - 1, z1 + 1).translate((xx, (yb + CLAVICLE[3]) / 2, 0)))      # up into the slot
        cuts.append(cyl_z(5.0, z0 - 1, z0 + 12.0).translate((xx, (yb + CLAVICLE[3]) / 2, 0)))   # head counterbore
        cuts.append(_teardrop_y(2.75, yb - 9.0, yb + 1.0, xx, (CLAVICLE[4] + CLAVICLE[5]) / 2 + 2.0))
    for yy in (24.0, 50.0):                                   # lighten (pockets from the top)
        cuts.append(box(x0 + 6.0, x1 - 6.0, yy, yy + 22.0, z0 + 6.0, z1 + 1))
    return cut_all(body, cuts)


def _face_xz(side, level):
    """(x, z) end points of a unit's foot face (local z = FOOT_Z0, y = +-WALL_Y) on a level."""
    m = unit_matrix(side, level)
    pts = []
    for yy in (DR.WALL_Y, -DR.WALL_Y):
        p = m @ np.array([0.0, yy, DR.FOOT_Z[0], 1.0])
        pts.append((p[0], p[2]))
    return pts


SADDLE_Y = (-32.0, SPINE_Y[0])      # front face .. against the spine posts
SADDLE_FLANGE_Y = 10.0               # rear flange thickness (y)
SADDLE_FLANGE_X = 64.0


def saddle():
    """Printed saddle for one level, in its LEVEL frame (TORSO frame shifted by the level
    height). Two faces carry a right and a left drive unit (4 M4 each, nuts in the
    centre cavity, reached from the open front); the rear flange bolts to both spine
    posts (2 M5 + T-nuts per post). Prints standing on its bottom face: the unit faces
    lean DRIVE_TILT out of vertical, the bolt holes are teardrops."""
    (xr0, zr0), (xr1, zr1) = _face_xz("R", 0)
    z0, z1 = sorted((zr0, zr1))
    zc = LEVEL_Z0
    # cross-section in XZ: the two faces (right face points (xr, z), left = mirror)
    top = max(zr0, zr1)
    bot = min(zr0, zr1)
    xt = xr0 if zr0 > zr1 else xr1
    xb = xr1 if zr0 > zr1 else xr0
    sec = [(xb, bot), (-xb, bot), (-xt, top), (xt, top)]
    y0, y1 = SADDLE_Y
    body = (cq.Workplane("XZ").polyline([(x, z - zc) for x, z in sec]).close().extrude(-(y1 - y0))
            .translate((0, y0, 0)))
    fl = box(-SADDLE_FLANGE_X, SADDLE_FLANGE_X, y1 - SADDLE_FLANGE_Y, y1, bot - zc, top - zc)
    body = body.union(fl)
    cuts = []
    # centre cavity for the nuts (open at the front, the top and the bottom)
    cuts.append(box(-7.5, 7.5, y0 - 1, 16.0, bot - zc - 1, top - zc + 1))
    cuts.append(box(-7.5, 7.5, 30.0, y1 - SADDLE_FLANGE_Y - 4.0, bot - zc - 1, top - zc + 1))   # lighten
    # unit bolts: along each unit's -Z from its foot face into the cavity
    for side in ("R", "L"):
        m = unit_matrix(side, 0)
        for x, y in DR.FOOT_BOLTS:
            p = (m @ np.array([x, y, DR.FOOT_Z[0] + 1.0, 1.0]))[:3]
            q = (m @ np.array([x, y, DR.FOOT_Z[0] - 30.0, 1.0]))[:3]
            p[2] -= zc
            q[2] -= zc
            cuts.append(_teardrop_rod(p, q, P.M4_CLR / 2))
    # flange bolts into the spine posts
    for sx in (1, -1):
        for zz in (bot - zc + 12.0, top - zc - 12.0):
            cuts.append(_teardrop_y(2.75, y1 - SADDLE_FLANGE_Y - 1, y1 + 1, sx * 40.0, zz))
    return cut_all(body, cuts)


def _teardrop_y(r, y0, y1, x, z):
    c, s = math.cos(math.radians(50)), math.sin(math.radians(50))
    tri = (cq.Workplane("XZ").workplane(offset=-y1)
           .polyline([(x + r * s, z + r * c), (x, z + r / c), (x - r * s, z + r * c), (x, z)]).close()
           .extrude(y1 - y0))
    return cyl_y(r, y0, y1).translate((x, 0, z)).union(tri)


def _teardrop_rod(p, q, r):
    """Round hole from p to q with a 50 deg roof pointing +Z (the axis is near horizontal)."""
    from .cad import rod
    v = np.array(q) - np.array(p)
    L = float(np.linalg.norm(v))
    u = v / L
    side = np.cross(u, [0, 0, 1.0])
    side /= np.linalg.norm(side)
    up = np.cross(side, u)
    c, s = math.cos(math.radians(50)), math.sin(math.radians(50))
    pl = cq.Plane(origin=tuple(p), xDir=tuple(side), normal=tuple(u))
    tri = (cq.Workplane(pl).polyline([(r * s, r * c), (0, r / c), (-r * s, r * c), (0, 0)]).close().extrude(L))
    return rod(tuple(p), tuple(q), 2 * r).union(tri)


def saddle_matrix(level):
    m = np.eye(4)
    m[2, 3] = LEVEL_Z0 + level * LEVEL_PITCH
    return m


# ------------------------------------------------------------------ shoulder beam
def shoulder_beam(s1):
    """Shoulder beam for one arm, BEAM frame (floor underside z = 0). The old U-channel's
    front part (S1 mount, mast walls) + wings that bolt onto the two deck rails.
    Prints floor down (6 walls). Same part for both arms (it is symmetric in x)."""
    x, (y0, y1) = BEAM_X, BEAM_Y
    body = box(-x, x, y0, y1, 0, BEAM_H)
    body = body.cut(box(-x + WALL, x - WALL, y0 + WALL, y1 - WALL, WALL, BEAM_H + 1))
    body = body.union(box(-WING_X, WING_X, WING_Y[0], WING_Y[1], 0, WALL))
    for sx in (1, -1):            # gussets wall -> wing (45 deg hypotenuse)
        for yy in (WING_Y[0] + 3.0, WING_Y[1] - 9.0):
            g = (cq.Workplane("XZ").polyline([(sx * x, WALL - 0.1), (sx * (x + 22.0), WALL - 0.1), (sx * x, WALL + 22.0)])
                 .close().extrude(-6.0).translate((0, yy, 0)))
            body = body.union(g)
    cuts = []
    # tube opening above the S1 base (pass-through slot + both sockets)
    cuts.append(box(-26, 26, S1_Y - 24, S1_Y + 24, -1, WALL + 1).edges("|Z").fillet(6))
    # S1 base bolts (M3 from inside the channel into the S1 inserts)
    for xx in (-(s1.lug_out + 7), s1.lug_out + 7):
        for yy in (-17, 17):
            cuts.append(cyl_z(P.M3_CLR / 2, -1, WALL + 1).translate((xx, S1_Y + yy, 0)))
    # notch in the front wall: the S1 cord tubes come down to their sockets from the front
    cuts.append(box(-32, 32, y0 - 1, y0 + WALL + 1, 14.0, BEAM_H + 1))
    # wing bolts (M5 into T-nuts in the deck rails), comb screws (M3 inserts in the floor)
    for xx, yy in WING_BOLTS:
        cuts.append(cyl_z(2.75, -1, WALL + 1).translate((xx, yy, 0)))
    for xx in (-COMB_LEG_X, COMB_LEG_X):
        cuts.append(cyl_z(P.INSERT_D / 2, WALL - P.INSERT_DEPTH, WALL + 1).translate((xx, S1_Y, 0)))
    # M4 bolts for the two balance-spring masts (arm/balance.py), through both side walls
    from . import balance as B
    for yy, zz in B.mast_bolt_points():
        cuts.append(box(-x - 1, x + 1, yy - P.M4_CLR / 2, yy + P.M4_CLR / 2, zz - P.M4_CLR / 2, zz + P.M4_CLR / 2))
    # lighten the rear wall
    cuts.append(box(-30, 30, y1 - WALL - 1, y1 + 1, 16.0, BEAM_H - 8.0))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ shoulder comb
COMB_Z = (30.0, 36.0)       # plate (BEAM frame)
COMB_PITCH = 5.8          # 4.5 mm holes with 1.3 mm walls; the tubes close up into the S1 slot below
COMB_COLS = [(k - 2) * COMB_PITCH for k in range(5)]        # x offsets
COMB_ROWS = [-COMB_PITCH, 0.0, COMB_PITCH]                   # y offsets from S1_Y
COMB_HOLE_R = P.TUBE_OD / 2 + 0.25
COMB_LEG_X = 33.0
COMB_HALF_Y = 10.0


def comb_holes():
    """[(x, y)] of the 15 comb holes (BEAM frame), inner column (towards the torso, +x
    for the right arm's beam frame) first, then outwards."""
    return [(x, S1_Y + y) for x in sorted(COMB_COLS, reverse=True) for y in COMB_ROWS]


def shoulder_comb():
    """Tube comb above the S1 floor opening (BEAM frame): a plate with the 5 x 3 tube grid
    on two narrow arms and legs screwed (M3 into floor inserts) to the floor; the S1 tubes
    drop past the arms at y +-12, the joints' encoder cables are tied to the arms.
    Prints plate down."""
    z0, z1 = COMB_Z
    yc = S1_Y
    h, c = COMB_HALF_Y, 3.5                                  # 3.5 mm corner chamfers (S1 tubes pass close)
    plate = (cq.Workplane("XY").polyline([(-16 + c, -h), (16 - c, -h), (16, -h + c), (16, h - c), (16 - c, h),
                                          (-16 + c, h), (-16, h - c), (-16, -h + c)]).close()
             .extrude(z1 - z0).translate((0, yc, z0)))
    body = plate.union(box(-COMB_LEG_X - 6, COMB_LEG_X + 6, yc - 4.0, yc + 4.0, z0, z1))
    for sx in (1, -1):                                       # pads over the legs (screw heads)
        body = body.union(box(sx * COMB_LEG_X - 6, sx * COMB_LEG_X + 6, yc - 6.0, yc + 6.0, z0, z1))
    for sx in (1, -1):
        body = body.union(box(sx * COMB_LEG_X - 5, sx * COMB_LEG_X + 5, yc - 6, yc + 6, WALL, z0 + 0.1))
    cuts = []
    for x, y in comb_holes():
        cuts.append(cyl_z(COMB_HOLE_R, z0 - 1, z1 + 1).translate((x, y, 0)))
    for sx in (1, -1):
        cuts.append(cyl_z(P.M3_CLR / 2, WALL - 1, z1 + 1).translate((sx * COMB_LEG_X, yc, 0)))
        cuts.append(cyl_z(P.M3_HEAD_D / 2 + 0.3, z1 - 3.0, z1 + 1).translate((sx * COMB_LEG_X, yc, 0)))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ electronics
TRAY = dict(x=(-75.0, 75.0), z=(300.0, 520.0), y0=SPINE_Y[1], t=4.0)
ESTOP = dict(x=(-28.0, 28.0), y=(380.0, 440.0), z0=CROSS_Z[1], h=60.0)


def electronics_tray():
    """Plate on the back of the spine posts (TORSO frame), 4 M5 into the posts. Carries a
    100 x 60 controller carrier (Teensy 4.1 class: 4 M3 inserts on 92 x 52), a servo-bus
    board (4 M3 inserts on 40 x 30), an M12 cable gland for the 12 V leads (Ø12) + a panel fuse holder (Ø16), with
    cable-tie slots along the edges. Prints flat."""
    (x0, x1), (z0, z1), y0, t = TRAY["x"], TRAY["z"], TRAY["y0"], TRAY["t"]
    body = box(x0, x1, y0, y0 + t, z0, z1)
    for xx in (x0 + 2, x1 - 6):                       # stiffening ribs
        body = body.union(box(xx, xx + 4, y0 + t - 0.1, y0 + t + 8, z0, z1))
    cuts = []
    for sx in (1, -1):
        for zz in (z0 + 15, z1 - 15):
            cuts.append(cyl_y(2.75, y0 - 1, y0 + t + 1).translate((sx * 40.0, 0, zz)))
    for dx in (-46.0, 46.0):
        for dz in (-26.0, 26.0):
            cuts.append(cyl_y(P.INSERT_D / 2, y0 + t - P.INSERT_DEPTH + 1.5, y0 + t + 1).translate((dx, 0, 440.0 + dz)))
    for dx in (-20.0, 20.0):
        for dz in (-15.0, 15.0):
            cuts.append(cyl_y(P.INSERT_D / 2, y0 + t - P.INSERT_DEPTH + 1.5, y0 + t + 1).translate((dx, 0, 360.0 + dz)))
    cuts.append(cyl_y(6.0, y0 - 1, y0 + t + 1).translate((-50.0, 0, 330.0)))     # 12 V cable gland (M12)
    cuts.append(cyl_y(8.0, y0 - 1, y0 + t + 1).translate((50.0, 0, 330.0)))      # fuse holder
    for zz in range(int(z0) + 30, int(z1) - 20, 40):
        for xx in (x0 + 12.0, x1 - 16.0):
            cuts.append(box(xx, xx + 4.0, y0 - 1, y0 + t + 1, zz, zz + 8.0))
    return cut_all(body, cuts)


def estop_box():
    """Bracket for a standard 22 mm mushroom E-stop (TORSO frame) on the rear cross rail,
    between the inner deck rails: a top plate (Ø22.5 hole) on two thick side walls, a back
    wall with a cable-gland hole, open front and bottom so the contact block stays
    reachable. Two M5 x 70 run down through the side walls into T-nuts in the rail.
    Prints top plate down."""
    (x0, x1), (y0, y1), z0, h = ESTOP["x"], ESTOP["y"], ESTOP["z0"], ESTOP["h"]
    sw = 10.0
    body = (box(x0, x1, y0, y1, z0 + h - 4, z0 + h)
            .union(box(x0, x0 + sw, y0, y1, z0, z0 + h)).union(box(x1 - sw, x1, y0, y1, z0, z0 + h))
            .union(box(x0, x1, y1 - 4, y1, z0, z0 + h)))
    xc, yc = (x0 + x1) / 2, (y0 + y1) / 2
    cuts = [cyl_z(11.25, z0 + h - 5, z0 + h + 1).translate((xc, yc - 3, 0)),
            cyl_y(6.0, y1 - 5, y1 + 1).translate((xc, 0, z0 + 22.0))]
    for xx in (x0 + sw / 2, x1 - sw / 2):
        cuts.append(cyl_z(2.75, z0 - 1, z0 + h + 1).translate((xx, 410.0, 0)))
    return cut_all(body, cuts)


def printed_parts():
    """name -> (workplane in PRINT orientation, qty, note) for export.py."""
    from .specs import S1
    rot = lambda w, ax, a: w.rotate((0, 0, 0), ax, a)
    return {
        "torso_shoulder_beam": (shoulder_beam(S1), 2, "shoulder beam, one per arm (symmetric) - floor down, 6 walls"),
        "torso_shoulder_comb": (shoulder_comb(), 2, "shoulder tube comb, 5 x 3 grid above the S1 opening - plate down"),
        "torso_s1_hanger": (s1_hanger(), 2, "S1 drive hanger under the clavicle (symmetric: same part both sides) - plate down"),
        "torso_saddle": (saddle(), LEVELS, "drive saddle, one per level (a right and a left drive unit) - bottom down"),
        "torso_electronics_tray": (rot(electronics_tray(), (1, 0, 0), 90), 1,
                                   "electronics tray on the back of the spine - flat, ribs up"),
        "torso_estop_box": (rot(estop_box(), (1, 0, 0), 180), 1, "E-stop box (22 mm mushroom) - top down"),
    }
