"""Torso for the first (single) arm: a shoulder beam + crank racks.

Shoulder beam: a U-channel that is screwed onto a table/shelf and overhangs its
edge. The S1 hinge base bolts UNDER its front end, so the arm hangs beside the
table. All Bowden tubes run inside the channel from the cranks to the shoulder.
Beam frame: table top at z=0, channel along Y, front (overhang) towards -Y.

Crank rack: plate for 3 crank modules side by side (print 3 for 7 cranks).

Tube comb: a plate across the channel that holds the 14 Bowden tubes in a
labelled 7 x 2 grid (columns S1 S2 S3 E F W1 W2, rows A / B). It stands on the
channel floor and two flex arms snap their lugs into a pair of side-wall windows
(press the lugs in from outside to remove it); arms and lugs fill the window
along y, so the comb cannot slide. Two combs, in the windows at y = 33 and
y = 60, plates at the back of the windows (clear of the wood screws at y = 25).
The tubes slide freely through 4.6 mm holes and run straight through, so the
combs never bend them.
"""
import cadquery as cq

from . import params as P
from .cad import box, cut_all, cyl_z

BEAM_X = 50.0          # half width
BEAM_H = 52.0
BEAM_Y = (-125.0, 115.0)
WALL = 6.0
S1_Y = -95.0           # S1 axis position along the beam


def shoulder_beam(s1):
    x, (y0, y1) = BEAM_X, BEAM_Y
    body = box(-x, x, y0, y1, 0, BEAM_H)
    body = body.cut(box(-x + WALL, x - WALL, y0 + WALL, y1 + 1, WALL, BEAM_H + 1))
    cuts = []
    # tube opening above the S1 base (pass-through slot + both sockets)
    cuts.append(box(-26, 26, S1_Y - 24, S1_Y + 24, -1, WALL + 1).edges("|Z").fillet(6))
    # S1 base bolts (M3 from inside the channel into the S1 inserts)
    for xx in (-(s1.lug_out + 7), s1.lug_out + 7):
        for yy in (-17, 17):
            cuts.append(cyl_z(P.M3_CLR / 2, -1, WALL + 1).translate((xx, S1_Y + yy, 0)))
    # lighten the side walls: three windows each, 48 deg pointed tops (6 mm bridge);
    # the tube combs snap into them
    cuts += _windows()
    for xx, yy in SCREWS_XY:
        cuts.append(cyl_z(2.4, -1, WALL + 1).translate((xx, yy, 0)))
        cuts.append(cyl_z(4.5, WALL - 2.5, WALL + 1).translate((xx, yy, 0)))
    # M4 bolts for the two balance-spring masts (arm/balance.py), through both side walls
    from . import balance as B
    for yy, zz in B.mast_bolt_points():
        cuts.append(box(-x - 1, x + 1, yy - P.M4_CLR / 2, yy + P.M4_CLR / 2, zz - P.M4_CLR / 2, zz + P.M4_CLR / 2)
                    .intersect(box(-x - 1, x + 1, -400, 400, -400, 400)))
    return cut_all(body, cuts)


WINDOWS_Y = (33.0, 60.0, 87.0)    # side-wall windows (centre y)
WIN_Z = 16.0                      # window bottom
WIN_HW = 10.0                     # window half width (y)
SCREWS_XY = [(xx, yy) for xx in (-28.0, 28.0) for yy in (25.0, 95.0)]   # wood screws in the floor


def _windows():
    x = BEAM_X
    z0, z1 = WIN_Z, BEAM_H - 10
    out = []
    for yc in WINDOWS_Y:
        hw, flat = WIN_HW, 3.0
        zs = z1 - (hw - flat) * 1.1
        out.append(cq.Workplane("YZ").workplane(offset=-x - 1)
                   .polyline([(yc - hw, z0), (yc + hw, z0), (yc + hw, zs), (yc + flat, z1),
                              (yc - flat, z1), (yc - hw, zs)]).close().extrude(2 * x + 2))
    return out


RACK_PITCH = 72.0          # crank arm + nut cup need ~70 mm per module


def crank_rack(n=3):
    w = RACK_PITCH * n
    body = box(-w / 2, w / 2, -48, 48, 0, 5).edges("|Z").fillet(6)
    cuts = []
    for k in range(n):
        xc = -w / 2 + RACK_PITCH * (k + 0.5)
        for dx in (-18.0, 18.0):
            for dy in (-32.0, 32.0):
                cuts.append(cyl_z(P.M4_CLR / 2, -1, 6).translate((xc + dx, dy, 0)))
                cuts.append(cq.Workplane("XY").center(xc + dx, dy).polygon(6, P.M4_NUT_AF / 0.866).extrude(3.0).translate((0, 0, -0.01)))
        cuts.append(box(xc - 12, xc + 12, -20, 20, -1, 6).edges("|Z").fillet(4))   # lighten
    for xx in (-w / 2 + 10, w / 2 - 10):
        for yy in (-38.0, 38.0):
            cuts.append(cyl_z(2.4, -1, 6).translate((xx, yy, 0)))
    return cut_all(body, cuts)


# ------------------------------------------------------------ tube comb
COMB_JOINTS = ("S1", "S2", "S3", "E", "F", "W1", "W2")
COMB_T = 10.0                # plate thickness along the channel (y)
COMB_SIDE = 0.4              # clearance to each side wall
COMB_TOP = BEAM_H - 6.0      # top edge (z, beam frame)
COMB_PITCH = 10.0            # column pitch (x)
COMB_ROWS = {"A": 32.0, "B": 24.0}   # row height (z, beam frame)
COMB_HOLE_R = P.TUBE_OD / 2 + 0.3    # 4.6 mm: the tube slides, never squeezed
COMB_ARM_T, COMB_SLOT = 2.6, 3.0     # snap arm thickness, gap behind it (>= lug depth)
COMB_LUG = 2.2               # lug reach into the window (wall is 6 mm)
COMB_LUG_Z = (WIN_Z + 2.0, WIN_Z + 10.0)
COMB_LUG_W = 2 * WIN_HW - 1.0        # arms + lugs fill the window along y: no play
COMB_Y = (WINDOWS_Y[0], WINDOWS_Y[1])     # the two combs (the y=87 window is too close to a wood screw)
TEXT_H, TEXT_UP = 5.0, 0.4


def comb_x(joint):
    """x of a joint's column in the comb (beam frame)."""
    return (COMB_JOINTS.index(joint) - 3) * COMB_PITCH


def tube_comb():
    """Tube comb in its PRINT frame: X = beam x, Y = beam z - WALL (bottom on the
    channel floor at Y=0), Z = towards -y of the beam. The plate (labels raised on
    its top face) is COMB_T thick; the two snap arms and their lugs stand
    COMB_LUG_W tall so they fill the window and the comb cannot slide. Installed,
    the labels face the shoulder (-y) and the plate sits at the BACK of its window,
    clear of the wood screws; see comb_in_beam()."""
    xw = BEAM_X - WALL - COMB_SIDE           # outer face of the arms
    xc = xw - COMB_ARM_T - COMB_SLOT         # core half width
    h = COMB_TOP - WALL
    bar = 5.0                                # top bar that carries the arms
    lz0, lz1 = COMB_LUG_Z[0] - WALL, COMB_LUG_Z[1] - WALL
    arm_y0 = lz0 - 2.0                       # free end of the arm
    plate = (cq.Workplane("XY").polyline([(-xc, 0), (xc, 0), (xc, h - bar), (xw, h - bar), (xw, h),
                                          (-xw, h), (-xw, h - bar), (-xc, h - bar)]).close().extrude(COMB_T))
    body = plate
    for sx in (1, -1):
        arm = [(xw - COMB_ARM_T, arm_y0), (xw, arm_y0), (xw, lz0), (xw + COMB_LUG, lz0 + COMB_LUG * 1.2),
               (xw + COMB_LUG, lz1), (xw, lz1), (xw, h), (xw - COMB_ARM_T, h)]
        body = body.union(cq.Workplane("XY").polyline([(sx * x, y) for x, y in arm]).close().extrude(COMB_LUG_W))
    cuts = []
    for j in COMB_JOINTS:
        for zr in COMB_ROWS.values():
            cuts.append(cyl_z(COMB_HOLE_R, -1, COMB_T + 1).translate((comb_x(j), zr - WALL, 0)))
    body = cut_all(body, cuts)
    # raised labels: joint names above the columns, A / B at both ends of the rows
    ya = COMB_ROWS["A"] - WALL
    yb = COMB_ROWS["B"] - WALL

    def txt(s, x, y):
        return (cq.Workplane("XY").workplane(offset=COMB_T - 0.01).center(x, y)
                .text(s, TEXT_H, TEXT_UP + 0.01, kind="bold", halign="center", valign="center"))
    for j in COMB_JOINTS:
        body = body.union(txt(j, comb_x(j), ya + COMB_HOLE_R + 1.2 + TEXT_H / 2))
    for r, y in (("A", ya), ("B", yb)):
        for sx in (-1, 1):
            body = body.union(txt(r, sx * (3 * COMB_PITCH + 5.5), y))
    return body


def comb_in_beam(yc):
    """A tube comb placed in the beam frame, in the window centred on y = yc."""
    y_back = yc + COMB_LUG_W / 2
    return tube_comb().rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y_back, WALL))
