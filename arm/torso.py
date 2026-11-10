"""Torso for the first (single) arm: a shoulder beam + crank racks.

Shoulder beam: a U-channel that is screwed onto a table/shelf and overhangs its
edge. The S1 hinge base bolts UNDER its front end, so the arm hangs beside the
table. All Bowden tubes run inside the channel from the cranks to the shoulder.
Beam frame: table top at z=0, channel along Y, front (overhang) towards -Y.

Crank rack: plate for 3 crank modules side by side (print 3 for 7 cranks).
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
    # wood screws into the table (behind the edge)
    for xx in (-28.0, 28.0):
        for yy in (25.0, 95.0):
            cuts.append(cyl_z(2.4, -1, WALL + 1).translate((xx, yy, 0)))
            cuts.append(cyl_z(4.5, WALL - 2.5, WALL + 1).translate((xx, yy, 0)))
    # lighten the side walls: three windows each, 48 deg pointed tops (6 mm bridge)
    z0, z1 = 16.0, BEAM_H - 10
    for yc in (33.0, 60.0, 87.0):
        hw, flat = 10.0, 3.0
        zs = z1 - (hw - flat) * 1.1
        win = (cq.Workplane("YZ").workplane(offset=-x - 1)
               .polyline([(yc - hw, z0), (yc + hw, z0), (yc + hw, zs), (yc + flat, z1),
                          (yc - flat, z1), (yc - hw, zs)]).close().extrude(2 * x + 2))
        cuts.append(win)
    return cut_all(body, cuts)


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
