"""Hand-crank drive module (one per joint). Swap for a NEMA17 later.

Local frame: shaft along X through the origin, Z up, Bowden tubes arrive from above.

* FRAME: two walls with 608 pockets, foot, top beam. Wall B (+X) carries a ring of
  36 index holes: a pin through the crank arm locks the joint every 10 deg of crank.
* SPOOL: two helical sections (cord A on -X, cord B on +X), clamped between the two
  608 inner races by the M8 shaft bolt -> turns with the bolt.
* CRANK ARM: M8 bolt head sits in its hex pocket. Knob turns on an M4 bolt.
* NUT CUP: holds the M8 nut on the -X side and clamps the stack.
* SOCKET BLOCK: holds both tube ends right above the spool tangents. Two M4
  screws through the top beam pull it up = tension both cords.
"""
import math
import cadquery as cq

from . import params as P
from .cad import box, cut_all, cyl_x, cyl_z, hex_x, hex_z, polar, rod, yz

R_SPOOL = 8.0                  # cord centre radius on the spool
PITCH = 1.6
SEC_A = (-12.5, -3.5)          # helical section for cord A
SEC_B = (3.5, 12.5)
SPOOL_HALF = 16.5              # bosses touch the inner races here
WALL_T = 9.0
WALL_IN = SPOOL_HALF           # inner faces of the walls
WALL_OUT = WALL_IN + WALL_T
FOOT_Z = (-78.0, -72.0)          # shaft high enough for the crank to clear the table
BEAM_Z = (44.0, 50.0)
BLOCK_Z = (22.0, 32.0)
SOCK_X = (SEC_A[0] + SEC_A[1]) / 2   # -8
INDEX_R, INDEX_N = 32.0, 36
ARM_R = 62.0
ARM_X = (WALL_OUT + 0.5, WALL_OUT + 8.5)


def _teardrop_x(r, x0, x1):
    """Horizontal hole along X that prints without support (pointed top)."""
    c = cyl_x(r, x0, x1)
    tri = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(-r * 0.7071, r * 0.7071), (0, r * 1.414), (r * 0.7071, r * 0.7071), (0, 0)]).close()
           .extrude(x1 - x0))
    return c.union(tri)


def spool():
    core_r = R_SPOOL + P.CORD_D / 2 + 0.3
    body = (cyl_x(12.0, -14.5, 14.5)
            .union(cyl_x(6.0, -SPOOL_HALF, SPOOL_HALF)))
    # turn the helical zones into cores
    for a, b in (SEC_A, SEC_B):
        body = body.cut(cyl_x(13.0, a, b).cut(cyl_x(core_r, a, b)))
    cuts = [cyl_x(P.M8_CLR / 2, -SPOOL_HALF - 1, SPOOL_HALF + 1)]
    for a, b in (SEC_A, SEC_B):
        h = b - a - 1.0
        path = cq.Wire.makeHelix(PITCH, h, R_SPOOL)
        g = cq.Workplane("XZ").center(R_SPOOL, 0).circle(P.CORD_D / 2 + 0.15).sweep(cq.Workplane().add(path), isFrenet=True)
        cuts.append(g.rotate((0, 0, 0), (0, 1, 0), 90).translate((a + 0.5, 0, 0)))
    # cord end holes through the end flanges
    for x0, x1, ang in ((-14.6, -12.4, -60), (12.4, 14.6, 60)):
        y, z = yz(R_SPOOL + 2.2, ang)
        cuts.append(rod((x0 - 0.5, y, z), (x1 + 0.5, y, z), 1.8))
    return cut_all(body, cuts)


def frame():
    yw, zw = 40.0, 40.0
    wall_b = box(WALL_IN, WALL_OUT, -yw, yw, FOOT_Z[0], max(zw, BEAM_Z[1]))
    wall_a = box(-WALL_OUT, -WALL_IN, -22, 22, FOOT_Z[0], BEAM_Z[1])
    foot = box(-WALL_OUT, WALL_OUT, -yw, yw, *FOOT_Z)
    beam = box(-WALL_OUT, WALL_OUT, -14, 14, *BEAM_Z)
    body = wall_a.union(wall_b).union(foot).union(beam)
    cuts = []
    for sgn in (1, -1):
        x_in, x_out = (WALL_IN, WALL_OUT) if sgn > 0 else (-WALL_OUT, -WALL_IN)
        pocket = (_teardrop_x(P.B608[1] / 2 + P.FIT, WALL_IN - 0.1, WALL_IN + P.B608[2]) if sgn > 0
                  else _teardrop_x(P.B608[1] / 2 + P.FIT, -WALL_IN - P.B608[2], -WALL_IN + 0.1))
        cuts.append(pocket)
        cuts.append(_teardrop_x(9.5, x_in - 1, x_out + 1))
    # index holes (blind, 8 deep from the outer face of wall B)
    for k in range(INDEX_N):
        y, z = yz(INDEX_R, 360.0 / INDEX_N * k)
        if z + 2.5 > max(zw, BEAM_Z[1]) or z - 2.5 < FOOT_Z[1]:
            continue
        cuts.append(cyl_x(2.2, WALL_OUT - 8, WALL_OUT + 1).translate((0, y, z)))
    # tension screws through the beam
    for x in (-12.0, 12.0):
        cuts.append(cyl_z(P.M4_CLR / 2, BEAM_Z[0] - 1, BEAM_Z[1] + 1).translate((x, 0, 0)))
    # tube pass holes in the beam (tubes go through to the block)
    for x, y in ((SOCK_X, R_SPOOL), (-SOCK_X, -R_SPOOL)):
        cuts.append(cyl_z(P.TUBE_OD / 2 + 1.0, BEAM_Z[0] - 1, BEAM_Z[1] + 1).translate((x, y, 0)))
    # foot screw holes
    for x in (-18.0, 18.0):
        for y in (-32.0, 32.0):
            cuts.append(cyl_z(P.M4_CLR / 2, FOOT_Z[0] - 1, FOOT_Z[1] + 1).translate((x, y, 0)))
    # lighten wall A
    cuts.append(box(-WALL_OUT - 1, -WALL_IN + 1, -14, 14, FOOT_Z[1] + 4, -16))
    return cut_all(body, cuts)


def socket_block():
    x0, x1 = -WALL_IN + 0.3, WALL_IN - 0.3
    body = box(x0, x1, -13, 13, *BLOCK_Z)
    cuts = []
    for x, y in ((SOCK_X, R_SPOOL), (-SOCK_X, -R_SPOOL)):
        cuts.append(cyl_z(P.TUBE_OD / 2 + 0.15, BLOCK_Z[1] - 7, BLOCK_Z[1] + 1).translate((x, y, 0)))
        cuts.append(cyl_z(P.TUBE_ID / 2 + 0.2, BLOCK_Z[0] - 1, BLOCK_Z[1]).translate((x, y, 0)))
    for x in (-12.0, 12.0):
        cuts.append(cyl_z(P.M4_CLR / 2 - 0.3, BLOCK_Z[0] - 1, BLOCK_Z[1] + 1).translate((x, 0, 0)))
        cuts.append(hex_z(BLOCK_Z[0] - 1, BLOCK_Z[0] + P.M4_NUT_T + 0.3, x, 0, P.M4_NUT_AF))
    return cut_all(body, cuts)


def crank_arm():
    x0, x1 = ARM_X
    hub = cyl_x(13.0, x0, x1)
    boss = cyl_x(6.0, WALL_IN + P.B608[2], x0 + 0.1)       # presses the 608-B inner race
    arm = box(x0, x1, -7, 7, 0, ARM_R).union(cyl_x(7.0, x0, x1).translate((0, 0, ARM_R)))
    body = hub.union(boss).union(arm)
    cuts = [cyl_x(P.M8_CLR / 2, WALL_IN, x1 + 1),
            hex_x(x1 - P.M8_HEAD_T - 0.3, x1 + 1, 0, 0, P.M8_HEAD_AF),
            cyl_x(2.25, x0 - 1, x1 + 1).translate((0, 0, INDEX_R)),          # lock pin
            cyl_x(P.M4_CLR / 2, x0 - 1, x1 + 1).translate((0, 0, ARM_R)),     # knob bolt
            hex_x(x0 - 1, x0 + P.M4_NUT_T + 0.3, 0, ARM_R, P.M4_NUT_AF)]
    return cut_all(body, cuts)


def nut_cup():
    x1 = -(WALL_IN + P.B608[2])
    x0 = -WALL_OUT - 8.5
    body = cyl_x(6.0, -WALL_OUT - 0.5, x1).union(cyl_x(12.0, x0, -WALL_OUT - 0.5))
    cuts = [cyl_x(P.M8_CLR / 2, x0 - 1, x1 + 1), hex_x(x0 - 1, x0 + P.M8_NUT_T + 0.3, 0, 0, P.M8_NUT_AF)]
    return cut_all(body, cuts)


def lock_pin():
    return cyl_z(2.0, 0, 20).union(cyl_z(6.0, 20, 24)).edges(">Z").fillet(1.0)


def knob():
    return cyl_z(7.0, 0, 26).cut(cyl_z(P.M4_CLR / 2 + 0.2, -1, 27)).edges().fillet(1.0)


def tangent_points():
    """Cord exits (bottom of the socket block) and spool tangents for cords A and B."""
    return [((SOCK_X, R_SPOOL, BLOCK_Z[0]), (SOCK_X, R_SPOOL, 0.0)),
            ((-SOCK_X, -R_SPOOL, BLOCK_Z[0]), (-SOCK_X, -R_SPOOL, 0.0))]
