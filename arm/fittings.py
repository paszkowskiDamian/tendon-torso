"""Small printed cord fittings.

cord_thimble(): joins a Dyneema cord to a spring hook, an S-hook or a screw eye.
A flat teardrop (5 mm thick): the hook goes through the 6 mm hole in the round
head, the cord wraps round the V-groove on the outside (6.5 mm bend radius at the
groove bottom, so the hook never bears on the cord) and both cord ends meet at the
tail, where they are knotted together or tied through the 1.6 mm tail hole.
Prints flat; the groove's upper flank is 40 deg from vertical, so no supports.
"""
import cadquery as cq

from .cad import cut_all, cyl_z

THIMBLE_R = 7.5            # head radius (outer, at the groove lips)
THIMBLE_TAIL = 16.0        # tail tip distance from the head centre
THIMBLE_T = 5.0
HOOK_R = 3.0               # hook / eye hole
GROOVE_D = 1.0             # V-groove depth (1 mm cord sits ~0.6 mm deep)
TAIL_HOLE = (-9.5, 0.8)    # (y, r) cord tie hole in the tail


def _outline(off=0.0):
    """Teardrop wire at z=0: head circle at the origin, tail tip at -y; off < 0 shrinks it."""
    import math
    r = THIMBLE_R + off
    d = THIMBLE_TAIL
    a = math.asin(THIMBLE_R / d)                    # tangent half angle at the tip
    tip = -d - off / math.sin(a)                    # offset tip (moves with the sides)
    # tangent points on the head circle
    tx, ty = r * math.cos(a), -r * math.sin(a)
    return (cq.Workplane("XY").moveTo(0, tip).lineTo(tx, ty).threePointArc((0, r), (-tx, ty)).close())


def cord_thimble():
    t, g = THIMBLE_T, GROOVE_D
    zl, zm, zh = 1.3, 2.5, 3.7                      # lower lip, groove bottom, upper lip
    low = _outline().extrude(zl)
    up = _outline().extrude(t - zh).translate((0, 0, zh))
    w0 = _outline().wires().val()
    w1 = _outline(-g).wires().val()
    cone_lo = cq.Solid.makeLoft([w0.translate((0, 0, zl)), w1.translate((0, 0, zm))])
    cone_hi = cq.Solid.makeLoft([w1.translate((0, 0, zm)), w0.translate((0, 0, zh))])
    body = low.union(cq.Workplane().add(cone_lo)).union(cq.Workplane().add(cone_hi)).union(up)
    y, r = TAIL_HOLE
    cuts = [cyl_z(HOOK_R, -1, t + 1),
            cyl_z(r, -1, t + 1).translate((0, y, 0))]
    return cut_all(body, cuts).edges(">Z or <Z").edges("%CIRCLE").chamfer(0.4)
