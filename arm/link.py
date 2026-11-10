"""Printable links: tapered tubes with outward bolting lips at both ends.

Bottom lip: bolt heads on its TOP face (outside the tube) going down into the
module below. Top lip: heads UNDER it going up into the module above.
Small 45-deg ribs between the top-lip holes make it printable without supports.
Print standing on the bottom lip.
"""
import math
import cadquery as cq

from . import params as P
from .cad import cyl_z, polar


def link(cls_a, cls_b, length, wall=3.0):
    A, B = P.FLANGES[cls_a], P.FLANGES[cls_b]
    lip = P.LIP_T
    outer = (cq.Workplane("XY").circle(A["tube_r"]).workplane(offset=length).circle(B["tube_r"]).loft())
    body = outer.union(cyl_z(A["od"], 0, lip)).union(cyl_z(B["od"], length - lip, length))
    # ribs under the top lip, between the bolts
    d = B["od"] - B["tube_r"]
    n = B["n"]
    for k in range(n):
        a = 360.0 / n * k             # bolts are at half steps -> ribs at whole steps
        r0 = B["tube_r"] - 1.0
        rib = (cq.Workplane("XZ").polyline([(r0, length - lip), (B["od"] - 0.5, length - lip),
                                            (r0, length - lip - d - 1.0)]).close()
               .extrude(1.5, both=True).rotate((0, 0, 0), (0, 0, 1), a))
        body = body.union(rib)
    inner = (cq.Workplane("XY").workplane(offset=-0.01).circle(A["tube_r"] - wall)
             .workplane(offset=length + 0.02).circle(B["tube_r"] - wall).loft())
    body = body.cut(inner)
    for spec, z0, z1 in ((A, -1, lip + 1), (B, length - lip - 1, length + 1)):
        for ang in P.bolt_angles(spec["n"]):
            x, y = polar(spec["bc"], ang)
            body = body.cut(cyl_z(P.M3_CLR / 2, z0, z1).translate((x, y, 0)))
    return body
