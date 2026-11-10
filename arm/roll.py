"""Roll module family: rotation about the link's own axis, hollow centre.

Local frame: roll axis = Z, stator plate below z=0, rotor above, origin on the
axis at the top face of the stator plate.

* STATOR: plate (heat-set inserts underneath) + a hollow tube that carries two
  thin-section bearings (inner races) + two idler brackets. Each idler is a 623
  bearing in a printed grooved tyre on an M3 bolt; it turns the cord coming up
  from a Bowden socket in the plate by 90 deg so it reaches the drum tangentially.
* ROTOR: a ring around the bearings (outer races) with two tendon grooves and
  knotted chord anchors, plus a top plate with inserts for the next link.
  Print it upside down (plate on the bed).
* RETAINER: ring screwed onto the top of the stator tube; clamps the upper
  inner race so the bearing pair is preloaded against the rotor's drum zone.

Cord A: groove at z_a, tangent at angle 0 (x=+R), arrives along -Y, wraps clockwise.
Cord B: groove at z_b, tangent at angle 180 (x=-R), arrives along -Y, wraps counter-clockwise.
Rotating +a (counter-clockwise about +Z): wrap A = WA - a, wrap B = WB + a.
"""
import math
from dataclasses import dataclass

import cadquery as cq

from . import params as P
from .cad import box, cut_all, cyl_x, cyl_z, hex_x, polar, rod


TYRE_R, TYRE_W = 8.0, 5.0              # printed idler tyre on a 623
IDLER_CORD_R = TYRE_R - 1.0 + P.CORD_D / 2   # cord centre radius on the idler (groove 1 mm deep)


@dataclass
class RollSpec:
    name: str
    cls: str            # stator plate flange class
    rotor_cls: str      # rotor plate flange class
    bearing: tuple      # (d, D, B) thin-section bearing, two of them
    lo: float
    hi: float
    tubes_through: int = 0

    @property
    def R(self):        # tendon centre radius on the drum
        return self.bearing[1] / 2 + 7.0

    @property
    def r_groove(self):
        return self.R - P.CORD_D / 2

    @property
    def r_o(self):
        return self.R + 2.0

    @property
    def r_t(self):      # stator tube outer radius (inner races)
        return self.bearing[0] / 2 - 0.05

    @property
    def r_in(self):     # free bore for passing tubes
        return self.r_t - 5.0

    @property
    def z_a(self):
        return 1.0 + self.bearing[2] + 10.0

    @property
    def z_b(self):
        return self.z_a + 6.0

    @property
    def z_top_pocket(self):      # bottom of the upper bearing pocket
        return self.z_b + 5.0

    @property
    def z_flange(self):
        return self.z_top_pocket + self.bearing[2]

    @property
    def out_z(self):
        return self.z_flange + P.PLATE_T

    @property
    def in_z(self):
        return -P.PLATE_T

    def idler_y(self):
        x_near = self.R - TYRE_W / 2 - 0.7
        need1 = TYRE_R + math.sqrt((self.r_o + 1.5) ** 2 - x_near ** 2)
        cheek_x = self.R - 6.2
        need2 = 4.5 + math.sqrt((self.r_o + 1.2) ** 2 - cheek_x ** 2)
        return max(need1, need2)

    def socket_xy(self):
        """(x, y) of the cord-A socket; cord B is at (-x, y)."""
        return self.R, self.idler_y() + IDLER_CORD_R

    def wraps(self):
        wa = max(25.0 + self.hi, min(135.0, 290.0 + self.lo))
        wb = max(25.0 - self.lo, min(135.0, 290.0 - self.hi))
        return wa, wb


def stator(s: RollSpec):
    F = P.FLANGES[s.cls]
    sx, sy = s.socket_xy()
    r_plate = max(F["od"], math.hypot(sx, sy) + 5)
    yi = s.idler_y()
    body = cyl_z(r_plate, s.in_z, 0)
    body = body.union(cyl_z(s.r_t, 0, s.z_flange))
    body = body.union(cyl_z(s.r_t + 2.5, -0.5, 1.0))           # shoulder under the lower inner race
    cuts = [cyl_z(s.r_in, s.in_z - 1, s.z_flange + 1)]
    for x, zg in ((s.R, s.z_a), (-s.R, s.z_b)):
        zc = zg - IDLER_CORD_R
        for side in (-1, 1):
            c0, c1 = sorted((x + side * 3.2, x + side * 6.2))
            body = body.union(box(c0, c1, yi - 4.5, yi + 4.5, -0.5, zc + 5.0))
        cuts.append(rod((x - 7, yi, zc), (x + 7, yi, zc), P.M3_CLR))
        # Bowden socket: tube from below, cord exits up to the idler
        cuts.append(cyl_z(P.TUBE_OD / 2 + 0.15, s.in_z - 1, -2.0).translate((x, sy, 0)))
        cuts.append(cyl_z(P.TUBE_ID / 2 + 0.2, -2.5, 1.0).translate((x, sy, 0)))
    # retainer screw pilots (self-tapping M3) in the top of the tube
    r_s = (s.r_in + s.r_t) / 2
    for a in (90, 210, 330):
        x, y = polar(r_s, a)
        cuts.append(cyl_z(1.3, s.z_flange - 10, s.z_flange + 1).translate((x, y, 0)))
    # inserts underneath for the link below
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        cuts.append(cyl_z(P.INSERT_D / 2, s.in_z - 1, s.in_z + P.INSERT_DEPTH).translate((x, y, 0)))
    return cut_all(body, cuts)


def rotor(s: RollSpec):
    d, D, B = s.bearing
    F = P.FLANGES[s.rotor_cls]
    body = cyl_z(s.r_o, 1.0, s.z_flange).union(cyl_z(F["od"], s.z_flange, s.out_z))
    cuts = [
        cyl_z(D / 2 + P.FIT, 0, 1.0 + B),                        # lower bearing pocket (opens down)
        cyl_z(s.r_t + 0.6, 0, s.out_z + 1),                      # clearance around the stator tube
        cyl_z(D / 2 + P.FIT, s.z_top_pocket, s.out_z + 1),       # upper pocket (opens up through the plate)
    ]
    w = 2.4
    wa, wb = s.wraps()
    for zg, ang, sign, wrap in ((s.z_a, 0.0, -1, wa), (s.z_b, 180.0, +1, wb)):
        cuts.append(cyl_z(s.r_o + 2, zg - w / 2, zg + w / 2).cut(cyl_z(s.r_groove, zg - w / 2, zg + w / 2)))
        anchor = ang + sign * wrap
        x0, y0 = polar(s.r_groove + 0.3, anchor)
        x1, y1 = polar(s.r_o + 1.0, anchor + sign * 65)
        cuts.append(rod((x0, y0, zg), (x1, y1, zg), P.CORD_D + 1.0))
        dv = cq.Vector(x1 - x0, y1 - y0, 0).normalized()
        cuts.append(rod((x1 - dv.x * 3.5, y1 - dv.y * 3.5, zg), (x1 + dv.x * 2, y1 + dv.y * 2, zg), 4.5))
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        cuts.append(cyl_z(P.INSERT_D / 2, s.out_z - P.INSERT_DEPTH, s.out_z + 1).translate((x, y, 0)))
    return cut_all(body, cuts)


def retainer(s: RollSpec):
    d = s.bearing[0]
    r_out = d / 2 + 1.5
    body = cyl_z(r_out, 0, 3.0).cut(cyl_z(s.r_in, -1, 4))
    r_s = (s.r_in + s.r_t) / 2
    for a in (90, 210, 330):
        x, y = polar(r_s, a)
        body = body.cut(cyl_z(P.M3_CLR / 2, -1, 4).translate((x, y, 0)))
    return body


def idler_tyre():
    """Printed tyre pressed onto a 623 (3x10x4): groove for the cord."""
    w = TYRE_W
    body = cyl_z(TYRE_R, 0, w)
    body = body.cut(cyl_z(P.B623[1] / 2 + 0.05, -1, w + 1))
    # V-ish groove: cut a ring 1 mm deep, 1.6 wide in the middle
    g = cyl_z(TYRE_R + 1, w / 2 - 0.8, w / 2 + 0.8).cut(cyl_z(TYRE_R - 1.0, w / 2 - 0.8, w / 2 + 0.8))
    return body.cut(g)
