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
* RETAINER: ring screwed onto the top of the stator tube (3x M3x10 into 2.5 mm
  pilots); the tube stops CLAMP_GAP below the upper inner race, so the retainer
  presses the race (not the tube) and preloads the pair through the rotor.

Print / support-free notes (all parts):
* stator prints plate down; idler cheeks are buttressed at 45 deg, the idler bolt
  (M3x12) goes in from the outer cheek and self-taps into the inner cheek (no nut
  next to the drum), small bosses space the 623 inner race off the cheeks.
* rotor prints upside down (top plate on the bed): the upper-pocket ceiling is a
  narrow seat (outer race only) + a 45 deg cone, the groove ceilings are 45 deg
  chamfers, knot pockets are teardrops; the lower pocket ceiling is relieved over
  the inner race so the rotor only touches the outer races.

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
BORE_WALL = 6.5            # stator tube wall (r_in .. r_t): room for the retainer screws
CLAMP_GAP = 0.4            # stator tube top below the upper inner race top
SEAT_W = 1.6               # rotor seat width on each outer race
RET_HOLE = 3.3             # retainer screw clearance (M3)
CHEEK_X0, CHEEK_X1 = 3.2, 6.2   # idler cheek faces from the idler centre (x)
CHEEK_HY = 4.5             # idler cheek half depth (y)
BOSS_HOLE = 3.2            # M3 through the 623 spacer bosses (d 4.6): 0.7 mm ring wall


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
        return self.r_t - BORE_WALL

    @property
    def r_screw(self):  # retainer screw circle (middle of the tube wall)
        return self.r_in + BORE_WALL / 2

    @property
    def r_seat(self):   # inner edge of the rotor's seats (outer race only)
        return self.bearing[1] / 2 - SEAT_W

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
    z_top = s.z_flange - CLAMP_GAP
    body = cyl_z(r_plate, s.in_z, 0)
    body = body.union(cyl_z(s.r_t, 0, z_top))
    body = body.union(cyl_z(s.r_t + 1.5, -0.5, 1.0))           # shoulder under the lower inner race (not its seal)
    cuts = [cyl_z(s.r_in, s.in_z - 1, s.z_flange + 1)]
    for x, zg in ((s.R, s.z_a), (-s.R, s.z_b)):
        zc = zg - IDLER_CORD_R
        inward = -1 if x > 0 else 1                             # towards the roll axis
        h = zc - 3.0                                            # buttress height
        for side in (-1, 1):
            c0, c1 = sorted((x + side * CHEEK_X0, x + side * CHEEK_X1))
            body = body.union(box(c0, c1, yi - CHEEK_HY, yi + CHEEK_HY, -0.5, zc + 5.0))
            # 45 deg buttress on the far side (away from the drum, clear of the cord leg)
            y0 = yi + CHEEK_HY
            web = (cq.Workplane("YZ").workplane(offset=c0)
                   .polyline([(y0 - 1, -0.5), (y0 + h, -0.5), (y0, h), (y0 - 1, h)]).close().extrude(c1 - c0))
            body = body.union(web)
            # spacer boss on the inside face: touches only the 623 inner race
            xf = x + side * CHEEK_X0
            body = body.union(rod((xf, yi, zc), (xf - side * 1.15, yi, zc), 4.6))
            # bolt: clearance in the outer cheek, self-tapping pilot in the inner one
            d = P.M3_CLR if side != inward else 2.5
            cuts.append(rod((c0 - 1, yi, zc), (c1 + 1, yi, zc), d))
        # bolt through both spacer bosses, face to face of the cheeks (the cheek holes
        # start 1 mm outside the cheek, so they don't reach the boss tips)
        cuts.append(rod((x - CHEEK_X0 - 0.2, yi, zc), (x + CHEEK_X0 + 0.2, yi, zc), BOSS_HOLE))
        # Bowden socket: tube from below, cord exits up to the idler
        cuts.append(cyl_z(P.TUBE_OD / 2 + 0.15, s.in_z - 1, -2.0).translate((x, sy, 0)))
        cuts.append(cyl_z(P.TUBE_ID / 2 + 0.2, -2.5, 1.0).translate((x, sy, 0)))
    # retainer screw pilots (self-tapping M3) in the top of the tube
    for a in (90, 210, 330):
        x, y = polar(s.r_screw, a)
        cuts.append(cyl_z(1.25, z_top - 10, z_top + 1).translate((x, y, 0)))
    # inserts underneath for the link below
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        cuts.append(cyl_z(P.INSERT_D / 2, s.in_z - 1, s.in_z + P.INSERT_DEPTH).translate((x, y, 0)))
    return cut_all(body, cuts)


def ring_cut(pts):
    """Solid of revolution about Z from an (r, z) polygon."""
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))


def teardrop(p0, p1, d, up=(0, 0, -1)):
    """Horizontal hole of diameter d from p0 to p1 with a 45 deg roof towards `up`
    (the print's +Z; the rotor prints upside down, so assembled -Z)."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    u = cq.Vector(*up)
    u = (u - v.normalized() * u.dot(v.normalized())).normalized()
    pl = cq.Plane(origin=p0, xDir=u, normal=v)
    r = d / 2
    k = r / math.sqrt(2)
    tip = cq.Workplane(pl).polyline([(0, 0), (k, k), (r * math.sqrt(2), 0), (k, -k)]).close().extrude(v.Length)
    return cq.Workplane(pl).circle(r).extrude(v.Length).union(tip)


def rotor(s: RollSpec):
    """Printed upside down (top plate on the bed): every ceiling in that
    orientation is either a narrow ledge or a 45 deg slope."""
    d, D, B = s.bearing
    F = P.FLANGES[s.rotor_cls]
    rp = D / 2 + P.FIT                  # bearing pocket radius
    rc = s.r_t + 0.6                    # clearance around the stator tube
    zl = 1.0 + B                        # lower pocket ceiling (seat on the lower outer race)
    body = cyl_z(s.r_o, 1.0, s.z_flange).union(cyl_z(F["od"], s.z_flange, s.out_z))
    zt = s.z_top_pocket
    cuts = [
        cyl_z(rp, 0, zl),                                       # lower bearing pocket (opens down)
        cyl_z(s.r_seat, zl - 0.1, zl + 0.5),                    # relief: no contact with the lower inner race
        cyl_z(rc, 0, s.out_z + 1),                              # clearance around the stator tube
        cyl_z(rp, zt, s.out_z + 1),                             # upper pocket (opens up through the plate)
        # upper seat = outer race only; below it a short wall and a cone (~50 deg from
        # horizontal) down to the tube clearance, so the print ceiling is a 1.75 mm ledge
        ring_cut([(0, zt + 0.01), (s.r_seat, zt + 0.01), (s.r_seat, zt - 0.6),
                  (rc, zt - 0.6 - 1.2 * (s.r_seat - rc)), (0, zt - 0.6 - 1.2 * (s.r_seat - rc))]),
    ]
    w = 2.4
    wa, wb = s.wraps()
    for zg, ang, sign, wrap in ((s.z_a, 0.0, -1, wa), (s.z_b, 180.0, +1, wb)):
        # groove: flat bottom + 1 mm flat lip, then the (print-)ceiling side opens at 45 deg
        r1, r2 = s.r_groove + 1.0, s.r_o + 1.0
        cuts.append(ring_cut([(s.r_groove, zg + w / 2), (s.r_groove, zg - w / 2), (r1, zg - w / 2),
                              (r2, zg - w / 2 - (r2 - r1)), (r2, zg + w / 2)]))
        anchor = ang + sign * wrap
        x0, y0 = polar(s.r_groove + 0.3, anchor)
        x1, y1 = polar(s.r_o + 1.0, anchor + sign * 65)
        cuts.append(teardrop((x0, y0, zg), (x1, y1, zg), P.CORD_D + 1.0))
        dv = cq.Vector(x1 - x0, y1 - y0, 0).normalized()
        cuts.append(teardrop((x1 - dv.x * 3.5, y1 - dv.y * 3.5, zg), (x1 + dv.x * 2, y1 + dv.y * 2, zg), 4.5))
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        cuts.append(cyl_z(P.INSERT_D / 2, s.out_z - P.INSERT_DEPTH, s.out_z + 1).translate((x, y, 0)))
    return cut_all(body, cuts)


def retainer(s: RollSpec):
    d = s.bearing[0]
    r_out = d / 2 + 1.2             # covers the inner race only (stays off the 2RS seal)
    body = cyl_z(r_out, 0, 3.0).cut(cyl_z(s.r_in, -1, 4))
    for a in (90, 210, 330):
        x, y = polar(s.r_screw, a)
        body = body.cut(cyl_z(RET_HOLE / 2, -1, 4).translate((x, y, 0)))
    return body


def idler_tyre():
    """Printed tyre pressed onto a 623 (3x10x4): groove for the cord."""
    w = TYRE_W
    body = cyl_z(TYRE_R, 0, w)
    body = body.cut(cyl_z(P.B623[1] / 2 + 0.05, -1, w + 1))
    # V-ish groove: cut a ring 1 mm deep, 1.6 wide in the middle
    g = cyl_z(TYRE_R + 1, w / 2 - 0.8, w / 2 + 0.8).cut(cyl_z(TYRE_R - 1.0, w / 2 - 0.8, w / 2 + 0.8))
    return body.cut(g)
