"""Hinge module family (grown out of J1-S / J2).

Local frame: joint axis = X, base below (z<0), rotor above (z>0), origin on the axis.

* BASE: mounting plate (heat-set inserts on its underside) + two lugs with
  captive M8 nuts (M8x20 stub axles into 608 bearings) + a boss with two
  angled Bowden sockets aimed exactly along the drum tangents + a centre slot
  that lets the tubes of the joints further out pass through.
* ROTOR: printed as two halves split on the motion plane (x=0), each printed
  flat on that face. A hollow drum: 608s at the ends, a cavity in the middle
  for passing tubes, one tendon groove + knotted chord anchor per half, and a
  top plate with heat-set inserts for the next link.

Cord A lives on the -X half, cord B on the +X half. Rotating +a winds cord A
on and cord B off (cord A wrap = WA + a, cord B wrap = WB - a).
"""
import math
from dataclasses import dataclass, field

import cadquery as cq

from . import params as P
from .cad import (box, cut_all, cyl_x, cyl_z, hex_x, insert_holes_up, insert_holes_down,
                  polar, rod, sector_x, yz)


@dataclass
class HingeSpec:
    name: str
    cls: str                 # flange class of base plate AND rotor plate
    R: float                 # tendon centre radius on the drum (moment arm)
    cav: float               # half width of the central tube cavity
    lo: float                # joint range (deg)
    hi: float
    lug_r: float = 16.0
    lug_t: float = 12.0
    slot_y: float = 6.0      # half length (y) of the pass-through slot
    base_style: str = "flange"   # or "torso": narrow rectangular plate for the shoulder beam
    tubes_through: int = 0   # how many Bowden tubes pass through (documentation / checks)
    rotor_cls: str = ""      # flange class of the rotor plate (default: same as cls)
    neck_extra: float = 0.0  # longer neck -> rotor plate further from the axis (more range)

    @property
    def rcls(self):
        return self.rotor_cls or self.cls

    # ---- derived
    @property
    def r_hub(self):
        return self.R + 2.0

    @property
    def r_groove(self):
        return self.R - P.CORD_D / 2

    @property
    def hub_half(self):
        return self.cav + 1.0 + P.B608[2]

    @property
    def gx(self):
        return self.cav + 1.0 + P.B608[2] / 2

    @property
    def lug_in(self):
        return self.hub_half + 1.0

    @property
    def lug_out(self):
        return self.lug_in + self.lug_t

    @property
    def boss_top(self):
        return -(self.r_hub + 8.0)

    @property
    def plate_top(self):
        return self.boss_top - 10.0

    @property
    def in_z(self):            # bottom face of the base plate
        return self.plate_top - P.PLATE_T

    @property
    def out_bot(self):         # underside of the rotor plate
        return self.r_hub + 12.0 + self.neck_extra

    @property
    def out_z(self):           # top face of the rotor plate
        return self.out_bot + P.PLATE_T

    @property
    def anchor_y(self):
        return max(5.0, 0.25 * self.R)

    def tangent(self):
        """Tube exit A (y, z) and drum tangent T (y, z, phi_deg) for cord A (+Y side)."""
        ay, az = self.anchor_y, self.boss_top
        d = math.hypot(ay, az)
        phi = math.atan2(az, ay) + math.acos(self.R / d)
        return (ay, az), (self.R * math.cos(phi), self.R * math.sin(phi), math.degrees(phi))

    def wraps(self):
        """Neutral wrap (deg, from the tangent point) of cord A and cord B."""
        wa = max(25.0 - self.lo, min(135.0, 290.0 - self.hi))
        wb = max(25.0 + self.hi, min(135.0, 290.0 + self.lo))
        return wa, wb


# ------------------------------------------------------------------ base
def base(s: HingeSpec):
    F = P.FLANGES[s.cls]
    if s.base_style == "torso":
        plate = box(-(s.lug_out + 14), s.lug_out + 14, -26, 26, s.in_z, s.plate_top).edges("|Z").fillet(6)
    else:
        plate = cyl_z(max(F["od"], math.hypot(s.lug_out, s.lug_r) + 2), s.in_z, s.plate_top)
    boss_x = s.lug_in - 0.5
    boss_y = s.anchor_y + 9.0
    body = plate.union(box(-boss_x, boss_x, -boss_y, boss_y, s.plate_top - 0.5, s.boss_top))
    for sx in (1, -1):
        x0, x1 = sorted((sx * s.lug_in, sx * s.lug_out))
        body = body.union(box(x0, x1, -s.lug_r, s.lug_r, s.plate_top - 0.5, 0).union(cyl_x(s.lug_r, x0, x1)))
    cuts = []
    # Bowden sockets: cord A at (-gx, +y), cord B at (+gx, -y)
    (ay, az), (ty, tz, _) = s.tangent()
    u = cq.Vector(0, ty - ay, tz - az).normalized()
    for sx, sy in ((-1, 1), (1, -1)):
        x = sx * s.gx
        A = cq.Vector(x, sy * ay, az)
        d = cq.Vector(0, sy * u.y, u.z)
        B = A - d * 8.0
        C = A - d * ((az - s.in_z + 2) / u.z)
        cuts.append(rod((B).toTuple(), (A + d).toTuple(), P.TUBE_ID + 0.4))
        cuts.append(rod(C.toTuple(), B.toTuple(), P.TUBE_OD + 0.3))
    # pass-through slot
    if s.cav >= 3:
        sx_ = s.cav - 1.0
        cuts.append(box(-sx_, sx_, -s.slot_y, s.slot_y, s.in_z - 1, s.boss_top + 1).edges("|Z").fillet(min(2.0, sx_ - 0.5)))
    # M8 stub axles + captive nuts
    for sx in (1, -1):
        x0, x1 = sorted((sx * s.lug_in, sx * s.lug_out))
        cuts.append(cyl_x(P.M8_CLR / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - P.M8_NUT_T / 2, xm + P.M8_NUT_T / 2, -P.M8_NUT_AF / 2, P.M8_NUT_AF / 2,
                        -P.M8_NUT_AF / 2 / math.cos(math.pi / 6), s.lug_r + 1))
    # mounting inserts on the underside
    if s.base_style == "torso":
        for x in (-(s.lug_out + 7), s.lug_out + 7):
            for y in (-17, 17):
                cuts.append(cyl_z(P.INSERT_D / 2, s.in_z - 1, s.in_z + P.INSERT_DEPTH).translate((x, y, 0)))
    else:
        cuts.append(insert_holes_up(F, s.in_z))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ rotor
def rotor_half(s: HingeSpec, side="A"):
    """Rotor half. side A = the -X half as modelled. side B is returned already in place (+X half)."""
    F = P.FLANGES[s.rcls]
    wa, wb = s.wraps()
    _, (_, _, phi_t) = s.tangent()
    if side == "A":
        wrap = wa
        sec0, sec1 = -90 - s.hi - 12, -90 - s.lo + 12
    else:
        # build B as a mirrored A (phi -> 180-phi under the final Rz180) with its own wrap
        wrap = wb
        sec0, sec1 = -90 + s.lo - 12, -90 + s.hi + 12
    hub = cyl_x(s.r_hub, -s.hub_half, 0)
    neck_x = -(s.gx - 2.2)
    neck_y = min(0.6 * s.r_hub, F["od"] - 6)
    neck = box(neck_x, 0, -neck_y, neck_y, 6, s.out_bot + 1)
    plate = cyl_z(F["od"], s.out_bot, s.out_z).intersect(box(-F["od"] - 1, 0, -F["od"] - 1, F["od"] + 1, s.out_bot - 1, s.out_z + 1))
    body = hub.union(neck).union(plate)

    r_cav = s.r_hub - 8.0 if s.R > 25 else s.r_hub - 7.5
    cuts = [
        cyl_x(r_cav, -s.cav, 0.1),
        cyl_x(9.5, -s.cav - 1.1, -s.cav + 0.1),
        cyl_x(P.B608[1] / 2 + P.FIT, -s.hub_half - 0.1, -s.cav - 1.0),
        box(-(s.cav - 0.5), 0.1, -(s.slot_y + 2), s.slot_y + 2, 4, s.out_z + 1),
        sector_x(-s.cav, 0.1, s.r_hub + 3, sec0, sec1),
    ]
    # tendon groove + anchor
    w = 2.4
    gx = -s.gx
    cuts.append(cyl_x(s.r_hub + 2, gx - w / 2, gx + w / 2).cut(cyl_x(s.r_groove, gx - w / 2, gx + w / 2)))
    anchor = phi_t + wrap
    y0, z0 = yz(s.r_groove + 0.3, anchor)
    y1, z1 = yz(s.r_hub + 1, anchor + 65)
    cuts.append(rod((gx, y0, z0), (gx, y1, z1), P.CORD_D + 1.0))
    dv = cq.Vector(y1 - y0, z1 - z0, 0).normalized()
    cuts.append(rod((gx, y1 - dv.x * 3.5, z1 - dv.y * 3.5), (gx, y1 + dv.x * 2, z1 + dv.y * 2), 4.5))
    # lighten the side wall (only on big drums)
    if s.R >= 30:
        r_l = (11.15 + s.R * math.cos(math.radians(32.5)) - 3) / 2 + 2
        dl = min(10.0, (s.R * math.cos(math.radians(32.5)) - 3 - 11.15) - 4)
        for k in range(6):
            yy, zz = yz(r_l, 30 + 60 * k)
            cuts.append(cyl_x(dl / 2, -s.hub_half - 1, -s.cav - 1).translate((0, yy, zz)))
    # inserts in the top plate (from the top face down), only the ones on this half
    for a in P.bolt_angles(F["n"]):
        x, y = polar(F["bc"], a)
        if x < 0:
            cuts.append(cyl_z(P.INSERT_D / 2, s.out_z - P.INSERT_DEPTH, s.out_z + 1).translate((x, y, 0)))
    # centre opening of the top plate
    cuts.append(box(-(s.cav - 0.5), 0.1, -(s.slot_y + 2), s.slot_y + 2, s.out_bot - 1, s.out_z + 1))
    body = cut_all(body, cuts)
    if side == "B":
        body = body.rotate((0, 0, 0), (0, 0, 1), 180)
    return body


def rotor_pair(s: HingeSpec):
    return rotor_half(s, "A"), rotor_half(s, "B")
