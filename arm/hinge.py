"""Hinge module family (grown out of J1-S / J2).

Local frame: joint axis = X, base below (z<0), rotor above (z>0), origin on the axis.

* BASE: mounting plate (heat-set inserts on its underside) + two lugs with
  captive M8 nuts (M8x20 stub axles into 608 bearings) + a boss with two
  angled Bowden sockets aimed exactly along the drum tangents + a centre slot
  that lets the tubes of the joints further out pass through.
* ROTOR: two halves split on the motion plane (x=0). A hollow drum: 608s at
  the ends, a cavity in the middle for passing tubes, one tendon groove +
  knotted chord anchor per half, and a top plate with heat-set inserts for
  the next link.

  PRINT ORIENTATION: each half prints TOP PLATE DOWN (model -Z up, i.e.
  rotate 180 deg about X). Printed split-face down instead, the drum end cap
  would have to bridge the open tube window (a ~40 mm cantilever) - no roof
  shape can fix that. Plate down, everything is self-supporting:
    - the half plate is the footprint, the neck rises from it, and the neck
      spans the full drum width (x = 0..-hub_half) and covers the drum top to
      within 44 deg of the side, so the drum never overhangs;
    - the tendon groove runs through the neck as a 2.4 mm wide tunnel
      (2.4 mm bridge) - the cord is threaded through it from the anchor;
    - the cavity end walls and the split face are vertical; the tube window
      is widened to cover every part of the cavity whose roof would be
      steeper than 45 deg (it opens at the top of the print). For the bends of
      the passing tubes it is also widened by a slot-wide band on each side and,
      on S1/E, opened up to the neck on the side of the big bend (tube_window,
      HingeSpec.neck_window, tools/check_tube_bends.py); none of that adds a roof;
    - bearing pockets, the axle clearance hole and lightening holes are
      horizontal -> teardrops (50 deg roofs) pointing up the print (-Z).

Cord A lives on the -X half, cord B on the +X half. Rotating +a winds cord A
on and cord B off (cord A wrap = WA + a, cord B wrap = WB - a).
"""
import math
from dataclasses import dataclass, field

import cadquery as cq

from . import params as P
from .cad import (box, cut_all, cyl_x, cyl_z, hex_x, insert_holes_up, insert_holes_down,
                  polar, rod, sector_x, yz)


WALL = 1.6        # wall between the tube cavity and each 608 pocket (bearing seat)
TEAR = 50.0       # teardrop / roof angle from horizontal in the print (deg)


def teardrop_x(r, x0, x1, y=0.0, z=0.0):
    """Hole along X with a teardrop roof pointing to -Z (= up when the rotor prints plate down)."""
    a = math.radians(90.0 - TEAR)               # half angle of the roof at the tip
    d = r / math.sin(a)
    t = math.radians(TEAR)
    tri = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(y + r * math.cos(t), z - r * math.sin(t)), (y, z - d),
                      (y - r * math.cos(t), z - r * math.sin(t))]).close().extrude(x1 - x0))
    return cyl_x(r, x0, x1).translate((0, y, z)).union(tri)


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
    bearing: tuple = P.B608  # drum bearings (d, D, B)
    axle: str = "M8"         # stub axles (P.AXLES)
    # open the tube window up to the neck opening on the side of this limit ("hi"/"lo"):
    # for big bends the passing tubes' curve leaves the cavity between the two
    # (see tools/check_tube_bends.py)
    neck_window: tuple = ()

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
        return self.cav + WALL + self.bearing[2]

    @property
    def gx(self):
        return self.cav + WALL + self.bearing[2] / 2

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
    # stub axles + captive nuts
    ax = P.AXLES[s.axle]
    for sx in (1, -1):
        x0, x1 = sorted((sx * s.lug_in, sx * s.lug_out))
        cuts.append(cyl_x(ax["clr"] / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - ax["nut_t"] / 2, xm + ax["nut_t"] / 2, -ax["nut_af"] / 2, ax["nut_af"] / 2,
                        -ax["nut_af"] / 2 / math.cos(math.pi / 6), s.lug_r + 1))
    # mounting inserts on the underside
    if s.base_style == "torso":
        for x in (-(s.lug_out + 7), s.lug_out + 7):
            for y in (-17, 17):
                cuts.append(cyl_z(P.INSERT_D / 2, s.in_z - 1, s.in_z + P.INSERT_DEPTH).translate((x, y, 0)))
    else:
        cuts.append(insert_holes_up(F, s.in_z))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ rotor
def tube_window(s: HingeSpec, a0, a1):
    """Tube window through the drum sleeve (cavity x-range only): the sector a0..a1 (deg,
    rotor frame) widened on both sides by a band as wide as the base slot, so the whole
    tube bundle - not just its centre line - clears the sleeve where the straight
    base-side run enters the drum near the axis. The bands' faces are parallel to the
    sector edges, so they print at the same slope."""
    r = s.r_hub + 3
    w = s.slot_y + 0.5
    x0, x1 = -s.cav, 0.1
    out = [sector_x(x0, x1, r, a0, a1)]
    for a, side in ((a0, -90.0), (a1, 90.0)):
        p0 = yz(w, a + side)
        pts = [(0.0, 0.0), yz(r, a), (yz(r, a)[0] + p0[0], yz(r, a)[1] + p0[1]), p0]
        out.append(cq.Workplane("YZ").workplane(offset=x0).polyline(pts).close().extrude(x1 - x0))
    return out


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
    H = s.hub_half
    hub = cyl_x(s.r_hub, -H, 0)
    # neck over the full drum width; wide enough (>= cos 44 deg * r_hub) that the drum
    # never overhangs when printed plate down, and inside the plate footprint
    neck_y = min(0.73 * s.r_hub, F["od"] - 6, math.sqrt(F["od"] ** 2 - H ** 2) - 0.8)
    neck = box(-H, 0, -neck_y, neck_y, 6, s.out_bot + 1)
    plate = cyl_z(F["od"], s.out_bot, s.out_z).intersect(box(-F["od"] - 1, 0, -F["od"] - 1, F["od"] + 1, s.out_bot - 1, s.out_z + 1))
    body = hub.union(neck).union(plate)

    # tube window: the joint sweep + margin, widened to [-138, -42] deg so that no part of
    # the cavity roof is steeper than 45 deg in the print (plate down = -Z up)
    sec0, sec1 = min(sec0, -138.0), max(sec1, -42.0)
    # big bends: the curved tube bundle swings out of the cavity between the window and
    # the neck opening -> open the window all the way to the neck on that side. Inside the
    # cavity x-range only (the grooves, anchors and bearing seats are outboard of it); the
    # end walls still carry the cord load into the neck, and in the print (plate down) the
    # extra cut has no roof: everything above it is the open cavity / window.
    if "hi" in s.neck_window:
        sec0, sec1 = (-270.0, sec1) if side == "A" else (sec0, 90.0)
    if "lo" in s.neck_window:
        sec0, sec1 = (sec0, 90.0) if side == "A" else (-270.0, sec1)
    r_cav = s.r_hub - 8.0 if s.R > 25 else s.r_hub - 7.5
    cuts = [
        cyl_x(r_cav, -s.cav, 0.1),
        teardrop_x(s.bearing[1] / 2 - 1.5, -s.cav - WALL - 0.1, -s.cav + 0.1),  # axle / inner-race clearance
        teardrop_x(s.bearing[1] / 2 + P.FIT, -H - 0.1, -s.cav - WALL),         # bearing pocket
        box(-(s.cav - 0.5), 0.1, -(s.slot_y + 2), s.slot_y + 2, 4, s.out_z + 1),
    ] + tube_window(s, sec0, sec1)
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
    # free-cord clearance: the straight cord from the Bowden socket to the drum tangent is
    # fixed in the base. Near the end of the range where a cord is almost unwound, its
    # tangent point moves under the neck, which would then press on the cord. Cut the band
    # that cord sweeps over the whole range (in the groove plane) out of the neck: a slot as
    # wide as the groove, so its roof in the print is the same 2.4 mm bridge as the tunnel.
    # (Side B is built mirrored, so it sees the joint angle negated.)
    (ay, az), _ = s.tangent()
    b0, b1 = (s.lo, s.hi) if side == "A" else (-s.hi, -s.lo)
    m = max(4.0, math.degrees(1.5 / s.R))
    th_a, r_a = math.degrees(math.atan2(az, ay)), math.hypot(ay, az) + 1.5
    n = int((b1 - b0 + 2 * m) / 2) + 2
    bs = [b0 - m + (b1 - b0 + 2 * m) * i / (n - 1) for i in range(n)]
    band = [yz(s.r_groove, phi_t - b) for b in bs] + [yz(r_a, th_a - b) for b in reversed(bs)]
    cuts.append(cq.Workplane("YZ").workplane(offset=gx - w / 2).polyline(band).close().extrude(w))
    # lighten the side wall (only on big drums)
    if s.R >= 30:
        r_p = s.bearing[1] / 2 + P.FIT
        r_l = (r_p + s.R * math.cos(math.radians(32.5)) - 3) / 2 + 2
        dl = min(10.0, (s.R * math.cos(math.radians(32.5)) - 3 - r_p) - 4)
        for k in range(6):
            yy, zz = yz(r_l, 60 * k)          # clear of the teardrop roof of the bearing pocket
            cuts.append(teardrop_x(dl / 2, -H - 1, -s.cav - WALL, yy, zz))
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
