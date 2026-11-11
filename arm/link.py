"""Printable links: tubes with outward bolting lips at both ends.

Bottom lip: bolt heads on its TOP face (outside the tube) going down into the
module below. Top lip: heads UNDER it going up into the module above.

Print standing on the bottom lip, no supports:
* the top lip sits on a full cone whose underside rises CONE_SLOPE mm per mm of
  radius (48 deg), from inside the tube wall out to the lip rim;
* every top-lip bolt gets a vertical counterbore (HEAD_R) up through that cone,
  open from below; its ceiling is a small round bridge around the M3 hole;
* links that get wider (B tube > A tube) keep the narrow A tube and let the cone do
  the flare; the bore widens in a 48 deg cone just below the top lip (the Bowden
  sockets of the roll stators above sit out at r ~ 37-39);
* where the cone would bury a bottom-lip head, a vertical counterbore from the bore
  runs down to it, so the bolt and a hex key go in from inside the link;
* short spacers (the cone reaches the bottom lip) get a radial slot at each bolt so
  both heads go in from the side, as before.
"""
import cadquery as cq

from . import params as P
from .cad import box, cyl_z, polar

CONE_SLOPE = 1.1               # rise per mm of radius (> 1 -> steeper than 45 deg)
HEAD_R = P.M3_HEAD_D / 2 + 0.4  # socket-head counterbore radius
KEY_CLEAR = 18.0               # free height wanted above a bottom-lip head (M3x10 + key)
WALL = 2.0                     # tube wall: 5 perimeters; bending stress stays < 2 MPa (statics)


def _revolve_z(pts):
    """Solid of revolution about Z from (r, z) profile points."""
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))


def link(cls_a, cls_b, length, wall=WALL):
    A, B = P.FLANGES[cls_a], P.FLANGES[cls_b]
    lip = P.LIP_T
    top0 = length - lip                          # underside of the top lip
    ra_b, rb_b = A["tube_r"] - wall, B["tube_r"] - wall
    flaring = B["tube_r"] > A["tube_r"]

    # tube + bore
    if flaring:
        outer = cyl_z(A["tube_r"], 0, top0 + 0.5)
        z_f = top0 - (rb_b - ra_b) * CONE_SLOPE  # bore starts to widen here
        assert z_f > lip, f"{cls_a}->{cls_b}: link too short for the bore flare"
        inner = _revolve_z([(0, -1), (ra_b, -1), (ra_b, z_f), (rb_b, top0), (rb_b, length + 1), (0, length + 1)])

        def r_bore(z):
            return ra_b if z <= z_f else min(rb_b, ra_b + (z - z_f) / CONE_SLOPE)
    else:
        outer = cq.Workplane("XY").circle(A["tube_r"]).workplane(offset=length).circle(B["tube_r"]).loft()
        inner = (cq.Workplane("XY").workplane(offset=-0.01).circle(ra_b)
                 .workplane(offset=length + 0.02).circle(rb_b).loft())

        def r_bore(z):
            return ra_b + (rb_b - ra_b) * z / length
    body = outer.union(cyl_z(A["od"], 0, lip)).union(cyl_z(B["od"], top0, length))

    # cone under the top lip: from inside the tube wall out to the lip rim
    r_in = min(A["tube_r"], B["tube_r"]) - 1.0
    zc = top0 - (B["od"] - r_in) * CONE_SLOPE
    cone = _revolve_z([(r_in, zc), (B["od"], top0), (B["od"], top0 + 0.5), (r_in, top0 + 0.5)])
    if zc < 0:
        cone = cone.intersect(box(-200, 200, -200, 200, 0, length))

    def z_cone(r):                               # cone underside at radius r
        return zc + (r - r_in) * CONE_SLOPE

    spacer = z_cone(B["bc"] - HEAD_R) < lip + P.M3_HEAD_H + 10.0
    for ang in P.bolt_angles(B["n"]):
        x, y = polar(B["bc"], ang)
        cone = cone.cut(cyl_z(HEAD_R, lip, top0).translate((x, y, 0)))
        if spacer:
            slot = (box(0, B["od"] + 2 - B["bc"], -HEAD_R, HEAD_R, lip, top0)
                    .translate((B["bc"], 0, 0)).rotate((0, 0, 0), (0, 0, 1), ang))
            cone = cone.cut(slot)
    body = body.union(cone).cut(inner)

    # bottom-lip heads buried by the cone/tube -> counterbore down from the bore
    if not spacer:
        x, y = polar(A["bc"], P.bolt_angles(A["n"])[0])
        probe = cyl_z(HEAD_R - 0.2, lip + 0.05, lip + P.M3_HEAD_H + KEY_CLEAR).translate((x, y, 0))
        hit = body.intersect(probe)
        if hit.vals() and hit.val().Volume() > 0.5:
            r_need = A["bc"] + HEAD_R + 0.3
            z_open = next((z / 4 for z in range(int(4 * lip), int(4 * top0)) if r_bore(z / 4) >= r_need), None)
            assert z_open is not None and z_open < top0 - 1, f"{cls_a}->{cls_b}: bottom bolts unreachable"
            for ang in P.bolt_angles(A["n"]):
                x, y = polar(A["bc"], ang)
                body = body.cut(cyl_z(HEAD_R, lip, z_open + 1).translate((x, y, 0)))

    for spec, z0, z1 in ((A, -1, lip + 1), (B, top0 - 1, length + 1)):
        for ang in P.bolt_angles(spec["n"]):
            x, y = polar(spec["bc"], ang)
            body = body.cut(cyl_z(P.M3_CLR / 2, z0, z1).translate((x, y, 0)))
    return body


def yoke_link(cls_a, cls_b, length, x_s, z_pin, top_tip=12.0, half_w=10.0, cheek=5.5, slot=3.0):
    """Spacer link + two gravity-balance yoke arms (see balance.py).

    Same flanges and length as link() (the kinematics do not change). Each arm is
    a flat bar along +-X on the bed (z = 0, the S1 side), y in +-half_w (clear of
    the radial bolt-head slots at +-22.5 deg), full height next to the lips and
    sloping down to top_tip at the tip (clear of the S2 rotor / upper arm swinging
    out at full abduction). The tip is a fork: a `slot`-wide cord slot in the plane
    x = +-x_s, open towards -Z and +-Y (the cord leaves towards the pulley over the
    whole flexion range), bridged above the pin; an M3 pin along X at z_pin carries
    the cord loop (a loop pivots on the pin, so the cord line always passes through
    the pin centre = point B).
    """
    body = link(cls_a, cls_b, length)
    A = P.FLANGES[cls_a]
    x0 = A["tube_r"] - 2.0                     # inside the tube wall, outside the bore
    x_flat = A["od"]                           # full height out to the lip radius
    x_tip = x_s + slot / 2 + cheek
    for sx in (1, -1):
        prof = [(x0, 0), (x_tip, 0), (x_tip, top_tip), (x_s + slot / 2 + 1.0, top_tip),
                (x_flat, length), (x0, length)]
        arm = (cq.Workplane("XZ").polyline([(sx * x, z) for x, z in prof]).close()
               .extrude(half_w).translate((0, half_w, 0)).union(
               cq.Workplane("XZ").polyline([(sx * x, z) for x, z in prof]).close()
               .extrude(-half_w).translate((0, -half_w, 0))))
        arm = arm.cut(box(-1, 1, -half_w - 1, half_w + 1, -1, z_pin + 3.5)
                      .translate((sx * x_s, 0, 0)).union(
                      box(-slot / 2, slot / 2, -half_w - 1, half_w + 1, -1, z_pin + 3.5)
                      .translate((sx * x_s, 0, 0))))
        pin = cq.Workplane("YZ").workplane(offset=x_s - slot / 2 - cheek - 1).circle(1.6).extrude(2 * cheek + slot + 2)
        if sx < 0:
            pin = pin.mirror("YZ")
        body = body.union(arm.cut(pin.translate((0, 0, z_pin))))
    return body
