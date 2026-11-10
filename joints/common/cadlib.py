"""Small CadQuery helpers shared by the joint models."""
import math
import cadquery as cq


def cyl_x(r, x0, x1):
    """Cylinder along X from x0 to x1."""
    return cq.Workplane("YZ").workplane(offset=x0).circle(r).extrude(x1 - x0)


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def _perp(v):
    a = cq.Vector(1, 0, 0) if abs(v.x) < 0.9 * v.Length else cq.Vector(0, 1, 0)
    return v.cross(a).normalized()


def hole_along(p0, p1, d):
    """Cylinder of diameter d from point p0 to p1."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane(cq.Plane(origin=p0, xDir=_perp(v), normal=v)).circle(d / 2).extrude(v.Length)


def polar_yz(r, phi_deg):
    """Point in the YZ plane; phi measured from +Y towards +Z."""
    p = math.radians(phi_deg)
    return r * math.cos(p), r * math.sin(p)


def groove_ring(x, r_in, r_out, w):
    """Annular cut centred at x: everything between r_in and r_out+2, width w."""
    return cyl_x(r_out + 2, x - w / 2, x + w / 2).cut(cyl_x(r_in, x - w / 2, x + w / 2))


def chord_anchor(x, r_groove, r_out, wrap_deg, sign=1, cord_d=1.0, knot_d=4.5):
    """Chord hole from the groove (at wrap_deg) through the wall to a knot pocket.

    sign=+1: tendon wraps over the top. sign=-1: tendon wraps under.
    """
    y0, z0 = polar_yz(r_groove + 0.3, sign * wrap_deg)
    y1, z1 = polar_yz(r_out + 1, sign * (wrap_deg + 65))
    h = hole_along((x, y0, z0), (x, y1, z1), cord_d + 1.0)
    d = cq.Vector(y1 - y0, z1 - z0, 0).normalized()
    k = hole_along((x, y1 - d.x * 3.5, z1 - d.y * 3.5), (x, y1 + d.x * 2, z1 + d.y * 2), knot_d)
    return h.union(k)


def hex_prism_x(x0, x1, y, z, af):
    """Hex nut pocket along X, flats `af` across."""
    return (cq.Workplane("YZ").workplane(offset=x0).center(y, z)
            .polygon(6, af / math.cos(math.pi / 6)).extrude(x1 - x0))


def hex_prism_z(z0, z1, x, y, af):
    return (cq.Workplane("XY").workplane(offset=z0).center(x, y)
            .polygon(6, af / math.cos(math.pi / 6)).extrude(z1 - z0))


def external_tangent(r1, r2, zc2):
    """Tangent line between a circle r1 at (y=0,z=0) and r2 at (y=0,z=zc2), both on +Y side.

    Returns ((y1, z1), (y2, z2)) tangent points.
    """
    nz = (r1 - r2) / zc2
    ny = math.sqrt(1 - nz * nz)
    return (r1 * ny, r1 * nz), (r2 * ny, zc2 + r2 * nz)
