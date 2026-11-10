"""Low-level CadQuery helpers."""
import math
import cadquery as cq


def cyl_x(r, x0, x1):
    return cq.Workplane("YZ").workplane(offset=x0).circle(r).extrude(x1 - x0)


def cyl_y(r, y0, y1):
    return cq.Workplane("XZ").workplane(offset=-y1).circle(r).extrude(y1 - y0)


def cyl_z(r, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).circle(r).extrude(z1 - z0)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _perp(v):
    a = cq.Vector(1, 0, 0) if abs(v.x) < 0.9 * v.Length else cq.Vector(0, 1, 0)
    return v.cross(a).normalized()


def rod(p0, p1, d):
    """Cylinder of diameter d from p0 to p1."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Workplane(cq.Plane(origin=p0, xDir=_perp(v), normal=v)).circle(d / 2).extrude(v.Length)


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def yz(r, deg):
    """Point in the YZ plane, angle from +Y towards +Z."""
    return polar(r, deg)


def hex_z(z0, z1, x, y, af, rot=0.0):
    return (cq.Workplane("XY").workplane(offset=z0).center(x, y)
            .polygon(6, af / math.cos(math.pi / 6)).extrude(z1 - z0)
            .rotate((x, y, 0), (x, y, 1), rot))


def hex_x(x0, x1, y, z, af):
    return (cq.Workplane("YZ").workplane(offset=x0).center(y, z)
            .polygon(6, af / math.cos(math.pi / 6)).extrude(x1 - x0))


def sector_x(x0, x1, r, a0, a1):
    """Pie slice (around the X axis) between angles a0..a1 (deg, from +Y towards +Z)."""
    am = (a0 + a1) / 2
    return (cq.Workplane("YZ").workplane(offset=x0).moveTo(0, 0)
            .lineTo(*yz(r, a0)).threePointArc(yz(r, am), yz(r, a1)).close().extrude(x1 - x0))


def sector_z(z0, z1, r, a0, a1):
    am = (a0 + a1) / 2
    return (cq.Workplane("XY").workplane(offset=z0).moveTo(0, 0)
            .lineTo(*polar(r, a0)).threePointArc(polar(r, am), polar(r, a1)).close().extrude(z1 - z0))


def insert_holes_down(spec, z_face, angles=None, depth=None):
    """Heat-set insert holes going DOWN from a top face at z_face."""
    from . import params as P
    depth = depth or P.INSERT_DEPTH
    out = None
    for a in angles or P.bolt_angles(spec["n"]):
        x, y = polar(spec["bc"], a)
        h = cyl_z(P.INSERT_D / 2, z_face - depth, z_face + 1).translate((x, y, 0))
        out = h if out is None else out.union(h)
    return out


def insert_holes_up(spec, z_face, angles=None, depth=None):
    """Heat-set insert holes going UP from a bottom face at z_face."""
    from . import params as P
    depth = depth or P.INSERT_DEPTH
    out = None
    for a in angles or P.bolt_angles(spec["n"]):
        x, y = polar(spec["bc"], a)
        h = cyl_z(P.INSERT_D / 2, z_face - 1, z_face + depth).translate((x, y, 0))
        out = h if out is None else out.union(h)
    return out


def cut_all(body, cuts):
    for c in cuts:
        if c is not None:
            body = body.cut(c)
    return body
