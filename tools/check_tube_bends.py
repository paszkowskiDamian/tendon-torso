"""Do the Bowden tubes that pass THROUGH a hinge's open centre still fit at its joint limits?

    .venv/bin/python tools/check_tube_bends.py            # all hinges
    .venv/bin/python tools/check_tube_bends.py W2 E       # some

Inside a hinge the passing tubes run from the base slot (along -Z) to the rotor opening
(along the rotor's +Z), so at joint angle q they bend by |q| around the joint axis. A PTFE
tube can't kink: it needs a bend radius >= R_MIN. For each hinge and each limit this builds
the tube bundle (tubes side by side across the cavity in x, extra rows stacked in the
bending plane) as a fillet of radius r_b between the two straight runs, and finds the
largest r_b (<= 60 mm) for which no tube touches the base or the rotor halves. The joint
passes when the innermost tube's radius (r_b - row offset) is >= R_MIN.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cadquery as cq  # noqa: E402

from arm import assembly as A  # noqa: E402
from arm import hinge as H  # noqa: E402
from arm import params as P  # noqa: E402
from arm.cad import rod  # noqa: E402

R_MIN = 25.0          # mm, minimum bend radius assumed for a 4x2 PTFE tube (estimate: measure yours)
PITCH = P.TUBE_OD + 0.3


def bundle(s):
    """[(x, o)] tube positions: x across the cavity, o = in-plane offset (+ = inner side)."""
    n = s.tubes_through
    kx = max(1, int((2 * (s.cav - 1.0) - 0.2) // PITCH))    # across the base slot (half width cav - 1)
    rows = math.ceil(n / kx)
    xs = [(i - (kx - 1) / 2) * PITCH for i in range(kx)]
    os_ = [(j - (rows - 1) / 2) * PITCH for j in range(rows)]
    out = []
    for o in os_:
        for x in xs:
            if len(out) < n:
                out.append((x, o))
    return out


def path(s, q, r_b, x, o, n_arc=16):
    """Tube centre line (list of 3D points) for joint angle q (deg), bend radius r_b."""
    th = math.radians(q)
    d1 = np.array([0, 0, 1.0])
    u2 = np.array([0, -math.sin(th), math.cos(th)])
    if abs(q) < 1e-6:
        return [np.array([x, 0, s.in_z - 5]), np.array([x, 0, s.out_z + 5])]
    n1 = np.array([0, -math.copysign(1, q), 0])         # towards the inside of the bend
    t = r_b * math.tan(abs(th) / 2)
    C = -t * d1 + r_b * n1
    pts = [C + (r_b - o) * (-n1)]                          # placeholder start of the arc
    arc = []
    for i in range(n_arc + 1):
        ph = abs(th) * i / n_arc
        v = -n1 * math.cos(ph) + d1 * math.sin(ph)        # from the centre to the arc
        arc.append(C + (r_b - o) * v)
    a0, a1 = arc[0], arc[-1]
    start = a0 - d1 * (a0[2] - (s.in_z - 5))               # straight run down to below the base
    end = a1 + u2 * max(0.0, (s.out_z + 5) - a1 @ u2)      # straight run up through the rotor
    pts = [start] + arc + [end]
    return [p + np.array([x, 0, 0]) for p in pts]


def tube_segments(pts):
    """The tube as separate rods (one per path segment, overlapping a little at the joints)."""
    out = []
    for p, q in zip(pts[:-1], pts[1:]):
        v = q - p
        L = np.linalg.norm(v)
        if L < 1e-3:
            continue
        u = v / L
        out.append(rod(tuple(p - u * 0.6), tuple(q + u * 0.6), P.TUBE_OD))
    return out


def _touch(a, b):
    pa, pb = a.val().BoundingBox(), b.val().BoundingBox()
    if pa.xmax < pb.xmin or pb.xmax < pa.xmin or pa.ymax < pb.ymin or pb.ymax < pa.ymin or pa.zmax < pb.zmin or pb.zmax < pa.zmin:
        return False
    try:
        i = a.intersect(b)
        return bool(i.vals()) and i.val().Volume() > 0.05
    except Exception:                       # OCC boolean failure: fall back to the distance
        return a.val().distance(b.val()) < 0.02


def hits(s, q, r_b, parts):
    for x, o in bundle(s):
        for seg in tube_segments(path(s, q, r_b, x, o)):
            if any(_touch(seg, w) for w in parts):
                return True
    return False


def check(name):
    s = A.joint_spec(name)
    if not isinstance(s, H.HingeSpec) or s.tubes_through == 0:
        return True
    base = H.base(s)
    ra, rb = H.rotor_pair(s)
    rows = max(o for _, o in bundle(s))
    ok = True
    for q in (s.lo, s.hi):
        if abs(q) < 1:
            continue
        parts = [base] + [r.rotate((0, 0, 0), (1, 0, 0), q) for r in (ra, rb)]
        best = None
        for r_b in (60, 50, 42, 36, 30, 25, 20, 15, 10, 6):
            if not hits(s, q, r_b, parts):
                best = r_b
                break
        inner = None if best is None else best - rows
        good = inner is not None and inner >= R_MIN
        print(f"  {name:3s} {q:+5.0f} deg, {s.tubes_through:2d} tubes ({len(set(o for _, o in bundle(s)))} rows): "
              f"largest free bend radius {best if best else '<6'} mm, innermost tube {inner if inner is not None else '-'} mm "
              f"-> {'ok' if good else f'TOO TIGHT (need >= {R_MIN:.0f})'}")
        ok &= good
    return ok


if __name__ == "__main__":
    names = sys.argv[1:] or [n for n in A.JOINTS if isinstance(A.joint_spec(n), H.HingeSpec)]
    print(f"== tubes through the hinges (PTFE bend radius >= {R_MIN:.0f} mm assumed) ==")
    res = {n: check(n) for n in names}
    print("\nRESULT:", {k: "PASS" if v else "FAIL" for k, v in res.items()})
