"""Do the Bowden tubes that pass THROUGH a hinge's open centre still fit at its joint limits?

    .venv/bin/python tools/check_tube_bends.py            # all hinges
    .venv/bin/python tools/check_tube_bends.py W2 E       # some

Inside a hinge the passing tubes run from the base slot (along -Z) to the rotor opening
(along the rotor's +Z), so at joint angle q they bend by |q| around the joint axis. A PTFE
tube can't kink: it needs a bend radius >= R_MIN.

Path model. The bundle (tubes side by side across the cavity in x, extra rows stacked in
the bending plane) is two straight runs - one in the base slot along -Z, one in the rotor
opening along the rotor's +Z - joined by a circular fillet of radius r_b (the innermost
row bends at r_b - row offset). Two placements of the straight runs are tried and the
better one counts:
  * centred: both runs on the slot centre lines (they cross on the joint axis);
  * pushed out: each run shifted to the OUTER side of the bend, as far as its slot lets the
    whole bundle go (minus 0.2 mm). That is where a bent tube actually lies - its stiffness
    pulls the bend open, so it rides on the outer wall of each slot - and it is no more
    than the slot allows, so it stays conservative (a real tube can also start bending
    inside the slots, which this model doesn't use).
For each hinge and limit it finds the largest r_b (<= 60 mm) for which no tube touches the
base or the rotor halves. The joint passes when the innermost tube's radius is >= R_MIN.
"""
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cadquery as cq  # noqa: E402

from arm import assembly as A  # noqa: E402
from arm import hinge as H  # noqa: E402
from arm import params as P  # noqa: E402
from arm.cad import rod  # noqa: E402

# Minimum bend radius of a 2 mm ID x 4 mm OD (1 mm wall) PTFE tube: 3/4" = 19.05 mm.
# Source: Pexco Altaflo 100 PTFE technical data sheet, "Bend Radius and Working Pressure
# Chart", metric row 100-0157-040-XX (0.157" OD, 0.040" wall, 0.079" ID): bend radius 3/4".
# https://www.biopharmworld.com/assets/altaflo-100-ptfe-tubing-datasheet.pdf (also pexco.com)
# It is a kink limit; a cord drags more in a tighter bend, so more radius is always better.
R_MIN = 19.05
PITCH = P.TUBE_OD + 0.3
MARGIN = 0.2          # clearance kept to the slot wall when the bundle is pushed out


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


def shifts(s):
    """How far the bundle can move to the outer side of the bend: (in the base slot, in the rotor opening)."""
    half = max(abs(o) for _, o in bundle(s)) + P.TUBE_OD / 2
    return max(0.0, s.slot_y - half - MARGIN), max(0.0, s.slot_y + 2.0 - half - MARGIN)


def corner(q, a1, a2):
    """(y, z) where the base run (y = a1) meets the rotor run (rotor y' = a2) at joint angle q."""
    th = math.radians(q)
    return np.array([0.0, a1, (a2 - a1 * math.cos(th)) / math.sin(th)])


def path(s, q, r_b, x, o, K=(0.0, 0.0, 0.0), n_arc=16):
    """Tube centre line (list of 3D points) for joint angle q (deg), bend radius r_b, corner K."""
    K = np.asarray(K, float)
    th = math.radians(q)
    d1 = np.array([0, 0, 1.0])
    u2 = np.array([0, -math.sin(th), math.cos(th)])
    if abs(q) < 1e-6:
        return [np.array([x, 0, s.in_z - 5]), np.array([x, 0, s.out_z + 5])]
    n1 = np.array([0, -math.copysign(1, q), 0])         # towards the inside of the bend
    t = r_b * math.tan(abs(th) / 2)
    C = K - t * d1 + r_b * n1
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


def hits(s, q, r_b, parts, K=(0.0, 0.0, 0.0)):
    for x, o in bundle(s):
        for seg in tube_segments(path(s, q, r_b, x, o, K)):
            if any(_touch(seg, w) for w in parts):
                return True
    return False


def placements(s, q):
    """Corner points to try: centred, and both runs pushed to the outer side of the bend."""
    s1, s2 = shifts(s)
    sg = math.copysign(1, q)
    out = [("centred", (0.0, 0.0, 0.0))]
    if s1 > 0 or s2 > 0:
        out.append(("pushed out", tuple(corner(q, sg * s1, sg * s2))))
    return out


def limit(name, q):
    """Largest free bend radius at joint angle q: (r_b or None, placement)."""
    s = A.joint_spec(name)
    base = H.base(s)
    parts = [base] + [r.rotate((0, 0, 0), (1, 0, 0), q) for r in H.rotor_pair(s)]
    rows = max(o for _, o in bundle(s))
    need = R_MIN + rows
    radii = sorted({60, 50, 42, 36, 30, 25, 20, 15, 10, 6, round(need, 2)}, reverse=True)
    best = (None, "-")
    for name_k, K in placements(s, q):
        for r_b in radii:
            if best[0] is not None and r_b <= best[0]:
                break
            if not hits(s, q, r_b, parts, K):
                best = (r_b, name_k)
                break
    return best


def check(names):
    jobs = []
    for name in names:
        s = A.joint_spec(name)
        if not isinstance(s, H.HingeSpec) or s.tubes_through == 0:
            continue
        for q in (s.lo, s.hi):
            if abs(q) >= 1:
                jobs.append((name, q))
    with ProcessPoolExecutor() as ex:
        res = list(ex.map(limit, *zip(*jobs)))
    ok = {n: True for n in names}
    for (name, q), (best, how) in zip(jobs, res):
        s = A.joint_spec(name)
        rows = max(o for _, o in bundle(s))
        inner = None if best is None else best - rows
        good = inner is not None and inner >= R_MIN - 1e-6
        print(f"  {name:3s} {q:+5.0f} deg, {s.tubes_through:2d} tubes ({len(set(o for _, o in bundle(s)))} rows): "
              f"largest free bend radius {best if best else '<6'} mm ({how}), "
              f"innermost tube {f'{inner:.1f}' if inner is not None else '-'} mm "
              f"-> {'ok' if good else f'TOO TIGHT (need >= {R_MIN:.1f})'}")
        ok[name] &= good
    return ok


def check_all(names=None):
    names = names or [n for n in A.JOINTS if isinstance(A.joint_spec(n), H.HingeSpec)]
    print(f"== tubes through the hinges (PTFE 4x2 bend radius >= {R_MIN:.2f} mm, Altaflo datasheet) ==")
    return check(names)


if __name__ == "__main__":
    res = check_all(sys.argv[1:])
    print("\nRESULT:", {k: "PASS" if v else "FAIL" for k, v in res.items()})
