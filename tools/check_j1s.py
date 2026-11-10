"""Sanity checks + preview renders for J1-S.

    .venv/bin/python tools/check_j1s.py
"""
import math
import sys
from pathlib import Path

import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1s-hinge"))
sys.path.insert(0, str(ROOT / "tools"))
import j1s_hinge as J  # noqa: E402
from render import render, to_mesh  # noqa: E402

IMG = ROOT / "docs" / "img"


def vol(a, b):
    i = a.intersect(b)
    return i.val().Volume() if i.vals() else 0.0


def rod(p0, p1, d):
    return J.hole_along(p0, p1, d)


if __name__ == "__main__":
    half, prox, sp, bs = J.distal_half(), J.proximal(), J.spool(), J.base()
    other = half.rotate((0, 0, 0), (0, 0, 1), 180)
    tb = J.tube_bot()
    for n, w in (("distal_half", half), ("proximal", prox), ("spool", sp), ("base", bs)):
        print(f"{n:12s} valid={w.val().isValid()} solids={len(w.solids().vals())} vol={w.val().Volume():8.0f} mm3")
    for f in sorted((J.OUT).glob("*.stl")):
        print(f"  {f.name:18s} watertight={trimesh.load(f).is_watertight}")

    print("\nrange of motion (distal vs proximal overlap):")
    for a in (-110, -100, -95, -90, 0, 90, 95, 100, 110):
        v = vol(half.rotate((0, 0, 0), (1, 0, 0), a), prox) + vol(other.rotate((0, 0, 0), (1, 0, 0), a), prox)
        print(f"  {a:5d} deg  {v:8.2f} mm3")

    # tendon centrelines as 1 mm rods: must not hit proximal or base
    (y1, z1), (y2, z2) = J.tendon_line()
    base_t = bs.translate((0, 0, tb))
    print("\ntendon rods vs parts (should be ~0):")
    for x, s in ((-J.GROOVE_X, 1), (J.GROOVE_X, -1)):
        r = rod((x, s * y1, z1 - 0.5), (x, s * y2, z2 + 0.5), J.CORD_D)
        print(f"  x={x:+.1f}: proximal {vol(r, prox):.3f}  base {vol(r, base_t):.3f}  distal@0 {vol(r, half)+vol(r, other):.3f}")
    # line passes over the drum top at 0 deg: check the neck is clear of the groove path
    for x in (-J.GROOVE_X, J.GROOVE_X):
        ring = J.cyl_x(J.R_CORD_DRUM + 0.5, x - 0.5, x + 0.5).cut(J.cyl_x(J.R_CORD_DRUM - 0.5, x - 0.5, x + 0.5))
        print(f"  cord ring at drum x={x:+.1f} vs distal: {vol(ring, half)+vol(ring, other):.3f}")

    # Bowden tube check: 4 mm tube, bend radius 25 mm, crossing the joint at 90 deg.
    # Path: up the proximal axis, arc of radius 25 centred inside the bend, then along the distal axis.
    print("\nBowden tube (OD 4, bend R25) at +/-90 deg vs parts:")
    for a in (90, -90):
        s = 1 if a > 0 else -1
        Rb = 25.0
        # arc tangent to z-axis (below) and to the distal axis (direction (0,-s,0) at 90 deg)
        # centre at (y=-s*Rb, z=-Rb) ... corner at origin
        cy, cz = -s * Rb, -Rb
        pts = []
        pts.append((0, 0, J.CAP_TOP + 1))
        for k in range(0, 31):
            t = math.radians(90 * k / 30)
            pts.append((0, cy + s * Rb * math.cos(t), cz + Rb * math.sin(t)))
        pts.append((0, -s * 140, 0))
        path = cq.Workplane().polyline(pts).val()
        tube = cq.Workplane("XY").add(cq.Solid.makeCylinder(0.1, 0.1))  # placeholder
        segs = None
        for p, q in zip(pts[:-1], pts[1:]):
            seg = rod(p, q, J.TUBE_OD)
            segs = seg if segs is None else segs.union(seg)
        dist = half.rotate((0, 0, 0), (1, 0, 0), a).union(other.rotate((0, 0, 0), (1, 0, 0), a).translate((0.01, 0, 0)))
        print(f"  {a:+d} deg: vs distal {vol(segs, dist):.2f} mm3, vs proximal {vol(segs, prox):.2f} mm3")

    # ---- renders
    pm = to_mesh(prox)
    bm = to_mesh(base_t)
    spm = to_mesh(sp.translate((0, 0, J.spool_z())))
    pair = cq.Workplane().add(cq.Compound.makeCompound([half.val(), other.val()]))
    lines = (((-J.GROOVE_X, y1, z1), (-J.GROOVE_X, y2, z2), "tab:red"),
             ((J.GROOVE_X, -y1, z1), (J.GROOVE_X, -y2, z2), "tab:blue"))
    render([(bm, "#9aa3ad", 1), (pm, "#e8a33d", 0.35), (spm, "#4c78a8", 1),
            (to_mesh(pair.rotate((0, 0, 0), (1, 0, 0), 40)), "#59a14f", 0.45)],
           IMG / "j1s_assembly.png", lines=lines)
    parts = [(pm, "#e8a33d", 0.6)]
    for a, col in ((-90, "#76b7b2"), (0, "#59a14f"), (90, "#b07aa1")):
        parts.append((to_mesh(pair.rotate((0, 0, 0), (1, 0, 0), a)), col, 0.35))
    render(parts, IMG / "j1s_range.png", elev=0, azim=0)
    render([(to_mesh(half), "#59a14f", 1)], IMG / "j1s_distal_half.png", elev=12, azim=-150)
    render([(to_mesh(sp), "#4c78a8", 1)], IMG / "j1s_spool.png", elev=20, azim=-60, size=5)
    render([(bm, "#9aa3ad", 1), (spm, "#4c78a8", 1)], IMG / "j1s_base.png", elev=25, azim=-130)
    print("images ->", IMG)
