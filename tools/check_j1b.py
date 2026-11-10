"""Checks + renders for J1b (J1-S elbow + J2 wrist + Bowden drive).

    .venv/bin/python tools/check_j1b.py
"""
import sys
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1b-2dof"))
sys.path.insert(0, str(ROOT / "tools"))
import j1b as B  # noqa: E402
from render import render, to_mesh  # noqa: E402

S = B.S
IMG = ROOT / "docs" / "img"


def vol(a, b):
    i = a.intersect(b)
    return i.val().Volume() if i.vals() else 0.0


def rot(w, ax, a):
    return w.rotate((0, 0, 0), ax, a)


if __name__ == "__main__":
    mount = B.j2_mount()
    h2 = B.j2_distal_half()
    h2b = rot(h2, (0, 0, 1), 180)
    print("J2 range (J2 distal vs J2 mount):")
    for a in (-110, -100, -90, 0, 90, 100, 110):
        v = vol(rot(h2, (1, 0, 0), a), mount) + vol(rot(h2b, (1, 0, 0), a), mount)
        print(f"  {a:5d}  {v:8.2f} mm3")

    # J2 cords: tube exit A -> tangent T, then a ring on the drum groove
    (ay, az), (ty, tz, phi) = B.anchor_and_tangent()
    print("J2 cord clearance (should be ~0):")
    for sx, sy in ((-1, 1), (1, -1)):
        x = sx * B.G2_X
        r = S.hole_along((x, sy * ay, az + 0.6), (x, sy * ty, tz), B.CORD_D)
        ring = B.cyl_x(B.R2_CORD + 0.5, x - 0.5, x + 0.5).cut(B.cyl_x(B.R2_CORD - 0.5, x - 0.5, x + 0.5))
        print(f"  x={x:+.1f}: A->T vs mount {vol(r, mount):.3f}, vs J2 distal {vol(r, h2) + vol(r, h2b):.3f};"
              f" groove ring vs J2 distal {vol(ring, h2) + vol(ring, h2b):.3f}")

    # J1 range with the whole J2 module on (J2 at -90/0/90) vs J1 proximal
    prox = S.proximal("through")
    print("J1 range with J2 attached (everything distal vs J1 proximal):")
    for a2 in (-90, 0, 90):
        asm = B.assembly(0, a2)
        distal = [asm[k] for k in ("j1_distal_a", "j1_distal_b", "j2_mount", "j2_distal_a", "j2_distal_b")]
        worst = 0.0
        for a1 in (-95, -90, -60, 0, 60, 90, 95):
            worst = max(worst, sum(vol(rot(d, (1, 0, 0), a1), prox) for d in distal))
        print(f"  J2 at {a2:+d}: worst overlap over J1 -95..95 = {worst:.2f} mm3")

    # J2 module vs J1 distal (static fit)
    asm = B.assembly(0, 0)
    fit = sum(vol(asm[k], asm[j]) for k in ("j2_mount",) for j in ("j1_distal_a", "j1_distal_b"))
    print(f"J2 mount vs J1 distal (should be ~0, they only touch): {fit:.2f} mm3")

    # ---- renders
    asm = B.assembly(35, 45)
    col = {"base": "#9aa3ad", "proximal": "#e8a33d", "spool1": "#4c78a8", "spool2": "#4c78a8",
           "j1_distal_a": "#59a14f", "j1_distal_b": "#77bb85", "j2_mount": "#b07aa1",
           "j2_distal_a": "#e15759", "j2_distal_b": "#f28e8c", "bowden_drive": "#9aa3ad"}
    alpha = {"proximal": 0.4, "j1_distal_a": 0.45, "j1_distal_b": 0.45, "base": 0.8}
    parts = [(to_mesh(w), col[k], alpha.get(k, 1.0)) for k, w in asm.items()]
    render(parts, IMG / "j1b_assembly.png", elev=16, azim=-50, size=8)
    j2 = [(to_mesh(B.j2_mount()), "#b07aa1", 1),
          (to_mesh(rot(B.j2_distal_half(), (1, 0, 0), 30)), "#e15759", 0.55),
          (to_mesh(rot(rot(B.j2_distal_half(), (0, 0, 1), 180), (1, 0, 0), 30)), "#f28e8c", 0.55)]
    flex = ((-B.G2_X, ay, az), (-B.G2_X, ty, tz), "tab:red")
    ext = ((B.G2_X, -ay, az), (B.G2_X, -ty, tz), "tab:blue")
    render(j2, IMG / "j1b_j2_module.png", elev=12, azim=-60, size=6, lines=(flex, ext))
    render([(to_mesh(B.bowden_drive()), "#9aa3ad", 1), (to_mesh(S.spool()), "#4c78a8", 1)],
           IMG / "j1b_bowden_drive.png", elev=25, azim=-130, size=6)
    print("images ->", IMG)
