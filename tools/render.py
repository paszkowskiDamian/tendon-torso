"""Render quick PNG previews + sanity checks for the J1 hinge.

    python tools/render.py
"""
import sys
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1-hinge"))
import j1_hinge as J  # noqa: E402
import cadquery as cq  # noqa: E402

IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)


def to_mesh(wp):
    shape = wp.val() if hasattr(wp, "val") else wp
    verts, tris = shape.tessellate(0.2, 0.3)
    return trimesh.Trimesh([(v.x, v.y, v.z) for v in verts], tris)


def draw(ax, mesh, color, alpha=1.0):
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    shade = 0.45 + 0.55 * np.clip(mesh.face_normals @ light, 0, 1)
    rgb = np.array(matplotlib.colors.to_rgb(color))
    fc = np.clip(shade[:, None] * rgb, 0, 1)
    pc = Poly3DCollection(mesh.vertices[mesh.faces], facecolors=fc, edgecolors="none", alpha=alpha)
    ax.add_collection3d(pc)


def frame(ax, meshes, elev, azim):
    allv = np.vstack([m.vertices for m in meshes])
    c = (allv.max(0) + allv.min(0)) / 2
    r = (allv.max(0) - allv.min(0)).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev, azim); ax.set_axis_off()


def render(parts, path, elev=18, azim=-55, size=7, lines=()):
    fig = plt.figure(figsize=(size, size), dpi=110)
    ax = fig.add_subplot(projection="3d")
    for m, col, a in parts:
        draw(ax, m, col, a)
    for p0, p1, col in lines:
        ax.plot(*zip(p0, p1), color=col, lw=2.2)
    frame(ax, [p[0] for p in parts], elev, azim)
    fig.tight_layout(); fig.savefig(path, facecolor="white"); plt.close(fig)


if __name__ == "__main__":
    prox_wp, tube_bot = J.proximal()
    prox = to_mesh(prox_wp)
    half = J.distal_half()
    spool = to_mesh(J.spool().translate((0, 0, tube_bot + J.SPOOL_Z)))
    base = to_mesh(J.base().translate((0, 0, tube_bot)))

    # --- sanity: interference distal vs proximal over the range of motion
    print("parts watertight:", {n: to_mesh(w).is_watertight for n, w in
                                (("proximal", prox_wp), ("distal_half", half), ("spool", J.spool()), ("base", J.base()))})
    other = half.rotate((0, 0, 0), (0, 0, 1), 180)
    for ang in (-110, -100, -95, -60, 0, 60, 95, 100, 110):
        vol = 0.0
        for hh in (half, other):
            i = hh.rotate((0, 0, 0), (1, 0, 0), ang).intersect(prox_wp)
            vol += i.val().Volume() if i.vals() else 0.0
        print(f"angle {ang:5d} deg  distal-proximal overlap = {vol:7.2f} mm^3")

    # tendon centrelines (flexor red, extensor blue) + a pass-through cord (green)
    zt, zs = 0.0, tube_bot + J.SPOOL_Z
    fl = ((-J.GROOVE_X, J.R_CORD, zt), (-J.GROOVE_X, J.R_CORD, zs), "tab:red")
    ex = ((J.GROOVE_X, -J.R_CORD, zt), (J.GROOVE_X, -J.R_CORD, zs), "tab:blue")
    pt = ((0, 0, 140), (0, 0, tube_bot - 10), "tab:green")

    ang = 40
    dfull = cq.Workplane().add(cq.Compound.makeCompound([half.val(), half.rotate((0, 0, 0), (0, 0, 1), 180).val()]))
    dist = to_mesh(dfull.rotate((0, 0, 0), (1, 0, 0), ang))
    render([(base, "#9aa3ad", 1), (prox, "#e8a33d", 0.35), (spool, "#4c78a8", 1), (dist, "#59a14f", 0.45)],
           IMG / "j1_assembly.png", lines=(fl, ex))

    # exploded / part views
    halfm = to_mesh(half)
    render([(halfm, "#59a14f", 1)], IMG / "j1_distal_half.png", elev=10, azim=-150)
    render([(to_mesh(dfull), "#59a14f", 0.5)], IMG / "j1_distal_pair_xray.png", elev=5, azim=-90,
           lines=(((0, 0, -40), (0, 0, 150), "tab:green"),))
    render([(prox, "#e8a33d", 1)], IMG / "j1_proximal.png", elev=30, azim=-40)
    render([(to_mesh(J.spool()), "#4c78a8", 1)], IMG / "j1_spool.png", elev=20, azim=-60)
    render([(base, "#9aa3ad", 1), (spool, "#4c78a8", 1)], IMG / "j1_base.png", elev=25, azim=-130)
    # side view along the joint axis, joint at 3 angles
    parts = [(prox, "#e8a33d", 0.6)]
    for a, col in ((-90, "#76b7b2"), (0, "#59a14f"), (90, "#b07aa1")):
        parts.append((to_mesh(dfull.rotate((0, 0, 0), (1, 0, 0), a)), col, 0.35))
    render(parts, IMG / "j1_range.png", elev=0, azim=0)
    print("images ->", IMG)
