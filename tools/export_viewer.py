"""Export the meshes used by docs/viewer as compact JSON ({s, v, f}).

    .venv/bin/python tools/export_viewer.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
for sub in ("j1-hinge", "j1s-hinge", "j1b-2dof"):
    sys.path.insert(0, str(ROOT / "joints" / sub))
import j1_hinge as J1  # noqa: E402
import j1b as B  # noqa: E402

S = B.S
OUT = ROOT / "docs" / "viewer" / "models"


def dump(wp, name, tol=0.12, ang=0.25):
    verts, tris = wp.val().tessellate(tol, ang)
    m = trimesh.Trimesh([(p.x, p.y, p.z) for p in verts], tris)
    m.merge_vertices()
    v = np.round(m.vertices * 20).astype(int)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / f"{name}.json", "w") as f:
        json.dump({"s": 0.05, "v": v.flatten().tolist(), "f": m.faces.flatten().tolist()}, f, separators=(",", ":"))
    print(f"{name:14s} {len(m.faces):6d} tris")


if __name__ == "__main__":
    which = sys.argv[1:] or ["j1", "j1b"]
    if "j1" in which:
        p, tb = J1.proximal()
        h = J1.distal_half()
        dump(J1.base().translate((0, 0, tb)), "base")
        dump(p, "proximal")
        dump(J1.spool(), "spool")
        dump(h, "distal_a")
        dump(h.rotate((0, 0, 0), (0, 0, 1), 180), "distal_b")
    if "j1b" in which:
        tb = S.tube_bot()
        d1 = S.distal_half("mount")
        d2 = B.j2_distal_half()
        dump(S.base().translate((0, 0, tb)), "b_base")
        dump(S.proximal("through"), "b_proximal")
        dump(S.spool(), "b_spool", 0.3, 0.6)
        dump(d1, "b_j1d_a")
        dump(d1.rotate((0, 0, 0), (0, 0, 1), 180), "b_j1d_b")
        dump(B.j2_mount(), "b_j2_mount")
        dump(d2, "b_j2d_a")
        dump(d2.rotate((0, 0, 0), (0, 0, 1), 180), "b_j2d_b")
        dump(B.bowden_drive(), "b_drive")
