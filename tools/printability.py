"""Printability audit for print-oriented STLs (the acceptance gate for the arm).

    .venv/bin/python tools/printability.py arm/stl            # all files in a folder
    .venv/bin/python tools/printability.py arm/stl/e_base.stl # one file
    .venv/bin/python tools/printability.py arm/stl --json out.json

A part PASSES when, in the orientation it is exported in, it prints on a
256 mm bed WITHOUT SUPPORTS:

  1. watertight single body, fits 250 x 250 x 250 mm
  2. no unsupported overhang steeper than 45 deg (from vertical), except:
       - near-horizontal downward faces ("bridges/ledges") whose
         characteristic span s = 2*area/perimeter <= BRIDGE_S (8 mm, i.e.
         a strip up to ~8 mm wide or a disc up to ~16 mm diameter), or
       - small features: region area <= 6 mm^2 (chamfer slivers, tiny holes)
       - sloped (>45 deg) downward regions with span s <= SLOPE_S (3 mm),
         e.g. the crown of horizontal holes up to ~5 mm
  3. no wall thinner than MIN_WALL (1.2 mm = 3 lines of 0.4) over a patch:
       fewer than THIN_FRAC of surface samples may be thinner (edges/corners
       always produce a few false hits)
  (islands are covered by the single-body rule)

Everything is a heuristic on the mesh. It reports WHERE each problem is
(bounding box in the print frame) so it can be fixed in the CAD.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

BED = 250.0
OVERHANG_DEG = 45.0
BRIDGE_S = 8.0
SLOPE_S = 3.0
SMALL_AREA = 6.0
MIN_WALL = 1.2
THIN_FRAC = 0.01
N_SAMPLES = 6000


def regions(mesh, mask):
    """Connected face regions (lists of face indices) among faces where mask is True."""
    idx = np.nonzero(mask)[0]
    if len(idx) == 0:
        return []
    adj = mesh.face_adjacency
    keep = mask[adj[:, 0]] & mask[adj[:, 1]]
    comps = trimesh.graph.connected_components(adj[keep], nodes=idx, min_len=1)
    return [np.asarray(c) for c in comps]


def boundary_length(mesh, faces):
    sub = mesh.submesh([faces], append=True, repair=False)
    if hasattr(sub, "outline"):
        try:
            return float(sum(e.length(sub.vertices) for e in sub.outline().entities))
        except Exception:
            pass
    # fallback: edges used once
    e = np.sort(sub.edges, axis=1)
    u, c = np.unique(e, axis=0, return_counts=True)
    b = u[c == 1]
    return float(np.linalg.norm(sub.vertices[b[:, 0]] - sub.vertices[b[:, 1]], axis=1).sum())


def has_support_below(mesh, faces, zmin):
    """True if a ray straight down from the region hits the part or reaches the bed level."""
    c = mesh.triangles_center[faces]
    pick = c[np.linspace(0, len(c) - 1, min(len(c), 12)).astype(int)]
    origins = pick - np.array([0, 0, 0.05])
    dirs = np.tile([0, 0, -1.0], (len(origins), 1))
    hits = mesh.ray.intersects_any(origins, dirs)
    return bool(hits.any())


def wall_thickness(mesh, n=N_SAMPLES):
    pts, fidx = trimesh.sample.sample_surface(mesh, n, seed=1)
    normals = mesh.face_normals[fidx]
    origins = pts - normals * 0.02
    locs, ray_idx, _ = mesh.ray.intersects_location(origins, -normals, multiple_hits=False)
    th = np.full(len(pts), np.inf)
    d = np.linalg.norm(locs - origins[ray_idx], axis=1)
    th[ray_idx] = d
    return pts, th


def analyze(path):
    m = trimesh.load(path, force="mesh")
    rep = {"file": Path(path).name, "problems": [], "notes": []}
    b = m.bounds
    dims = b[1] - b[0]
    rep["size"] = [round(float(v), 1) for v in dims]
    if not m.is_watertight:
        rep["problems"].append("not watertight")
    bodies = m.split(only_watertight=False)
    if len(bodies) != 1:
        rep["problems"].append(f"{len(bodies)} separate bodies")
    if dims.max() > BED:
        rep["problems"].append(f"too big for the bed: {dims.round(0).tolist()}")
    zmin = b[0][2]

    n = m.face_normals
    c = m.triangles_center
    on_bed = c[:, 2] < zmin + 0.25
    lim = -math.cos(math.radians(OVERHANG_DEG + 1.0))    # n_z below this = steeper than 45 deg overhang (1 deg tolerance)
    over = (n[:, 2] < lim) & ~on_bed
    worst_bridge = 0.0
    for faces in regions(m, over):
        area = float(m.area_faces[faces].sum())
        if area <= SMALL_AREA:
            continue
        per = boundary_length(m, faces)
        s = 2 * area / per if per > 0 else 0
        nz = float((n[faces, 2] * m.area_faces[faces]).sum() / area)
        fb = m.triangles[faces].reshape(-1, 3)
        where = f"x {fb[:,0].min():.0f}..{fb[:,0].max():.0f}, y {fb[:,1].min():.0f}..{fb[:,1].max():.0f}, z {fb[:,2].min():.1f}..{fb[:,2].max():.1f}"
        horiz = nz < -0.985
        if horiz:
            worst_bridge = max(worst_bridge, s)
            if s > BRIDGE_S:
                rep["problems"].append(f"flat overhang/bridge too wide (span {s:.1f} mm, area {area:.0f} mm2) at {where}")
        elif s > SLOPE_S:
            rep["problems"].append(f"overhang steeper than 45 deg (span {s:.1f} mm, area {area:.0f} mm2, avg {math.degrees(math.acos(-nz)):.0f} deg from down) at {where}")
    rep["worst_bridge_span"] = round(worst_bridge, 1)

    pts, th = wall_thickness(m)
    thin = th < MIN_WALL
    frac = float(thin.mean())
    rep["thin_wall_fraction"] = round(frac, 4)
    if frac > THIN_FRAC:
        p = pts[thin]
        cells = {}
        for q in np.round(p / 6.0).astype(int):
            cells[tuple(q)] = cells.get(tuple(q), 0) + 1
        top = sorted(cells.items(), key=lambda kv: -kv[1])[:4]
        locs = "; ".join(f"near ({k[0]*6},{k[1]*6},{k[2]*6})" for k, _ in top)
        rep["problems"].append(f"thin walls < {MIN_WALL} mm on {frac*100:.1f}% of the surface, {locs}")
    rep["pass"] = not rep["problems"]
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args()
    files = []
    for p in a.paths:
        p = Path(p)
        files += sorted(p.glob("*.stl")) if p.is_dir() else [p]
    reps = []
    for f in files:
        r = analyze(f)
        reps.append(r)
        flag = "PASS" if r["pass"] else "FAIL"
        print(f"{flag}  {r['file']:26s} size {r['size']}  bridge {r['worst_bridge_span']} mm  thin {r['thin_wall_fraction']*100:.1f}%")
        for pr in r["problems"]:
            print(f"        - {pr}")
    n_ok = sum(r["pass"] for r in reps)
    print(f"\n{n_ok}/{len(reps)} parts printable without supports")
    if a.json:
        Path(a.json).write_text(json.dumps(reps, indent=1))
    sys.exit(0 if n_ok == len(reps) else 1)


if __name__ == "__main__":
    main()
