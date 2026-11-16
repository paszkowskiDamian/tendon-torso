"""Sanity checks + preview renders for J1-T (hand-held rolling elbow, block-and-tackle drive).

    .venv/bin/python tools/check_j1t.py

  * every part is one valid solid; exported STLs are watertight
  * printed thread: bolt thread vs nut at the matching phase must not overlap
  * cord lengths vs joint angle: F + spool + E constant, X1 / X2 / T1 constant
  * cord-in-groove: 0.4 mm balls along every cord arc sit free in the grooves
    (discs, sheaves, guide, spool) at several joint angles
  * straight cord runs (2.5 mm rods) vs every part, incl. the free ends past the pin
  * forearm assembly vs upper-arm assembly over the range + first contact beyond it;
    centre links vs the block / guide bolts; knob vs everything
"""
import math
import sys
from pathlib import Path

import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1t-tackle"))
sys.path.insert(0, str(ROOT / "tools"))
import j1t_tackle as J  # noqa: E402
from render import render, to_mesh  # noqa: E402

IMG = ROOT / "docs" / "img"


def ball(p, r=0.4):
    return cq.Workplane().add(cq.Solid.makeSphere(r, cq.Vector(*p)))


def along(p, q, s):
    v = cq.Vector(*q) - cq.Vector(*p)
    return (cq.Vector(*p) + v.normalized() * s).toTuple()


def path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))


def straight_runs(pts, min_len=10.0):
    return [(a, b) for a, b in zip(pts[:-1], pts[1:]) if math.dist(a, b) > min_len]


class Group:
    """A bag of solids that behaves like one for the overlap tests (no boolean fusing needed)."""
    def __init__(self, parts):
        self.parts = list(parts)


def vol(a, b):
    if isinstance(a, Group):
        return sum(vol(p, b) for p in a.parts)
    if isinstance(b, Group):
        return sum(vol(a, p) for p in b.parts)
    i = a.intersect(b)
    return i.val().Volume() if i.vals() else 0.0


def fuse(parts):
    return Group(parts)


if __name__ == "__main__":
    singles = {"plate_handle": J.plate_handle(), "plate_forearm": J.plate_forearm(), "disc_thread": J.disc("thread"), "disc_pocket": J.disc("pocket"), "sheave": J.sheave(),
               "anchor_ring": J.anchor_ring(), "guide": J.guide(), "centre_link": J.centre_link(), "spool": J.spool(),
               "knob": J.knob(), "grip": J.grip(), "bolt": J.bolt(), "bolt_block": J.block_bolt(), "bolt_pin": J.pin_bolt(), "bolt_spool": J.spool_bolt(), "nut": J.nut()}
    for n, w in singles.items():
        print(f"{n:14s} valid={w.val().isValid()} solids={len(w.solids().vals())} vol={w.val().Volume():8.0f} mm3")
    for f in sorted(J.OUT.glob("*.stl")):
        m = trimesh.load(f)
        print(f"  {f.name:22s} watertight={m.is_watertight}  size={[round(v, 1) for v in m.extents]}")

    # ---- printed thread fit
    thr = J.external_thread(J.NUT_T + 4).translate((0, 0, -2))
    print(f"\nthread: bolt thread inside the nut at the matching phase, overlap {vol(thr, J.nut()):.3f} mm3 (0 = free);"
          f" at half a pitch {vol(thr.translate((0, 0, J.P_PITCH / 2)), J.nut()):.1f} mm3 (must be > 0)")
    print(f"radial clearance crest/root {J.INT_MAJOR - J.EXT_MAJOR:.2f} / {J.INT_MINOR - J.EXT_MINOR:.2f} mm, pitch {J.P_PITCH} mm")

    # ---- lengths
    print(f"\nratio (elbow torque / knob torque) = {J.RATIO0:.2f} cos(theta/2)-ish; spool r {J.R_SPOOL}, {J.N_PASS} passes, blocks at {J.H_BLK} mm")
    L0 = {k: path_length(v) for k, v in J.routing(0.0, n_arc=90).items()}
    print("cord length change vs 0 deg (F shortens, E lengthens, F + S + E and X1, X2, T1 must stay constant):")
    for th in (-20, 0, 30, 60, 90, 120, 130):
        L = {k: path_length(v) for k, v in J.routing(th, n_arc=90).items()}
        print(f"  {th:+4d} deg: " + "  ".join(f"{k} {L[k] - L0[k]:+7.2f}" for k in ("F", "E", "X1", "X2", "T1"))
              + f"   F+S+E {sum(L[k] - L0[k] for k in ('F', 'S', 'E')):+6.2f}   ratio {J.ratio(th):4.2f}  knob {J.spool_turns(th) / 360:+5.2f} turns")
    print(f"  one drive cord {L0['F'] + L0['S'] + L0['E']:.0f} mm + knots; X1, X2 {L0['X1']:.0f} mm each")

    # ---- assemblies
    arm = J.link_parts(1)
    wheels = {k for k in arm if k.split("_")[-1] in ("s1", "s3", "anchor", "s2") or k == "guide"}
    arm_static = fuse([v for k, v in arm.items() if k not in wheels])
    arm_sheaves = fuse([v for k, v in arm.items() if k in wheels])
    print("\ncord-in-groove test (0.4 mm balls, volume inside the parts; 0 = free):")
    for th in (-20.0, 0.0, 70.0, 130.0):
        fore = J.forearm_at(th)
        fore_all = fuse(list(fore.values()))
        sp = J.spool_parts(th)
        spool_w = sp["spool"]
        tot = {}
        for cord, pts in J.routing(th, n_arc=10).items():
            tot[cord] = sum(vol(ball(p), arm_static) + vol(ball(p), arm_sheaves) + vol(ball(p), fore_all) + vol(ball(p), spool_w) for p in pts)
        print(f"  theta {th:+5.0f}: " + "  ".join(f"{k} {v:.3f}" for k, v in tot.items()))
    bad = [(x, y + 1.6, z) for x, y, z in J.routing(0.0, n_arc=10)["X1"][3:8]]
    print(f"  negative control (balls 1.6 mm off the X1 groove): {sum(vol(ball(p), arm_static) for p in bad):.3f} (must be > 0)")

    print("\nstraight cord runs (2.5 mm rods) vs parts, should be 0:")
    for th in (-20.0, 0.0, 70.0, 130.0):
        fore_all = fuse(list(J.forearm_at(th).values()))
        pins = fuse(list(J.pin_parts(th).values()))
        sp = J.spool_parts(th)
        tot = {}
        for cord, pts in J.routing(th, n_arc=48).items():
            for a, b in straight_runs(pts):
                r = J.hole_along(along(a, b, 1.0), along(b, a, 1.0), J.CORD_D)
                for nm, part in (("arm", arm_static), ("wheels", arm_sheaves), ("fore", fore_all), ("links", pins), ("spool", sp["spool"]), ("knob", sp["knob"])):
                    v = vol(r, part)
                    if v > 1e-3:
                        tot[f"{cord}/{nm}"] = tot.get(f"{cord}/{nm}", 0.0) + v
        print(f"  theta {th:+5.0f}: {sum(tot.values()):.3f} mm3" + (f"  {tot}" if tot else ""))

    print("\nforearm (all parts) vs upper arm (all parts), centre links vs block bolts, knob vs all (overlap mm3):")
    arm_all = fuse(list(arm.values()))
    sp0 = J.spool_parts(0.0)
    knob0 = sp0["knob"]
    for th in (-40, -30, -20, 0, 45, 90, 120, 130, 140, 160):
        fore_all = fuse(list(J.forearm_at(th).values()))
        pins = J.pin_parts(th)
        links = Group([pins["link_a"], pins["link_b"]])
        print(f"  {th:+5d} deg  forearm {vol(fore_all, arm_all) + vol(fore_all, knob0):8.2f}   links {vol(links, arm_all) + vol(links, fore_all):8.2f}")
    print(f"  knob vs upper arm at 0 deg: {vol(knob0, arm_all):.2f}; spool vs upper arm: {vol(sp0['spool'], arm_all):.2f}")

    # ---- renders
    IMG.mkdir(parents=True, exist_ok=True)
    def col(k):
        if k.startswith("fore_"):
            k2 = k[5:]
            if k2.startswith("plate"):
                return "#59a14f"
            if k2.startswith("disc"):
                return "#8cd17d"
            return "#4c78a8" if ("_s" in k2 or "anchor" in k2) else "#e8a33d"
        if k.startswith("arm_"):
            k2 = k[4:]
            if k2.startswith("plate"):
                return "#9aa3ad"
            if k2.startswith("disc"):
                return "#c7ccd1"
            if k2 == "grip":
                return "#6b7280"
            return "#4c78a8" if ("_s" in k2 or "anchor" in k2 or k2 == "guide") else "#e8a33d"
        if k.startswith("link"):
            return "#b07aa1"
        if k in ("spool",):
            return "#4c78a8"
        if k == "knob":
            return "#d1495b"
        return "#e8a33d"
    ccol = {"F": "tab:red", "S": "tab:red", "E": "tab:blue", "X1": "tab:green", "X2": "tab:olive", "T1": "tab:purple"}

    def lines(th, cords=None):
        out = []
        for cord, pts in J.routing(th, n_arc=24).items():
            if cords and cord not in cords:
                continue
            out += [(a, b, ccol[cord]) for a, b in zip(pts[:-1], pts[1:])]
        return out

    th = 40.0
    asm = J.assembly(th)
    render([(to_mesh(v), col(k), 1.0) for k, v in asm.items()], IMG / "j1t_assembly.png", elev=22, azim=-55, size=8, lines=lines(th))
    # side view without the +X plates, links and knob: the tackle and cords
    hide = {"arm_plate_p", "fore_plate_p", "arm_disc_p", "fore_disc_p", "link_b", "knob", "knob_nut", "spool_w+42", "spool_w+36"}
    keep = {k: v for k, v in asm.items() if k not in hide and not k.endswith("_nut") and "pinp" not in k and "pin_w+" not in k}
    render([(to_mesh(v), col(k), 1.0) for k, v in keep.items()], IMG / "j1t_side.png", elev=0, azim=0, size=8, lines=lines(th))
    # range: forearm at 4 angles, cords of the tackle
    base = [(to_mesh(v), col(k), 1.0) for k, v in J.link_parts(1).items() if k not in ("plate_p",)]
    ln = []
    for a, c in ((-20, "#76b7b2"), (0, "#59a14f"), (90, "#b07aa1"), (130, "#d1495b")):
        base += [(to_mesh(v), c, 0.35) for k, v in J.forearm_at(a).items() if k not in ("plate_p",)]
        ln += lines(a, ("F", "E", "T1"))
    render(base, IMG / "j1t_range.png", elev=0, azim=0, size=8, lines=ln)
    # pass-through demo: T1 at three angles, discs only
    discs = [(to_mesh(J.link_parts(1)[k]), "#c7ccd1", 0.18) for k in ("disc_p", "disc_n")]
    ln = []
    for a, c in ((0, "#59a14f"), (70, "#b07aa1"), (130, "#d1495b")):
        discs += [(to_mesh(v), c, 0.12) for k, v in J.forearm_at(a).items() if k.startswith("fore_disc")]
        ln += [(p, q, c) for p, q, _ in lines(a, ("T1",))]
    render(discs, IMG / "j1t_passthrough.png", elev=0, azim=0, size=7, lines=ln)
    render([(to_mesh(J.plate_handle()), "#9aa3ad", 1.0), (to_mesh(J.disc()), "#c7ccd1", 1.0), (to_mesh(J.sheave().translate((0, 0, J.H_BLK))), "#4c78a8", 1.0),
            (to_mesh(J.bolt().translate((60, 0, -40))), "#e8a33d", 1.0), (to_mesh(J.nut().translate((60, 20, -40))), "#e8a33d", 1.0),
            (to_mesh(J.spool().translate((0, *J.SPOOL_C))), "#4c78a8", 1.0), (to_mesh(J.knob().translate((50, J.SPOOL_C[0], J.SPOOL_C[1]))), "#d1495b", 1.0)],
           IMG / "j1t_parts.png", elev=25, azim=-60, size=8)
    print("images ->", IMG)
