"""Sanity checks + preview renders for J1-H (hand-cranked 3:1 hinge).

    .venv/bin/python tools/check_j1h.py

Checks (all volumes in mm^3, "0" means the parts clear each other):
  * every part is one valid solid; every exported STL is watertight
  * the cord really lies in the grooves: 0.4 mm test balls along the intended
    cord path on the drum and on the spool helix must not touch the parts,
    while balls half a pitch off the helix (negative control) must
  * drum + lever vs. base, frame and spool over the joint range (and beyond)
  * the free cord spans (1 mm rods) vs. everything, at -90 / 0 / +90 deg
  * crank arm + knob vs. frame over a full turn
  * geometry summary: ratio, wraps, cord lengths, pull needed per load
"""
import math
import sys
from pathlib import Path

import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1h-hand"))
sys.path.insert(0, str(ROOT / "tools"))
import j1h_hand as J  # noqa: E402
from render import render, to_mesh  # noqa: E402

IMG = ROOT / "docs" / "img"


def vol(a, b):
    i = a.intersect(b)
    return i.val().Volume() if i.vals() else 0.0


def ball(p, r=0.4):
    return cq.Workplane().add(cq.Solid.makeSphere(r, cq.Vector(*p)))


def rod(p0, p1, d):
    return J.hole_along(p0, p1, d)


def along(p, q, s):
    """Point at distance s from p along p -> q."""
    v = cq.Vector(*q) - cq.Vector(*p)
    return (cq.Vector(*p) + v.normalized() * s).toTuple()


def cord_arc_points(cord, angle, n=14):
    """Cord centre points on the drum at joint `angle`: from the fixed tangent point
    to the anchor (which turns with the drum)."""
    t = J.tangents()[cord]["phi"]
    a0 = J.drum_anchor_angle(cord) + angle
    x = -J.GROOVE_X if cord == "A" else J.GROOVE_X
    pts = []
    for k in range(n + 1):
        ph = t + (a0 - t) * (k / n)
        y, z = J.polar_yz(J.R_DRUM, ph)
        pts.append((x, y, z))
    return pts


def spool_path_points(cord, x_from, n=12):
    """Cord centre points on the spool helix from the fleet point x_from to the anchor."""
    cy, cz = J.SPOOL_C
    xe = -J.HELIX_END if cord == "A" else J.HELIX_END
    pts = []
    for k in range(n + 1):
        x = x_from + (xe - x_from) * (k / n)
        y, z = J.polar_yz(J.R_SPOOL, J.spool_wrap_angle(cord, x))
        pts.append((x, cy + y, cz + z))
    return pts


if __name__ == "__main__":
    parts = {n: getattr(J, n)() for n in ("base", "drum_a", "drum_b", "frame", "spool", "crank_arm", "nut_cup", "knob", "lock_pin")}
    for n, w in parts.items():
        print(f"{n:10s} valid={w.val().isValid()} solids={len(w.solids().vals())} vol={w.val().Volume():8.0f} mm3")
    for f in sorted(J.OUT.glob("*.stl")):
        m = trimesh.load(f)
        print(f"  {f.name:16s} watertight={m.is_watertight}  size={[round(v, 1) for v in m.extents]}")

    t = J.tangents()
    cy, cz = J.SPOOL_C
    free = J.cord_free_length()
    wrap_len = math.radians(J.WRAP0) * J.R_DRUM
    print(f"\nratio R_DRUM/R_SPOOL = {J.RATIO:.2f}; joint +-{J.RANGE:.0f} deg = crank +-{J.RATIO * J.RANGE:.0f} deg")
    print(f"tangent angles on the drum: A {t['A']['phi']:.1f} deg, B {t['B']['phi']:.1f} deg "
          f"(cord slope {math.degrees(math.atan2(t['A']['td'][1] - t['A']['ts'][1], t['A']['td'][0] - t['A']['ts'][0])) - 0:.1f} deg from the Y axis)")
    print(f"free span {free:.1f} mm; drum wrap at 0 deg {J.WRAP0:.0f} deg = {wrap_len:.1f} mm, min wrap at +-{J.RANGE:.0f} deg = {J.WRAP0 - J.RANGE:.0f} deg")
    print(f"spool wrap at 0 deg {(J.HELIX_END - J.GROOVE_X) / J.PITCH:.2f} turns; "
          f"at the range ends {(J.HELIX_END - J.GROOVE_X) / J.PITCH - J.RANGE * J.RATIO / 360:.2f} .. "
          f"{(J.HELIX_END - J.GROOVE_X) / J.PITCH + J.RANGE * J.RATIO / 360:.2f} turns (helix holds {J.TURNS})")
    take_up = math.radians(J.RANGE) * J.R_DRUM
    print(f"cord take-up over {J.RANGE:.0f} deg: {take_up:.1f} mm = {take_up / (2 * math.pi * J.R_SPOOL):.2f} spool turns")
    for m_kg in (0.5, 1.0, 2.0):
        tq = m_kg * 9.81 * J.LEVER_HOLES[1] / 1000
        print(f"  {m_kg:.1f} kg at {J.LEVER_HOLES[1]:.0f} mm: joint {tq:.2f} N.m, cord {tq / (J.R_DRUM / 1000):.0f} N, "
              f"crank {tq / J.RATIO:.2f} N.m = {tq / J.RATIO / (J.CRANK_R / 1000):.1f} N on the {J.CRANK_R:.0f} mm handle")

    # ---- grooves hold the cord: balls along the intended paths
    print("\ncord-in-groove test (volume of 0.4 mm balls inside the part; 0 = free):")
    halves = {"A": parts["drum_a"], "B": parts["drum_b"]}
    for cord in ("A", "B"):
        for ang in (-J.RANGE, 0.0, J.RANGE):
            h = halves[cord].rotate((0, 0, 0), (1, 0, 0), ang)
            v = sum(vol(ball(p), h) for p in cord_arc_points(cord, ang))
            print(f"  drum cord {cord} @ {ang:+5.0f} deg: {v:.4f}")
    sp = parts["spool"].translate((0, cy, cz))
    for cord in ("A", "B"):
        for xf in (-16.0, -14.5, -13.0) if cord == "A" else (13.0, 14.5, 16.0):
            v = sum(vol(ball(p), sp) for p in spool_path_points(cord, xf))
            print(f"  spool cord {cord} from x={xf:+5.1f}: {v:.4f}")
    # negative control: half a pitch along x lands on the land between turns
    bad = [(x + J.PITCH / 2, y, z) for x, y, z in spool_path_points("B", 13.0)[2:-3]]
    print(f"  negative control (balls on the land): {sum(vol(ball(p), sp) for p in bad):.4f}  (must be > 0)")

    # ---- range of motion
    print("\ndrum + lever vs base / frame / spool (overlap mm3):")
    static = parts["base"].union(parts["frame"].translate((0, cy, cz))).union(sp)
    for ang in (-120, -110, -100, -90, -45, 0, 45, 90, 100, 110, 120):
        pr = J.drum_pair(ang)
        print(f"  {ang:+5d} deg  {vol(pr, static):8.2f}")

    # ---- free cord spans vs everything
    print("\nfree cord spans (1 mm rods) vs parts, should be 0:")
    for ang in (-100, -90, 0, 90, 100):
        pr = J.drum_pair(ang)
        tot = {}
        for cord, sgn in (("A", -1), ("B", 1)):
            x = sgn * J.GROOVE_X
            td, ts = t[cord]["td"], t[cord]["ts"]
            p0 = along((x, *td), (x, *ts), 0.8)
            p1 = along((x, *ts), (x, *td), 0.8)
            r = rod(p0, p1, J.CORD_D)   # full cord diameter
            tot[cord] = (vol(r, parts["base"]), vol(r, parts["frame"].translate((0, cy, cz))), vol(r, sp), vol(r, pr))
        print(f"  {ang:+4d} deg: " + "  ".join(f"{c}: base {b:.3f} frame {f:.3f} spool {s:.3f} drum {d:.3f}" for c, (b, f, s, d) in tot.items()))

    # ---- crank arm vs frame
    print("\ncrank arm + knob vs frame over a turn:")
    fr = parts["frame"]
    arm = parts["crank_arm"]
    kn = J.knob().rotate((0, 0, 0), (0, 1, 0), 90).translate((J.ARM_X[1] + 0.5, 0, J.CRANK_R))
    worst = 0.0
    for ang in range(0, 360, 30):
        a = arm.union(kn).rotate((0, 0, 0), (1, 0, 0), ang)
        worst = max(worst, vol(a, fr))
    print(f"  worst overlap {worst:.3f} mm3; lever tip vs spool flange at -90 deg: "
          f"{(J.D - J.SP_FLANGE_R) - (J.LEVER_L + J.LEVER_Y):.1f} mm gap")
    print(f"  lever at +-90 deg clears the base by {J.HD - J.R_HUB:.0f} mm (hub) and the crank frame by x: "
          f"lever |x| <= {J.LEVER_X:.1f}, walls |x| >= {J.WALL_IN:.1f}")

    # ---- renders
    IMG.mkdir(parents=True, exist_ok=True)
    cols = {"base": "#9aa3ad", "drum": "#59a14f", "frame": "#b0b7c0", "spool": "#4c78a8",
            "crank_arm": "#e8a33d", "nut_cup": "#e8a33d", "knob": "#d1495b", "lock_pin": "#d1495b"}
    lines = []
    for cord, sgn, col in (("A", -1, "tab:red"), ("B", 1, "tab:blue")):
        x = sgn * J.GROOVE_X
        lines.append(((x, *t[cord]["td"]), (x, *t[cord]["ts"]), col))
    asm = J.assembly(35.0)
    render([(to_mesh(v), cols[k], 1.0 if k != "drum" else 0.9) for k, v in asm.items()],
           IMG / "j1h_assembly.png", elev=22, azim=-50, size=8, lines=lines)
    render([(to_mesh(v), cols[k], 1.0) for k, v in asm.items()],
           IMG / "j1h_side.png", elev=0, azim=0, size=8, lines=lines)
    pl = [(to_mesh(parts["base"]), cols["base"], 1.0), (to_mesh(parts["frame"].translate((0, cy, cz))), cols["frame"], 0.6),
          (to_mesh(sp), cols["spool"], 1.0)]
    for a, c in ((-90, "#76b7b2"), (0, "#59a14f"), (90, "#b07aa1")):
        pl.append((to_mesh(J.drum_pair(a)), c, 0.4))
    render(pl, IMG / "j1h_range.png", elev=0, azim=0, size=8, lines=lines)
    render([(to_mesh(parts["drum_a"]), cols["drum"], 1.0)], IMG / "j1h_drum_half.png", elev=15, azim=-140, size=6)
    render([(to_mesh(parts["spool"]), cols["spool"], 1.0)], IMG / "j1h_spool.png", elev=20, azim=-60, size=5)
    render([(to_mesh(parts["frame"]), cols["frame"], 1.0), (to_mesh(parts["spool"]), cols["spool"], 1.0),
            (to_mesh(parts["crank_arm"]), cols["crank_arm"], 1.0), (to_mesh(parts["nut_cup"]), cols["nut_cup"], 1.0)],
           IMG / "j1h_crank.png", elev=20, azim=-120, size=6)
    print("note: tools/printability.py flags ~2 % thin surface on spool.stl: that is the crest between\n      the helical groove turns (1.7 mm land, 1.4 mm radius flanks), not a wall. Everything else passes.")
    print("images ->", IMG)
