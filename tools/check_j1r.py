"""Sanity checks + preview renders for J1-R (rolling elbow, hand-cranked 3:1).

    .venv/bin/python tools/check_j1r.py

  * every part is one valid solid; exported STLs are watertight
  * cord-in-groove: 0.4 mm balls along every cord arc (spool helices, rollers,
    idler, pulley 1 discs, pulley 2) must sit free in the grooves at several
    joint angles; a negative control on the spool land must not
  * straight cord runs (full 2.5 mm rods) vs every part
  * forearm + pulley 2 vs upper arm / idler / rollers / frame over the range,
    and a scan for the first contact beyond it
  * side plates vs everything; crank arm vs frame
  * cord length changes vs joint angle (must be +-R*theta for A/B, 0 for X1/X2)
"""
import math
import sys
from pathlib import Path

import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "joints" / "j1r-rolling"))
sys.path.insert(0, str(ROOT / "tools"))
import j1r_rolling as J  # noqa: E402
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
    v = cq.Vector(*q) - cq.Vector(*p)
    return (cq.Vector(*p) + v.normalized() * s).toTuple()


def path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))


def straight_runs(pts, min_len=12.0):
    """Long straight pieces of a polyline (the free spans between wraps)."""
    return [(a, b) for a, b in zip(pts[:-1], pts[1:]) if math.dist(a, b) > min_len]


def spool_points(cord, x_from, n=12):
    cy, cz = J.SPOOL_C
    xe = J.HELIX_END[cord]
    out = []
    for k in range(n + 1):
        x = x_from + (xe - x_from) * k / n
        y, z = J.polar_yz(J.R_SPOOL, J.spool_wrap_angle(cord, x))
        out.append((x, cy + y, cz + z))
    return out


if __name__ == "__main__":
    base, idler, roller, side, fore0, spool = J.base(), J.idler(), J.roller(), J.side_plate(), J.forearm(), J.spool()
    for n, w in (("base", base), ("forearm", fore0), ("idler", idler), ("roller", roller), ("side_plate", side), ("spool", spool)):
        print(f"{n:10s} valid={w.val().isValid()} solids={len(w.solids().vals())} vol={w.val().Volume():8.0f} mm3")
    for f in sorted(J.OUT.glob("*.stl")):
        m = trimesh.load(f)
        print(f"  {f.name:22s} watertight={m.is_watertight}  size={[round(v, 1) for v in m.extents]}")

    cy, cz = J.SPOOL_C
    rollers = {c: roller.translate((J.ST[c], *J.ROLLER_C[c])) for c in ("A", "B")}
    frame = J.frame().translate((0, cy, cz)) if hasattr(J, "frame") else J.H.frame().translate((0, cy, cz))
    spool_w = spool.translate((0, cy, cz))
    static = base.union(frame).union(spool_w).union(rollers["A"]).union(rollers["B"])

    print(f"\nratio R/R_SPOOL = {J.RATIO:.2f}; centre distance {J.LC} mm; range {J.TH_MIN:.0f}..{J.TH_MAX:.0f} deg "
          f"= crank {J.RATIO * J.TH_MIN:.0f}..{J.RATIO * J.TH_MAX:.0f} deg")
    print("cord length change vs 0 deg (A, B should be -+R*theta; X1, X2 ~0):")
    L0 = {k: path_length(v) for k, v in J.routing(0.0, n_arc=90).items()}
    for th in (-20, 0, 45, 90, 140):
        L = {k: path_length(v) for k, v in J.routing(th, n_arc=90).items()}
        print(f"  {th:+4d} deg: " + "  ".join(f"{k} {L[k] - L0[k]:+7.2f}" for k in ("A", "B", "X1", "X2"))
              + f"   R*theta = {math.radians(th) * J.R:+.2f}")
    for cord in ("A", "B"):
        print(f"  cord {cord}: total length at 0 deg {L0[cord]:.0f} mm + spool/anchor allowance")

    # ---- balls in grooves
    print("\ncord-in-groove test (0.4 mm balls, volume inside the parts; 0 = free):")
    for th in (-20.0, 0.0, 70.0, 140.0):
        fore = J.forearm_at(th)
        moving = fore.union(idler)
        tot = {}
        for cord, pts in J.routing(th, n_arc=10).items():
            tot[cord] = sum(vol(ball(p), static) + vol(ball(p), moving) for p in pts)
        print(f"  theta {th:+5.0f}: " + "  ".join(f"{k} {v:.3f}" for k, v in tot.items()))
    for cord, xs in (("A", (-9.0, -7.0, -5.0)), ("B", (5.0, 7.0, 9.0))):
        for xf in xs:
            print(f"  spool cord {cord} from x={xf:+5.1f}: {sum(vol(ball(p), spool_w) for p in spool_points(cord, xf)):.4f}")
    bad = [(x + J.PITCH / 2, y, z) for x, y, z in spool_points("B", 5.0)[2:-3]]
    print(f"  negative control (balls on the spool land): {sum(vol(ball(p), spool_w) for p in bad):.3f} (must be > 0)")

    # ---- straight runs vs parts
    print("\nstraight cord runs (2.5 mm rods) vs parts, should be 0:")
    for th in (-20.0, 0.0, 70.0, 140.0):
        fore = J.forearm_at(th)
        plates = J.side_plate_at(th, -1).union(J.side_plate_at(th, 1))
        tot = 0.0
        for cord, pts in J.routing(th, n_arc=48).items():      # fine arcs: only the real free spans count as straight
            for a, b in straight_runs(pts):
                r = rod(along(a, b, 1.0), along(b, a, 1.0), J.CORD_D)
                tot += vol(r, static) + vol(r, fore) + vol(r, idler) + vol(r, plates)
        print(f"  theta {th:+5.0f}: {tot:.3f} mm3")

    # ---- forearm / side plates vs the rest
    print("\nforearm+pulley 2 and side plates vs upper arm, idler, rollers, frame (overlap mm3):")
    for th in (-40, -30, -20, 0, 45, 90, 120, 140, 150, 160):
        fore = J.forearm_at(th)
        plates = J.side_plate_at(th, -1).union(J.side_plate_at(th, 1))
        print(f"  {th:+5d} deg  forearm {vol(fore, static) + vol(fore, idler):8.2f}   plates {vol(plates, static) + vol(plates, idler) + vol(plates, fore):8.2f}")
    arm = J.H.crank_arm().union(J.H.knob().rotate((0, 0, 0), (0, 1, 0), 90).translate((J.H.ARM_X[1] + 0.5, 0, J.H.CRANK_R)))
    worst = max(vol(arm.rotate((0, 0, 0), (1, 0, 0), a), J.H.frame()) for a in range(0, 360, 30))
    print(f"  crank arm + knob vs frame over a turn: worst {worst:.3f} mm3")

    # ---- renders
    IMG.mkdir(parents=True, exist_ok=True)
    cols = {"base": "#9aa3ad", "forearm": "#59a14f", "idler": "#4c78a8", "side_plate_a": "#e8a33d", "side_plate_b": "#e8a33d",
            "frame": "#b0b7c0", "spool": "#4c78a8", "crank_arm": "#e8a33d", "nut_cup": "#e8a33d", "knob": "#d1495b",
            "lock_pin": "#d1495b", "roller_A": "#4c78a8", "roller_B": "#4c78a8"}
    ccol = {"A": "tab:red", "B": "tab:blue", "X1": "tab:green", "X2": "tab:olive"}

    def lines(th):
        out = []
        for cord, pts in J.routing(th, n_arc=24).items():
            out += [(a, b, ccol[cord]) for a, b in zip(pts[:-1], pts[1:])]
        return out

    th = 30.0
    asm = J.assembly(th)
    render([(to_mesh(v), cols[k], 1.0) for k, v in asm.items()], IMG / "j1r_assembly.png", elev=22, azim=-50, size=8, lines=lines(th))
    render([(to_mesh(v), cols[k], 0.55 if k in ("side_plate_b",) else 1.0) for k, v in asm.items()],
           IMG / "j1r_side.png", elev=0, azim=0, size=8, lines=lines(th))
    pl = [(to_mesh(base), cols["base"], 1.0), (to_mesh(idler), cols["idler"], 1.0), (to_mesh(frame), cols["frame"], 0.6),
          (to_mesh(spool_w), cols["spool"], 1.0)] + [(to_mesh(r), cols["idler"], 1.0) for r in rollers.values()]
    ln = []
    for a, c in ((-20, "#76b7b2"), (0, "#59a14f"), (90, "#b07aa1"), (140, "#d1495b")):
        pl.append((to_mesh(J.forearm_at(a)), c, 0.4))
        pl.append((to_mesh(J.side_plate_at(a, 1)), c, 0.25))
        ln += lines(a)
    render(pl, IMG / "j1r_range.png", elev=0, azim=0, size=8, lines=ln)
    # close-up of the elbow without the far side plate, at 70 deg
    th = 70.0
    asm = J.assembly(th)
    keep = {k: v for k, v in asm.items() if k not in ("side_plate_b", "frame", "spool", "crank_arm", "nut_cup", "knob", "lock_pin")}
    render([(to_mesh(v), cols[k], 1.0) for k, v in keep.items()], IMG / "j1r_elbow.png", elev=15, azim=-35, size=7, lines=lines(th))
    render([(to_mesh(fore0), cols["forearm"], 1.0)], IMG / "j1r_forearm.png", elev=20, azim=-120, size=5)
    print("note: tools/printability.py flags the 3.4 mm wide ceilings of the restraint grooves where they tunnel\n      through the tine stems (base.stl) and the spool groove crests: short bridges, print fine.")
    print("images ->", IMG)
