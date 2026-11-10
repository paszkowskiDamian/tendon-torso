"""Export every printable part, oriented for printing, + a print list.

    .venv/bin/python -m arm.export
"""
import sys
from pathlib import Path

import cadquery as cq

from . import crank as C
from . import fittings as FT
from . import hinge as H
from . import link as L
from . import roll as R
from . import torso as T
from . import balance as BAL
from .assembly import BALANCE_YOKE_ON, CHAIN, balance_yoke_link, link_length
from .specs import E, F, S1, S2, S3, W1, W2

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "arm" / "stl"
BED = 250.0


def lay(wp):
    """Move a part so it sits on z=0, centred on the bed."""
    bb = wp.val().BoundingBox()
    return wp.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def rotY(wp, a):
    return wp.rotate((0, 0, 0), (0, 1, 0), a)


def rotX(wp, a):
    return wp.rotate((0, 0, 0), (1, 0, 0), a)


def parts():
    """name -> (workplane in print orientation, quantity, note)."""
    out = {}
    for s in (S1, S2, E, W1, W2):
        k = s.name.lower()
        a, b = H.rotor_pair(s)
        out[f"{k}_base"] = (H.base(s), 1, f"{s.name} hinge base - plate down")
        out[f"{k}_rotor_a"] = (rotX(a, 180), 1, f"{s.name} rotor half A - top plate down")
        out[f"{k}_rotor_b"] = (rotX(b, 180), 1, f"{s.name} rotor half B - top plate down")
    for s in (S3, F):
        k = s.name.lower()
        out[f"{k}_stator"] = (R.stator(s), 1, f"{s.name} roll stator - plate down")
        out[f"{k}_rotor"] = (rotX(R.rotor(s), 180), 1, f"{s.name} roll rotor - top plate down")
        out[f"{k}_retainer"] = (R.retainer(s), 1, f"{s.name} bearing retainer")
    out["idler_tyre"] = (R.idler_tyre(), 4, "idler tyre for a 623 bearing (2 per roll joint)")
    for name, spec, _ in CHAIN:
        if name == BALANCE_YOKE_ON:
            out[name] = (balance_yoke_link(), 1,
                         f"link {spec[0]}->{spec[1]}, {spec[2]:.0f} mm + balance yoke arms - bottom lip down, arms flat on the bed")
        elif isinstance(spec, tuple):
            out[name] = (L.link(spec[0], spec[1], link_length(name, spec)), 1,
                         f"link {spec[0]}->{spec[1]}, {link_length(name, spec):.0f} mm - bottom lip down, 48 deg cone under the top lip")
    out["crank_frame"] = (C.frame(), 7, "crank frame - foot down (teardrop bearing holes, 48 deg roof under the beam)")
    out["crank_spool"] = (rotY(C.spool(), -90), 7, "crank spool - standing on its small -X boss, 0.12 mm layers, brim")
    out["crank_socket_block_a"] = (C.socket_block_a(), 7, "socket / tension block for cord A (1 M4x45 + nut) - nut trap down")
    out["crank_socket_block_b"] = (C.socket_block_b(), 7, "socket / tension block for cord B (1 M4x45 + nut) - nut trap down")
    out["crank_tube_clip"] = (C.tube_clip(), 7, "tube clip on the crank top, under both tension screw heads - plate down")
    out["crank_arm"] = (rotY(C.crank_arm(), 90), 7, "crank arm - flat")
    out["crank_nut_cup"] = (rotY(C.nut_cup(), -90), 7, "M8 nut cup - nut pocket down")
    out["crank_lock_pin"] = (C.lock_pin(), 7, "lock pin - head up")
    out["crank_knob"] = (C.knob(), 7, "crank knob")
    out["torso_shoulder_beam"] = (T.shoulder_beam(S1), 1, "shoulder beam - floor down, 6 walls")
    out["torso_crank_rack"] = (T.crank_rack(3), 3, "crank rack (3 cranks each)")
    out["torso_tube_comb"] = (T.tube_comb(), 2, "labelled tube comb, snaps into the beam windows at y=33 and y=60 - plate down, labels up")
    out["fitting_cord_thimble"] = (FT.cord_thimble(), 4, "cord thimble (cord to spring hook / eye) - flat")
    out.update(BAL.printed_parts())
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    too_big = []
    for name, (wp, qty, note) in parts().items():
        wp = lay(wp)
        bb = wp.val().BoundingBox()
        dims = (bb.xlen, bb.ylen, bb.zlen)
        if max(dims[:2]) > BED or dims[2] > BED:
            too_big.append((name, dims))
        cq.exporters.export(wp, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        rows.append((name, qty, dims, note))
        print(f"{name:22s} x{qty}  {dims[0]:6.1f} x {dims[1]:6.1f} x {dims[2]:6.1f}  {note}")
    with open(OUT / "PRINT_LIST.md", "w") as f:
        f.write("| File | Qty | Size (mm) | Notes |\n|---|---|---|---|\n")
        for name, qty, d, note in rows:
            f.write(f"| `{name}.stl` | {qty} | {d[0]:.0f} x {d[1]:.0f} x {d[2]:.0f} | {note} |\n")
    print("TOO BIG:", too_big or "none")


if __name__ == "__main__":
    main()
