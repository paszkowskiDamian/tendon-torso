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
    which = [w for w in sys.argv[1:] if w != "arm"] or ([] if "arm" in sys.argv[1:] else ["j1", "j1b"])
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


# ---------------------------------------------------------------- 7-DOF arm
def export_arm():
    sys.path.insert(0, str(ROOT))
    from arm import assembly as A
    from arm import crank as C
    from arm import hinge as H
    from arm import roll as R
    from arm import torso as T
    chain = []
    for name, spec, mount in A.CHAIN:
        st, mv = A.element_parts(name, spec)
        for k, w in list(st.items()) + list(mv.items()):
            dump(w, "arm_" + k, 0.15, 0.3)
        z_in, z_out = A.element_io(spec, name)
        e = {"name": name, "mount": mount, "in_z": z_in, "out_z": z_out,
             "static": ["arm_" + k for k in st], "moving": ["arm_" + k for k in mv]}
        if isinstance(spec, H.HingeSpec):
            (ay, az), (ty, tz, phi) = spec.tangent()
            wa, wb = spec.wraps()
            e.update(kind="hinge", lo=spec.lo, hi=spec.hi, label=A.JOINT_LABEL[name],
                     cord=dict(R=spec.R, gx=spec.gx, ay=ay, az=az, ty=ty, tz=tz, phi=phi, wa=wa, wb=wb,
                               in_z=spec.in_z, plate=spec.plate_top))
        elif isinstance(spec, R.RollSpec):
            sx, sy = spec.socket_xy()
            wa, wb = spec.wraps()
            e.update(kind="roll", lo=spec.lo, hi=spec.hi, label=A.JOINT_LABEL[name],
                     cord=dict(R=spec.R, sx=sx, sy=sy, yi=spec.idler_y(), ri=R.IDLER_CORD_R,
                               za=spec.z_a, zb=spec.z_b, wa=wa, wb=wb, in_z=spec.in_z),
                     # thin-section bearing pair (d, D, B, z of the lower face) for the cutaway view
                     bearings=[[*spec.bearing, 1.0], [*spec.bearing, spec.z_top_pocket]])
        else:
            e.update(kind="link")
        chain.append(e)
    for k, w in A.crank_parts().items():
        dump(w, "arm_" + k, 0.4 if k == "crank_spool" else 0.15, 0.8 if k == "crank_spool" else 0.3)
    dump(T.shoulder_beam(A.S1), "arm_torso_beam", 0.2, 0.3)
    from arm import balance as BL
    bal = A.balance_parts(None, cords=False)
    meta_bal = []
    for k, w in bal.items():
        dump(w, "arm_bal_" + k, 0.2, 0.3)
        meta_bal.append("arm_bal_" + k)
    dump(T.crank_rack(3), "arm_crank_rack", 0.2, 0.3)
    meta = {
        "world": A.WORLD.tolist(),
        "chain": chain,
        "cranks": {j: {"m": m.tolist(), "ratio": A.crank_ratio(j)} for j, m in A.crank_frames().items()},
        "racks": [[x, y] for x, y in A.RACK_CENTRES],
        "balance": meta_bal,
        "balance_geo": {"x_s": BL.X_S, "b_z": BL.B_Z, "c_y": BL.C_Y, "c_z": BL.C_Z, "r": BL.R_SHEAVE,
                        "z_line": BL.Z_LINE, "y_fair": BL.Y_FAIRLEAD, "yoke": A.BALANCE_YOKE_ON},
        "crank": {"r": C.R_SPOOL, "sock_x": C.SOCK_X, "sock_b": C.SOCK_B, "block_z": C.BLOCK_Z[0], "beam_z": C.BEAM_Z[1]},
        "beam": {"y0": T.BEAM_Y[0], "y1": T.BEAM_Y[1], "h": T.BEAM_H, "s1_y": T.S1_Y},
    }
    from arm import kinematics as KN
    meta["shoulder"] = KN.shoulder_frame().tolist()
    meta["tool"] = KN.TOOL_OFFSET
    from arm import statics as ST
    for e, m in zip(meta["chain"], ST.mass_model()["elements"]):
        e.update(ms=m["m_static_g"] / 1000, cs=m["com_static"], mm=m["m_moving_g"] / 1000, cm=m["com_moving"])
        if e["kind"] != "link":
            e["R"] = A.joint_spec(e["name"]).R
    meta["statics"] = {"payload": ST.PAYLOAD, "pretension": ST.PRETENSION, "spool_r": ST.SPOOL_R, "knob_r": ST.KNOB_R,
                       "mass_kg": sum(x["m_static_g"] + x["m_moving_g"] for x in ST.mass_model()["elements"]) / 1000}
    with open(OUT / "arm_chain.json", "w") as f:
        json.dump(meta, f, separators=(",", ":"))
    print("arm_chain.json written")


if __name__ == "__main__" and "arm" in sys.argv[1:]:
    export_arm()
