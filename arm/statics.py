"""Statics of the 7-DOF arm: masses, gravity torques and the loads on every element.

    .venv/bin/python -m arm.statics            # load report (worst cases over the workspace)
    .venv/bin/python -m arm.statics --rebuild  # re-measure the CAD (after geometry changes)

Mass model
----------
Printed parts: CAD volume x PETG density (1.27 g/cm3) x fill_factor(part). The
skin (perimeters + top/bottom layers, SHELL_T thick over the whole surface,
holes included) prints solid, the rest at the part's infill (PROFILES: links
3 walls + 15 % gyroid, modules 4 walls + 25 %), so thin walls come out solid
and only chunky parts gain from low infill. Each part's centre of mass comes
from the CAD solid. Hardware (bearings, bolts, inserts, PTFE tubes
running through) is added per element. PAYLOAD sits at the tool point.

The model is measured once from the CAD and cached in `arm/mass_model.json`.
Everything else (torques, loads) is fast pure numpy on top of `arm/kinematics.py`.

Loads
-----
For a pose q, gravity acts on every body b (mass m_b, centre c_b). For joint i
(axis a_i through p_i), with all bodies distal to it:

    tau_i  = a_i . sum_b (c_b - p_i) x (m_b g)          joint torque (carried by the cords)
    F_i    = sum_b m_b g                                 force through the joint
    M_i    = sum_b (c_b - p_i) x (m_b g)                 full moment; the part
                                                         perpendicular to a_i goes into the bearings

    cord tension    T = |tau| / R_drum + pretension
    crank knob      F_knob = T * r_spool / r_knob
    bearing radial  ~ |F_perp| / 2 + |M_perp| / s + T_total / 2     (s = bearing spacing)
    link bending    sigma = M_perp * r_o / I_tube   at the link's narrowest section
    flange bolts    F_bolt ~ 2 M_perp / (n * r_bc)   (tension in the most loaded bolt / insert)
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

from . import kinematics as K

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "arm" / "mass_model.json"

G = 9.81
RHO = 1.27e-3          # g/mm3 PETG
PROFILES = {             # print profile: (solid skin mm, infill)   -> PRINT_LIST.md
    "link": (1.3, 0.15),       # 3 walls x 0.42 + 4 top/bottom layers, 15 % gyroid
    "module": (1.8, 0.25),     # 4 walls x 0.45, 25 % gyroid (inserts, bearing seats)
}
PAYLOAD = 0.3          # kg at the tool point (gripper + object)
PRETENSION = 20.0      # N per cord
TUBE_G_PER_MM = 0.021  # PTFE 2x4 mm
KNOB_R = 62.0          # mm (crank arm)
SPOOL_R = 8.0

# hardware masses (g)
HW = {"608": 12.0, "625": 4.5, "6806": 22.0, "6805": 16.0, "623": 2.0, "insert": 0.6, "m3": 1.0}
AXLE_G = {"M8": 21.0, "M5": 5.0}   # stub axle set: socket bolt + nut (+ washer)
BEARING_G = {(8.0, 22.0, 7.0): HW["608"], (5.0, 16.0, 5.0): HW["625"]}


def print_profile(part):
    return "link" if part.startswith("link") else "module"


def fill_factor(shape, part=""):
    """Effective solid fraction of a printed part (see the module docstring)."""
    t, infill = PROFILES[print_profile(part)]
    v, a = shape.Volume(), shape.Area()
    shell = min(v, 0.5 * a * t)        # 0.5: the two faces of a thin wall share it
    return (shell + infill * (v - shell)) / v
ALLOW = {"link_bending_MPa": 8.0, "insert_pull_N": 250.0, "bearing_608_N": 1300.0}   # conservative, for printed PETG


# ------------------------------------------------------------------ mass model (from CAD)
def build_mass_model():
    import cadquery as cq
    from . import assembly as A
    from . import params as P
    from .hinge import HingeSpec
    from .roll import RollSpec
    els = []
    names = [n for n, _, _ in A.CHAIN]
    joint_idx = {j: names.index(j) for j in K.JOINTS}
    for idx, (name, spec, _) in enumerate(A.CHAIN):
        st, mv = A.element_parts(name, spec)

        def body(parts):
            m, mc = 0.0, np.zeros(3)
            for k, w in parts.items():
                s = w.val()
                v = s.Volume() * RHO * fill_factor(s, k)
                c = cq.Shape.centerOfMass(s)
                m += v
                mc += v * np.array([c.x, c.y, c.z])
            return m, (mc / m if m else mc)
        ms, cs = body(st)
        mm, cm = body(mv)
        z_in, z_out = A.element_io(spec, name)
        hw_s = hw_m = 0.0
        if isinstance(spec, HingeSpec):
            hw_m += 2 * BEARING_G[tuple(spec.bearing)] + P.FLANGES[spec.rcls]["n"] * HW["insert"]
            hw_s += 2 * AXLE_G[spec.axle] + P.FLANGES[spec.cls]["n"] * HW["insert"]
        elif isinstance(spec, RollSpec):
            b = "6806" if spec.bearing[0] >= 30 else "6805"
            hw_m += HW[b] + P.FLANGES[spec.rotor_cls]["n"] * HW["insert"]
            hw_s += HW[b] + 2 * (HW["623"] + HW["m3"]) + P.FLANGES[spec.cls]["n"] * HW["insert"]
        else:
            ca, cb, _ = spec
            hw_s += (P.FLANGES[ca]["n"] + P.FLANGES[cb]["n"]) * HW["m3"]
        # PTFE tubes running through this element: those of every joint at or after it
        n_tubes = 2 * sum(1 for j in K.JOINTS if joint_idx[j] >= idx)
        hw_s += n_tubes * TUBE_G_PER_MM * (z_out - z_in)
        # hardware sits roughly on the axis, mid element
        cs = (cs * ms + np.array([0, 0, (z_in + 0) / 2]) * hw_s) / (ms + hw_s) if ms + hw_s else cs
        cm = (cm * mm + np.array([0, 0, z_out / 2]) * hw_m) / (mm + hw_m) if mm + hw_m else cm
        els.append({"name": name, "m_static_g": ms + hw_s, "com_static": cs.tolist(),
                    "m_moving_g": mm + hw_m, "com_moving": cm.tolist(),
                    "printed_g": ms + mm, "hardware_g": hw_s + hw_m})
    model = {"profiles": PROFILES, "rho": RHO, "elements": els}
    CACHE.write_text(json.dumps(model, indent=1))
    return model


_MODEL = None


def mass_model():
    global _MODEL
    if _MODEL is None:
        _MODEL = json.loads(CACHE.read_text()) if CACHE.exists() else build_mass_model()
    return _MODEL


# ------------------------------------------------------------------ frames of every element
def element_frames(q_deg, tool=K.TOOL_OFFSET):
    """For each chain element: (static frame, moving frame) as world 4x4, plus the tool frame."""
    W, els = K.chain()
    M = W.copy()
    out, k = [], 0
    for e in els:
        M = M @ K.Rz(math.radians(e.mount)) @ K.Tz(-e.in_z)
        base = M.copy()
        if e.kind != "link":
            a = math.radians(q_deg[k]); k += 1
            M = M @ (K.Rx(a) if e.kind == "hinge" else K.Rz(a))
        out.append((base, M.copy()))
        M = M @ K.Tz(e.out_z)
    return out, M @ K.Tz(tool)


def bodies(q_deg, payload=PAYLOAD):
    """List of (element index, mass kg, world centre) for the pose; the payload has index len(chain)."""
    fr, T = element_frames(q_deg)
    res = []
    for i, (e, (Fs, Fm)) in enumerate(zip(mass_model()["elements"], fr)):
        res.append((i, e["m_static_g"] / 1000, (Fs @ np.r_[e["com_static"], 1])[:3]))
        res.append((i + 0.5, e["m_moving_g"] / 1000, (Fm @ np.r_[e["com_moving"], 1])[:3]))
    if payload:
        res.append((len(fr), payload, T[:3, 3]))
    return res


GRAV = np.array([0, 0, -G])


def _joint_elements():
    _, els = K.chain()
    return [i for i, e in enumerate(els) if e.kind != "link"]


def gravity_torques(q_deg, payload=PAYLOAD):
    """Gravity torque (N*m) the cords must hold at each of the 7 joints."""
    frames, axes, _ = K.fk_all(q_deg)
    bs = bodies(q_deg, payload)
    tau = np.zeros(7)
    for j, ei in enumerate(_joint_elements()):
        p = frames[j][:3, 3]
        M = np.zeros(3)
        for idx, m, c in bs:
            if idx >= ei + 0.5:                         # moving part of this element and everything after
                M += np.cross((c - p) / 1000, m * GRAV)
        tau[j] = axes[j] @ M
    return tau


def joint_wrenches(q_deg, payload=PAYLOAD):
    """Per chain element: force (N) and moment (N*m) carried through its proximal face, + joint data."""
    fr, _ = element_frames(q_deg)
    bs = bodies(q_deg, payload)
    W, els = K.chain()
    res = []
    for i, (e, (Fs, Fm)) in enumerate(zip(els, fr)):
        # proximal face of the element = static frame shifted by in_z
        face = (Fs @ np.r_[0, 0, e.in_z, 1])[:3]
        pivot = Fs[:3, 3]
        Fsum, Mface, Mpiv = np.zeros(3), np.zeros(3), np.zeros(3)
        for idx, m, c in bs:
            if idx >= i:
                f = m * GRAV
                Fsum += f
                Mface += np.cross((c - face) / 1000, f)
            if idx >= i + 0.5:
                Mpiv += np.cross((c - pivot) / 1000, m * GRAV)
        res.append({"name": e.name, "kind": e.kind, "F": Fsum, "M_face": Mface, "M_pivot": Mpiv,
                    "axis": Fs[:3, 0] if e.kind == "hinge" else Fs[:3, 2], "link_axis": Fs[:3, 2]})
    return res


# ------------------------------------------------------------------ element loads
def element_loads(q_deg, payload=PAYLOAD):
    """Loads on every element for pose q (dict per element name)."""
    from . import assembly as A
    from . import params as P
    from .hinge import HingeSpec
    from .roll import RollSpec
    out = {}
    specs = {n: s for n, s, _ in A.CHAIN}
    tau = gravity_torques(q_deg, payload)
    jt = dict(zip(K.JOINTS, tau))
    for w in joint_wrenches(q_deg, payload):
        name, spec = w["name"], specs[w["name"]]
        d = {"F_N": float(np.linalg.norm(w["F"]))}
        # bending at the proximal face (perpendicular to the link axis)
        z = w["link_axis"]
        Mf = w["M_face"] - (w["M_face"] @ z) * z
        d["bend_Nm"] = float(np.linalg.norm(Mf))
        if isinstance(spec, HingeSpec):
            a = w["axis"]
            t = float(w["M_pivot"] @ a)
            Mp = w["M_pivot"] - t * a
            Fp = w["F"] - (w["F"] @ a) * a
            s = 2 * (spec.hub_half - spec.bearing[2] / 2) / 1000
            T = abs(t) / (spec.R / 1000) + PRETENSION
            d.update(tau_Nm=t, cord_N=T, knob_N=T * SPOOL_R / KNOB_R,
                     bearing_N=float(np.linalg.norm(Fp) / 2 + np.linalg.norm(Mp) / s + (T + PRETENSION) / 2))
            fl = P.FLANGES[spec.cls]
        elif isinstance(spec, RollSpec):
            a = w["axis"]
            t = float(w["M_pivot"] @ a)
            Mp = w["M_pivot"] - t * a
            Fp = w["F"] - (w["F"] @ a) * a
            s = (spec.z_top_pocket - 1) / 1000
            T = abs(t) / (spec.R / 1000) + PRETENSION
            d.update(tau_Nm=t, cord_N=T, knob_N=T * SPOOL_R / KNOB_R,
                     bearing_N=float(np.linalg.norm(Fp) / 2 + np.linalg.norm(Mp) / s + (T + PRETENSION) / 2),
                     axial_N=float(abs(w["F"] @ a)))
            fl = P.FLANGES[spec.cls]
        else:
            ca, cb, L = spec
            fa, fb = P.FLANGES[ca], P.FLANGES[cb]
            ro = min(fa["tube_r"], fb["tube_r"])
            from .link import WALL as LINK_WALL
            ri = ro - LINK_WALL
            I = math.pi / 4 * (ro ** 4 - ri ** 4)
            d["stress_MPa"] = d["bend_Nm"] * 1000 * ro / I
            fl = fa
        d["bolt_N"] = 2 * d["bend_Nm"] * 1000 / (fl["n"] * fl["bc"])
        out[name] = d
    return out


def worst_case(n=1500, seed=3, payload=PAYLOAD):
    """Sample the joint space; return the max of each load per element and the pose it came from."""
    lo, hi = K.limits()
    rng = np.random.default_rng(seed)
    poses = [np.zeros(7), np.array([90, 0, 0, 0, 0, 0, 0.]), np.array([0, 90, 0, 0, 0, 0, 0.]),
             np.array([90, 0, 0, 90, 0, 0, 0.]), np.array([120, 0, 0, 0, 0, 0, 0.])]
    poses += [rng.uniform(lo, hi) for _ in range(n)]
    worst = {}
    for q in poses:
        for name, d in element_loads(q, payload).items():
            w = worst.setdefault(name, {})
            for k, v in d.items():
                v = abs(v)
                if v > w.get(k, (0, None))[0]:
                    w[k] = (v, q.round(0).tolist())
    return worst


def total_mass():
    els = mass_model()["elements"]
    printed = sum(e["printed_g"] for e in els)
    hw = sum(e["hardware_g"] for e in els)
    return printed, hw


def report():
    printed, hw = total_mass()
    print(f"Arm mass: {printed:.0f} g printed (links {PROFILES['link'][1]:.0%}, modules {PROFILES['module'][1]:.0%} infill) + {hw:.0f} g hardware = {(printed + hw) / 1000:.2f} kg"
          f"  (+ {PAYLOAD:.1f} kg payload at the tool)\n")
    print(f"{'element':14s} {'mass g':>7s}")
    for e in mass_model()["elements"]:
        print(f"{e['name']:14s} {e['m_static_g'] + e['m_moving_g']:7.0f}")
    W = worst_case()
    print("\nWorst case over the workspace (1500 random poses + horizontal poses):")
    print(f"{'joint':5s} {'torque N*m':>10s} {'cord N':>8s} {'knob N':>7s} {'bearing N':>10s}   pose (S1..W2)")
    for j in K.JOINTS:
        w = W[j]
        print(f"{j:5s} {w['tau_Nm'][0]:10.2f} {w['cord_N'][0]:8.0f} {w['knob_N'][0]:7.1f} {w['bearing_N'][0]:10.0f}   {w['tau_Nm'][1]}")
    print(f"\n{'element':14s} {'bend N*m':>9s} {'stress MPa':>11s} {'bolt N':>7s}")
    for name, w in W.items():
        st = f"{w['stress_MPa'][0]:11.1f}" if "stress_MPa" in w else " " * 11
        flag = ""
        if "stress_MPa" in w and w["stress_MPa"][0] > ALLOW["link_bending_MPa"]:
            flag += "  HIGH STRESS"
        if w["bolt_N"][0] > ALLOW["insert_pull_N"]:
            flag += "  HIGH BOLT LOAD"
        print(f"{name:14s} {w['bend_Nm'][0]:9.2f} {st} {w['bolt_N'][0]:7.0f}{flag}")
    return W


if __name__ == "__main__":
    if "--rebuild" in sys.argv or not CACHE.exists():
        build_mass_model()
    report()
