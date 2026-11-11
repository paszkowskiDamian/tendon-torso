"""Checks for the 7-DOF arm.

    .venv/bin/python tools/check_arm.py           # all checks
    .venv/bin/python tools/check_arm.py ranges    # only joint ranges
    .venv/bin/python tools/check_arm.py torso bends drives   # the two-arm torso
"""
import itertools
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cadquery as cq  # noqa: E402
import numpy as np  # noqa: E402

from arm import assembly as A  # noqa: E402
from arm import crank as C  # noqa: E402
from arm import params as P  # noqa: E402

EL = [n for n, _, _ in A.CHAIN]


def vol(a, b):
    try:
        i = a.intersect(b)
        return i.val().Volume() if i.vals() else 0.0
    except Exception:
        return -1.0


def _bb(w):
    """Bounding box of a workplane, computed once (it is expensive on the big solids)."""
    bb = getattr(w, "_bb_cache", None)
    if bb is None:
        bb = w.val().BoundingBox()
        w._bb_cache = bb
    return bb


def bb_overlap(a, b, tol=0.5):
    p, q = _bb(a), _bb(b)
    return not (p.xmax < q.xmin - tol or q.xmax < p.xmin - tol or p.ymax < q.ymin - tol or
                q.ymax < p.ymin - tol or p.zmax < q.zmin - tol or q.zmax < p.zmin - tol)


def owner(part):
    """Which chain element a part belongs to."""
    for n, spec, _ in A.CHAIN:
        k = n.lower()
        if part == k or part.startswith(k + "_"):
            return n
    return None


def collisions(q, joint_idx=None, min_vol=1.0):
    """Pairs of parts from DIFFERENT, non-adjacent... (any) elements that overlap > min_vol."""
    solids = A.arm_solids(q)
    names = list(solids)
    hits = []
    for a, b in itertools.combinations(names, 2):
        ea, eb = owner(a), owner(b)
        if ea == eb and not (a.endswith("_rotor") or b.endswith("_rotor") or "rotor_" in a or "rotor_" in b):
            continue
        if not bb_overlap(solids[a], solids[b]):
            continue
        v = vol(solids[a], solids[b])
        if v > min_vol or v < 0:
            hits.append((a, b, round(v, 1)))
    return hits


def check_ranges():
    print("== joint ranges (each joint at its limits, others straight) ==")
    ok = True
    for j in A.JOINTS:
        s = A.joint_spec(j)
        for a in (s.lo, s.hi):
            h = collisions({j: a})
            print(f"  {j:3s} {a:+5.0f}: {'clear' if not h else h}")
            ok &= not h
    return ok


def check_combos():
    print("== combined poses ==")
    poses = [
        {"E": 115, "F": 85, "W1": 60},
        {"E": 115, "F": -85, "W1": -60, "W2": 30},
        {"S2": 100, "S3": 80, "E": 90},
        {"S2": 100, "S3": -80, "E": 115},
        {"S1": 120, "S2": 60, "E": 90, "W1": 45},
        {"S1": -30, "E": 115},
        {"S3": 80, "F": 85, "W1": 60, "W2": -30},
    ]
    ok = True
    for q in poses:
        h = collisions(q)
        print(f"  {q}: {'clear' if not h else h}")
        ok &= not h
    return ok


def check_drives():
    """Drive units (drive.py) in the torso: the parts of one unit fit together for every
    motor (phase 1 and the DM-J4310 upgrade), the 16 units clear each other, the saddles,
    hangers and extrusions, and a DM-J4310 fits on every column level."""
    from arm import drive as DR
    from arm import torso as T
    print("== drive units ==")
    ok = True
    # parts that are meant to touch: the spool sits on the bearings (not modelled), the motor
    # bolts to the adapter face, the cup's pillars stand on wall B
    for m in DR.MOTORS:
        p = DR.unit_parts(m)
        hits = [(a, b, round(v, 1)) for (a, wa), (b, wb) in itertools.combinations(p.items(), 2)
                if bb_overlap(wa, wb) and (v := vol(wa, wb)) > 0.5]
        print(f"  {m:9s} unit: {'parts clear' if not hits else hits}")
        ok &= not hits
    statics = A.torso_solids()
    units = A.drive_solids()
    hits = []
    names = list(units)
    owner_u = lambda n: "_".join(n.split("_")[:3])
    for a, b in itertools.combinations(names, 2):
        if owner_u(a) == owner_u(b):
            continue
        if bb_overlap(units[a], units[b]) and (v := vol(units[a], units[b])) > 0.5:
            hits.append((a, b, round(v, 1)))
    for a, wa in units.items():
        for b, wb in statics.items():
            if bb_overlap(wa, wb) and (v := vol(wa, wb)) > 0.5:
                hits.append((a, b, round(v, 1)))
    print(f"  16 units vs each other, saddles, hangers, extrusions, shoulder beams: {'clear' if not hits else hits}")
    ok &= not hits
    # upgrade path: every column level and the S1 hangers take a DM-J4310 unit
    dm = {k: w for k, w in DR.unit_parts("DM-J4310").items() if k in ("cup", "motor", "adapter")}
    up_hits = []
    frames = A.drive_frames()
    placed = {(s, j): {k: A.place(w, m) for k, w in dm.items()} for (s, j), m in frames.items()}
    for (key_a, pa), (key_b, pb) in itertools.combinations(placed.items(), 2):
        for ka, wa in pa.items():
            for kb, wb in pb.items():
                if bb_overlap(wa, wb) and (v := vol(wa, wb)) > 0.5:
                    up_hits.append((key_a, ka, key_b, kb, round(v, 1)))
    for key, pa in placed.items():
        for ka, wa in pa.items():
            for b, wb in statics.items():
                if bb_overlap(wa, wb) and (v := vol(wa, wb)) > 0.5:
                    up_hits.append((key, ka, b, round(v, 1)))
    print(f"  DM-J4310 on all 16 positions (upgrade path): {'fits' if not up_hits else up_hits}")
    ok &= not up_hits
    top = max(w.val().BoundingBox().zmax for w in list(statics.values()) + list(units.values()))
    print(f"  column: {T.LEVELS} levels x {T.LEVEL_PITCH:.0f} mm from z {T.LEVEL_Z0:.0f}, S1 units under the clavicle;"
          f" torso top {top:.0f} mm above the table")
    return ok


def _torso_hits(qr, ql, statics, min_vol=1.0):
    """Arm-vs-arm and arm-vs-torso interference for the right arm at qr and the left at ql."""
    arms = {"R": A.arm_world("R", qr), "L": A.arm_world("L", ql)}
    hits = []
    for a, wa in arms["R"].items():
        for b, wb in arms["L"].items():
            if bb_overlap(wa, wb) and (v := vol(wa, wb)) > min_vol:
                hits.append((f"R.{a}", f"L.{b}", round(v, 1)))
    for s, arm in arms.items():
        for a, wa in arm.items():
            for b, wb in statics.items():
                if a == "s1_base" and b == f"shoulder_beam_{s}":       # bolted face to face
                    continue
                if bb_overlap(wa, wb) and (v := vol(wa, wb)) > min_vol:
                    hits.append((f"{s}.{a}", b, round(v, 1)))
    return hits


def _ik_pose(p_shoulder):
    """Low-lever IK joint dict for a tool point in an arm's SHOULDER frame (same for both arms)."""
    from arm import kinematics as K
    r, _ = K.ik_low_lever(p_shoulder, seeds=2)
    return dict(zip(K.JOINTS, [float(v) for v in r.q])), r.ok


def check_torso():
    """Two arms on the torso: arm-vs-arm and arm-vs-torso (extrusions, saddles, drive units,
    shoulder beams, combs, hangers, balance hardware of both arms) for both arms at every
    joint limit (mirror-symmetric) and with both hands in the shared middle zone."""
    print("== two arms on the torso ==")
    statics = {**A.torso_solids(), **A.drive_solids()}
    for s in A.SIDES:
        statics.update({f"{s}.{k}": w for k, w in A.balance_world(s).items() if "cord" not in k})
    ok = True
    poses = [("rest", {}, {})]
    for j in A.JOINTS:
        sp = A.joint_spec(j)
        for a in (sp.lo, sp.hi):
            poses.append((f"both {j} {a:+.0f}", {j: a}, {j: a}))
    poses += [("both shoulders up, elbows in", {"S1": 90, "S2": -10, "E": 90}, {"S1": 90, "S2": -10, "E": 90}),
              ("both swung back", {"S1": -30, "S2": 30, "E": 60}, {"S1": -30, "S2": 30, "E": 60})]
    # hands in the shared middle zone (work surface z = -650 in the SHOULDER frame, x < 0 = inwards)
    zone = [((-120.0, 200.0, -600.0), (-120.0, 200.0, -600.0), "both hands 110 mm apart, mid table"),
            ((-140.0, 300.0, -560.0), (-140.0, 300.0, -560.0), "both hands 70 mm apart, far"),
            ((-130.0, 120.0, -500.0), (-130.0, 120.0, -500.0), "both hands 90 mm apart, high"),
            ((-188.0, 150.0, -600.0), (40.0, 150.0, -600.0), "right hand at the inner edge, left at home"),
            ((40.0, 300.0, -620.0), (-188.0, 300.0, -620.0), "left hand at the inner edge, right at home")]
    known = []
    for pr, pl, lab in zone:
        qr, okr = _ik_pose(pr)
        ql, okl = _ik_pose(pl)
        if not (okr and okl):
            print(f"  {lab}: IK failed ({okr}, {okl})")
            ok = False
            continue
        poses.append((lab + f" (q_R {', '.join(f'{v:.0f}' for v in qr.values())})", qr, ql))
    # both hands reaching the same inner edge at once overlap by construction (documented)
    known.append(("both hands at their inner edge x = -188 (cross the mid-plane)",) + tuple(
        _ik_pose((-188.0, 200.0, -600.0))[0] for _ in range(2)))
    for lab, qr, ql in poses:
        h = _torso_hits(qr, ql, statics)
        print(f"  {lab}: {'clear' if not h else h}")
        ok &= not h
    for lab, qr, ql in known:
        h = _torso_hits(qr, ql, statics)
        arm_arm = [x for x in h if x[0].startswith("R.") and x[1].startswith("L.")]
        other = [x for x in h if x not in arm_arm]
        print(f"  {lab}: {'clear' if not arm_arm else 'KNOWN (shared zone, one hand at a time): ' + str(arm_arm[:3])}")
        ok &= not other
    return ok


def check_bends():
    """Bend budget of every fixed tube run, drive unit -> shoulder (target <= 90 deg, R >= 25),
    and the runs clear the torso (rods along the routes vs printed parts and extrusions)."""
    from arm.cad import rod
    print("== tube bend budget (drive unit -> shoulder, fixed) ==")
    routes = A.tube_routes()
    ok = True
    print("  side joint cord helix   bend = arc1 + arc2   (straight before / after the arcs)  length")
    for r in routes:
        L = float(sum(np.linalg.norm(np.diff(r["pts"], axis=0), axis=1)))
        r["len"] = L
        flag = "ok" if r["bend"] <= 90.0 else "OVER BUDGET"
        print(f"  {r['side']}    {r['joint']:3s}  {r['cord']}    {r['helix']}     {r['bend']:5.1f} = {r['t1']:4.1f} + {r['t2']:4.1f}"
              f"     ({r['a']:4.0f} / {r['b']:4.0f} mm)                    {L:4.0f} mm  {flag}")
        ok &= r["bend"] <= 90.0
    worst = max(r["bend"] for r in routes)
    print(f"  worst fixed bend {worst:.1f} deg (old bench: ~180 deg U-turn); friction factor e^(mu*theta) at mu 0.25:"
          f" {math.exp(0.25 * math.radians(worst)):.2f} (U-turn: {math.exp(0.25 * math.pi):.2f})")
    statics = {k: w for k, w in A.torso_solids().items()}
    units = A.drive_solids()
    hits = []
    for r in routes:
        pts = r["pts"]
        own = f"drive_{r['side']}_{r['joint']}_"
        keep = [pts[1]] + [q for q in pts[2:-1] if np.linalg.norm(q - pts[1]) > 0.5]
        wire = cq.Workplane().add(cq.Wire.makePolygon([cq.Vector(*map(float, q)) for q in keep]))
        for n, w in list(statics.items()) + [(k, v) for k, v in units.items() if not k.startswith(own)]:
            if n.startswith("shoulder_comb") and r["joint"] != "S1":
                continue                                           # the comb holds these tubes
            if not bb_overlap(wire, w, P.TUBE_OD / 2):
                continue
            d = wire.val().distance(w.val())                       # tube centre line -> part
            if d < P.TUBE_OD / 2 - 0.1:
                hits.append((r["side"], r["joint"], r["cord"], n, round(d, 1)))
    print(f"  tube centre lines vs torso parts and other drive units (need >= {P.TUBE_OD / 2:.0f} mm):"
          f" {'clear' if not hits else sorted(set(hits))}")
    ok &= not hits
    return ok


def check_tubes():
    print("== Bowden tube passages ==")
    ok = True
    for j in A.JOINTS:
        s = A.joint_spec(j)
        n = s.tubes_through
        if hasattr(s, "cav"):
            w, h = 2 * (s.cav - 1), 2 * s.slot_y
            cap = int(w // (P.TUBE_OD + 0.3)) * int(h // (P.TUBE_OD + 0.3))
            kind = f"slot {w:.0f} x {h:.0f} mm"
        else:
            d = 2 * s.r_in
            cap = {12: 7, 13: 8, 15: 10, 16: 12, 17: 12, 18: 14, 19: 16, 20: 19}.get(int(d), int((d / (P.TUBE_OD + 0.3)) ** 2 * 0.78))
            kind = f"bore {d:.0f} mm"
        print(f"  {j:3s}: {n:2d} tubes through, {kind}, fits ~{cap}  {'ok' if cap >= n else 'TOO SMALL'}")
        ok &= cap >= n
    # roll joints: the Bowden sockets open under the stator plate, inside the link below it;
    # the link bore there must take the tube (+ 0.5 mm)
    from arm import link as L
    from arm.roll import RollSpec
    names = [c[0] for c in A.CHAIN]
    for j in A.JOINTS:
        s = A.joint_spec(j)
        if not isinstance(s, RollSpec):
            continue
        prev = A.CHAIN[names.index(j) - 1]
        bore = P.FLANGES[prev[1][1]]["tube_r"] - L.WALL
        need = s.socket_r_max() + P.TUBE_OD / 2 + 0.5
        good = bore >= need
        print(f"  {j:3s}: socket exits at r {s.socket_r_max():.1f}, {prev[0]} bore r {bore:.1f}"
              f" (needs {need:.1f})  {'ok' if good else 'TUBE BLOCKED'}")
        ok &= good
    return ok


def check_tensions():
    print("== tendon tension / crank effort (static, arm 2 kg, 0.5 kg payload) ==")
    # rough static torques with the arm horizontal
    g = 9.81
    torques = {"S1": 2.0 * g * 0.35 + 0.5 * g * 0.75, "S2": 2.0 * g * 0.30 + 0.5 * g * 0.70,
               "S3": 1.5, "E": 0.9 * g * 0.18 + 0.5 * g * 0.40, "F": 0.4, "W1": 0.3 * g * 0.08 + 0.5 * g * 0.15,
               "W2": 0.5 * g * 0.12}
    for j, t in torques.items():
        s = A.joint_spec(j)
        tension = t / (s.R / 1000)
        crank = tension * C.R_SPOOL / 1000
        print(f"  {j:3s}: joint {t:5.2f} N*m -> tendon {tension:5.0f} N, crank torque {crank:4.2f} N*m"
              f" ({crank / (C.ARM_R / 1000):4.1f} N on the knob), ratio {A.crank_ratio(j):.2f}:1")
    return True


def _balance_hits(q, statics, min_vol=0.5):
    """Balance parts (posed for q) against the arm (posed), the torso + cranks and each other."""
    arm = A.arm_solids(q)
    bal = A.balance_parts(q)
    hits = []
    for bn, bw in bal.items():
        cord = "cord" in bn
        for an, aw in arm.items():
            if cord and an == A.BALANCE_YOKE_ON:      # the cord starts on the yoke pin
                continue
            if bb_overlap(bw, aw) and (v := vol(bw, aw)) > min_vol:
                hits.append((bn, an, round(v, 1)))
        for sn, sw in statics.items():
            if bb_overlap(bw, sw) and (v := vol(bw, sw)) > min_vol:
                hits.append((bn, sn, round(v, 1)))
    skip = {("sheave", "cord"), ("cordback", "spring")}
    for (a, aw), (b, bw) in itertools.combinations(bal.items(), 2):
        ka, kb = a.split("_")[1], b.split("_")[1]
        if a[-1] != b[-1] or (ka, kb) in skip or (kb, ka) in skip:
            continue
        if bb_overlap(aw, bw) and (v := vol(aw, bw)) > min_vol:
            hits.append((a, b, round(v, 1)))
    return hits


def check_balance():
    """Shoulder balance hardware: yoke (on link_s1_s2), masts, sheaves, cords, springs,
    anchor posts vs the arm at the joint limits + combined poses, and the torso (shoulder
    beam, deck rails, drive units, ...), in the right arm's BEAM frame (the left arm is its
    mirror). Plus the yoke's clearance at full abduction."""
    print("== gravity balance hardware ==")
    inv = np.linalg.inv(A.MOUNT)
    statics = {k: A.place(w, inv) for k, w in {**A.torso_solids(), **A.drive_solids()}.items()}
    s1 = [(n, b) for n, _, b, _ in A.chain_frames({}) if n == "S1"][0][1]
    from arm import hinge as H
    statics["s1_base"] = A.place(H.base(A.joint_spec("S1")), s1)
    poses = [{}] + [{j: a} for j in ("S1", "S2") for a in (A.joint_spec(j).lo, A.joint_spec(j).hi)]
    poses += [{"S1": 60}, {"S1": 120, "S2": 100}, {"S1": -30, "S2": 100}, {"S1": 90, "S2": 100, "E": 115},
              {"S2": 100, "S3": 80, "E": 90}, {"S2": 100, "S3": -80, "E": 90}, {"S2": 85, "S3": -80, "E": 115},
              {"S1": 120, "S2": 60, "E": 90, "W1": 45}, {"S1": -30, "E": 115}]
    # documented reach limit: hand folded back over the shoulder (S2 > ~90, S3 < ~-70, E > ~105
    # together) touches the -X mast (the plain arm clears the beam there by ~37 mm)
    limits = [{"S2": 100, "S3": -80, "E": 115}]
    ok = True
    for q in poses + limits:
        h = _balance_hits(q, statics)
        if q in limits:
            print(f"  {q}: {'clear' if not h else 'KNOWN REACH LIMIT (hand vs mast, documented): ' + str(h)}")
            continue
        print(f"  {q}: {'clear' if not h else h}")
        ok &= not h
    # how close does the abducted upper arm come to the yoke tips?
    for q in ({"S2": 100}, {"S2": 100, "S3": 80, "E": 115}, {"S2": 100, "S3": -80, "E": 115}):
        arm = A.arm_solids(q)
        yoke = arm.pop(A.BALANCE_YOKE_ON).val()
        d = min(yoke.distance(w.val()) for n, w in arm.items()
                if A.CHAIN[[c[0] for c in A.CHAIN].index(owner(n))][0] not in ("S1", "link_s1_s2")
                and not n.startswith("s2_base"))
        print(f"  yoke clearance to the abducted arm at {q}: {d:.1f} mm")
        ok &= d > 2.0
    return ok


def check_gripper():
    """Gripper: finger/pad/palm interference over the whole opening, the cord and the
    opening spring run free, the open stops work, tool point = grasp centre, forces."""
    from arm import gripper as G
    from arm import kinematics as K
    from arm.cad import rod
    print("== gripper ==")
    ok = True
    good = abs(K.TOOL_OFFSET - G.GRASP_Z) < 1e-6
    print(f"  tool point {K.TOOL_OFFSET:.1f} mm past the W2 plate = grasp centre {G.GRASP_Z:.1f}  {'ok' if good else 'MISMATCH'}")
    ok &= good
    own = ({"gripper_finger_a", "gripper_pad_a"}, {"gripper_finger_b", "gripper_pad_b"})
    cord = rod((0, 0, G.BOSS_TOP + 0.3), (0, 0, G.ZP), P.CORD_D)
    yc = (G.SPRING[0] + G.FY) / 2            # spring centre plane
    for th in [0, 5, 10, 15, 20, 25, 30, 35, G.THETA_MAX]:
        p = G.parts(th)
        hits = [(a, b, round(v, 2)) for a, b in itertools.combinations(p, 2)
                if {a, b} not in own and (v := vol(p[a], p[b])) > 0.01]
        # the cord runs up the axis onto finger A's drum (touches it only at the tangent point)
        hits += [("cord", k, round(v, 2)) for k, w in p.items() if (v := vol(cord, w)) > 0.05]
        # the opening spring between the two posts (body without the hooks)
        L, F, _ = G.spring_state(th)
        x = L / 2 - G.POST_R - 2.5
        ang = math.radians(th)
        z = G.ZP + G.TAIL[0] * math.sin(ang) + G.TAIL[1] * math.cos(ang)
        spring = rod((-x, yc, z), (x, yc, z), G.SPRING_OD)
        hits += [("spring", k, round(v, 2)) for k, w in p.items() if (v := vol(spring, w)) > 0.01]
        print(f"  opening {th:4.1f} deg ({G.opening(th):4.1f} mm): {'clear' if not hits else hits}")
        ok &= not hits
    p = G.parts(G.THETA_MAX + 2)
    stop = vol(p["gripper_palm"], p["gripper_finger_a"]) > 0.05 and vol(p["gripper_palm"], p["gripper_finger_b"]) > 0.05
    print(f"  open stops on the palm end the travel at {G.THETA_MAX:.0f} deg: {'ok' if stop else 'NO STOP'}")
    ok &= stop
    w = G.opening(G.THETA_MAX)
    T10 = max(G.cord_for_force(10.0, th) for th in range(0, int(G.THETA_MAX) + 1))
    pull = min(2 * G.spring_state(th)[2] / G.R_C for th in range(0, int(G.THETA_MAX) + 1))
    print(f"  opening 0..{w:.0f} mm (need >= 60); cord for 10 N per pad <= {T10:.0f} N (need < 60);"
          f" spring pull-back on the cord >= {pull:.1f} N (need >= 1)")
    ok &= w >= 60 and T10 < 60 and pull >= 1.0
    return ok


if __name__ == "__main__":
    which = sys.argv[1:] or ["tubes", "tensions", "gripper", "ranges", "combos", "drives", "balance", "bends", "torso"]
    res = {}
    for w in which:
        res[w] = {"ranges": check_ranges, "combos": check_combos, "drives": check_drives, "torso": check_torso,
                  "bends": check_bends,
                  "tubes": check_tubes, "tensions": check_tensions, "balance": check_balance,
                  "gripper": check_gripper}[w]()
    print("\nRESULT:", {k: ("PASS" if v else "FAIL") for k, v in res.items()})
