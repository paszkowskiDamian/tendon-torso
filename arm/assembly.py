"""The 7-DOF arm as a chain of modules + links, plus torso and cranks.

Chain frames are 4x4 matrices. Each module has its own local frame (see hinge.py /
roll.py); links run along +Z from 0 to their length. The chain is built "upwards"
from the S1 axis and then placed in the world hanging under the shoulder beam.
"""
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
import numpy as np

from . import crank as C
from . import gripper as G
from . import hinge as H
from . import link as L
from . import params as P
from . import roll as R
from . import torso as T
from .specs import E, F, S1, S2, S3, W1, W2

UPPER_ARM = 300.0      # S2 axis -> elbow axis
FOREARM = 250.0        # elbow axis -> wrist (W1) axis
FAN1, FAN2, SPACER = 60.0, 55.0, 22.0


def _len_upper():
    return UPPER_ARM - (S2.out_z + FAN1 + (S3.out_z - S3.in_z) - E.in_z)


def _len_fore():
    return FOREARM - (E.out_z + FAN2 + (F.out_z - F.in_z) - W1.in_z)


# (name, spec-or-link, mount rotation about local Z in deg)
CHAIN = [
    ("S1", S1, 0),
    ("link_s1_s2", ("L", "L", SPACER), 0),
    ("S2", S2, 90),
    ("link_s2_s3", ("L", "L", FAN1), 0),
    ("S3", S3, 0),
    ("link_upper", ("M", "S", None), 0),
    ("E", E, 90),
    ("link_e_f", ("S", "ML", FAN2), 0),
    ("F", F, 0),
    ("link_forearm", ("S", "S", None), 0),
    ("W1", W1, 0),
    ("link_w1_w2", ("S", "S", SPACER), 0),
    ("W2", W2, 90),
    ("gripper", G.GRIPPER, 0),     # end element: in = out = 0, its frame is the W2 plate top
]
JOINTS = ["S1", "S2", "S3", "E", "F", "W1", "W2"]
JOINT_LABEL = {"S1": "Shoulder flexion", "S2": "Shoulder abduction", "S3": "Upper-arm rotation",
               "E": "Elbow", "F": "Forearm twist", "W1": "Wrist flexion", "W2": "Wrist tilt", "G": "Gripper"}
# the gripper's fingers in the CAD assembly (collision checks, renders): fully open, the widest pose
GRIP_THETA = G.THETA_MAX


def link_length(name, spec):
    if spec[2] is not None:
        return spec[2]
    return _len_upper() if name == "link_upper" else _len_fore()


# ------------------------------------------------------------ matrices
def Tm(x=0, y=0, z=0):
    m = np.eye(4); m[:3, 3] = (x, y, z); return m


def Rx(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[1, 1], m[1, 2], m[2, 1], m[2, 2] = c, -s, s, c; return m


def Ry(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c; return m


def Rz(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c; return m


def place(wp, m):
    """Apply a rigid 4x4 matrix to a Workplane/Shape and return a Workplane."""
    from OCP.gp import gp_Trsf
    shape = wp.val() if hasattr(wp, "val") else wp
    t = gp_Trsf()
    t.SetValues(*[float(m[i, j]) for i in range(3) for j in range(4)])
    return cq.Workplane().add(shape.moved(cq.Location(t)))


# world placement of the chain frame: S1 axis under the beam, arm hanging, forward = -Y
WORLD = Tm(0, T.S1_Y, 0) @ Ry(180)       # frame on the underside of the beam floor


# ------------------------------------------------------------ parts per element
_cache = {}
BALANCE_YOKE_ON = "link_s1_s2"     # this spacer carries the gravity-balance yoke (balance.py)


def balance_yoke_link():
    """link_s1_s2 with the two balance yoke arms (same flanges/length as the plain spacer)."""
    from . import balance as BAL
    ca, cb, ln = dict((n, s) for n, s, _ in CHAIN)[BALANCE_YOKE_ON]
    return L.yoke_link(ca, cb, ln, BAL.X_S, BAL.B_Z, top_tip=BAL.YOKE_TIP_H)


def element_parts(name, spec):
    """(static parts, moving parts) in the element's local frame, as {part_name: workplane}."""
    if name in _cache:
        return _cache[name]
    key = name.lower()
    if isinstance(spec, H.HingeSpec):
        a, b = H.rotor_pair(spec)
        out = ({f"{key}_base": H.base(spec)}, {f"{key}_rotor_a": a, f"{key}_rotor_b": b})
    elif isinstance(spec, R.RollSpec):
        st = {f"{key}_stator": R.stator(spec),
              f"{key}_retainer": R.retainer(spec).translate((0, 0, spec.z_flange))}
        yi = spec.idler_y()
        tyre = R.idler_tyre()
        for tag, x, zg in (("a", spec.R, spec.z_a), ("b", -spec.R, spec.z_b)):
            st[f"{key}_idler_{tag}"] = (tyre.translate((0, 0, -R.TYRE_W / 2)).rotate((0, 0, 0), (0, 1, 0), 90)
                                        .translate((x, yi, zg - R.IDLER_CORD_R)))
        out = (st, {f"{key}_rotor": R.rotor(spec)})
    elif isinstance(spec, G.GripperSpec):
        out = (G.parts(GRIP_THETA), {})
    elif name == BALANCE_YOKE_ON:
        out = ({key: balance_yoke_link()}, {})
    else:
        ca, cb, _ = spec
        out = ({key: L.link(ca, cb, link_length(name, spec))}, {})
    _cache[name] = out
    return out


def element_io(spec, name):
    if isinstance(spec, (H.HingeSpec, R.RollSpec, G.GripperSpec)):
        return spec.in_z, spec.out_z
    return 0.0, link_length(name, spec)


def chain_frames(q=None):
    """World matrix of each element's local frame + of its moving part, for joint angles q (dict)."""
    q = q or {}
    m = WORLD.copy()
    frames = []
    for name, spec, mount in CHAIN:
        z_in, z_out = element_io(spec, name)
        base_m = m @ Rz(mount) @ Tm(0, 0, -z_in)
        if isinstance(spec, H.HingeSpec):
            mov = base_m @ Rx(q.get(name, 0.0))
        elif isinstance(spec, R.RollSpec):
            mov = base_m @ Rz(q.get(name, 0.0))
        else:
            mov = base_m
        frames.append((name, spec, base_m, mov))
        m = mov @ Tm(0, 0, z_out)
    return frames


def arm_solids(q=None, only=None):
    """{part_name: workplane in world} for the posed arm."""
    out = {}
    for name, spec, base_m, mov in chain_frames(q):
        if only and name not in only:
            continue
        st, mv = element_parts(name, spec)
        for k, w in st.items():
            out[k] = place(w, base_m)
        for k, w in mv.items():
            out[k] = place(w, mov)
    return out


# ------------------------------------------------------------ joints / ratios
DRIVE_ORDER = ["S1", "S2", "S3", "E", "F", "W1", "W2", "G"]   # G: gripper, one cord (spool side A)
CRANK_ORDER = DRIVE_ORDER


def joint_spec(j):
    return {"S1": S1, "S2": S2, "S3": S3, "E": E, "F": F, "W1": W1, "W2": W2, "G": G.GRIPPER}[j]


def crank_ratio(j):
    """Spool turns per joint turn (= drum radius / spool radius). For G: per finger turn.
    (The drive units use the old hand crank's spool, so the name stays.)"""
    return joint_spec(j).R / C.R_SPOOL


# ------------------------------------------------------------ gravity balance
def balance_parts(q=None, cords=True):
    """{part_name: workplane in the BEAM frame} of the shoulder balance for joint angles q:
    masts, sheaves, anchor posts, springs and cords (the yoke is link_s1_s2)."""
    from . import balance as BAL
    return BAL.posed_parts(q, cords=cords)


# ------------------------------------------------------------ two arms on the torso
SIDES = ("R", "L")
MOUNT = T.mount_matrix()             # right arm's BEAM frame (= parent of WORLD) in the TORSO frame
MIRROR = np.diag([-1.0, 1.0, 1.0, 1.0])


def arm_base(side="R"):
    """4x4 of an arm's chain base in the TORSO frame. The left arm is the mirror image of the
    right one: its solids are mirror('YZ') of the right arm's solids posed with the SAME
    joint values (q_left = q_right gives a mirror-symmetric pose); as a proper frame,
    every left matrix is MIRROR @ M_right @ MIRROR (with mirrored part geometry)."""
    m = MOUNT @ WORLD
    return m if side == "R" else MIRROR @ m @ MIRROR


def to_torso(w, side="R"):
    """Place a workplane given in the right arm's BEAM frame into the TORSO frame for side."""
    w = place(w, MOUNT)
    return w if side == "R" else w.mirror("YZ")


def point_torso(p, side="R"):
    """Point (3,) from the right arm's BEAM frame into the TORSO frame for side."""
    q = (MOUNT @ np.r_[np.asarray(p, float), 1.0])[:3]
    if side == "L":
        q[0] = -q[0]
    return q


def dir_torso(d, side="R"):
    d = np.array(d, float).copy()
    if side == "L":
        d[0] = -d[0]
    return d


def arm_world(side="R", q=None, only=None):
    """{part_name: workplane in the TORSO frame} of one posed arm."""
    return {k: to_torso(w, side) for k, w in arm_solids(q, only).items()}


def balance_world(side="R", q=None, cords=True):
    return {k: to_torso(w, side) for k, w in balance_parts(q, cords).items()}


def shoulder_frame_torso(side="R"):
    """SHOULDER frame (kinematics.py) in the TORSO frame; for 'L' its mirror (X outwards = +X)."""
    from . import kinematics as K
    m = MOUNT @ K.shoulder_frame()
    return m if side == "R" else MIRROR @ m @ MIRROR


# ------------------------------------------------------------ drive column
def drive_motor(j):
    from . import drive as DR
    return DR.PHASE1[j]


def drive_frames():
    """{(side, joint): 4x4 in the TORSO frame} of the 16 drive units."""
    out = {}
    for s in SIDES:
        out[(s, "S1")] = T.s1_unit_matrix(s)
        for k, j in enumerate(T.COLUMN_ORDER):
            out[(s, j)] = T.unit_matrix(s, k)
    return out


_unit_cache = {}


def drive_unit_parts(m, gripper=False):
    """{part: workplane} of one drive unit with motor m (unit frame). The gripper unit has
    only socket block A (one cord)."""
    from . import drive as DR
    key = (m, gripper)
    if key not in _unit_cache:
        p = dict(DR.unit_parts(m))
        if gripper:
            p["socket_block"] = C.socket_block_a()
        _unit_cache[key] = p
    return _unit_cache[key]


def drive_solids(only=None):
    """{name: workplane in the TORSO frame} of all drive units (spools at their zero)."""
    out = {}
    for (s, j), mtx in drive_frames().items():
        if only and j not in only:
            continue
        for k, w in drive_unit_parts(drive_motor(j), j == "G").items():
            out[f"drive_{s}_{j}_{k}"] = place(w, mtx)
    return out


def torso_solids(extrusions=True):
    """{name: workplane in the TORSO frame}: extrusions, saddles, shoulder beams and combs
    (both arms), electronics tray, E-stop box. (Drive units: drive_solids().)"""
    out = {}
    if extrusions:
        for n in T.extrusions():
            out[f"alu_{n}"] = T.extrusion_solid(n)
    sad = T.saddle()
    for k in range(T.LEVELS):
        out[f"saddle_{k}"] = place(sad, T.saddle_matrix(k))
    beam, comb = T.shoulder_beam(S1), T.shoulder_comb()
    for s in SIDES:
        out[f"shoulder_beam_{s}"] = to_torso(beam, s)
        out[f"shoulder_comb_{s}"] = to_torso(comb, s)
    hanger = T.s1_hanger()
    out["s1_hanger_R"], out["s1_hanger_L"] = hanger, hanger.mirror("YZ")
    out["electronics_tray"] = T.electronics_tray()
    out["estop_box"] = T.estop_box()
    return out


# ------------------------------------------------------------ Bowden tube routes (fixed part)
TUBE_R_MIN = 25.0          # PTFE 4x2: smallest allowed bend radius


def s1_sockets():
    """[(cord, point, direction)] of S1's two Bowden sockets in the right arm's BEAM frame:
    the point where the socket axis passes 2 mm above the S1 base plate, and the direction
    pointing into the socket (towards the drum)."""
    (ay, az), (ty, tz, _) = S1.tangent()
    u = np.array([0.0, ty - ay, tz - az])
    u /= np.linalg.norm(u)
    base = [b for n, _, b, _ in chain_frames({}) if n == "S1"][0]
    out = []
    for cord, s in (("A", -1), ("B", 1)):
        sy = -s
        A_ = np.array([s * S1.gx, sy * ay, az])
        d = np.array([0.0, sy * u[1], u[2]])
        Cp = A_ - d * ((az - S1.in_z + 2) / u[2])
        out.append((cord, (base @ np.r_[Cp, 1])[:3], base[:3, :3] @ d))
    return out


def comb_assignment():
    """{(joint, cord): (x, y) comb hole in the BEAM frame} for the 13 pass-through tubes.
    Lower drive units take the inner columns (their tubes then run inside the higher ones)."""
    holes = T.comb_holes()                 # 5 columns (inner first) x rows (-, 0, +)
    col = lambda c, r: holes[3 * c + {"-": 0, "0": 1, "+": 2}[r]]
    a = {}
    for c, j in enumerate(["S2", "S3", "E", "F", "W1"]):
        a[(j, "A")], a[(j, "B")] = col(c, "-"), col(c, "+")
    a[("W2", "A")], a[("W2", "B")], a[("G", "A")] = col(3, "0"), col(4, "0"), col(2, "0")
    return a


def _ang(u, v):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(u @ v) / (np.linalg.norm(u) * np.linalg.norm(v))))))


def bend_path(p0, d0, p1, d1, r=TUBE_R_MIN):
    """Smallest total bend of a tube that leaves p0 along d0 and enters p1 along d1, as
    straight - arc - straight - arc - straight with arcs of radius >= r: the two corner
    points slide along the start and end lines. Returns (total, theta1, theta2, a, b)."""
    p0, d0, p1, d1 = (np.asarray(v, float) for v in (p0, d0, p1, d1))
    d0, d1 = d0 / np.linalg.norm(d0), d1 / np.linalg.norm(d1)

    def ev(a, b):
        A_, B_ = p0 + a * d0, p1 - b * d1
        w = B_ - A_
        L = np.linalg.norm(w)
        if L < 1e-6:
            t1, t2 = _ang(d0, d1), 0.0
        else:
            t1, t2 = _ang(d0, w), _ang(w, d1)
        h1, h2 = r * math.tan(math.radians(t1) / 2), r * math.tan(math.radians(t2) / 2)
        if a < h1 - 1e-6 or b < h2 - 1e-6 or (L > 1e-6 and L < h1 + h2 - 1e-6):
            return None
        return (t1 + t2 + 1e-4 * (a + b), t1, t2, a, b)

    span = float(np.linalg.norm(p1 - p0)) + 50
    best = None
    for a in np.linspace(0.0, span, 61):
        for b in np.linspace(0.0, span, 61):
            e = ev(a, b)
            if e and (best is None or e[0] < best[0]):
                best = e
    if best is None:
        return None
    step = span / 60
    for _ in range(4):                      # refine around the best point
        a0, b0 = best[3], best[4]
        for a in np.linspace(max(0, a0 - step), a0 + step, 21):
            for b in np.linspace(max(0, b0 - step), b0 + step, 21):
                e = ev(a, b)
                if e and e[0] < best[0]:
                    best = e
        step /= 8
    return (best[1] + best[2], best[1], best[2], best[3], best[4])


def tube_routes(sides=SIDES):
    """Fixed tube runs from each drive unit to its shoulder (TORSO frame): list of dicts with
    side, joint, cord, p0, d0 (tube clip exit), p1, d1 (comb hole top or S1 socket),
    bend (deg, total), t1, t2, a, b, pts (polyline with the arcs)."""
    from . import drive as DR
    frames = drive_frames()
    comb = comb_assignment()
    sock = {c: (p, d) for c, p, d in s1_sockets()}
    out = []
    for s in sides:
        for j in DRIVE_ORDER:
            M = frames[(s, j)]
            exits = DR.tube_exits()
            options = []
            for helix in (("A",), ) if j == "G" else (("A", "B"), ("B", "A")):
                rows = []
                for cord, hx in zip("AB", helix):
                    pl, dl = exits["AB".index(hx)]
                    p0 = (M @ np.r_[pl, 1.0])[:3]
                    d0 = M[:3, :3] @ np.array(dl, float)
                    if j == "S1":
                        pb, db = sock[cord]
                        p1, d1 = point_torso(pb, s), dir_torso(db, s)
                    else:
                        x, y = comb[(j, cord)]
                        p1 = point_torso((x, y, T.COMB_Z[1]), s)
                        d1 = np.array([0.0, 0.0, -1.0])
                    tot, t1, t2, a, b = bend_path(p0, d0, p1, d1)
                    rows.append(dict(side=s, joint=j, cord=cord, helix=hx, p0=p0, d0=d0, p1=p1, d1=d1, bend=tot,
                                     t1=t1, t2=t2, a=a, b=b, pts=route_pts(p0, d0, p1, d1, a, b)))
                options.append(rows)
            # a joint cord may go to either spool helix (the firmware flips the motor sign):
            # take the mapping with the smaller worst bend
            out += min(options, key=lambda rows: max(r["bend"] for r in rows))
    return out


def route_pts(p0, d0, p1, d1, a, b, r=TUBE_R_MIN, n=10):
    """Polyline of a route: straight lines with r-radius arcs at the two corners."""
    d0, d1 = d0 / np.linalg.norm(d0), d1 / np.linalg.norm(d1)
    A_, B_ = p0 + a * d0, p1 - b * d1
    w = B_ - A_
    L = np.linalg.norm(w)
    w = w / L if L > 1e-6 else d1
    pts = [p0]
    for P_, u, v in ((A_, d0, w), (B_, w, d1)):
        th = math.radians(_ang(u, v))
        if th < 1e-4:
            pts.append(P_)
            continue
        h = r * math.tan(th / 2)
        s0, s1 = P_ - h * u, P_ + h * v
        perp = v - (v @ u) * u                 # arc centre: from s0, perpendicular to u, inside the bend
        perp /= np.linalg.norm(perp)
        c = s0 + r * perp
        e0, e1 = s0 - c, s1 - c
        for k in range(n + 1):
            t = k / n
            pts.append(c + (math.sin((1 - t) * th) * e0 + math.sin(t * th) * e1) / math.sin(th))
    pts.append(p1)
    return np.array(pts)
