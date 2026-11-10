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
    ("link_s2_s3", ("L", "XL", FAN1), 0),
    ("S3", S3, 0),
    ("link_upper", ("M", "S", None), 0),
    ("E", E, 90),
    ("link_e_f", ("S", "XM", FAN2), 0),
    ("F", F, 0),
    ("link_forearm", ("S", "S", None), 0),
    ("W1", W1, 0),
    ("link_w1_w2", ("S", "S", SPACER), 0),
    ("W2", W2, 90),
]
JOINTS = ["S1", "S2", "S3", "E", "F", "W1", "W2"]
JOINT_LABEL = {"S1": "Shoulder flexion", "S2": "Shoulder abduction", "S3": "Upper-arm rotation",
               "E": "Elbow", "F": "Forearm twist", "W1": "Wrist flexion", "W2": "Wrist tilt"}


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
    else:
        ca, cb, _ = spec
        out = ({key: L.link(ca, cb, link_length(name, spec))}, {})
    _cache[name] = out
    return out


def element_io(spec, name):
    if isinstance(spec, (H.HingeSpec, R.RollSpec)):
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


# ------------------------------------------------------------ torso + cranks
CRANK_ORDER = ["S1", "S2", "S3", "E", "F", "W1", "W2"]
RACK_CENTRES = [(175.0, 40.0), (175.0, 150.0), (175.0, 260.0)]
SHAFT_Z = 5.0 - C.FOOT_Z[0]          # rack thickness + foot depth below the shaft


def crank_frames():
    """World matrix of each crank module (shaft frame)."""
    frames = {}
    for i, j in enumerate(CRANK_ORDER):
        rack, slot = divmod(i, 3)
        cx, cy = RACK_CENTRES[rack]
        x = cx - T.RACK_PITCH * 1.5 + T.RACK_PITCH * (slot + 0.5)
        frames[j] = Tm(x, cy, SHAFT_Z)
    return frames


def crank_parts():
    return {"crank_frame": C.frame(), "crank_socket_block": C.socket_block(), "crank_nut_cup": C.nut_cup(),
            "crank_spool": C.spool(), "crank_arm": C.crank_arm(), "crank_knob": C.knob()}


def joint_spec(j):
    return {"S1": S1, "S2": S2, "S3": S3, "E": E, "F": F, "W1": W1, "W2": W2}[j]


def crank_ratio(j):
    """Crank turns per joint turn (= drum radius / spool radius)."""
    return joint_spec(j).R / C.R_SPOOL
