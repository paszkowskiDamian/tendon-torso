"""Forward and inverse kinematics of the 7-DOF tendon arm.

The kinematic chain is read from `arm/assembly.py` (the same numbers that place the
printed parts), so the maths can never drift away from the CAD.

Frames
------
* WORLD: table top at z=0, shoulder beam along Y, the arm hangs at y = -95.
* SHOULDER frame (what you normally give targets in): origin on the S1 axis,
  +X = outwards (the way abduction lifts the arm), +Y = forwards (the way
  flexion lifts the arm), +Z = up. Units: mm and degrees.
* TOOL point: centre of the W2 rotor plate (the gripper mount) + TOOL_OFFSET
  along the hand axis = the gripper's grasp centre (between the pad centres,
  gripper.GRASP_Z; the last chain element, the gripper, has in = out = 0).

Chain (see docs/kinematics.md for the equations)
------------------------------------------------
For every element i (module or link) the transform is

    A_i(q_i) = Rz(m_i) * Tz(-in_i) * R_i(q_i) * Tz(out_i)

with R_i = Rx(q) for a hinge, Rz(q) for a roll joint, identity for a link, and

    T_tool(q) = WORLD * A_1(q_1) * ... * A_14 * Tz(TOOL_OFFSET)
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

JOINTS = ["S1", "S2", "S3", "E", "F", "W1", "W2"]
TOOL_OFFSET = 72.0     # = gripper.GRASP_Z (tools/check_arm.py gripper checks it)


# ------------------------------------------------------------------ small SE(3) helpers
def Tz(d):
    m = np.eye(4); m[2, 3] = d; return m


def Rx(rad):
    c, s = math.cos(rad), math.sin(rad)
    m = np.eye(4); m[1, 1], m[1, 2], m[2, 1], m[2, 2] = c, -s, s, c; return m


def Rz(rad):
    c, s = math.cos(rad), math.sin(rad)
    m = np.eye(4); m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c; return m


def rpy_to_R(roll, pitch, yaw):
    """Rotation from roll/pitch/yaw in degrees (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) < 1e-6:
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    else:
        roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def rot_error(R_target, R_now):
    """Axis * angle (rad) that rotates R_now onto R_target, in the base frame."""
    Re = R_target @ R_now.T
    c = max(-1.0, min(1.0, (np.trace(Re) - 1) / 2))
    ang = math.acos(c)
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    if math.pi - ang < 1e-6:                      # 180 deg: pick the axis from the diagonal
        axis = np.sqrt(np.clip((np.diag(Re) + 1) / 2, 0, 1))
        k = int(np.argmax(axis))
        axis = axis * np.sign(np.r_[Re[k, 0] + Re[0, k], Re[k, 1] + Re[1, k], Re[k, 2] + Re[2, k]][:3] + 1e-12)
        return axis / np.linalg.norm(axis) * ang
    return v / (2 * math.sin(ang)) * ang


# ------------------------------------------------------------------ the chain
@dataclass
class Element:
    name: str
    mount: float      # deg about local Z
    in_z: float
    out_z: float
    kind: str         # "hinge" (about local X), "roll" (about local Z) or "link"
    lo: float = 0.0
    hi: float = 0.0


@lru_cache(maxsize=1)
def chain():
    """(WORLD matrix, [Element...]) taken from the CAD assembly."""
    from . import assembly as A
    from .hinge import HingeSpec
    from .roll import RollSpec
    els = []
    for name, spec, mount in A.CHAIN:
        z_in, z_out = A.element_io(spec, name)
        if isinstance(spec, HingeSpec):
            els.append(Element(name, mount, z_in, z_out, "hinge", spec.lo, spec.hi))
        elif isinstance(spec, RollSpec):
            els.append(Element(name, mount, z_in, z_out, "roll", spec.lo, spec.hi))
        else:
            els.append(Element(name, mount, z_in, z_out, "link"))
    return np.array(A.WORLD, dtype=float), els


def limits():
    _, els = chain()
    j = [e for e in els if e.kind != "link"]
    return np.array([e.lo for e in j]), np.array([e.hi for e in j])


def shoulder_frame():
    """World pose of the SHOULDER frame (origin on the S1 axis, X out, Y forward, Z up)."""
    W, els = chain()
    s1 = W @ Rz(math.radians(els[0].mount)) @ Tz(-els[0].in_z)
    # axes: work them out from the zero pose so they follow the CAD conventions
    q0 = np.zeros(7)
    fwd = _point_after(np.array([10.0, 0, 0, 0, 0, 0, 0]), q0)     # flexion moves the hand forwards
    out = _point_after(np.array([0, 10.0, 0, 0, 0, 0, 0]), q0)     # abduction moves it outwards
    y = fwd * np.array([1, 1, 0]); y /= np.linalg.norm(y)
    x = out * np.array([1, 1, 0]); x -= y * (x @ y); x /= np.linalg.norm(x)
    z = np.cross(x, y)
    F = np.eye(4); F[:3, 0], F[:3, 1], F[:3, 2], F[:3, 3] = x, y, z, s1[:3, 3]
    return F


def _point_after(dq, q0):
    return fk(q0 + dq, frame="world")[:3, 3] - fk(q0, frame="world")[:3, 3]


# ------------------------------------------------------------------ forward kinematics
def fk_all(q_deg, tool=TOOL_OFFSET):
    """World 4x4 of every joint frame (before its rotation) and of the tool.

    Returns (joint_frames[7], axes[7] (unit, world), T_tool).
    """
    W, els = chain()
    M = W.copy()
    frames, axes = [], []
    k = 0
    for e in els:
        M = M @ Rz(math.radians(e.mount)) @ Tz(-e.in_z)
        if e.kind != "link":
            frames.append(M.copy())
            axes.append(M[:3, 0].copy() if e.kind == "hinge" else M[:3, 2].copy())
            a = math.radians(q_deg[k]); k += 1
            M = M @ (Rx(a) if e.kind == "hinge" else Rz(a))
        M = M @ Tz(e.out_z)
    return frames, axes, M @ Tz(tool)


_SF = None


def fk(q_deg, frame="shoulder", tool=TOOL_OFFSET):
    """Tool pose (4x4) for joint angles q (deg, order S1 S2 S3 E F W1 W2)."""
    global _SF
    T = fk_all(q_deg, tool)[2]
    if frame == "world":
        return T
    if _SF is None:
        _SF = np.linalg.inv(shoulder_frame())
    return _SF @ T


def jacobian(q_deg, frame="shoulder", tool=TOOL_OFFSET):
    """6x7 geometric Jacobian [dv (mm/rad); dw (rad/rad)] in the chosen frame."""
    frames, axes, T = fk_all(q_deg, tool)
    p = T[:3, 3]
    J = np.zeros((6, 7))
    for i, (F, a) in enumerate(zip(frames, axes)):
        J[:3, i] = np.cross(a, p - F[:3, 3])
        J[3:, i] = a
    if frame == "shoulder":
        global _SF
        if _SF is None:
            _SF = np.linalg.inv(shoulder_frame())
        R = _SF[:3, :3]
        J = np.vstack([R @ J[:3], R @ J[3:]])
    return J


# ------------------------------------------------------------------ inverse kinematics
@dataclass
class IKResult:
    q: np.ndarray          # joint angles, deg
    ok: bool
    pos_err: float         # mm
    rot_err: float         # deg (0 when orientation was not asked for)
    iters: int


def ik(target_pos, target_R=None, q0=None, tool=TOOL_OFFSET, frame="shoulder",
       pos_tol=0.5, rot_tol=0.5, max_iter=200, rot_weight=150.0, restarts=None, seed=0):
    """Joint angles that put the tool at target_pos (mm), optionally with orientation target_R.

    Damped least squares with joint limits; the 4 (position only) or 1 (full pose)
    spare degrees of freedom pull gently towards the middle of each joint's range.
    """
    lo, hi = limits()
    mid = (lo + hi) / 2
    rng = np.random.default_rng(seed)
    if restarts is None:
        restarts = 30 if target_R is not None else 8
    best = None
    seeds = [np.clip(q0 if q0 is not None else mid, lo, hi)]
    seeds += [mid] + [rng.uniform(lo, hi) for _ in range(restarts)]
    for qs in seeds:
        r = _ik_once(np.array(target_pos, float), target_R, qs.astype(float), lo, hi, mid, tool, frame,
                     pos_tol, rot_tol, max_iter, rot_weight)
        if best is None or (r.ok and not best.ok) or (r.ok == best.ok and r.pos_err + r.rot_err < best.pos_err + best.rot_err):
            best = r
        if r.ok:
            break
    return best


def _err(p_t, R_t, q, frame, tool, w_rot):
    T = fk(q, frame, tool)
    e_p = p_t - T[:3, 3]
    if R_t is None:
        return e_p, np.zeros(3), T
    return e_p, rot_error(R_t, T[:3, :3]), T


def _ik_once(p_t, R_t, q, lo, hi, mid, tool, frame, pos_tol, rot_tol, max_iter, w_rot):
    """Levenberg-Marquardt / damped least squares with an active set for joint limits."""
    lam = 2.0
    half = (hi - lo) / 2
    e_p, e_w, _ = _err(p_t, R_t, q, frame, tool, w_rot)
    for it in range(1, max_iter + 1):
        pe, re = float(np.linalg.norm(e_p)), math.degrees(float(np.linalg.norm(e_w)))
        if pe < pos_tol and re < rot_tol:
            return IKResult(q, True, pe, re, it)
        J = jacobian(q, frame, tool) * (np.pi / 180)            # per degree
        if R_t is not None:
            e = np.r_[e_p, w_rot * e_w]
            Jt = np.vstack([J[:3], w_rot * J[3:]])
        else:
            e, Jt = e_p, J[:3]
        free = np.ones(7, bool)
        for _ in range(3):                                      # drop joints pushing into a limit
            Jf = Jt * free
            JJt = Jf @ Jf.T
            dq = Jf.T @ np.linalg.solve(JJt + lam ** 2 * np.eye(len(e)), e)
            Jp = Jf.T @ np.linalg.inv(JJt + 1e-4 * np.eye(len(e)))
            N = (np.eye(7) - Jp @ Jf) * free
            dq += N @ (0.1 * (mid - q) * (1 + 3 * ((q - mid) / half) ** 4))
            push = ((q <= lo + 1e-6) & (dq < 0)) | ((q >= hi - 1e-6) & (dq > 0))
            if not (push & free).any():
                break
            free &= ~push
        step = float(np.max(np.abs(dq)))
        if step > 15:
            dq *= 15 / step
        q_new = np.clip(q + dq, lo, hi)
        ne_p, ne_w, _ = _err(p_t, R_t, q_new, frame, tool, w_rot)
        if np.linalg.norm(np.r_[ne_p, w_rot * ne_w]) < np.linalg.norm(np.r_[e_p, w_rot * e_w]):
            q, e_p, e_w, lam = q_new, ne_p, ne_w, max(lam * 0.5, 0.05)
        else:
            lam = min(lam * 3.0, 1e3)
            if lam >= 1e3:
                break
    pe, re = float(np.linalg.norm(e_p)), math.degrees(float(np.linalg.norm(e_w)))
    return IKResult(q, pe < pos_tol and re < rot_tol, pe, re, it)


def crank_turns(q_deg):
    """Crank turns from the straight pose for each joint (= joint angle * ratio / 360)."""
    from . import assembly as A
    return np.array([q_deg[i] * A.crank_ratio(j) / 360.0 for i, j in enumerate(JOINTS)])


# ------------------------------------------------------------------ low-lever IK
def tension_cost(q_deg, payload=None):
    """Sum of squared gravity cord tensions (N^2): small = arm held close, short levers."""
    from . import statics as S
    from . import assembly as A
    tau = S.gravity_torques(q_deg, S.PAYLOAD if payload is None else payload)
    R = np.array([A.joint_spec(j).R / 1000 for j in JOINTS])
    return float(np.sum((tau / R) ** 2))


def _task_jac(q, R_t, frame, tool, w_rot):
    J = jacobian(q, frame, tool) * (np.pi / 180)
    return np.vstack([J[:3], w_rot * J[3:]]) if R_t is not None else J[:3]


def optimize_posture(q, target_pos, target_R=None, tool=TOOL_OFFSET, frame="shoulder", iters=40,
                     pos_tol=0.5, rot_tol=0.5, w_rot=150.0):
    """Keep the hand where it is and move the spare joints to lower the cord tensions."""
    lo, hi = limits()
    p_t = np.asarray(target_pos, float)
    cost = tension_cost(q)
    alpha = 4.0
    for _ in range(iters):
        g = np.zeros(7)
        for i in range(7):
            d = np.zeros(7); d[i] = 0.5
            g[i] = (tension_cost(np.clip(q + d, lo, hi)) - tension_cost(np.clip(q - d, lo, hi))) / 1.0
        Jt = _task_jac(q, target_R, frame, tool, w_rot)
        N = np.eye(7) - np.linalg.pinv(Jt) @ Jt
        # joints at a limit that would be pushed further don't move
        step = -N @ g
        nrm = np.max(np.abs(step))
        if nrm < 1e-9:
            break
        improved = False
        while alpha > 0.05:
            qn = np.clip(q + step / nrm * alpha, lo, hi)
            r = _ik_once(p_t, target_R, qn, lo, hi, (lo + hi) / 2, tool, frame, pos_tol, rot_tol, 30, w_rot)
            if r.ok:
                c = tension_cost(r.q)
                if c < cost - 1e-6:
                    q, cost, improved = r.q, c, True
                    alpha = min(alpha * 1.5, 10.0)
                    break
            alpha *= 0.5
        if not improved:
            break
    return q, cost


def ik_low_lever(target_pos, target_R=None, q0=None, tool=TOOL_OFFSET, frame="shoulder", seeds=6, seed=0):
    """IK that, among all joint solutions reaching the target, prefers short gravity levers
    (minimum sum of squared cord tensions). Uses several starting postures + null-space descent."""
    lo, hi = limits()
    rng = np.random.default_rng(seed)
    starts = [] if q0 is None else [np.asarray(q0, float)]
    starts += [np.zeros(7), np.array([30, 10, 0, 90, 0, 0, 0.]), np.array([60, 20, -40, 100, 0, 0, 0.]),
               np.array([60, 20, 40, 100, 0, 0, 0.])]
    starts += [rng.uniform(lo, hi) for _ in range(seeds)]
    best = None
    for s in starts:
        r = ik(target_pos, target_R, q0=s, tool=tool, frame=frame, restarts=0, seed=seed)
        if not r.ok:
            continue
        q, c = optimize_posture(r.q, target_pos, target_R, tool, frame)
        if best is None or c < best[1]:
            T = fk(q, frame, tool)
            best = (q, c, IKResult(q, True, float(np.linalg.norm(np.asarray(target_pos) - T[:3, 3])),
                                   math.degrees(float(np.linalg.norm(rot_error(target_R, T[:3, :3])))) if target_R is not None else 0.0, 0))
    if best is None:
        return ik(target_pos, target_R, q0=q0, tool=tool, frame=frame), None
    return best[2], best[1]
