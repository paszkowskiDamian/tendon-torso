"""Command line for the arm's kinematics.

  Inverse (hand target -> joint angles + crank turns), shoulder frame, mm / deg:
    python -m arm.ik 150 250 -500                 # position only
    python -m arm.ik 150 250 -500 --rpy 0 90 0    # position + hand orientation
    python -m arm.ik 150 250 -500 --tool 0        # target the W2 plate centre

  Forward (joint angles -> hand pose):
    python -m arm.ik --fk 45 25 20 70 40 30 10

Shoulder frame: origin on the S1 axis, +X outwards, +Y forwards, +Z up.
"""
import argparse
import sys

import numpy as np

from . import kinematics as K


def show_pose(T):
    p = T[:3, 3]
    r = K.R_to_rpy(T[:3, :3])
    print(f"  hand position  x {p[0]:8.1f}  y {p[1]:8.1f}  z {p[2]:8.1f}  mm")
    print(f"  hand rotation  roll {r[0]:7.1f}  pitch {r[1]:7.1f}  yaw {r[2]:7.1f}  deg")


def show_joints(q):
    lo, hi = K.limits()
    turns = K.crank_turns(q)
    print("  joint                    angle    range        crank turns")
    from .assembly import JOINT_LABEL
    for i, j in enumerate(K.JOINTS):
        flag = "  at limit" if min(q[i] - lo[i], hi[i] - q[i]) < 0.5 else ""
        print(f"  {j:3s} {JOINT_LABEL[j]:20s} {q[i]:7.1f}°  {lo[i]:5.0f}..{hi[i]:<5.0f}   {turns[i]:+6.2f}{flag}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xyz", nargs="*", type=float, help="target x y z (mm, shoulder frame)")
    ap.add_argument("--rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"), help="hand orientation (deg)")
    ap.add_argument("--fk", nargs=7, type=float, metavar="Q", help="forward kinematics for 7 joint angles (deg)")
    ap.add_argument("--tool", type=float, default=K.TOOL_OFFSET, help="tool point beyond the W2 plate (mm)")
    ap.add_argument("--from", dest="q0", nargs=7, type=float, metavar="Q", help="start from these joint angles")
    a = ap.parse_args(argv)

    if a.fk:
        q = np.array(a.fk)
        print("forward kinematics")
        show_joints(q)
        show_pose(K.fk(q, tool=a.tool))
        return 0
    if len(a.xyz) != 3:
        ap.error("give x y z (or --fk)")
    R = K.rpy_to_R(*a.rpy) if a.rpy else None
    r = K.ik(np.array(a.xyz), R, q0=np.array(a.q0) if a.q0 else None, tool=a.tool)
    print("inverse kinematics:", "SOLVED" if r.ok else "NOT REACHABLE - closest pose shown",
          f"(position error {r.pos_err:.2f} mm" + (f", rotation error {r.rot_err:.2f} deg)" if R is not None else ")"))
    show_joints(r.q)
    show_pose(K.fk(r.q, tool=a.tool))
    return 0 if r.ok else 1


if __name__ == "__main__":
    sys.exit(main())
