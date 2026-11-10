"""Check the shoulder gravity balance (arm/balance.py).

    .venv/bin/python tools/check_balance.py

Over a grid of S1 x S2 x E (S3 = F = W = 0) it computes the gravity torque at S1 and
S2 from the CAD mass model, with and without the two balance cords (real sheave
geometry: cord tangent to the sheave, loop on the yoke pin), and reports the peak
holding torque and the crank-knob force. The spring offset s0 (= anchor screw
position) is tuned for the smallest S1 peak, within the +-20 mm screw range.

PASS: S1 peak reduced by >= 60 %, the screw setting is in range, the springs stay
open (cord tension > initial tension) and under their safe load everywhere.
Extra (info): S3 at +-80 deg, and a 0.5 kg payload.
"""
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm import assembly as A  # noqa: E402
from arm import balance as B  # noqa: E402
from arm import crank as C  # noqa: E402

S1S = list(range(-30, 121, 10))
S2S = [-10, 0, 25, 50, 75, 100]
ES = [0, 30, 60, 90, 115]


def knob(tau, j="S1"):
    """Force (N) on the crank knob that holds joint torque tau (N*m)."""
    tension = abs(tau) / (A.joint_spec(j).R / 1000)
    return tension * C.R_SPOOL / C.ARM_R


def grid(s3s=(0,), payload=0.0):
    ms = B.masses(payload)
    rows = []
    for s1, s2, e, s3 in itertools.product(S1S, S2S, ES, s3s):
        q = {"S1": s1, "S2": s2, "E": e, "S3": s3}
        rows.append((q, B.gravity_torque(q, "S1", ms=ms), B.gravity_torque(q, "S2", ms=ms)))
    return rows


def spring_table():
    """Spring geometry per S1 angle only (the yoke sits before S2)."""
    st = {}
    for s1 in S1S:
        q = {"S1": s1}
        fr = {n: (b, mv) for n, _, b, mv in A.chain_frames(q)}
        o, ax = B.joint_axis(fr, "S1")
        items = []
        for Bw in B.pins_world(q):
            sx = 1 if Bw[0] > 0 else -1
            _, u, s = B.cord_geometry(Bw, sx)
            items.append((s, np.cross(Bw - o, u) @ ax / 1000.0))
        st[s1] = items
    return st


def spring_tau(st, s1, s0):
    return sum(B.cord_tension(s, s0) * lever for s, lever in st[s1])


def main():
    ok = True
    rows = grid()
    st = spring_table()
    g1 = np.array([r[1] for r in rows])
    g2 = np.array([r[2] for r in rows])
    s1v = np.array([r[0]["S1"] for r in rows])
    peak0 = np.abs(g1).max()
    q0 = rows[int(np.abs(g1).argmax())][0]

    # tune s0 (anchor screw) for the smallest S1 peak
    best = None
    for s0 in np.arange(B.S0_DESIGN - 20, B.S0_DESIGN + 20.01, 0.5):
        res = g1 + np.array([spring_tau(st, s, s0) for s in s1v])
        p = np.abs(res).max()
        if best is None or p < best[1]:
            best = (s0, p, res)
    s0, peak1, res = best
    q1 = rows[int(np.abs(res).argmax())][0]
    red = 1 - peak1 / peak0

    print("== shoulder balance: grid S1 -30..120 x S2 -10..100 x E 0..115 ==")
    print(f"  design: a = {B.A_DIST} mm, b = {B.B_DIST:.0f} mm, X_S = +-{B.X_S:.0f} mm, k = {B.K_SPRING} N/mm per side,"
          f" k*a*b x2 = {2 * B.K_SPRING * B.A_DIST * B.B_DIST / 1000:.2f} N*m")
    ideal = max(abs(B.spring_torque({"S1": s}, s0) - B.ideal_torque({"S1": s})) for s in S1S)
    print(f"  real sheave vs ideal zero-length law: max deviation {ideal:.2f} N*m")
    shift = s0 - B.S0_DESIGN
    print(f"  anchor screw setting: {shift:+.1f} mm from nominal (range +-20)")
    ok &= abs(shift) <= 20
    print(f"  S1 peak holding torque  WITHOUT springs: {peak0:5.2f} N*m  ({knob(peak0):4.1f} N on the knob)  at {q0}")
    print(f"  S1 peak holding torque  WITH    springs: {peak1:5.2f} N*m  ({knob(peak1):4.1f} N on the knob)  at {q1}")
    print(f"  reduction: {red:.0%}  (need >= 60 %)")
    ok &= red >= 0.60
    for lab, cond in (("straight arm (S2 = 0, E = 0)", lambda q: q["S2"] == 0 and q["E"] == 0),
                      ("S2 = 0, any elbow", lambda q: q["S2"] == 0)):
        m = np.array([cond(r[0]) for r in rows])
        print(f"    {lab:30s}: {np.abs(g1[m]).max():5.2f} -> {np.abs(res[m]).max():5.2f} N*m")
    p2 = np.abs(g2).max()
    print(f"  S2 peak holding torque (springs act on S1 only, unchanged): {p2:5.2f} N*m ({knob(p2, 'S2'):4.1f} N on the knob)")

    print("  S1 sweep, straight arm:  S1   gravity   springs   residual   cord N   spring ext mm")
    for s in (-30, 0, 30, 60, 90, 120):
        q = {"S1": s}
        g = B.gravity_torque(q)
        sp = spring_tau(st, s, s0)
        t, e, _, _ = B.spring_state(q, s0)[1]
        print(f"                          {s:+4d}  {g:+7.2f}   {sp:+7.2f}   {g + sp:+7.2f}   {t:6.1f}   {e:6.1f}")

    # springs: always open, under the safe load
    ts = [B.spring_state({"S1": s}, s0)[1][0] for s in S1S]
    print(f"  cord tension per side {min(ts):.1f} .. {max(ts):.1f} N (spring initial tension {B.FI_SPRING} N,"
          f" safe load {B.SPRING_F_SAFE} N)")
    ok &= min(ts) > B.FI_SPRING and max(ts) < B.SPRING_F_SAFE

    print("== info ==")
    rows3 = grid(s3s=(-80, 0, 80))
    g13 = np.array([r[1] for r in rows3])
    res3 = g13 + np.array([spring_tau(st, r[0]["S1"], s0) for r in rows3])
    print(f"  with S3 -80/0/+80 too: {np.abs(g13).max():.2f} -> {np.abs(res3).max():.2f} N*m"
          f" ({1 - np.abs(res3).max() / np.abs(g13).max():.0%})")
    rowsp = grid(payload=500.0)
    g1p = np.array([r[1] for r in rowsp])
    resp = g1p + np.array([spring_tau(st, r[0]["S1"], s0) for r in rowsp])
    print(f"  with 0.5 kg payload:   {np.abs(g1p).max():.2f} -> {np.abs(resp).max():.2f} N*m"
          f" ({knob(np.abs(g1p).max()):.1f} -> {knob(np.abs(resp).max()):.1f} N on the knob)")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
