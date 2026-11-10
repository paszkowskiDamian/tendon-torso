"""Compare arm variants with fewer joints: mass, shoulder load and what the hand can still do.

    .venv/bin/python tools/dof_study.py

A removed joint is replaced by a rigid printed spacer of the same length (so the
arm keeps its 300 / 250 mm segments) and its module mass is swapped for the spacer's.
Removing W2 also removes the W1->W2 spacer and the W2 module, so the hand gets
110 mm shorter.

Metrics (targets sampled from the full 7-DOF arm):
  * mass, and S1 torque with the arm held horizontal + 0.3 kg payload
  * position reach: share of 7-DOF hand positions the variant still reaches
  * pose reach: share of 7-DOF hand poses (position + orientation) it still reaches
  * rms cord tension over the reached positions (lower = easier to crank, lighter loads)
"""
import copy
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm import kinematics as K  # noqa: E402
from arm import statics as S  # noqa: E402

BASE_CHAIN = K.chain()
BASE_MODEL = copy.deepcopy(S.mass_model())
SPACER_G = {"S3": 70.0, "F": 55.0, "W2": 25.0}     # printed spacer replacing a removed module (g)

VARIANTS = {
    "7 DOF (current)": [],
    "6 DOF, no wrist tilt (W2)": ["W2"],
    "6 DOF, no forearm twist (F)": ["F"],
    "6 DOF, no upper-arm rotation (S3)": ["S3"],
    "5 DOF, no S3 and no W2": ["S3", "W2"],
}


def apply_variant(removed):
    W, els = BASE_CHAIN
    els2 = []
    for e in els:
        e2 = copy.copy(e)
        if e.name in removed:
            e2.lo, e2.hi = -1e-3, 1e-3
        els2.append(e2)
    K.chain = lambda: (W, els2)               # noqa: E731
    model = copy.deepcopy(BASE_MODEL)
    for el in model["elements"]:
        if el["name"] in removed:
            el["m_static_g"], el["m_moving_g"] = SPACER_G[el["name"]], 0.0
        if "W2" in removed and el["name"] == "link_w1_w2":
            el["m_static_g"] = 0.0
    S._MODEL = model
    K._SF = None
    tool = K.TOOL_OFFSET - (110.0 if "W2" in removed else 0.0)
    return tool


def mass_kg():
    return sum(e["m_static_g"] + e["m_moving_g"] for e in S.mass_model()["elements"]) / 1000


def main():
    lo7, hi7 = K.limits()
    rng = np.random.default_rng(11)
    q7 = [rng.uniform(lo7, hi7) for _ in range(120)]
    poses7 = [K.fk(q) for q in q7]
    rows = []
    for label, removed in VARIANTS.items():
        tool = apply_variant(removed)
        Rj = np.array([0.042, 0.042, 0.028, 0.036, 0.0255, 0.02, 0.02])
        # shoulder torque, arm horizontal forwards
        S1 = abs(S.gravity_torques(np.array([90, 0, 0, 0, 0, 0, 0.]))[0])
        pos_ok = pose_ok = 0
        tens = []
        for T in poses7:
            # the same hand-point targets, scaled to this variant's hand length along the hand axis
            p = T[:3, 3] - T[:3, 2] * (110.0 if "W2" in removed else 0.0)
            r = K.ik(p, None, tool=tool, restarts=4)
            if r.ok:
                pos_ok += 1
                tens.append(np.sqrt(np.mean((S.gravity_torques(r.q) / Rj) ** 2)))
            rp = K.ik(p, T[:3, :3], tool=tool, restarts=10, rot_tol=2.0, pos_tol=1.0)
            pose_ok += rp.ok
        n = len(poses7)
        rows.append((label, mass_kg(), S1, pos_ok / n, pose_ok / n, float(np.mean(tens)) if tens else float("nan")))
        print(f"{label:36s} mass {rows[-1][1]:.2f} kg | S1 horizontal {S1:5.2f} N*m | "
              f"positions {pos_ok / n:5.0%} | poses {pose_ok / n:5.0%} | rms cord {rows[-1][5]:5.1f} N", flush=True)
    return rows


if __name__ == "__main__":
    main()
