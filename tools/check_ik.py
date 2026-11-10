"""Checks for arm/kinematics.py.

    .venv/bin/python tools/check_ik.py

1. FK equals the CAD assembly (every joint at random angles).
2. The analytic Jacobian equals finite differences.
3. IK round trip: random reachable poses -> IK from the straight pose -> error.
4. Workspace: reach envelope sampled from random joint angles.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm import assembly as A  # noqa: E402
from arm import kinematics as K  # noqa: E402

rng = np.random.default_rng(7)
lo, hi = K.limits()
ok_all = True

worst = 0.0
for _ in range(25):
    q = rng.uniform(lo, hi)
    _, _, T = K.fk_all(q, tool=0)
    fr = A.chain_frames(dict(zip(K.JOINTS, q)))
    name, spec, _, mov = fr[-1]
    cad = mov @ A.Tm(0, 0, A.element_io(spec, name)[1])
    worst = max(worst, float(np.abs(cad - T).max()))
print(f"1. FK vs CAD assembly: worst difference {worst:.2e} mm")
ok_all &= worst < 1e-6

q = rng.uniform(lo, hi)
J = K.jacobian(q)
h = 1e-4
Jn = np.zeros((6, 7))
for i in range(7):
    d = np.zeros(7); d[i] = h
    Tp, Tm = K.fk(q + d), K.fk(q - d)
    Jn[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * np.radians(h))
    Jn[3:, i] = K.rot_error(Tp[:3, :3], Tm[:3, :3]) / (2 * np.radians(h))
jerr = float(np.abs(J - Jn).max())
print(f"2. Jacobian vs finite differences: worst {jerr:.1e}")
ok_all &= jerr < 1e-4

for mode in ("position", "position + orientation"):
    n, ok, t0, errs = 150, 0, time.time(), []
    for k in range(n):
        qt = rng.uniform(lo, hi)
        T = K.fk(qt)
        r = K.ik(T[:3, 3], T[:3, :3] if "orientation" in mode else None, q0=np.zeros(7), seed=k)
        ok += r.ok
        errs.append((r.pos_err, r.rot_err))
    e = np.max(errs, axis=0)
    print(f"3. IK {mode:24s}: {ok}/{n} solved, worst {e[0]:.2f} mm / {e[1]:.2f} deg, "
          f"{(time.time() - t0) / n * 1000:.0f} ms per solve")
    ok_all &= ok == n

P = np.array([K.fk(rng.uniform(lo, hi))[:3, 3] for _ in range(4000)])
r = np.linalg.norm(P, axis=1)
print(f"4. Workspace (shoulder frame): x {P[:,0].min():.0f}..{P[:,0].max():.0f}, y {P[:,1].min():.0f}..{P[:,1].max():.0f}, "
      f"z {P[:,2].min():.0f}..{P[:,2].max():.0f} mm; reach from the S1 axis {r.min():.0f}..{r.max():.0f} mm")

print("\nRESULT:", "PASS" if ok_all else "FAIL")
sys.exit(0 if ok_all else 1)
