"""Where should the arm work? Scores a grid of hand positions in the shoulder frame.

    .venv/bin/python tools/workspace_study.py            # full grid (multi-core, a few minutes)
    .venv/bin/python tools/workspace_study.py --step 100 # coarser / faster

For every grid point (shoulder frame: +X outwards, +Y forwards, +Z up, origin on the S1 axis):
  * reach      - the tool point gets there at all
  * dexterity  - share of 8 grasp approach directions the hand can take there (each tried at
                 4 roll angles about the approach axis; a direction counts if any roll works)
  * cord load  - rms gravity cord tension over the 7 joints (0.3 kg payload) in the
                 lowest-lever posture, top-down grasp if possible, else position only
  * tube bend  - sum of |hinge angles| the distal tubes pass (S1+S2+E+W1): adds to the
                 Bowden friction of the wrist cords, e^(mu * bend)

Then: the table height with the largest "good" area (dexterity >= 6/8, rms cord <= LOAD_OK) and the
largest axis-aligned rectangle of good cells on it. Writes docs/workspace.json (viewer overlay),
docs/img/workspace_*.png and prints a summary.
"""
import argparse
import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm import kinematics as K  # noqa: E402
from arm import statics as S     # noqa: E402

HINGES = [0, 1, 3, 5, 6]          # S1, S2, E, W1, W2 in q
DIRS = {                          # grasp approach = hand axis (tool z) in the shoulder frame
    "down": (0, 0, -1), "forward": (0, 1, 0), "fwd-down": (0, 1, -1), "inward": (-1, 0, 0),
    "outward": (1, 0, 0), "in-down": (-1, 0, -1), "out-down": (1, 0, -1), "in-fwd": (-1, 1, 0),
}
ROLLS = (0.0, 90.0, 180.0, 270.0)
GOOD_DEX = 6 / 8
LOAD_OK = 70.0                    # N rms cord tension (gravity + payload, no springs)


def frame_from(a, roll_deg):
    z = np.asarray(a, float); z /= np.linalg.norm(z)
    ref = np.array([0, 0, 1.0]) if abs(z[2]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    x, y = c * x + s * y, -s * x + c * y
    return np.column_stack([x, y, z])


def rms_cord(q):
    from arm import assembly as A
    tau = S.gravity_torques(q, S.PAYLOAD)
    R = np.array([A.joint_spec(j).R / 1000 for j in K.JOINTS])
    return float(np.sqrt(np.mean((tau / R) ** 2)))


def score(p):
    p = np.asarray(p, float)
    r = K.ik(p, None, restarts=6)
    if not r.ok:
        return None
    ok_dirs, q_down = [], None
    for name, a in DIRS.items():
        for roll in ROLLS:
            rr = K.ik(p, frame_from(a, roll), q0=r.q, restarts=8, rot_tol=2.0, pos_tol=1.0)
            if rr.ok:
                ok_dirs.append(name)
                if name == "down" and q_down is None:
                    q_down = rr.q
                break
    if q_down is not None:       # top-down grasp: only the elbow swing is free -> lowest levers
        q, _ = K.optimize_posture(q_down, p, K.fk(q_down)[:3, :3], iters=10, rot_tol=2.0, pos_tol=1.0)
    else:
        q, _ = K.optimize_posture(r.q, p, None, iters=15)
    return dict(p=p.tolist(), dex=len(ok_dirs) / len(DIRS), dirs=ok_dirs, load=rms_cord(q),
                bend=float(sum(abs(q[i]) for i in HINGES[:4])), q=[round(float(v), 1) for v in q])


def best_rect(mask):
    """Largest all-True axis-aligned rectangle in a 2D boolean grid: (area, i0, i1, j0, j1)."""
    best = (0, 0, -1, 0, -1)
    nx, ny = mask.shape
    h = np.zeros(ny, int)
    for i in range(nx):
        h = np.where(mask[i], h + 1, 0)
        stack = []
        for j in range(ny + 1):
            cur = h[j] if j < ny else 0
            start = j
            while stack and stack[-1][1] >= cur:
                s0, hh = stack.pop()
                area = hh * (j - s0)
                if area > best[0]:
                    best = (area, i - hh + 1, i, s0, j - 1)
                start = s0
            stack.append((start, cur))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=50.0)
    ap.add_argument("--reuse", action="store_true", help="reuse the cached grid (docs/workspace_cells.json)")
    args = ap.parse_args()
    st = args.step
    xs = np.arange(-450, 650 + 1, st)
    ys = np.arange(-150, 800 + 1, st)
    zs = np.arange(-850, 250 + 1, st)
    pts = [(x, y, z) for z in zs for y in ys for x in xs]
    cache = ROOT / "docs" / "workspace_cells.json"
    if args.reuse and cache.exists():
        res = json.loads(cache.read_text())["res"]
    else:
        with Pool() as pool:
            res = pool.map(score, pts, chunksize=8)
        cache.write_text(json.dumps({"step": st, "res": res}))
    cells = {tuple(p): r for p, r in zip(pts, res)}
    ok = [c for c in res if c]
    dex = np.array([c["dex"] for c in ok]); load = np.array([c["load"] for c in ok])
    print(f"{len(ok)} reachable cells of {len(pts)}")
    print("dexterity histogram (share of 8 grasp directions):",
          {f"{k}/8": int(np.sum(np.isclose(dex * 8, k))) for k in range(9)})
    print(f"rms cord tension: min {load.min():.0f}, median {np.median(load):.0f}, max {load.max():.0f} N")
    from collections import Counter
    print("grasp directions reached (cells):", dict(Counter(d for c in ok for d in c["dirs"])))
    global GOOD_DEX, LOAD_OK
    # "good" = the best ~quarter of the reachable cells on both counts
    GOOD_DEX = float(np.quantile(dex, 0.75))
    LOAD_OK = float(np.quantile(load, 0.5))
    print(f"good cell: dexterity >= {GOOD_DEX * 8:.0f}/8 and rms cord <= {LOAD_OK:.0f} N")

    # table height: largest good area
    rows = []
    for z in zs:
        good = [(x, y) for x in xs for y in ys if (c := cells[(x, y, z)]) and c["dex"] >= GOOD_DEX and c["load"] <= LOAD_OK]
        reach = sum(1 for x in xs for y in ys if cells[(x, y, z)])
        rows.append((z, len(good) * st * st / 1e4, reach * st * st / 1e4))
    print("height z (mm)  good area (dm2)  reachable area (dm2)")
    for z, g, a in rows:
        print(f"  {z:6.0f}       {g:6.1f}          {a:6.1f}")
    z_best = max(rows, key=lambda r: r[1])[0]

    def plane(z):
        return np.array([[cells[(x, y, z)] is not None and cells[(x, y, z)]["dex"] >= GOOD_DEX
                          and cells[(x, y, z)]["load"] <= LOAD_OK for y in ys] for x in xs])
    mask = plane(z_best)
    area, i0, i1, j0, j1 = best_rect(mask)
    box = dict(x0=float(xs[i0] - st / 2), x1=float(xs[i1] + st / 2), y0=float(ys[j0] - st / 2), y1=float(ys[j1] + st / 2), z=float(z_best))
    inside = [cells[(x, y, z_best)] for x in xs[i0:i1 + 1] for y in ys[j0:j1 + 1]]
    print(f"\nbest table height: z = {z_best:.0f} mm (shoulder frame)")
    print(f"best work rectangle on it: x {box['x0']:.0f}..{box['x1']:.0f}, y {box['y0']:.0f}..{box['y1']:.0f} mm "
          f"({(box['x1'] - box['x0']):.0f} x {(box['y1'] - box['y0']):.0f})")
    print(f"  inside: dexterity {np.mean([c['dex'] for c in inside]):.2f} (min {min(c['dex'] for c in inside):.2f}), "
          f"rms cord {np.mean([c['load'] for c in inside]):.0f} N (max {max(c['load'] for c in inside):.0f}), "
          f"tube bend {np.mean([c['bend'] for c in inside]):.0f} deg (max {max(c['bend'] for c in inside):.0f})")
    # sweet spot: best single cell (dexterity first, then load)
    sweet = max((c for c in cells.values() if c), key=lambda c: (c["dex"], -c["load"] - 0.2 * c["bend"]))
    print(f"sweet spot: {sweet['p']} dexterity {sweet['dex']:.2f}, rms cord {sweet['load']:.0f} N, bend {sweet['bend']:.0f} deg")
    # bimanual: dexterity on the best plane vs x (x < 0 = towards the torso centre)
    print("\ndexterity / load on the best plane per x (mean over y), for choosing the shoulder spacing:")
    for x in xs:
        cs = [cells[(x, y, z_best)] for y in ys if cells[(x, y, z_best)]]
        if cs:
            print(f"  x {x:5.0f}: reach {len(cs):2d} cells, dexterity {np.mean([c['dex'] for c in cs]):.2f}, "
                  f"good {sum(c['dex'] >= GOOD_DEX and c['load'] <= LOAD_OK for c in cs):2d}")

    out = dict(step=st, z_best=float(z_best), box=box, sweet=sweet, heights=rows, good_dex=GOOD_DEX, load_ok=LOAD_OK,
               plane=[c for c in (cells[(x, y, z_best)] for x in xs for y in ys) if c])
    (ROOT / "docs" / "workspace.json").write_text(json.dumps(out, indent=0))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    img = ROOT / "docs" / "img"
    img.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 3, figsize=(16, 5.2))
    for ax, key, lab, cmap in ((axs[0], "dex", "dexterity (share of 8 grasp directions)", "viridis"),
                                (axs[1], "load", "rms cord tension, N (lower = better)", "magma_r"),
                                (axs[2], "bend", "tube bend S1+S2+E+W1, deg", "magma_r")):
        Z = np.full((len(ys), len(xs)), np.nan)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                c = cells[(x, y, z_best)]
                if c:
                    Z[j, i] = c[key]
        im = ax.imshow(Z, origin="lower", extent=(xs[0] - st / 2, xs[-1] + st / 2, ys[0] - st / 2, ys[-1] + st / 2), cmap=cmap)
        ax.add_patch(plt.Rectangle((box["x0"], box["y0"]), box["x1"] - box["x0"], box["y1"] - box["y0"], fill=False, ec="w", lw=2))
        ax.plot([0], [0], "w+", ms=12)
        ax.set_title(lab, fontsize=10); ax.set_xlabel("x (mm, + outwards)"); ax.set_ylabel("y (mm, + forwards)")
        plt.colorbar(im, ax=ax, fraction=0.046)
    fig.suptitle(f"Work plane z = {z_best:.0f} mm below/above the shoulder axis (white box: recommended work area, + = shoulder)")
    plt.tight_layout(); plt.savefig(img / "workspace_plane.png", dpi=100)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot([r[0] for r in rows], [r[1] for r in rows], "o-", label="good area")
    ax.plot([r[0] for r in rows], [r[2] for r in rows], "s--", label="reachable area")
    ax.set_xlabel("table height z (mm, shoulder frame)"); ax.set_ylabel("area (dm2)"); ax.legend(); ax.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(img / "workspace_heights.png", dpi=100)


if __name__ == "__main__":
    main()
