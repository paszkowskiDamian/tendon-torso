"""Record scripted pick-and-place demos as a LeRobot dataset (lerobot 0.7, dataset v3).

    .venv/bin/python -m sim.record --episodes 100                     # -> data/sim_pick_place
    .venv/bin/python -m sim.record --episodes 100 --side both --root data/sim_pick_place_both

Per frame (25 fps):
    observation.state         (16,)  joint angles (rad) + grip (0..1) of both arms (env.py)
    action                    (16,)  joint targets sent to the position servos
    observation.images.top    overhead camera
    observation.images.wrist  wrist camera of the working arm (the right one with --side R)
Only successful episodes are kept (the scripted demos succeed ~100%).
--noise: the free moves are executed with a slowly drifting joint offset (DART), the
recorded action stays the expert's clean target, so the policy sees how to recover.
"""
import argparse
import shutil
import time
from pathlib import Path

import numpy as np

from sim import env as E
from sim import scripted as SC

ROOT = Path(__file__).resolve().parents[1]
TASK = E.TASK
NAMES = [f"{s}_{j}" for s in "rl" for j in E.JOINTS + ["grip"]]


def features(h, w, cams):
    f = {"observation.state": {"dtype": "float32", "shape": (16,), "names": NAMES},
         "action": {"dtype": "float32", "shape": (16,), "names": NAMES}}
    for c in cams:
        f[f"observation.images.{c}"] = {"dtype": "video", "shape": (h, w, 3), "names": ["height", "width", "channels"]}
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--side", default="R", choices=["R", "L", "both"])
    ap.add_argument("--root", default="data/sim_pick_place")
    ap.add_argument("--repo-id", default="local/tendon_torso_sim_pick_place")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--noise", type=float, default=2.0,
                    help="max DART noise scale (scripted.NOISE_DEG); each episode draws 0..max")
    ap.add_argument("--workers", type=int, default=1, help="record in parallel processes, then merge")
    args = ap.parse_args()
    root = ROOT / args.root
    if root.exists():
        if not args.overwrite:
            raise SystemExit(f"{root} exists (use --overwrite)")
        shutil.rmtree(root)
    if args.workers > 1:
        return parallel(args, root)
    record(args, root)


def parallel(args, root):
    """Each worker records its share into <root>_partK (seeds far apart), then merge."""
    import subprocess
    import sys
    from lerobot.datasets.aggregate import aggregate_datasets
    n = args.workers
    share = [args.episodes // n + (k < args.episodes % n) for k in range(n)]
    procs, parts = [], []
    for k in range(n):
        part = root.parent / f"{root.name}_part{k}"
        parts.append(part)
        cmd = [sys.executable, "-m", "sim.record", "--episodes", str(share[k]), "--seed", str(args.seed + 100_000 * k),
               "--side", args.side, "--root", str(part.relative_to(ROOT)), "--repo-id", f"{args.repo_id}_part{k}",
               "--width", str(args.width), "--height", str(args.height), "--noise", str(args.noise), "--overwrite"]
        procs.append(subprocess.Popen(cmd, cwd=ROOT))
    if any(p.wait() for p in procs):
        raise SystemExit("a recording worker failed")
    aggregate_datasets([f"{args.repo_id}_part{k}" for k in range(n)], args.repo_id, roots=parts, aggr_root=root)
    for part in parts:
        shutil.rmtree(part)
    print(f"dataset: {root}  (merged from {n} workers)")


def record(args, root):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    sim_cams = ("top", "r_wrist", "l_wrist")
    env = E.PickPlaceEnv(cameras=sim_cams, side=args.side, width=args.width, height=args.height)
    ds = LeRobotDataset.create(args.repo_id, E.FPS, features(args.height, args.width, ("top", "wrist")),
                               root=root, robot_type="tendon_torso_sim", use_videos=True)
    kept, seed, t0 = 0, args.seed, time.time()
    while kept < args.episodes:
        nz = float(np.random.default_rng(seed).uniform(0, args.noise))
        ok, log = SC.episode(env, seed, noise=nz)
        seed += 1
        if not ok:
            print(f"seed {seed - 1}: scripted demo failed, skipped")
            continue
        w = "r_wrist" if env.episode_side == "R" else "l_wrist"
        for state, action, ims in log:
            ds.add_frame({"observation.state": state.astype(np.float32), "action": action.astype(np.float32),
                          "observation.images.top": ims["top"], "observation.images.wrist": ims[w], "task": TASK})
        ds.save_episode()
        kept += 1
        print(f"episode {kept}/{args.episodes} (seed {seed - 1}, {len(log)} frames, {time.time() - t0:.0f} s)")
    ds.finalize()
    print(f"dataset: {root}  ({ds.num_episodes} episodes, {ds.num_frames} frames)")


if __name__ == "__main__":
    main()
