"""Run a trained LeRobot policy (ACT, SmolVLA, ...) in the sim and measure its success rate.

    .venv/bin/python -m sim.eval_policy outputs/act_sim/checkpoints/last/pretrained_model --episodes 50
    .venv/bin/python -m sim.eval_policy <ckpt> --episodes 3 --video eval.mp4

Uses seeds the demos never saw (default from 50000), the same cameras and image size as
sim/record.py, and the policy's saved pre/post-processors (normalisation).
"""
import argparse
from pathlib import Path

import numpy as np
import torch

from sim import env as E

MAX_STEPS = 300            # the scripted demos take 184 steps


def load(path, device, ensemble=None, n_action_steps=None):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.device = device
    if ensemble is not None:                      # ACT: average overlapping chunks every step
        cfg.temporal_ensemble_coeff, cfg.n_action_steps = ensemble, 1
    elif n_action_steps is not None:              # how many actions of each chunk run before re-planning
        cfg.n_action_steps = n_action_steps
    policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(path),
                                         preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post


def frame(obs, side, device):
    w = "r_wrist" if side == "R" else "l_wrist"
    out = {"observation.state": torch.from_numpy(obs["state"].astype(np.float32))[None].to(device)}
    for k, im in (("top", obs["images"]["top"]), ("wrist", obs["images"][w])):
        out[f"observation.images.{k}"] = (torch.from_numpy(im).to(device).float() / 255).permute(2, 0, 1)[None]
    out["task"] = [E.TASK]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=50000)
    ap.add_argument("--side", default="R", choices=["R", "L", "both"])
    ap.add_argument("--device", default="mps" if torch.backends.mps.is_available() else "cpu")
    ap.add_argument("--video", default=None)
    ap.add_argument("--n-action-steps", type=int, default=None, help="override the policy's n_action_steps")
    ap.add_argument("--ensemble", type=float, default=None,
                    help="ACT temporal ensembling coefficient (e.g. 0.01): predict every step, average chunks")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    args = ap.parse_args()
    policy, pre, post = load(Path(args.checkpoint), args.device, args.ensemble, args.n_action_steps)
    cams = ("top", "r_wrist", "l_wrist") + (("front",) if args.video else ())
    env = E.PickPlaceEnv(cameras=cams, side=args.side, width=args.width, height=args.height)
    frames, ok = [], 0
    for e in range(args.episodes):
        obs = env.reset(seed=args.seed + e)
        policy.reset()
        done = False
        for t in range(MAX_STEPS):
            with torch.inference_mode():
                a = post(policy.select_action(pre(frame(obs, env.episode_side, args.device))))
            obs = env.step(a.squeeze(0).float().cpu().numpy())
            if args.video:
                w = "r_wrist" if env.episode_side == "R" else "l_wrist"
                frames.append(np.concatenate([obs["images"]["front"], obs["images"]["top"], obs["images"][w]], axis=1))
            if env.success():
                done = True
                # let it finish releasing / retreating a little for the video
                if not args.video:
                    break
        ok += done
        print(f"episode {e:3d} seed {args.seed + e}: {'OK' if done else 'fail'}  ({t + 1} steps)")
    print(f"success {ok}/{args.episodes} = {100 * ok / args.episodes:.0f}%")
    if args.video:
        import imageio.v2 as iio
        iio.mimwrite(args.video, frames, fps=E.FPS, quality=7, macro_block_size=8)
        print("video:", args.video)


if __name__ == "__main__":
    main()
