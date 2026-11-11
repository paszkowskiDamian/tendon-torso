"""Scripted pick-and-place with the arm's own IK (arm/kinematics.py) - the demo generator.

    .venv/bin/python -m sim.scripted --episodes 50            # success rate
    .venv/bin/python -m sim.scripted --episodes 1 --video out.mp4

The hand comes straight down onto the cube (gripper axis vertical, fingers across two
faces of the cube), lifts it, carries it over the bin and lets go. Every control step
solves the IK for the next point on a smooth Cartesian path, seeded with the previous
joint angles, so the arm never jumps to another configuration mid-move.

The left arm uses the right arm's IK on the mirrored target (q_left = q_right is the
mirror pose).
"""
import argparse
import math

import numpy as np

from arm import assembly as A
from arm import kinematics as K
from sim import env as E

SF_INV = np.linalg.inv(A.shoulder_frame_torso("R"))      # TORSO (mm) -> right SHOULDER frame
GRASP_Z = 0.019             # tool point (pad centres) above the table when grasping a 35 mm cube
CARRY_Z = 0.095             # hand height while approaching / carrying (top-down reach ends ~0.10)
RELEASE_Z = 0.080
OPEN = 0.85                 # gripper opening while approaching (0..1 of 62 mm)


def R_down(yaw):
    """Tool rotation (TORSO frame): hand axis down, fingers opening along yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.c_[x, np.cross(z, x), z]


def min_jerk(t):
    t = min(max(t, 0.0), 1.0)
    return t ** 3 * (10 - 15 * t + 6 * t ** 2)


class Planner:
    """Joint targets for one arm along Cartesian waypoints (positions in the TORSO frame, m)."""

    def __init__(self, side, q_now_rad):
        self.side = side
        self.q = np.degrees(q_now_rad)

    def _local(self, p, yaw):
        """Mirror a left-arm target into the right arm's frame; to the shoulder frame in mm."""
        p = np.array(p, float) * 1000
        if self.side == "L":
            p[0], yaw = -p[0], -yaw
        ps = (SF_INV @ np.r_[p, 1.0])[:3]
        Rs = SF_INV[:3, :3] @ R_down(yaw)
        return ps, Rs

    def solve(self, p, yaw, q0=None, restarts=0):
        ps, Rs = self._local(p, yaw)
        r = K.ik(ps, Rs, q0=self.q if q0 is None else q0, restarts=restarts, max_iter=60)
        return r

    def pick_yaw(self, cube_yaw, cube_p, bin_p):
        """Grasp yaw (cube yaw + k 90 deg) reachable above the cube, at the grasp and over the bin.

        Demos for a learned policy must be consistent in JOINT space: the same scene should
        always give the same arm configuration. So every candidate is solved by continuation
        from the current (home) posture without random restarts, and the one closest to home
        wins; random restarts (which can land in the other forearm-roll branch) are only a
        fallback. If no aligned grasp is reachable that way, the hand may come in up to
        20 deg off the cube's faces (the closing fingers square the cube up: a 35 mm cube
        at 20 deg is 45 mm across, inside the 53 mm approach opening).
        Returns (yaw, joint angles above the cube) or None."""
        for restarts, off in ((0, 0), (0, 10), (0, -10), (0, 20), (0, -20), (4, 0)):
            best = None
            for k in range(4):
                yaw = cube_yaw + k * math.pi / 2 + math.radians(off)
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                q, ok, cost, q_above = self.q, True, 0.0, None
                for p in ((cube_p[0], cube_p[1], CARRY_Z), (cube_p[0], cube_p[1], GRASP_Z),
                          (bin_p[0], bin_p[1], CARRY_Z)):
                    r = self.solve(p, yaw, q0=q, restarts=restarts)
                    if not r.ok or np.abs(r.q - q).max() > (60 if p[2] == GRASP_Z else 120):
                        ok = False
                        break
                    q_above = r.q if q_above is None else q_above
                    cost += float(np.abs(r.q - self.q).sum())
                    q = r.q
                if ok and (best is None or cost < best[2]):
                    best = (yaw, q_above, cost)
            if best is not None:
                return best[0], best[1]
        return None

    def joint_path(self, q_goal, steps):
        """Joint-space min-jerk move to q_goal (deg): the free move from home, where the hand
        has to turn a lot and a straight Cartesian path may leave the reach."""
        q0 = self.q.copy()
        out = [np.radians(q0 + min_jerk(i / steps) * (q_goal - q0)) for i in range(1, steps + 1)]
        self.q = np.asarray(q_goal, float)
        return out

    def path(self, p0, p1, yaw0, yaw1, steps):
        """Joint targets (rad) along a min-jerk straight line p0 -> p1."""
        out = []
        for i in range(1, steps + 1):
            s = min_jerk(i / steps)
            p = (1 - s) * np.asarray(p0) + s * np.asarray(p1)
            yaw = yaw0 + s * (yaw1 - yaw0)
            r = self.solve(p, yaw)
            if not r.ok:
                r2 = self.solve(p, yaw, restarts=3)
                if r2.ok and np.abs(r2.q - self.q).max() < 25:
                    r = r2
            if np.abs(r.q - self.q).max() < 30:          # never jump
                self.q = r.q
            out.append(np.radians(self.q))
        return out


def cube_yaw(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# DART-style noise: the arm executes the expert's target plus a slowly drifting offset, but
# the recorded action is the clean target. The demos then show how to get back onto the
# path from states a policy will actually land in (deg, 1 sigma per joint; OU time constant).
NOISE_DEG = np.array([1.5, 1.5, 2.0, 1.5, 3.0, 3.0, 3.0])
NOISE_TAU = 0.6


def episode(env, seed, noise=0.0):
    """Run one scripted episode. Returns (success, list of (state, action, images)).
    noise > 0 scales NOISE_DEG (executed actions are perturbed, recorded ones are not)."""
    obs = env.reset(seed=seed)
    nrng = np.random.default_rng(seed + 7_000_000)
    off = np.zeros(7)
    a_ou = math.exp(-1.0 / (E.FPS * NOISE_TAU))
    side = env.episode_side
    k = 0 if side == "R" else 8
    action = env.home()
    cube_p, cyaw = obs["cube"][:3].copy(), cube_yaw(obs["cube"][3:])
    bin_p = obs["bin"].copy()
    pl = Planner(side, action[k:k + 7])
    pick = pl.pick_yaw(cyaw, cube_p, bin_p)
    yaw, q_above = pick if pick else (None, None)               # TORSO-frame targets; the planner mirrors for L
    log = []

    def run(targets, grip, noisy=False):
        """noisy: perturb this free move (full strength until 60 % of it, gone at 90 %)."""
        nonlocal obs
        for i, qt in enumerate(targets):
            a = action.copy()
            a[k:k + 7] = qt
            a[k + 7] = grip(i / max(1, len(targets) - 1)) if callable(grip) else grip
            log.append((obs["state"], a, obs.get("images")))
            ex = a.copy()
            if noise > 0 and noisy:
                off[:] = a_ou * off + math.sqrt(1 - a_ou ** 2) * nrng.normal(0, 1, 7) * np.radians(NOISE_DEG) * noise
                u = i / max(1, len(targets) - 1)
                ex[k:k + 7] += off * min(1.0, max(0.0, (0.9 - u) / 0.3))
            obs = env.step(ex)
            action[:] = a

    if yaw is None:
        return False, log
    hold = lambda n: [np.radians(pl.q)] * n                            # noqa: E731
    above = (cube_p[0], cube_p[1], CARRY_Z)
    grasp = (cube_p[0], cube_p[1], GRASP_Z)
    over = (bin_p[0], bin_p[1], CARRY_Z)
    drop = (bin_p[0], bin_p[1], RELEASE_Z)
    n = lambda s: int(round(s * E.FPS))                    # noqa: E731
    run(pl.joint_path(q_above, n(1.6)), lambda t: 1.0 - (1.0 - OPEN) * t, noisy=True)
    run(pl.path(above, grasp, yaw, yaw, n(1.0)), OPEN)
    run(hold(n(0.6)), lambda t: OPEN * (1 - min(1.0, 1.6 * t)))
    run(pl.path(grasp, above, yaw, yaw, n(0.8)), 0.0)
    run(pl.path(above, over, yaw, yaw, n(1.4)), 0.0, noisy=True)
    run(pl.path(over, drop, yaw, yaw, n(0.5)), 0.0)
    run(hold(n(0.5)), lambda t: OPEN * min(1.0, 2 * t))
    run(pl.path(drop, over, yaw, yaw, n(0.6)), OPEN)
    run(hold(n(0.4)), OPEN)
    return env.success(), log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--side", default="R", choices=["R", "L", "both"])
    ap.add_argument("--video", default=None, help="write an mp4 of the episodes (front camera)")
    args = ap.parse_args()
    cams = ("front", "top", "r_wrist", "l_wrist") if args.video else ()
    env = E.PickPlaceEnv(cameras=cams, side=args.side, width=480, height=360)
    frames = []
    ok = 0
    for e in range(args.episodes):
        s, log = episode(env, args.seed + e)
        ok += s
        print(f"episode {e:3d} seed {args.seed + e:4d} side {env.episode_side}: {'OK' if s else 'fail'}  ({len(log)} steps)")
        if args.video:
            w = "r_wrist" if env.episode_side == "R" else "l_wrist"
            for _, _, im in log:
                top = np.concatenate([im["front"], im["top"]], axis=1)
                frames.append(np.concatenate([top, np.concatenate([im[w], np.zeros_like(im[w])], axis=1)], axis=0))
    print(f"success {ok}/{args.episodes} = {100 * ok / args.episodes:.0f}%")
    if args.video:
        import imageio.v2 as iio
        iio.mimwrite(args.video, frames, fps=E.FPS, quality=7, macro_block_size=8)
        print("video:", args.video)


if __name__ == "__main__":
    main()
