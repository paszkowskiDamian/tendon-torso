"""Pick-and-place environment for the two-arm robot (MuJoCo).

    from sim.env import PickPlaceEnv
    env = PickPlaceEnv(cameras=("top", "r_wrist"))
    obs = env.reset(seed=0)
    obs = env.step(action)          # action: 16 joint targets (see below)
    env.success()

Action / state layout (16 numbers, the same for both):
    [r_S1 r_S2 r_S3 r_E r_F r_W1 r_W2 r_grip  l_S1 ... l_W2 l_grip]
joints in radians (the CAD convention: q_left = q_right is the mirror pose), grip
0 = closed .. 1 = fully open (62 mm between the pads).

One env step = 1/FPS s of simulation (the position servos track the targets).
Observation: {"state": (16,), "images": {camera: HxWx3 uint8}, "cube": (7,) pose, "bin": (3,)}.

Task: a 35 mm cube and a bin are placed at random in the right arm's part of the work
area; the right arm puts the cube into the bin (the left arm waits at its home pose).
`side="L"` mirrors the task to the left arm, `side="both"` picks a side per episode.
"""
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
TASK = "Put the red cube in the blue bin."   # language instruction stored with the demos
JOINTS = ["S1", "S2", "S3", "E", "F", "W1", "W2"]
FPS = 25
THETA_MAX = np.radians(41.0)                 # gripper.THETA_MAX
CUBE_HALF = 0.0175
# where things are spawned (torso frame, m), right side; the left side is the mirror (x -> -x)
CUBE_X, CUBE_Y = (-0.270, -0.130), (-0.370, -0.180)
BIN_X, BIN_Y = (-0.270, -0.110), (-0.400, -0.180)
MIN_SEPARATION = 0.15
# cube yaw (deg, right side; mirrored for the left). The cube looks the same every 90 deg,
# but the arm's top-down grasps only cover yaw -90..0, so somewhere in any 90 deg band the
# grasp has to jump to a very different arm posture. That jump is kept out of the data
# (a policy can't learn it from images): the yaw stays 10 deg clear of the wrap.
CUBE_YAW = (-80.0, -10.0)
BIN_INNER = 0.057                            # half inner size of the bin
# home: hand above the work area, close to the body (deg, from arm.kinematics.ik)
HOME_DEG = np.array([73.1, -10.0, -16.3, 103.2, -83.6, -18.0, -27.7])


class PickPlaceEnv:
    def __init__(self, cameras=("top", "r_wrist"), width=320, height=240, side="R", render=True):
        self.model = mujoco.MjModel.from_xml_path(str(HERE / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.cameras, self.side = tuple(cameras), side
        self.width, self.height = width, height
        self.renderer = mujoco.Renderer(self.model, height, width) if render and cameras else None
        m = self.model
        self.qadr = np.array([m.joint(f"{s}_{j}").qposadr[0] for s in "rl" for j in JOINTS + ["finger_a"]])
        self.act = np.array([m.actuator(f"{s}_{j}").id for s in "rl" for j in JOINTS + ["gripper"]])
        self.fingers_b = [m.joint(f"{s}_finger_b").qposadr[0] for s in "rl"]
        self.cube_q = m.joint("cube").qposadr[0]
        self.bin_mocap = m.body_mocapid[m.body("bin").id]
        self.substeps = int(round(1.0 / FPS / m.opt.timestep))
        self.rng = np.random.default_rng()
        self.episode_side = "R"

    # ------------------------------------------------------------------ layout helpers
    def home(self):
        h = np.zeros(16)
        h[:7] = h[8:15] = np.radians(HOME_DEG)
        h[7] = h[15] = 1.0
        return h

    def _to_ctrl(self, a):
        c = np.asarray(a, float).copy()
        c[[7, 15]] = np.clip(c[[7, 15]], 0, 1) * THETA_MAX
        return c

    # ------------------------------------------------------------------ api
    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        side = self.side if self.side in ("R", "L") else ("R" if self.rng.random() < 0.5 else "L")
        self.episode_side = side
        sx = 1.0 if side == "R" else -1.0
        while True:
            c = np.array([self.rng.uniform(*CUBE_X), self.rng.uniform(*CUBE_Y)])
            b = np.array([self.rng.uniform(*BIN_X), self.rng.uniform(*BIN_Y)])
            if np.linalg.norm(c - b) >= MIN_SEPARATION:
                break
        yaw = sx * np.radians(self.rng.uniform(*CUBE_YAW))
        d.qpos[self.cube_q:self.cube_q + 3] = (sx * c[0], c[1], CUBE_HALF + 0.0005)
        d.qpos[self.cube_q + 3:self.cube_q + 7] = (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2))
        d.mocap_pos[self.bin_mocap] = (sx * b[0], b[1], 0.0)
        h = self._to_ctrl(self.home())
        d.qpos[self.qadr] = h
        d.qpos[self.fingers_b] = h[[7, 15]]
        d.ctrl[self.act] = h
        mujoco.mj_forward(m, d)
        for _ in range(10):                    # let the cube settle
            mujoco.mj_step(m, d)
        return self.observe()

    def step(self, action):
        self.data.ctrl[self.act] = self._to_ctrl(action)
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        return self.observe()

    def state(self):
        q = self.data.qpos[self.qadr].copy()
        q[[7, 15]] = q[[7, 15]] / THETA_MAX
        return q

    def render(self, camera):
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render().copy()

    def observe(self):
        d = self.data
        obs = {"state": self.state(),
               "cube": d.qpos[self.cube_q:self.cube_q + 7].copy(),
               "bin": d.mocap_pos[self.bin_mocap].copy()}
        if self.renderer is not None:
            obs["images"] = {c: self.render(c) for c in self.cameras}
        return obs

    def success(self):
        """Cube resting inside the bin (and no longer held up by the gripper)."""
        c = self.data.qpos[self.cube_q:self.cube_q + 3]
        b = self.data.mocap_pos[self.bin_mocap]
        inside = np.all(np.abs(c[:2] - b[:2]) < BIN_INNER - CUBE_HALF * 0.5)
        return bool(inside and c[2] < 0.006 + CUBE_HALF + 0.01)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
