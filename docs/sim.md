# Simulation: teaching the arms without building a second arm

A MuJoCo model of the whole robot (torso + both arms + gripper), a pick-and-place
scene, a scripted demonstrator that uses the arm's own IK, a recorder that writes
LeRobot datasets, and an evaluator that runs a trained policy back in the sim.

```
sim/build_model.py   CAD -> sim/robot.xml + sim/assets/*.stl        (.venv, needs CadQuery)
sim/scene.xml        table, cube, bin, overhead + front cameras (includes robot.xml)
sim/env.py           PickPlaceEnv: reset / step(16 joint targets) / images / success
sim/scripted.py      scripted top-down pick-and-place (arm/kinematics.py IK) = demo generator
sim/record.py        scripted demos -> LeRobot dataset (data/, not in git)
sim/eval_policy.py   trained LeRobot policy -> success rate / video
```

## The model

`python -m sim.build_model` (about 2 minutes) rebuilds the model from the same numbers
that place the printed parts, so it can't drift from the CAD:

* **Chain:** `arm/assembly.py` CHAIN (mount, in_z, out_z per element), joint limits from
  the joint specs. MuJoCo's tool site matches `arm.kinematics.fk` to 1e-12 mm for random
  joint angles on both arms.
* **Left arm:** the mirror image of the right one, built from mirrored frames and
  meshes. `q_left = q_right` is the mirror pose (the same convention as the CAD and the
  viewer).
* **Masses:** from `arm/statics.py` (printed parts at the print-profile infill plus
  hardware), 1.62 kg per arm.
* **Joints:** each joint is a position servo. Its torque limit is the drive servo's stall
  torque times the tendon ratio. The gripper's two fingers are geared together
  (equality constraint), and pad force is limited to about 12 N.
* **Collisions:** the gripper pads, palm and arm hulls collide with the table, cube and
  bin. The two arms don't collide with each other in the sim (check_arm does that).
* **Not modelled:** the tendons, their friction and stretch, and the balance springs.
  Gravity on the arm is compensated, standing in for the servo's integral term plus
  the springs. A policy trained here still has to be fine-tuned on the real arm.

World frame = TORSO frame in metres: table top z = 0, -Y forwards, +X the robot's left,
shoulders at x = ±0.175, z = 0.65.

## The task

* **Objects:** a 35 mm cube (50 g) and an open bin (110 × 110 inner, 30 mm walls).
* **Placement:** both are placed at random in the right arm's part of the work area, at
  least 150 mm apart. The cube yaw is random between −80° and −10° (mirrored for the
  left arm); the next section explains why it isn't the full 90°.
* **Variants:** `side="L"` mirrors the task to the left arm; `side="both"` picks a side
  per episode.
* **Success:** the cube rests inside the bin.

**Action / state (16 numbers):** `[r_S1 r_S2 r_S3 r_E r_F r_W1 r_W2 r_grip, l_S1 … l_grip]`

* joints in radians
* grip from 0 (closed) to 1 (62 mm between the pads)
* control at 25 Hz

**Cameras:** `top` (overhead), `front`, and `r_wrist` / `l_wrist` (on the palm rim,
looking between the fingers).

**Top-down grasp limit:** with the hand pointing straight down, the arm reaches only up
to about 100 mm above the table (the elbow limit is 115°). So the hand approaches and
carries at 95 mm, which still clears the 30 mm bin walls.

## Scripted demonstrations

```bash
.venv/bin/python -m sim.scripted --episodes 50 --side both
.venv/bin/python -m sim.scripted --episodes 2 --video demo.mp4
```

The scripted episode runs these steps:

1. Joint-space move from home to above the cube. The grasp yaw is the cube's yaw plus
   k·90°, chosen as reachable over the cube, at the grasp height and over the bin.
2. Straight-line descent, close, lift, carry, lower, release, retreat. Each step is an IK
   solve seeded with the previous joint angles, so the arm never switches configuration
   mid-move.

**Demos have to be consistent in joint space.** A 7-DOF arm can reach the same grasp
with very different joint angles; here the forearm roll F has a branch near −60° and
one near +50°. The first dataset let the IK pick freely with random restarts, and 11 %
of the demos used the other branch. The ACT policy trained on it followed the demo for
about 1 s, then drifted between branches and never reached the cube (0/20 at 10k
steps).

The demonstrator now solves every candidate grasp by continuation from the home pose,
without random restarts, and keeps the one closest to home. If no face-aligned grasp is
reachable that way, it comes in up to 20° off the cube's faces, and closing squares the
cube up. With that, 1 % of the demos use the other branch, against 11 % before, and
every scene has a solution.

**The grasp choice must not jump either.** The cube looks the same every 90°, but the
arm's top-down grasps only cover yaw −90…0°. Somewhere in any 90° band of cube yaw, the
grasp therefore has to wrap to a posture 90° away. For example, a cube at −3° was grasped
at −3° (S2 −8°) and one at +1° at −89° (S2 +35°). Two nearly identical images then ask
for completely different arm motions. The policy blended them, drifted 20–35° off on S2
and S3 after about 1 s, and never succeeded (0/20 at 20k steps, even on training seeds).
The cube yaw is now drawn from −80…−10°, which keeps the wrap 10° clear of the data.

**Recovery data (DART).** A policy never hits the demo path exactly, and clean demos
don't show how to get back onto it. With `--noise`, the free moves (home → above the cube,
and the carry to the bin) are executed with a slowly drifting joint offset of up to a few
degrees, faded out before the descent. The recorded action stays the expert's clean
target, so the data teaches the way back. Each episode draws its noise scale between 0
and the maximum. The demos still succeed 40/40 at twice the base noise.

**Result:** 100 % on 50 right-arm episodes and 100 % on 50 episodes with both sides.

## Record a dataset

```bash
.venv/bin/python -m sim.record --episodes 200 --workers 8 --noise 2.0   # -> data/sim_pick_place
```

The output is LeRobot dataset v3 (lerobot 0.7):

* `observation.state` (16) and `action` (16)
* `observation.images.top` and `observation.images.wrist`, 320 × 240 AV1 video
* the task string "Put the red cube in the blue bin."

## Train and evaluate (ACT)

LeRobot's command-line parser (draccus) crashes on Python 3.14. Training therefore runs
in a separate Python 3.12 environment. MPS (Apple GPU) also needs torch 2.9 on macOS 14:
torch 2.15 aborts with an MPSNDArray error.

```bash
uv venv .venv-train --python 3.12
uv pip install --python .venv-train/bin/python 'lerobot[dataset,training]==0.7.0' mujoco imageio imageio-ffmpeg \
    torch==2.9.1 torchvision==0.24.1 'torchcodec==0.8.*'
```

```bash
.venv-train/bin/lerobot-train --dataset.repo_id=local/tendon_torso_sim_pick_place \
    --dataset.root=data/sim_pick_place --policy.type=act --policy.device=mps \
    --policy.push_to_hub=false --policy.chunk_size=50 --policy.n_action_steps=50 \
    --batch_size=8 --steps=30000 --save_freq=10000 --num_workers=4 \
    --output_dir=outputs/act_sim --job_name=act_sim
```

```bash
.venv-train/bin/python -m sim.eval_policy outputs/act_sim/checkpoints/last/pretrained_model \
    --episodes 50 --n-action-steps 50
```

The evaluator uses seeds from 50000 on, so none of them appear in the demos. Add
`--video eval.mp4` to record a video. `--n-action-steps` overrides how many actions of
each chunk run before the policy is asked again; `--ensemble 0.01` turns on ACT's
temporal ensembling instead.

On an M3 Max, ACT takes about 0.18 s per step at batch 8, so 30k steps is roughly 90 minutes.

### Results (200 noise-injected demos, ACT, 50 unseen seeds)

| checkpoint | re-plan every 10 steps | re-plan every 25 | every 50 (whole chunk) |
|---|---|---|---|
| 10k | 0/20 | 3/10 | 34/50 = 68 % |
| 20k | | | 38/50 = 76 % |
| 30k | | | 42/50 = 84 % |

Re-planning in the middle of the approach makes the policy hover above the cube. At
those points the state looks the same whether the approach has just started or is nearly
done, so the new chunk keeps saying "approach". Running each 2 s chunk to the end
avoids that. A policy that re-plans often needs a phase cue in its input: a time step,
or a short history of frames (ACT's `n_obs_steps`).

## Next

* Bimanual tasks (handover between the arms, two cubes at once) use the same env with
  `side="both"`.
* Domain randomisation (lighting, colours, camera jitter, joint friction) before any
  sim-to-real attempt.
* Retarget human videos / handheld-gripper (UMI-style) demos into the same action space.
