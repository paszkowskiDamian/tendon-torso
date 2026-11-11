# AI control: teaching the tendon arm by demonstration

Snapshot from **2026-11-11**. The question: how do we connect this 7-DOF, Bowden-tendon arm (later two of them on a torso) to the current open robot-learning stack, so that we can show it a task a few dozen times and have it repeat the task on its own?

Robot facts used below come from [arm/README.md](../arm/README.md), [docs/kinematics.md](kinematics.md), [docs/loads_report.txt](loads_report.txt) and [arm/specs.py](../arm/specs.py). Everything else is linked to its source. Numbers marked *(est.)* are our own estimates, not from a source.

## Summary

* **Software: use [LeRobot](https://github.com/huggingface/lerobot)** (v0.7.0, released 2026-11-05, [PyPI](https://pypi.org/project/lerobot/)). It records demonstrations, trains ACT, Diffusion, SmolVLA, π0/π0.5, GR00T N1.7, X-VLA and about a dozen newer policies, and deploys them, all behind one `Robot` interface. A custom robot is a pip-installable plugin package named `lerobot_robot_<name>`. No fork of LeRobot is needed ([Bring Your Own Hardware](https://huggingface.co/docs/lerobot/integrate_hardware)).
* **Action space: absolute joint positions in degrees**, 7 joints + gripper = 8 numbers per arm, 16 for two arms. Every policy above accepts this. End-effector actions are optional and only needed for phone/VR teleop.
* **Joint angles must come from encoders at the joints, not from the motors.** Cord stretch, Bowden friction and slack put the motor angle several degrees, maybe more than 10°, away from the joint angle *(est.)*. Put a 14-bit magnetic encoder (AS5048A or MT6701) at each of the 8 axes. A **microcontroller in the torso** (Teensy 4.1 class) runs a 500–1000 Hz joint loop: encoder → motor. LeRobot only sends joint targets at 30 Hz.
* **Motors (phase 1, cheapest and closest to LeRobot):** Feetech serial servos with the 8 mm spool on the horn. STS3095 at S1/S2 (keep the balance springs), STS3250 at S3/E, STS3215 (12 V) at F/W1/W2/gripper. Roughly **$550 per arm** *(est., some prices unverified)*. **Upgrade path:** Damiao DM-J4310-2EC at S1/S2 (and S3/E if needed). It is a geared, backdrivable QDD motor with torque control, 3 N·m rated / 7 N·m peak, about $125–185, and LeRobot supports it natively.
* **Teleop: a passive 7-DOF leader arm**, a scaled copy of our kinematics in the style of GELLO or the SO-101 leader. LeRobot's leader/follower teleop is joint-to-joint, which is the only mode that fixes all 7 DOF (VR or phone gives a 6-DOF pose, and the elbow swivel is then left to IK). Two leaders for the bimanual phase.
* **Phased plan:** (1) one arm + gripper + leader → 50 pick-and-place demos → **ACT** (trains in hours on one GPU, runs on a laptop). (2) Same data plus more tasks with language → fine-tune **SmolVLA** (450M, about 4 h on an A100), then **π0.5** (LoRA needs a GPU with more than 22.5 GB, so a rented or 4090/5090-class card). (3) Two arms, 16-D actions, ACT/π0.5, optionally X-VLA or GR00T N1.7.

---

## 1. The landscape (as of November 2026)

### 1.1 LeRobot is the hub

* **Releases:** 0.4.2 (2025-11-27) → 0.4.4 (2026-02-27: Damiao motors + CAN, **OpenArm 7-DOF single/bimanual**) → 0.5.x (Python ≥ 3.12, relative actions for π0/π0.5, HIL/DAgger collection) → 0.6.0 (2026-07-06: GR00T N1.5 replaced by **N1.7**, new `lerobot-rollout` deploy CLI) → **0.7.0 (2026-11-05)**. In 0.7.0, Feetech and Dynamixel share one rustypot-backed `SerialMotorsBus`, async inference was replaced by `lerobot-policy-server`, LoRA comes via `--policy.use_peft`, and Linux wheels are CUDA 13 (driver ≥ 580.65). macOS needs version 14+ (15+ for MPS). Sources: [PyPI history](https://pypi.org/project/lerobot/), [releases](https://github.com/huggingface/lerobot/releases), [v0.7.0 notes](https://github.com/huggingface/lerobot/releases/tag/v0.7.0), [v0.6.0 blog](https://huggingface.co/blog/lerobot-release-v060).
* **Policies listed in the README:** imitation learning (ACT, Diffusion, VQ-BeT, Multitask DiT); RL (HIL-SERL, TDMPC); VLAs (π0, π0-FAST, π0.5, GR00T N1.7, SmolVLA, X-VLA, EO-1, MolmoAct2, LingBot-VLA 2.0, WALL-OSS, EVO1, OpenGalaxea G0.5, FineART-VLA, DM05); world-model policies (VLA-JEPA, FastWAM, LingBot-VA and others) ([README](https://github.com/huggingface/lerobot)).
* **Deployment:** `lerobot-rollout` with strategies `base`, `sentry`, `highlight`, `episodic` and `dagger` (human corrections tagged `intervention`). It also offers Real-Time Chunking (RTC) for VLAs, a remote policy server, and an interactive `/subtask` command to re-instruct language policies mid-run ([Policy Deployment](https://huggingface.co/docs/lerobot/inference)).

### 1.2 Policy comparison

| Policy | Size | Demos to start | Cameras | Action space | Train / fine-tune compute | Run compute, rate | Licence | Bimanual |
|---|---|---|---|---|---|---|---|---|
| **ACT** ([docs](https://huggingface.co/docs/lerobot/act), [paper](https://arxiv.org/abs/2304.13705)) | ~80M | "often … just 50" | 1–4 RGB | any vector, chunk of `k` actions (joint pos. typical) | "a few hours for 100k steps on a single GPU"; batch 8 | Laptop GPU/CPU class *(est.)*, 30–50 Hz | Apache-2.0 (LeRobot) | Yes, designed on ALOHA (14-D) |
| **Diffusion Policy** ([paper](https://arxiv.org/abs/2303.04137), in LeRobot) | ~250M *(from memory, not checked)* | 50–200 *(est.)* | 1–2 | any vector | more steps than ACT, single GPU | slower per chunk (denoising) | Apache-2.0 / MIT | Possible |
| **SmolVLA** ([docs](https://huggingface.co/docs/lerobot/smolvla), [paper](https://huggingface.co/papers/2506.01844)) | 450M (`lerobot/smolvla_base`) | **~50 episodes**; "25 … was not enough"; 10 demos per variation | up to 3 (`camera1..3` via `--rename_map`) | joint/state vectors, padded | 20k steps ≈ **4 h on one A100**; Colab notebook | consumer GPU; RTC supported | Apache-2.0 | Possible (pad) |
| **π0 / π0-FAST / π0.5** ([openpi](https://github.com/Physical-Intelligence/openpi), [LeRobot π0.5](https://huggingface.co/docs/lerobot/pi05)) | ~3B (PaliGemma base) | 10k+ h pre-training; fine-tune on tens–hundreds of episodes *(est.)* | up to 3 (224×224) | state & action **padded to 32**, loss on real dims; absolute or relative actions | openpi: **LoRA > 22.5 GB (RTX 4090)**, full > 70 GB (A100/H100 80 GB); LeRobot example "sized for a single 80 GB GPU", or `train_expert_only` to save memory | inference **> 8 GB** (RTX 4090); websocket remote server; RTC | Apache-2.0 + Gemma terms (gated PaliGemma tokenizer) | Yes (ALOHA checkpoints) |
| **GR00T N1.7** ([Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T), [LeRobot](https://huggingface.co/docs/lerobot/groot)) | 3B (`nvidia/GR00T-N1.7-3B`, Cosmos-Reason2/Qwen3-VL) | not stated; "a few thousand steps" | 1–n | `new_embodiment` tag + `modality.json`; **relative EE recommended**, LeRobot example uses relative joint actions, chunk 16 | **≥ 40 GB GPU** (H100/L40; A6000 works) | **≥ 16 GB**; ~40 Hz RTX PRO 6000, ~28 Hz Jetson AGX Thor (TensorRT) | code Apache-2.0; weights NVIDIA Open Model License, "fully commercially usable" | trained on bimanual & humanoid data |
| **X-VLA** ([LeRobot](https://huggingface.co/docs/lerobot/xvla), [paper](https://arxiv.org/abs/2510.10274)) | 0.9B (`lerobot/xvla-base`) | "minimal demonstrations" | `num_image_views`, `empty_cameras` | `action_mode=auto` pads to 20-D; joint or EE6D; chunk 32 | bf16, full fine-tune recommended (GPU size not stated) | not stated | not stated in the docs (check model card) | **Yes**, documented bimanual SO-101 recipe |
| **OpenVLA-OFT** ([repo](https://github.com/moojink/openvla-oft), [paper](https://arxiv.org/abs/2502.19645)) | 7B | paper's ALOHA tasks | 1–3 | continuous chunks (L1 head), K=25 | 1–8 GPUs × 27–80 GB | ~16–18 GB; **77.9 Hz throughput on an A100** (ALOHA), executed at 25 Hz | code MIT (base OpenVLA on Llama 2: check its licence) | Yes (ALOHA) |
| Others in LeRobot 0.6/0.7 ([blog](https://huggingface.co/blog/lerobot-release-v060)) | MolmoAct2 (~12 GB bf16 inference, LoRA fits 24 GB), EO-1 (Qwen2.5-VL-3B), EVO1 (0.77B), Multitask DiT (~450M), VLA-JEPA (Qwen3-VL-2B), LingBot-VLA (4B, [repo](https://github.com/robbyant/lingbot-vla)) | | | | | | varies | |

**Closed or hosted:** Physical Intelligence's newer models (π\*0.6 in Nov 2025, π0.7 in Apr 2026) are not open-weight. On **2026-10-13** PI opened a **Pi API beta** for hosted fine-tuning and cloud inference on your own robot ([PI blog](https://www.pi.website/blog)). The only open PI weights are π0, π0-FAST and π0.5 via openpi. Community "π0.6" re-implementations exist, for example [OpenPIE-0.6](https://huggingface.co/exla-ai/openpie-0.6), but they are not from PI.

**What this means for us:**
* **ACT first.** It is the documented starting point, data-efficient (50 demos), cheap to train and indifferent to embodiment. It knows nothing about language or the world, so it learns exactly one task per model.
* **SmolVLA second.** It is the smallest VLA, built for LeRobot datasets and for adding a language instruction. It is the lowest-cost way to test whether one model can do several tasks on our arm.
* **π0.5 when we want the best generalisation.** It needs a 24 GB+ card for LoRA or a rented 80 GB GPU. GR00T N1.7 is comparable and has a commercial-friendly licence, but needs ≥ 40 GB to fine-tune. X-VLA has the clearest documented recipe for a new bimanual embodiment.
* **Our robot is not in any pre-training set.** Every VLA therefore needs fine-tuning on our own demos. The pre-training mostly helps with vision, language and robustness, not with our tendon dynamics.

---

## 2. Integration path for a custom robot in LeRobot

### 2.1 Interfaces we implement

From [Bring Your Own Hardware](https://huggingface.co/docs/lerobot/integrate_hardware):

* **`RobotConfig` subclass**, registered with `@RobotConfig.register_subclass("tendon_arm")`: port, cameras (`OpenCVCameraConfig`), limits.
* **`Robot` subclass** with:
  * `observation_features` / `action_features`: dicts such as `{"S1.pos": float, …, "gripper.pos": float, "wrist": (480, 640, 3)}`. These must be callable before `connect()`.
  * `connect(calibrate=True)`, `disconnect()`, `is_connected`.
  * `is_calibrated`, `calibrate()`, `configure()`. Calibration data is saved per robot `id` and reloaded via `_keep_calibration_file()`.
  * `get_observation()` and `send_action(action) -> dict`. The second must **return what was actually applied**, after clipping.
  * Optional `supports_position_hold = True`: lets rollout pause safely, for example during `/subtask` or slow inference. Only set it if re-sending the last position target is safe.
* **Motor bus:** `SerialMotorsBus` drives Feetech STS/SMS (`sts3215`, `sts3250`, `sm8512bl`), SCS (`scs0009`) and Dynamixel X (XL330, XL430, XM430, XM540, XH540 …). It provides `sync_read`, `sync_write`, `record_calibration`, `write_calibration` and `set_operating_mode`. Other Feetech models (for example **STS3095**) need an entry in [rustypot](https://github.com/pollen-robotics/rustypot) first. `DamiaoMotorsBus` handles Damiao over Linux SocketCAN/CAN-FD ([Damiao guide](https://huggingface.co/docs/lerobot/damiao)).
* **Teleoperator:** a `Teleoperator` subclass with `get_action()` and optional `send_feedback()`.
* **Packaging:** packages named `lerobot_robot_*`, `lerobot_teleoperator_*`, `lerobot_camera_*` or `lerobot_strategy_*` are discovered automatically. Each needs a `FooConfig`/`Foo` class pair exported from `__init__.py`. Then `lerobot-record --robot.type=tendon_arm --teleop.type=tendon_leader …` works unchanged. Example plugins: [lerobot-robot-xarm](https://github.com/SpesRobotics/lerobot-robot-xarm), [lerobot-teleoperator-teleop](https://github.com/SpesRobotics/lerobot-teleoperator-teleop).

**Our design: a thin plugin and a smart microcontroller.**

```
 PC (LeRobot)                                     torso MCU (Teensy 4.1 class)               arm
 ────────────────────────────                     ───────────────────────────────            ────────────
 lerobot_robot_tendon_arm                          1 kHz loop:                                8 joint encoders
   get_observation():  ◄── USB serial 200 Hz ──    read 8 joint encoders (SPI/SSI) ◄───────── (AS5048A / MT6701)
     S1..W2.pos, gripper.pos (deg, joint enc.)     q_err = q_target − q_joint
     (+ motor.pos, motor.load as extra state)      motor_cmd = coupling⁻¹ · (PID + feedforward)
   send_action(): joint targets (deg) ──► 30 Hz    clamp to specs.py ranges, rate & torque limits ──► 8 motors (Feetech TTL
   cameras: wrist + scene (OpenCV/UVC)             watchdog: no target for 200 ms → hold / limp      bus and/or Damiao CAN)
```

* The plugin's `calibrate()` saves the zero offset and direction of each joint encoder: the hanging pose q = 0 from [kinematics.md](kinematics.md), plus the range limits from `arm/specs.py`. Units are degrees (`MotorNormMode.DEGREES` semantics), so a dataset matches the IK and viewer conventions.
* The **tendon coupling matrix** (roadmap: `l = R·θ`) is applied **in the firmware**. The policy and the dataset never see it, so the same dataset stays valid after a re-routing.
* A USB-CDC microcontroller works on macOS and Linux. The Damiao CAN adapter path is **Linux-only** ([OpenArm docs](https://huggingface.co/docs/lerobot/openarm)), but because our firmware talks to the motors itself, the PC side stays OS-independent.

### 2.2 Dataset format

`LeRobotDataset v3.0` ([docs](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)):
* `meta/info.json` holds the schema, fps and path templates. `meta/stats.json` holds normalisation stats (π0.5 needs q01/q99, recomputed with `lerobot-edit-dataset … recompute_stats`). `meta/tasks.parquet` holds the language task strings. `meta/episodes/` holds the episode offsets.
* `data/` holds Parquet shards (`observation.state`, `action`, timestamps). `videos/` holds MP4 shards, one per camera.
* Record with `lerobot-record … --dataset.single_task="Put the cube in the bowl"`, push to the Hub, and inspect in the [online visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset).
* GR00T's own repo still uses a v2-style layout plus `meta/modality.json`, with a v3→v2 conversion script ([Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T)). Inside LeRobot (`--policy.type=groot`) the v3 data is used directly.

**Our feature layout (one arm):**
* `observation.state`: 8 joint angles from the **joint encoders**, in degrees.
* Optional extras: `motor.pos` (spool angle) and `motor.load` (servo current). They are useful for diagnosing friction and cheap to log.
* `action`: 8 joint targets, in degrees.
* `observation.images.wrist` and `observation.images.scene` at 640×480, 30 fps.
* Bimanual: prefix every key with `left_` / `right_` (LeRobot's `bi_*` wrappers do the same).

### 2.3 Teleoperation options

| Option | In LeRobot | Gives | Fit for our 7-DOF arm |
|---|---|---|---|
| **Leader arm** (joint-to-joint): SO-101 leader, Koch, OpenArm leader, [GELLO](https://github.com/wuphilipp/gello_software) plugin (7-DOF) | native (`so101_leader`, `openarm_leader`), GELLO as [plugin](https://huggingface.co/docs/lerobot/third_party_robots) | all joints directly, smooth, intuitive, best data for ACT | **Best**, but needs a **custom leader with our joint layout**. Off-the-shelf leaders are 5–6 DOF (SO-101) or match other arms (GELLO: Franka/UR/xArm/YAM). |
| Phone (iOS HEBI Mobile I/O, Android WebXR) | native, [Phone docs](https://huggingface.co/docs/lerobot/phone_teleop) | 6-DOF EE pose → IK (Placo/Pinocchio) → joints; **needs a URDF** | Works once we export a URDF. Elbow swivel is undetermined: IK picks it, so demos are less consistent. |
| VR: Meta Quest / PICO via WebXR ([vr-teleop-kit](https://huggingface.co/docs/lerobot/third_party_robots), `lerobot-teleoperator-teleop`, `pico4`); Isaac Teleop for SO-101 ([v0.6 blog](https://huggingface.co/blog/lerobot-release-v060)) | plugins | 6-DOF per hand, natural for **bimanual** | Good for phase 3 (two hands at once). Needs a URDF and IK, same swivel issue. |
| SpaceMouse ([plugin](https://huggingface.co/docs/lerobot/third_party_robots)) | plugin, IK for SO arms | 6-DOF velocity | Workable for slow tasks. One hand. |
| Gamepad / keyboard | native | EE deltas | Debugging only |
| Haptic (Haply Inverse3, omega.7) | plugins | force feedback | Expensive, not needed |

**Recommendation: build a passive leader arm with our kinematics.** It should have the same 7 axes in the same order (S1, S2, S3, E, F, W1, W2), scaled to about 0.5× (joint distances 166/130/170/119/131/110/101 mm → about 83/65/85/60/65/55/50 mm), plus a trigger for the gripper. There are two ways to build the joints:
1. **STS3215 servos with low gear ratios** as joint sensors, exactly like the SO-101 leader. It uses 1/191 and 1/147 gearing so a hand can move it, and 1/345 on the joint that carries the most weight ([SO-101 docs](https://huggingface.co/docs/lerobot/so101)). This is LeRobot-native (`SerialMotorsBus`) and can later hold the leader against gravity or give force feedback. About 8 × $28 ([RobotShop](https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215)).
2. **Plain bearings + MT6701/AS5600 magnets** read by a small MCU. Cheapest and lightest, but needs friction or springs so the leader stays where you leave it.

Option 1 is less work and gives the same calibration path as the follower. The leader also carries the deadman switch: a foot pedal or a button on the handle.

### 2.4 Bimanual in LeRobot

LeRobot already ships bimanual wrappers that combine two arm instances and prefix their keys: `bi_openarm_follower`/`bi_openarm_leader`, BiSO, BiYAM, BiRebot ([OpenArm](https://huggingface.co/docs/lerobot/openarm), [robot support table](https://huggingface.co/docs/lerobot/integrate_hardware#waiting-between-predictions)). We copy that pattern: `bi_tendon_arm` with `left_arm_config` / `right_arm_config`, with each arm on its own MCU/USB port, and `bi_tendon_leader`. [OpenArm](https://openarm.dev) is the closest reference overall: an open 7-DOF humanoid arm, bimanual, about $6,500 for the pair, integrated in LeRobot with leader arms.

---

## 3. Hardware implications for this robot

### 3.1 What a motor has to deliver at the 8 mm spool

Spool torque = cord force × 8 mm. Cord forces are the worst case over the workspace with 0.3 kg payload and no springs ([loads_report.txt](loads_report.txt)). Friction factor 1.4–2.5 as given; up to ~5 at the wrist dry and in extreme poses. Speed: spool rpm = joint speed × (moment arm / 8 mm).

| Joint | Moment arm | Cord force | Spool torque, nominal | × 2.5 friction | Spool rpm for 90°/s at the joint *(est.)* |
|---|---|---|---|---|---|
| S1 | 42 mm | 204 N | 1.6 N·m (≈ 0.5 N·m with balance springs, no payload) | 4.1 N·m | 79 |
| S2 | 42 mm | 146 N | 1.2 N·m | 2.9 N·m | 79 |
| S3 | 28 mm | 99 N | 0.8 N·m | 2.0 N·m | 53 |
| E | 36 mm | 83 N | 0.7 N·m | 1.7 N·m | 68 |
| F | 25.5 mm | 44 N | 0.35 N·m | 0.9 N·m | 48 |
| W1 | 20 mm | 57 N | 0.46 N·m | 1.1 N·m (2.3 at × 5) | 38 |
| W2 | 20 mm | 35 N | 0.28 N·m | 0.7 N·m | 38 |
| Gripper | n/a | ~30–50 N *(est.)* | ~0.3 N·m | ~0.8 N·m | — |

Demos are usually recorded slower than 90°/s. A spool speed of **40–80 rpm** is enough, and **≥ 30 rpm** is the minimum for usable teleop *(est.)*.

**Safety: the motor can break the cord.** 1 mm Dyneema breaks at about 1000 N, which is only **8 N·m at the spool**. A Damiao DM4310 peaks at 7–12.5 N·m and an STS3095 stalls at about 9 N·m. Set a current or torque limit in firmware and keep the lock-pin idea as a mechanical fallback.

### 3.2 Motor options

| Option | Torque | Speed | Cost each | Interface / LeRobot | Pros | Cons |
|---|---|---|---|---|---|---|
| **Feetech STS3215-C018 (12 V, 1/345)** | 10 kg·cm rated (0.98 N·m), 30 kg·cm stall (2.9 N·m) ([evelta](https://evelta.com/sts3215-c018-12v-30kg-cm-dual-shaft-ttl-serial-servo-metal-gears-magnetic-encoder/)) | ~45 rpm *(not verified)* | $26–31 ([RobotShop](https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215), [OpenELAB](https://openelab.io/it/products/feetech-sts3215-c018-servo-12v)) | TTL half-duplex bus, **native** (`sts3215`) | cheap, LeRobot standard, reads position, load and current | 12-bit spool encoder only; gear backlash; not backdrivable |
| **Feetech STS3250 (12 V)** | 16 kg·cm rated (1.57 N·m), 50 kg·cm peak (4.9 N·m), 4.2 A stall | not verified | ~$99 (C001), $74 (C002) ([OpenELAB](https://openelab.io/products/feetech-sts3250-c001-ttl-servo)) | **native** (`sts3250`) | 1.6× the STS3215 | price |
| **Feetech STS3095 (C002 12 V)** | 22.5 kg·cm rated (2.2 N·m), 95 kg·cm stall (9.3 N·m), 9.8 A stall | ~31 rpm at 12 V | not verified (~$100–140 *(est.)*) ([OpenELAB](https://openelab.io/products/feetech-st3095-c002-serial-servo)) | TTL bus; **not in LeRobot's model list**, add to rustypot or drive from our MCU | strongest Feetech for S1/S2 | slow (S1 full range ~4 s); large stall current |
| **Damiao DM-J4310-2EC** (10:1 QDD, dual encoder) | 3 N·m rated / 7 N·m peak (V1.1); 3.5 / 12.5 N·m (newer version) | 120 rpm nominal (24 V) | ~$125–185 ([OpenELAB](https://openelab.com/products/damiao-brushless-servo-dm-j4310), [roboticscenter](https://www.roboticscenter.ai/store/product/damiao-dm-j4310-2ec-v11)) | CAN 1 Mbps (+ UART), **native** `DamiaoMotorsBus` (Linux) | backdrivable, real torque control (MIT mode), fast, absolute output encoder; same family as OpenArm | 4× the price of STS3215; 24 V + CAN; ~300 g each *(est.)* |
| **Closed-loop NEMA17 + MKS SERVO42D** (+ 3–5:1 belt or planetary for S1/S2) | ~0.4–0.6 N·m direct *(est., motor-dependent)* | high | board ~$27 ([Abra](https://abra-electronics.com/electromechanical/motors/motor-controllers/mot-servo42d-can-mks-servo42d-can-mb-closed-loop-stepper-motor-board-without-motor.html)) + motor ~$15 + gearbox | CAN/RS-485, Modbus; **no LeRobot driver** (fine with our MCU) | cheap torque, our original plan, holds well | no torque sensing, noise, heat at standstill, gearbox for S1/S2 |
| **Dynamixel XM430-W350-T** | ~4 N·m stall *(from memory)* | ~46 rpm *(from memory)* | ~$310 ([RobotShop](https://www.robotshop.com/products/robotis-dynamixel-xm430-w350-t-smart-servo-actuator)) | **native** | quality, current control | price/performance worse than Damiao |
| **QDD BLDC + moteus-c1 / ODrive S1** | motor-dependent | high | controller $69 (moteus-c1, [reseller](https://reach.dog/shop/mjbots-robotic-systems/products/moteus-c1)) / $149 (ODrive S1, [shop](https://shop.odriverobotics.com/products/odrive-s1)) + motor + gearbox | CAN-FD; no LeRobot driver | best torque control | most integration work; needs gearing and own encoders |

**Decision:**
* **Phase 1: all Feetech.**
  * S1, S2: **STS3095**. The balance springs from `balance.py` stay on: they bring S1 down to about 0.5 N·m at the spool.
  * S3, E: **STS3250**.
  * F, W1, W2, gripper: **STS3215-C018**.
  * Everything runs on one 12 V TTL bus, driven by the torso MCU.
  * Cost per arm: 2 × ~$120 + 2 × $99 + 4 × $28 ≈ **$550** *(est.)*.
* **Upgrade:** design the motor bay with a **common spool hub and a 4-hole pattern** so S1/S2 (and S3/E) can swap to **DM-J4310** if S1/S2 turn out slow or too weak once we have measured the friction. For the two-arm torso, 16 × DM4310 (~$2.0–2.9k) gives the cleanest result: torque sensing, backdrivability and LeRobot-native support.
* Keep **one motor per joint** with both cords on one helical spool, as on today's crank (2 tubes per joint). Antagonistic pretension stays mechanical (the M4 tension screws), which avoids a second motor per joint.

### 3.3 Why joint encoders, and how to fit and read them

**Why.** The motor sees the spool, not the joint. Between them are:
* **Cord stretch.** Braided UHMWPE stretches roughly 0.3–1% at 20% of break load. Over a ~1.4 m run that is about 4–14 mm, or **about 5–19° at S1** (42 mm moment arm) and more at the wrist (20 mm) *(est.: measure it)*.
* **Bowden friction and hysteresis.** These are pose-dependent (factor 1.4–2.5, up to ~5).
* **Slack after creep** and **coupling** with the joints upstream.

A policy trained on motor angles would learn the hysteresis too. The arm would also land in different places for the same command depending on load. Measuring the joint directly gives:
1. true state for the dataset and policy;
2. a closed joint-position loop that cancels stretch, friction and coupling;
3. a way to detect a cord slipping or breaking (spool moves, joint doesn't).

**Sensors.**
* **AS5048A**: 14-bit (0.022°), SPI, 3.3/5 V, **daisy-chainable**, so one 4-wire chain reads every sensor ([datasheet](https://cdn.sparkfun.com/assets/e/d/e/1/d/AS5048_DS000298_4-00.pdf)). Ready-made boards with magnets are available. The best fit for a chain through the arm.
* **MT6701**: 14-bit over I²C or SSI, plus ABZ/UVW/PWM outputs, 3.3–5 V, very cheap chip ([datasheet copy](https://voltiq.ru/datasheets/MT6701.pdf), [SimpleFOC thread](https://community.simplefoc.com/t/mt6701-magnetic-position-encoder-support/2618)). SSI with one CS line per sensor.
* **AS5600**: 12-bit, I²C with **one fixed address**. 8 sensors need an I²C multiplexer (TCA9548A) or the analog output, and long I²C runs through the arm are fragile. Fine for the leader arm, not recommended on the follower *(general knowledge, not re-checked)*.

**Fitting.**
* **Hinges (S1, S2, E, W1, W2):** a 6×2.5 mm diametric magnet glued into the end of the rotor drum on the stub-axle axis. The sensor PCB goes on the outside of one lug, 1–2 mm away. The stub axles are steel (M8/M5): keep the magnet on a plastic boss or use a non-magnetic axle on the sensor side, so it doesn't distort the field *(est.)*.
* **Rolls (S3, F):** the bore is full of tubes, so on-axis sensing is impossible. Use a **printed 2:1 gear** from the rotor ring to a small side shaft carrying the magnet. ±80° and ±85° become ±160° and ±170°, still inside one turn, and the gear halves the backlash error. Alternatives: an off-axis ring-magnet sensor, or a printed potentiometer track.
* **Gripper:** a magnet on the jaw pivot, or skip it and use the servo position plus current.

**Reading 8 (one arm) to 16 (two arms) sensors.**
* One MCU per arm. One SPI/SSI bus at ≤ 1 MHz with twisted pairs through the hollow links: 6 wires (VCC, GND, CLK, MOSI, MISO, CS) for an AS5048A chain.
* 8 × 16 bits at 1 MHz is about 0.15 ms per sweep, so a **1 kHz** loop is easy.
* If the runs are noisy, add RS-422 transceivers, or put a tiny MCU (RP2040) at the elbow that streams the distal sensors over CAN *(est.)*.

### 3.4 Controller board, power and safety

* **Torso MCU (one per arm): Teensy 4.1** (600 MHz Cortex-M7, many UARTs, CAN incl. one CAN-FD, USB HS) or an STM32G474 Nucleo (CAN-FD). It drives:
  * the Feetech bus through a half-duplex buffer (for example a 74LVC1G125, or a Waveshare bus-servo adapter);
  * the Damiao motors through a CAN transceiver;
  * the encoders over SPI.
* **Power:**
  * 12 V for Feetech, 15–20 A per arm because the STS3095 stall current is 9.8 A ([OpenELAB](https://openelab.io/products/feetech-sts3095-c001-ttl-servo)).
  * 24 V for Damiao.
  * A separate 5 V for logic and encoders.
* **Safety:**
  * A hardware E-stop that cuts motor power.
  * Firmware limits: joint range from `specs.py` minus 3°, maximum joint speed, current/torque limit below cord break, and a watchdog (no target for 200 ms → hold).
  * LeRobot's `max_relative_target`-style clipping in `send_action()`.

### 3.5 Control-loop rates

| Loop | Rate | Where |
|---|---|---|
| Policy inference / dataset frames | **30 Hz** (LeRobot default `--fps 30`); ACT/π0.5 output chunks (π0.5 chunk 50, GR00T 16) | PC / GPU |
| PC → MCU joint targets | 30 Hz, or 60–90 Hz with `--interpolation_multiplier` ([deploy docs](https://huggingface.co/docs/lerobot/inference)); the MCU also interpolates | USB |
| Joint loop (encoder → motor command) | **500–1000 Hz** | MCU |
| Feetech bus | sync-write all 8 targets in ~0.5 ms; sync-read of 8 servos is a few ms → ~200 Hz full read/write *(est.)*; the servo's internal position loop runs much faster | TTL 1 Mbps |
| Damiao bus | CAN 1 Mbps (FD 5 Mbps data); 8 motors ≈ 1 ms per cycle → 1 kHz *(est.)* | CAN |

With Feetech motors, the outer joint loop sets the servo's **goal position** (cascade: joint PID → spool target). With Damiao motors, it can command **torque** directly (impedance control), which is better at hiding Bowden friction.

### 3.6 Cameras

* **Wrist camera:** small UVC module, 640×480 at 30 fps, wide angle (~120°), MJPG, under 20 g. The USB cable runs with the tubes; keep it off the moving cords.
* **Scene camera:** one fixed UVC webcam (C920 class) looking over the table from the front or above.
* An optional second scene view helps π0.5/SmolVLA, which take up to 3 views (`camera1..3`).
* Depth (RealSense) is supported since 0.6 but not needed.
* All policies resize to about 224–256 px, so a higher resolution only costs USB bandwidth.

### 3.7 Compute

| Job | Minimum | Comfortable |
|---|---|---|
| Recording, ACT inference | the Mac/laptop | — |
| Training ACT | Mac MPS (slow) / Colab | any 12–24 GB NVIDIA card, a few hours |
| SmolVLA fine-tune | Colab / rented A100 | ~4 h per 20k steps on an A100 ([docs](https://huggingface.co/docs/lerobot/smolvla)) |
| π0.5 LoRA fine-tune / inference | 24 GB (RTX 4090) / 8 GB | rented A100/H100 80 GB for full fine-tune ([openpi](https://github.com/Physical-Intelligence/openpi)) |
| GR00T N1.7 | ≥ 40 GB train / ≥ 16 GB run | H100 / L40 ([Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T)) |

Large models can run on a remote GPU through `lerobot-policy-server` / `--inference.type=remote`, with the robot loop on the laptop ([deploy docs](https://huggingface.co/docs/lerobot/inference)). Hugging Face Jobs (`--job.target`, from T4 to 8×H200) can host the training ([v0.6 blog](https://huggingface.co/blog/lerobot-release-v060)).

---

## 4. Recommended stack and phased plan

### 4.1 Stack

| Layer | Choice |
|---|---|
| Learning framework | LeRobot ≥ 0.7, Python 3.12, plugins `lerobot_robot_tendon_arm`, `lerobot_teleoperator_tendon_leader`, later `bi_*` |
| Data | LeRobotDataset v3 on the Hugging Face Hub; joint-space state/action in degrees |
| Policies | ACT → SmolVLA → π0.5 (LoRA); X-VLA or GR00T N1.7 as bimanual alternatives |
| Low level | firmware on Teensy 4.1 (C++): encoder reading, joint PID, coupling, limits, watchdog; simple binary protocol over USB CDC (targets in, state out) |
| Motors | Feetech STS3095 / STS3250 / STS3215 (phase 1), DM-J4310 upgrade path |
| Sensors | AS5048A (or MT6701) per joint; wrist + scene UVC cameras |
| Teleop | 0.5-scale leader arm with STS3215 joints (low-ratio), foot pedal |
| Tools we already have | `arm/kinematics.py` for FK (log EE pose), a URDF export for phone/VR teleop later, the three.js viewer to replay datasets |

### 4.2 Phase 1: one motorised arm + gripper, leader-arm teleop, ACT (≈ 6–10 weeks *(est.)*)

1. **Motor bay:** replace the cranks with servo mounts carrying the same helical spool, keeping the tension screws and tube clips. Begin with E/F/W1/W2 (cheap STS3215), then the shoulder.
2. **Joint encoders** on all 8 axes, plus a 2:1 gear on S3/F.
3. **Firmware:**
   * read encoders, then drive motors open-loop and record spool angle, joint angle and current over the full range (this measures stretch and friction);
   * close the joint loop;
   * identify the coupling matrix (roadmap J1 pass-through test);
   * add limits and the watchdog.
4. **Plugin:** `TendonArm` / `TendonArmConfig` with joint-encoder calibration. Test `lerobot-teleoperate`, `lerobot-record` and `lerobot-replay`. The replay test passes when a recorded episode replays to within ~2° *(target)*.
5. **Leader arm** (0.5×, 8 × STS3215) as `TendonLeader`.
6. **Data:** **50 episodes** of one pick-and-place: 5 cube positions × 10 demos, the structure SmolVLA used ([docs](https://huggingface.co/docs/lerobot/smolvla)). 10–20 s each at 30 fps, wrist + scene camera, constant lighting.
7. **Train ACT** (`--policy.type=act`, defaults, ~100k steps) and roll out with `lerobot-rollout --strategy.type=episodic`. **Goal: ≥ 70% success** on the trained positions *(target)*. Then use `--strategy.type=dagger` to collect corrections for the failures.

### 4.3 Phase 2: language and generalisation (SmolVLA → π0.5)

1. Record 3–5 tasks × 50 episodes, each with a clear `single_task` string ("put the red cube in the bowl", "open the box", …). Vary the objects and lighting.
2. Fine-tune **SmolVLA** from `lerobot/smolvla_base`, mapping the cameras with `--rename_map`. Compare it with per-task ACT.
3. Fine-tune **π0.5** from `lerobot/pi05_base` with `use_relative_actions=true` (gripper excluded). Recompute the quantile stats. Use LoRA (`--policy.use_peft=true`) or `train_expert_only` on a 24 GB card, or a rented 80 GB card. Deploy with RTC, on the remote server if needed.
4. Use `lerobot-rollout --interactive=true` and `/subtask` to switch instructions live.
5. If the shoulder can't keep up (slow STS3095, Bowden hysteresis), swap S1/S2 to DM-J4310 before scaling the data.

### 4.4 Phase 3: two arms on the torso

1. Second arm (mirrored CAD), second MCU, second leader. `bi_tendon_arm` / `bi_tendon_leader` wrappers with `left_` / `right_` keys: 16-D state and action, 3 cameras (2 wrists + 1 scene).
2. Torso motor bank: 16 motors and 28–32 tubes. Plan the cable/tube routing, power (2 × 12/24 V supplies) and cooling.
3. Policies: ACT (originally bimanual), π0.5 (ALOHA heritage), **X-VLA** (`action_mode=auto`, bimanual recipe documented), GR00T N1.7 (`new_embodiment`, relative actions). Start with 50–100 episodes per bimanual task *(est.)*.
4. Optional: VR teleop (Quest/PICO via WebXR) for faster bimanual collection, once a URDF exists.

### 4.5 Rough bill of materials (one arm, phase 1)

| Item | Qty | Unit | Total |
|---|---|---|---|
| Feetech STS3095-C002 (S1, S2) | 2 | ~$120 *(unverified)* | $240 |
| Feetech STS3250 (S3, E) | 2 | $99 | $198 |
| Feetech STS3215-C018 (F, W1, W2, gripper) | 4 | $28 | $112 |
| Leader arm: STS3215 (low-ratio variants) + bus adapter + PETG | 8 + 1 | $28 / $15 | ~$250 |
| AS5048A boards + magnets (8 follower; leader uses servo encoders) | 8 | ~$10–15 *(est.)* | ~$100 |
| Teensy 4.1 + half-duplex buffer + CAN transceiver + PCB | 1 | | ~$60 |
| 12 V 20 A PSU, 5 V buck, E-stop, wiring, connectors | | | ~$90 |
| Wrist camera (UVC wide-angle) + scene webcam | 2 | | ~$90 |
| Spare Dyneema, PTFE tube, printed parts | | | ~$40 |
| **Phase 1 total** | | | **≈ $1,200** |
| GPU: rent (~$1–3/h for A100/H100 class *(est.)*) or own RTX 4090/5090 | | | $50–200 rented per phase / $2–3k bought |
| Upgrade: DM-J4310-2EC for S1/S2 (+ CAN, 24 V PSU) | 2 | $125–185 | ~$400 |
| Phase 3 (second arm + leader + MCU + cameras) | | | ≈ $1,100 more |

### 4.6 Main risks

1. **Bowden friction and hysteresis** (factor up to 2.5, about 5 at the wrist). This causes stick-slip in the joint loop, slow motion, and different behaviour depending on pose.
   * *Mitigation:* joint encoders and an integrator with anti-windup; friction feed-forward identified from the logged data; lubricated PTFE and larger bend radii; DM4310 torque control at the worst joints.
2. **Cord stretch and creep.** Pretension drifts, and backlash appears when the load reverses.
   * *Mitigation:* the joint-angle loop hides it. Re-tension periodically; log the motor-vs-joint offset as a health signal; pre-stretch the cords.
3. **Coupling between joints** (distal cords pass through proximal joints).
   * *Mitigation:* the identified routing matrix in firmware. The policy never sees it.
4. **Slow shoulder with STS3095** (~31 rpm). Demos get slow, and so does the policy.
   * *Mitigation:* the DM-J4310 upgrade path.
5. **Breaking a cord at stall** (8 N·m at the spool breaks the cord).
   * *Mitigation:* current/torque limits and E-stop.
6. **Wrist camera cable and roll-joint encoders** add mass and parts at the distal end, and tangling risk through the S3/F bores.
7. **Policy quality with few demos.** ACT is single-task; VLAs need GPUs and careful normalisation (quantile stats, relative actions).
   * *Mitigation:* start with one well-structured task, use DAgger corrections, keep the cameras fixed.
8. **Software churn.** LeRobot had breaking changes in each of 0.5, 0.6 and 0.7.
   * *Mitigation:* pin the version in `requirements.txt` and keep the plugin small.
9. **Licences.** π0.5 needs the gated PaliGemma/Gemma terms; GR00T weights use the NVIDIA Open Model License; X-VLA's licence is unchecked. Fine for a public hobby repo, but check before any commercial use.

---

## Not verified

* SmolVLA/π0 `max_state_dim` / `max_action_dim` defaults (π0.5 pads actions to 32 per the LeRobot docs; 16-D bimanual fits in any case).
* STS3095 price; STS3215 and STS3250 no-load speeds; DM-J4310 mass; Dynamixel XM430 torque/speed (from memory).
* X-VLA licence and fine-tuning VRAM; OpenVLA-OFT base-model licence; Diffusion Policy parameter count.
* Whether MT6701 SSI `DO` tri-states, which decides whether sensors can share a data line.
* Cord-stretch, friction and bus-timing figures marked *(est.)*. All of them must be measured on the printed arm (phase 1, step 3).
