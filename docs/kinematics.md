# Kinematics of the 7-DOF tendon arm

Code: [`arm/kinematics.py`](../arm/kinematics.py) · command line: [`arm/ik.py`](../arm/ik.py) · checks: [`tools/check_ik.py`](../tools/check_ik.py)

All numbers come from the CAD chain in `arm/assembly.py`. `check_ik.py` confirms that the forward kinematics reproduces the placed CAD parts exactly.

## 1. Frames and conventions

* **Shoulder frame** (targets are given in this frame):
  * origin on the S1 (shoulder flexion) axis;
  * **+X outwards** (the way abduction lifts the arm), **+Y forwards** (the way flexion lifts it), **+Z up**;
  * millimetres and degrees.
* At **q = 0** the arm hangs straight down, and the tool point is at (0, 0, −939).
* **Tool point:** the centre of the W2 plate (gripper mount) plus `TOOL_OFFSET` = 72 mm along the hand axis: the grasp centre between the gripper pads (`arm/gripper.py`, `GRASP_Z`).
* **Joint vector:** q = (q₁ … q₇) = (S1 shoulder flexion, S2 abduction, S3 upper-arm rotation, E elbow, F forearm twist, W1 wrist flexion, W2 wrist tilt).

## 2. Forward kinematics

Every element of the chain (14 in all: 7 joints, 6 links and the gripper, whose a = b = 0) is

$$A_i(q_i) = R_z(m_i)\; T_z(a_i)\; J_i(q_i)\; T_z(b_i)$$

where:
* $m_i$ is the mounting rotation;
* $a_i$ runs from the incoming flange to the joint axis, and $b_i$ from the joint axis to the outgoing flange;
* $J_i$ is $R_x(q_i)$ for a hinge, $R_z(q_i)$ for a roll joint and the identity for a link.

| # | element | type | m | a (mm) | b (mm) | range |
|---|---|---|---|---|---|---|
| 1 | S1 | hinge, Rx | 0° | 69 | 75 | −30…120° |
| 2 | link | – | 0° | 0 | 22 | |
| 3 | S2 | hinge, Rx | 90° | 69 | 63 | −10…100° |
| 4 | link | – | 0° | 0 | 60 | |
| 5 | S3 | roll, Rz | 0° | 7 | 43 | ±80° |
| 6 | link (upper arm) | – | 0° | 0 | 64 | |
| 7 | E | hinge, Rx | 90° | 63 | 57 | 0…115° |
| 8 | link | – | 0° | 0 | 55 | |
| 9 | F | roll, Rz | 0° | 7 | 43 | ±85° |
| 10 | link (forearm) | – | 0° | 0 | 41 | |
| 11 | W1 | hinge, Rx | 0° | 47 | 41 | ±60° |
| 12 | link | – | 0° | 0 | 22 | |
| 13 | W2 | hinge, Rx | 90° | 47 | 41 | ±90° |
| 14 | gripper | – | 0° | 0 | 0 | (fingers 0…41°) |

All seven axes cross the arm's centreline, and $R_z$ commutes with $T_z$. The chain therefore collapses to joint-to-joint distances along the arm:

$$
{}^{S}T_{tool}(q) = R_x(q_1)\,T_z(166)\,R_z(90°)\,R_x(q_2)\,T_z(130)\,R_z(q_3)\,T_z(170)\,R_z(90°)\,R_x(q_4)\,T_z(119)\,R_z(q_5)\,T_z(131)\,R_x(q_6)\,T_z(110)\,R_z(90°)\,R_x(q_7)\,T_z(113)
$$

followed by a fixed flip from the hanging chain into the shoulder frame (the arm points down, so the chain's $z$ is $-Z$). The distances between the axes are:

| | S1→S2 | S2→S3 | S3→E | E→F | F→W1 | W1→W2 | W2→tool |
|---|---|---|---|---|---|---|---|
| mm | 166 | 130 | 170 | 119 | 131 | 110 | 113 |

Upper arm (S2→E) = 300 mm, forearm (E→W1) = 250 mm.

The shoulder axes S1/S2 are 166 mm apart and the wrist axes W1/W2 are 110 mm apart, so neither the shoulder nor the wrist is *spherical*. The textbook closed-form split (solve the wrist centre for position, then the wrist for orientation) doesn't apply exactly, so the arm is solved numerically (below). That's robust and takes about 4 ms per solve.

## 3. Jacobian

For each joint, let $\hat a_i$ be its world axis (local x of a hinge frame, local z of a roll frame), $p_i$ a point on that axis, and $p_e$ the tool point. The geometric Jacobian ($6 \times 7$) is

$$
J_i = \begin{bmatrix} \hat a_i \times (p_e - p_i) \\ \hat a_i \end{bmatrix},
\qquad
\begin{bmatrix} \dot p_e \\ \omega_e \end{bmatrix} = J(q)\,\dot q .
$$

`check_ik.py` checks it against finite differences (agreement about 1e‑8).

## 4. Inverse kinematics

Given a target position $p^*$, and optionally an orientation $R^*$, find $q$ within the joint limits.

**Error:**

$$
e = \begin{bmatrix} p^* - p_e(q) \\ w\,\log(R^* R_e(q)^\top)^\vee \end{bmatrix}
$$

with $w$ = 150 mm/rad. That weight makes 1° of orientation error count about as much as 2.6 mm of position error. Without an orientation target only the 3 position rows are used.

**Step** (damped least squares, Levenberg–Marquardt):

$$
\Delta q = J^\top\left(J J^\top + \lambda^2 I\right)^{-1} e \;+\; \left(I - J^{+}J\right)\,k\,(q_{mid} - q)
$$

* **Damping $\lambda$:** adapts each step. It shrinks after a step that reduces the error and grows after one that doesn't, so the solver stays stable near singularities, for example with the arm straight.
* **Redundancy:** the second term uses the spare joints without moving the hand. There are 4 spare joints for a position target and 1 for a full pose (the "elbow swivel"). It pulls each joint towards the middle of its range, more strongly near the ends. That keeps the solution away from limits and gives natural-looking poses.
* **Joint limits:** the step is clipped to $[q_{lo}, q_{hi}]$. A joint that is at a limit and being pushed further is removed from $J$ for that step (an active set), so the other joints take over.
* **Restarts:** the solver starts from the current pose and then from the mid pose. If those fail it tries random seeds: 8 for a position target, 30 for a full pose.
* **Result:** a solution within 0.5 mm / 0.5°. If the target is out of reach, it returns the closest pose and says so.

Measured by `tools/check_ik.py` on 150 random reachable poses, each solved from the straight pose:

| target | solved | worst error | time |
|---|---|---|---|
| position | 150 / 150 | 0.49 mm | ~3 ms |
| position + orientation | 150 / 150 | 0.50 mm, 0.18° | ~4 ms |

**Workspace** (shoulder frame): x −465…766, y −633…928, z −920…495 mm. The tool reaches 167–937 mm from the S1 axis.

## 5. From joint angles to cranks

Every joint is a closed tendon loop: a drum of radius $R_j$ at the joint and a spool of radius $r$ = 8 mm at its crank. Each cord runs inside its own Bowden tube, so the joints don't couple. That gives

$$
n_j = \frac{q_j}{360°}\cdot\frac{R_j}{r}
$$

crank turns, counted from the straight (zero) pose.

| | S1 | S2 | S3 | E | F | W1 | W2 |
|---|---|---|---|---|---|---|---|
| $R_j$ (mm) | 42 | 42 | 28 | 36 | 25.5 | 20 | 20 |
| crank turns per joint turn | 5.25 | 5.25 | 3.5 | 4.5 | 3.19 | 2.5 | 2.5 |

The crank lock pin indexes every 10°, so the joint resolution is $10°/(R_j/r)$: about 1.9° at the shoulder and 4° at the wrist.

## 6. Using it

```bash
# where do these joint angles put the hand?
.venv/bin/python -m arm.ik --fk 45 25 20 70 40 30 10

# which joint angles (and crank turns) put the hand here?
.venv/bin/python -m arm.ik -40 164 -695
.venv/bin/python -m arm.ik -40 164 -695 --rpy 170 42 6     # also the hand orientation
```

```python
from arm import kinematics as K
r = K.ik([-40, 164, -695])            # shoulder frame, mm
r.q, r.ok, r.pos_err                  # joint angles (deg), success, error (mm)
K.crank_turns(r.q)                    # turns for each crank
K.fk(r.q)                             # 4x4 tool pose
```

The same solver runs in the browser viewer (*Arm · 7 DOF* → **Reach mode**). Move the hand target and the arm and cranks follow.
