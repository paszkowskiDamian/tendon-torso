# Roadmap: torso with two tendon-driven arms

**End goal:** a 3D-printed torso with two arms and simple grippers. **All motors sit in the torso**, and the tendons run hidden inside the hollow links, through shoulder, elbow and wrist, down to the gripper.

```
           ┌───────────── torso: motor bank ─────────────┐
           │  [M][M][M][M][M][M]      [M][M][M][M][M][M]  │
           └──┬─────────────────────────────────────┬────┘
   shoulder ( o ) ── all tendons cross here ──      ( o ) shoulder
             ║  upper arm (hollow)                    ║
      elbow ( o )                                    ( o )
             ║  forearm                               ║
      wrist ( o )                                    ( o )
             /\  gripper                              /\
```

## The core problem: one tendon, many joints

A gripper tendon starting in the torso crosses shoulder, elbow and wrist. Each time it passes a joint at distance `r` from the axis, its length changes by `r·Δθ`. Bending the elbow would then squeeze or open the gripper. That's **coupling**, and there are three tools against it, used together:

1. **Cross on the axis** (`r≈0`): the J1 approach. It works for 1-DOF hinges. For a 2–3-DOF shoulder, all axes need to intersect in roughly one point with an open center (gimbal-style), which is the hardest mechanical part of this project.
2. **Idler pulleys on the joint axis**: the classic cable-robot solution, used in surgical robots and Barrett WAM-style arms. The tendon wraps an idler that is concentric with the joint. The length change is then predictable (`r_idler·Δθ`), so you…
3. **…compensate in software**: the motors for the distal joints move by the known coupling amount. With a tendon routing matrix `l = R·θ`, motor commands are `R·θ_desired`. Cheap, but it requires accurate geometry and is the reason to log the pass-through measurement on J1.

Bowden sheaths (cable inside a tube, like bike brakes) avoid coupling completely but add a lot of friction (`e^(μ·total bend)`). Use them only for the gripper's single tendon, if at all.

## Milestones

| # | Milestone | Proves | Status |
|---|---|---|---|
| **J1** | 1-DOF hinge, servo in base, open center | tendon pair, tensioning, printability, pass-through | **designed**, ready to print |
| J1b | Two J1 stacked (elbow + wrist), both motors in the base | multi-joint pass-through, measured coupling, compensation code | next |
| G1 | Simple gripper: 1 tendon + return spring (or antagonistic pair) at the end of J1b | end effector driven from far away | |
| S1 | 2-DOF shoulder (pitch + roll) with an open center | the hard routing problem | |
| A1 | Full arm: shoulder (2–3) + elbow + wrist (1–2) + gripper = 5–7 DOF | | |
| T1 | Torso with motor bank, 2× A1 mirrored | final goal | |

## Decisions to make along the way

- **Actuators.** Servos are fine up to J1b. For the shoulder (~5–10 N·m), consider steppers or small BLDCs with a printed capstan reduction in the torso (see Forte arm and the capstan test stand in [research](research.md)).
- **Sensing.** With tendons, a motor angle ≠ joint angle (stretch, slack). Add joint encoders (e.g. AS5600 magnetic). Because the joint axis is hollow, the magnet goes on the drum flange with the sensor on the lug, off-axis, or use a ring magnet.
- **Tension.** Over time, slack from creep or knots slipping will appear. Plan for a spring tensioner per tendon pair in the torso (series-elastic, which also lets us estimate force).
- **Control software.** Arduino/ESP32 for J1. Later ROS 2 + CARDSflow-style routing model, or LeRobot for teleop and learning.
