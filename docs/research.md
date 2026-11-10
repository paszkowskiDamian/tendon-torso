# Research notes: tendon-driven, 3D-printed joints

Snapshot from 2026-11. The question: which open designs exist for tendon-driven joints that can be printed, and what should we borrow for a torso with two arms whose motors all live in the torso?

## Open projects worth knowing

| Project | What it is | Open? | What to take from it |
|---|---|---|---|
| [Roboy](https://en.wikipedia.org/wiki/Roboy) / [roboy.org](https://roboy.org/research/) | Musculoskeletal, tendon-driven humanoid, 3D-printed body | Yes: code + CAD on GitHub, BSD / CC-BY 4.0 ([roboy.org](https://roboy.org/?p=2323)) | Closest to our end goal. Its **CARDSflow** framework handles design, simulation and control of cable-driven robots ([devanthro](https://www.devanthro.com/?p=382)) |
| InMoov hand & forearm | Printed humanoid, servos in the forearm pull finger tendons | Yes | Proven, simple servo + braided-line approach. Its tendons don't cross many joints, though |
| [ORCA hand](https://www.roboticgizmos.com/orca-v1/amp) | Open-source tendon-driven humanoid hand | Yes | Tendon routing through printed parts, for when we upgrade the gripper |
| [Aero Hand Open](https://feedbagel.com/post/aero-hand-open-open-source-3d-printable-robotic-hand-for-research-and-makers) | Printable tendon hand, < $314 | Yes | Same |
| Faive hand (ETH) ([3dprinting.com](https://3dprinting.com/news/eth-zurich-3d-prints-tendon-driven-robotic-hand)) | Tendon hand with **rolling-contact joints** | Not confirmed | Rolling-contact joints suit fingers, not load-bearing arm joints |
| [Open Parametric Hand](https://arxiv.org/pdf/2410.18633) | Single-piece printed hand, 56+ parameters | Stated open | Parametric approach like ours |
| [CRAFT hand](https://arxiv.org/pdf/2603.12120) | Tendon hand with rolling-contact PIP/DIP joints | Announced, not yet confirmed | — |
| [DLR tendon continuum mechanism](https://elib.dlr.de/188830/1/deutschmann2022_robosoft_final_submission.pdf) | Open-source printed continuum joint, ~$100 | Yes | A candidate wrist (a 2-DOF continuum wrist driven by 4 tendons) |
| [Forte arm](https://arxiv.org/html/2507.15693v1) | Printed arm with **steel-cable capstans** at shoulder/elbow, < $215, 0.63 kg payload | Paper | Proof that printed capstan reduction works for arm joints |
| [Capstan-Drive test stand](https://github.com/jshet/Capstan-Drive) | PLA capstan, 8.55:1, lead-screw tensioner | Yes | Tensioner design and helical drum grooves |
| Aaed Musa robot dog ([Hackaday](https://hackaday.com/2024/06/03/gears-are-old-and-busted-capstans-are-cool)) | Capstan-driven legs | Partly | **Steel cable fatigued and snapped; Dyneema DM-20 worked.** |
| [Justin Hart quadruped leg](https://sites.google.com/view/justinhart/home/personal-projects/quadruped-robot-leg) | Capstan leg, Dyneema, 10:1 | Write-up | Dyneema "essentially zero creep" for this use |
| Tendon-driven linkage ([3dsearch](https://3dsearch.net/model/tendon-driven-linkage-tv7357304)) | Simple printed joint, M3 pivot, 2 mm Kevlar line in channels | Aggregator, license unchecked | The simplest possible baseline |
| [SO-ARM100/101](https://aiwiki.ai/wiki/the_robot_studio/raw) | Popular printed arm, Apache 2.0 | Yes | Not tendon-driven (servo in every joint), but useful reference for proportions and the LeRobot software stack |

None of these is a ready-made **arm joint built for multi-joint tendon pass-through**. The hands solve routing at finger scale, Roboy solves it with muscles spanning joints, and the capstan arms keep motors near their joints. So we design our own joint and borrow the ideas.

## Design decisions for J1 (and why)

1. **Pin joint on bearings, not rolling contact.** Rolling-contact joints (Faive, CRAFT) are great for fingers: compact and compliant. But they need tendon tension to hold together and handle off-axis loads poorly. Shoulder and elbow carry the whole arm, so they get real bearings (608s are cheap and everywhere).
2. **Antagonistic closed loop, one motor per DOF.** Two tendons per joint, wrapped in opposite directions on the same radius on drum and spool. The alternative is N+1 tendons for N joints, with fewer motors but control coupled across every joint. That's worth revisiting for the fingers later, not for the arm.
3. **Route distal tendons through the joint axis.** A cord's length changes by `r·Δθ`, where `r` is its distance from the axis. Through the axis, `r≈0`, so no coupling. This is why the drum is hollow and the bearings sit at its ends.
4. **Dyneema/UHMWPE braid.** Every capstan project converged on it: low stretch, practically no creep at hobby loads, and it survives small bend radii. Steel fatigues over printed pulleys, and nylon mono stretches.
5. **Keep bend radius ≥ ~10–15× the cord diameter.** Drum radius 15.5 mm for 1 mm cord.
6. **Friction budget.** Every wrap angle θ over a fixed surface multiplies tension by `e^(μθ)` (capstan equation). With PE on PLA (μ≈0.15–0.25), a 90° bend over a fixed edge costs 25–50% of the force. So: straight runs, rotating idlers or bearings at every bend, and PTFE sleeves only where unavoidable.

## Open questions to answer with hardware

- How much does PETG creep under constant tendon pre-tension?
- How good is the coupling through the J1 center slot in practice? The cords rub on the slot edges, and later they'll need tiny idler rollers.
- Servo or stepper/BLDC + capstan in the torso? Servos are fine for J1, but the shoulder needs ~10× the torque.
