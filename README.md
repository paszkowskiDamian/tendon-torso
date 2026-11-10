# Tendon Torso

An open, fully 3D-printed **tendon-driven torso with two arms**. All motors sit in the torso, and the tendons run hidden through the hollow core of every link, through shoulder, elbow and wrist, down to simple grippers.

We build it up one joint at a time.

![7-DOF tendon arm](docs/img/arm_assembly.png)

## Status

| Part | Status |
|---|---|
| [J1: 1-DOF tendon hinge test rig](joints/j1-hinge/) | designed, STL ready, **not printed/tested yet** |
| [J1-S: NEMA17, 6:1 tendon reduction, Bowden-ready center](joints/j1s-hinge/) | designed, STL ready, **not printed/tested yet** |
| [J1b: 2-DOF, J1-S elbow + J2 wrist, both motors remote (J2 via Bowden)](joints/j1b-2dof/) | designed, STL ready, **not printed/tested yet** |
| [**Arm: 7 DOF, hand-cranked**](arm/), shoulder (3), elbow, forearm twist, wrist (2) | designed, all 37 parts print-ready and CAD-checked, **not printed yet** |
| Gripper, second arm, motors | see [roadmap](docs/roadmap.md) |

## Interactive viewer

An animated 3D assembly showing how the tendons wind is in [`docs/viewer/`](docs/viewer/). Run `python3 -m http.server -d docs/viewer` and open http://localhost:8000.

## Repo layout

```
docs/research.md      existing open designs + why J1 looks the way it does
docs/roadmap.md       path to the torso, the tendon-routing problem
joints/j1-hinge/      J1 (servo, 1:1): CadQuery source, STL/STEP, build guide
joints/j1s-hinge/     J1-S (stepper, 6:1, Bowden center)
joints/j1b-2dof/      J1b: J1-S + J2 wrist module + Bowden drive module
joints/common/        shared CAD helpers
arm/                  the full 7-DOF arm: modules, links, cranks, torso, STLs, build guide
arm/kinematics.py     forward + inverse kinematics (python -m arm.ik X Y Z), see docs/kinematics.md
tools/                preview renders + collision / clearance checks
```

## Building the CAD

Everything is parametric Python ([CadQuery](https://cadquery.readthedocs.io/)). Change a number, then regenerate:

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python joints/j1-hinge/j1_hinge.py
.venv/bin/python tools/render.py
```

STEP files open in Fusion, FreeCAD, Onshape or Rhino.

## License

[MIT](LICENSE): use it, remix it, sell it, just keep the notice. Contributions and build reports are welcome.
