"""Build the MuJoCo model of the two-arm robot from the CAD (arm/ package).

    .venv/bin/python -m sim.build_model          # writes sim/assets/*.stl + sim/robot.xml

Everything comes from the same numbers that place the printed parts:

* kinematic chain: arm/assembly.py CHAIN (mount, in_z, out_z per element), joint
  limits from the joint specs, the gripper's finger pivots from arm/gripper.py;
* meshes: every printed part, tessellated in its element frame (mm, scaled to m);
* masses + centres of mass: arm/statics.py mass_model() (printed parts with the
  print-profile infill + hardware), inertia from the part's bounding box;
* torso: extrusions, shoulder beams, saddles and the 16 drive units (visual only).

Frames: MuJoCo world = the TORSO frame (arm/torso.py) in metres: origin on the
table top on the robot's mid-plane, +X = the robot's left, -Y = forwards, +Z = up.

The left arm is the mirror image of the right one. MuJoCo bodies can't carry a
reflection, so the left chain is built from mirrored frames (S M S with
S = diag(-1, 1, 1), a proper rotation again) and mirrored meshes; a joint axis a
becomes -S a. With that, q_left = q_right gives a mirror-symmetric pose - the same
convention as the CAD, the viewer and arm/kinematics.py.

Simplifications (the real arm is tendon driven):
* gravity on the arm is compensated (body gravcomp): the real servos' integral term and
  the balance springs hold the static load, the P-D loop here only moves the arm;
* every joint is a position servo (the drive unit's servo + the tendon, lumped);
  its force limit is the servo's stall torque times the tendon ratio;
* the gravity balance springs are not modelled (the servos hold the arm);
* collisions: the table, the objects and the gripper pads/palm collide; the arm
  parts collide with the table and the objects through their convex hulls, the
  fingers only through their pads; the arms don't collide with each other.
"""
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arm import assembly as A  # noqa: E402
from arm import gripper as G  # noqa: E402
from arm import hinge as H  # noqa: E402
from arm import roll as R  # noqa: E402
from arm import statics as ST  # noqa: E402
from arm import torso as T  # noqa: E402

OUT = ROOT / "sim"
ASSETS = OUT / "assets"
S = np.diag([-1.0, 1.0, 1.0, 1.0])
SIDES = {"R": "r", "L": "l"}

# servo stall torque (N m) x tendon ratio (= joint torque) - drive.py PHASE1 motors, rounded down
JOINT_FORCE = {"S1": 20.0, "S2": 20.0, "S3": 10.0, "E": 12.0, "F": 6.0, "W1": 5.0, "W2": 4.0}
JOINT_KP = {"S1": 120.0, "S2": 120.0, "S3": 40.0, "E": 60.0, "F": 15.0, "W1": 12.0, "W2": 8.0}
FINGER_FORCE = 0.6        # N m on finger A ~ 12 N per pad at the pad centre (gripper cord ~30 N)
COLORS = {"base": "0.85 0.45 0.20 1", "rotor": "0.30 0.55 0.85 1", "link": "0.80 0.80 0.82 1",
          "stator": "0.85 0.45 0.20 1", "idler": "0.25 0.25 0.25 1", "retainer": "0.55 0.55 0.6 1",
          "gripper": "0.35 0.75 0.45 1", "pad": "0.15 0.15 0.15 1", "torso": "0.62 0.64 0.68 1",
          "drive": "0.25 0.27 0.30 1"}


def color_of(part):
    for k in ("pad", "idler", "retainer", "stator", "rotor", "base", "link", "gripper"):
        if k in part:
            return COLORS[k]
    return COLORS["link"]


# ------------------------------------------------------------------ meshes
def tess(wp, tol=0.3, ang=0.5):
    verts, tris = (wp.val() if hasattr(wp, "val") else wp).tessellate(tol, ang)
    m = trimesh.Trimesh([(p.x, p.y, p.z) for p in verts], tris, process=True)
    m.merge_vertices()
    return m


def save(mesh, name, mirror=False):
    m = mesh.copy()
    if mirror:
        m.vertices[:, 0] *= -1
        m.invert()
    m.export(ASSETS / f"{name}.stl")
    return name


# ------------------------------------------------------------------ xml helpers
def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def quat(Rm):
    """wxyz quaternion of a rotation matrix."""
    q = np.empty(4)
    t = np.trace(Rm)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q[:] = 0.25 * s, (Rm[2, 1] - Rm[1, 2]) / s, (Rm[0, 2] - Rm[2, 0]) / s, (Rm[1, 0] - Rm[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(Rm)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + Rm[i, i] - Rm[j, j] - Rm[k, k]) * 2
        q[0] = (Rm[k, j] - Rm[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (Rm[j, i] + Rm[i, j]) / s
        q[1 + k] = (Rm[k, i] + Rm[i, k]) / s
    return q / np.linalg.norm(q)


def pose(M):
    """pos (m) + quat attributes of a 4x4 (mm)."""
    return f'pos="{fmt(M[:3, 3] / 1000)}" quat="{fmt(quat(M[:3, :3]))}"'


def Rz(deg):
    a = math.radians(deg)
    m = np.eye(4); m[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return m


def Tz(d):
    m = np.eye(4); m[2, 3] = d
    return m


def Tx(x, y, z):
    m = np.eye(4); m[:3, 3] = (x, y, z)
    return m


# ------------------------------------------------------------------ the arm
WRIST_CAM = (0.0, -48.0, 18.0)      # on the palm rim (palm frame, mm), outside the fingers' swing
WRIST_LOOK = (0.0, 0.0, 115.0)      # looks at a point just beyond the pad tips


def wrist_camera(p, Mp, ind):
    """<camera> on the palm (palm frame Mp, already mirrored for the left arm)."""
    c = (Mp @ np.r_[WRIST_CAM, 1.0])[:3]
    f = (Mp @ np.r_[WRIST_LOOK, 1.0])[:3] - c
    f /= np.linalg.norm(f)
    up = Mp[:3, :3] @ np.array([0, 0, -1.0])           # image up = back towards the palm
    y = up - f * (up @ f); y /= np.linalg.norm(y)
    x = np.cross(y, -f)
    return f'{ind}<camera name="{p}_wrist" pos="{fmt(c / 1000)}" xyaxes="{fmt(np.r_[x, y])}" fovy="80"/>'


class Arm:
    """Writes one arm's body tree."""

    def __init__(self, side, meshes, masses):
        self.side, self.p = side, SIDES[side]
        self.mir = side == "L"
        self.meshes, self.masses = meshes, masses
        self.lines = []

    def M(self, m):
        return S @ m @ S if self.mir else m

    def axis(self, a):
        a = np.asarray(a, float)
        return -(S[:3, :3] @ a) if self.mir else a

    def mesh(self, part):
        return f"{part}_{'l' if self.mir else 'r'}"

    def geoms(self, parts, M_rel, ind, hull=True):
        """Visual + (optionally) collision geoms of parts placed at M_rel in the current body."""
        out = []
        for part in parts:
            at = pose(self.M(M_rel))
            out.append(f'{ind}<geom class="visual" mesh="{self.mesh(part)}" {at} rgba="{color_of(part)}"/>')
            if hull:
                out.append(f'{ind}<geom class="hull" mesh="{self.mesh(part)}" {at}/>')
        return out

    def build(self):
        p = self.p
        base = A.arm_base(self.side)            # chain base in the torso frame (already mirrored for L)
        L = self.lines
        L.append(f'    <body name="{p}_base" pos="{fmt(base[:3, 3] / 1000)}" quat="{fmt(quat(base[:3, :3]))}">')
        depth = 3
        ind = "  " * depth
        M_rel = np.eye(4)
        pend_static = []                        # (mass, com, M_rel, size) to fold into the current body
        body_mass = []
        for name, spec, mount in A.CHAIN:
            z_in, z_out = A.element_io(spec, name)
            M_rel = M_rel @ Rz(mount) @ Tz(-z_in)
            mm = self.masses[name]
            if isinstance(spec, G.GripperSpec):
                self.gripper(M_rel, ind, depth, mm)
                break
            st, mv = A.element_parts(name, spec)
            L.extend(self.geoms(st, M_rel, ind))
            body_mass.append((mm["m_static_g"], mm["com_static"], M_rel.copy(), self.meshes[next(iter(st))][1]))
            if isinstance(spec, (H.HingeSpec, R.RollSpec)):
                # close the current body's inertial, open the joint's body
                L.extend(self.merged_inertial(body_mass, ind))
                body_mass = []
                Mb = self.M(M_rel)
                L.append(f'{ind}<body name="{p}_{name}" pos="{fmt(Mb[:3, 3] / 1000)}" quat="{fmt(quat(Mb[:3, :3]))}">')
                depth += 1
                ind = "  " * depth
                ax = self.axis((1, 0, 0) if isinstance(spec, H.HingeSpec) else (0, 0, 1))
                L.append(f'{ind}<joint name="{p}_{name}" class="{"hinge" if isinstance(spec, H.HingeSpec) else "roll"}" '
                         f'axis="{fmt(ax)}" range="{math.radians(spec.lo):.5f} {math.radians(spec.hi):.5f}"/>')
                M_rel = np.eye(4)
                L.extend(self.geoms(mv, M_rel, ind))
                body_mass.append((mm["m_moving_g"], mm["com_moving"], M_rel.copy(), self.meshes[next(iter(mv))][1]))
            M_rel = M_rel @ Tz(z_out)
        L.append(f'{"  " * 3}</body>')
        self.depth_close = depth

    def merged_inertial(self, items, ind):
        """One inertial for a body from several (mass g, com, M_rel, bbox size) pieces."""
        items = [it for it in items if it[0] > 0]
        if not items:
            return []
        m = sum(it[0] for it in items) / 1000
        cs = [(self.M(Mr) @ np.r_[np.asarray(c, float), 1.0])[:3] / 1000 for _, c, Mr, _ in items]
        c = sum(it[0] / 1000 * ci for it, ci in zip(items, cs)) / m
        I = np.zeros(3)
        for (mg, _, Mr, size), ci in zip(items, cs):
            mi = mg / 1000
            sx, sy, sz = np.asarray(size, float) / 1000 * 0.7       # hollow parts: smaller than the box
            I += mi / 12 * np.array([sy ** 2 + sz ** 2, sx ** 2 + sz ** 2, sx ** 2 + sy ** 2])
            d = ci - c
            I += mi * (d @ d - d ** 2)
        return [f'{ind}<inertial pos="{fmt(c)}" mass="{m:.5f}" diaginertia="{fmt(np.maximum(I, 1e-7))}"/>']

    def gripper(self, M_rel, ind, depth, mm):
        """Palm (fixed to W2's rotor) + two geared fingers with pads."""
        p, L = self.p, self.lines
        L.extend(self.geoms(["gripper_palm"], M_rel, ind))
        L.extend(self.merged_inertial([(mm["m_static_g"], mm["com_static"], M_rel.copy(), (62, 62, 37))], ind))
        tool = self.M(M_rel @ Tz(G.GRASP_Z))
        L.append(f'{ind}<site name="{p}_tool" {pose(tool)} size="0.006" rgba="1 0.2 0.2 0.6" group="3"/>')
        # wrist camera: on the palm rim, beside the fingers' plane, looking at the grasp zone
        L.append(wrist_camera(p, self.M(M_rel), ind))
        for w in ("a", "b"):
            W = w.upper()
            pv = M_rel @ Tx(*G.pivot(W))
            Mb = self.M(pv)
            L.append(f'{ind}<body name="{p}_finger_{w}" pos="{fmt(Mb[:3, 3] / 1000)}" quat="{fmt(quat(Mb[:3, :3]))}">')
            i2 = ind + "  "
            ax = self.axis((0, -1, 0) if W == "A" else (0, 1, 0))    # + = opening
            L.append(f'{i2}<joint name="{p}_finger_{w}" class="finger" axis="{fmt(ax)}" range="0 {math.radians(G.THETA_MAX):.5f}"/>')
            back = np.linalg.inv(Tx(*G.pivot(W)))                     # parts are in the palm frame
            L.extend(self.geoms([f"gripper_finger_{w}", f"gripper_pad_{w}"], back, i2, hull=False))
            # pad collision box (pad bbox at closed, in the palm frame)
            bb = self.meshes[f"gripper_pad_{w}"][2]
            c = (bb[0] + bb[1]) / 2
            hs = (bb[1] - bb[0]) / 2
            Mp = self.M(back @ Tx(*c))
            L.append(f'{i2}<geom name="{p}_pad_{w}" class="pad" type="box" size="{fmt(hs / 1000)}" {pose(Mp)}/>')
            L.append(f'{i2}<inertial pos="{fmt(self.M(back @ Tx(0, 0, G.ZP + 30))[:3, 3] / 1000)}" mass="0.015" diaginertia="4e-6 4e-6 1e-6"/>')
            L.append(f"{ind}</body>")
        while depth > 3:
            depth -= 1
            L.append(f'{"  " * depth}</body>')


# ------------------------------------------------------------------ torso (visual)
def torso_meshes():
    import cadquery as cq
    from arm import drive as DR
    parts = []
    alu = [T.extrusion_solid(n).val() for n in T.extrusions()]
    parts.append(("torso_alu", tess(cq.Compound.makeCompound(alu), 0.5, 0.5), "torso"))
    beam = tess(T.shoulder_beam(A.S1), 0.4, 0.5)
    for s in ("R", "L"):
        b = beam.copy()
        b.apply_transform(A.MOUNT)
        if s == "L":
            b.vertices[:, 0] *= -1
            b.invert()
        parts.append((f"torso_beam_{s.lower()}", b, "torso"))
    sad = tess(T.saddle(), 0.4, 0.5)
    lv = []
    for k in range(T.LEVELS):
        m = sad.copy(); m.apply_transform(T.saddle_matrix(k)); lv.append(m)
    parts.append(("torso_saddles", trimesh.util.concatenate(lv), "torso"))
    drives = []
    for (s, j), M in A.drive_frames().items():
        m_ = A.drive_motor(j)
        up = A.drive_unit_parts(m_, j == "G")
        for k in ("frame", "spool"):
            d = tess(up[k], 0.6, 0.8); d.apply_transform(M); drives.append(d)
        d = tess(DR.motor_body(m_), 0.6, 0.8); d.apply_transform(M); drives.append(d)
    parts.append(("torso_drives", trimesh.util.concatenate(drives), "drive"))
    return parts


# ------------------------------------------------------------------ write
def build(torso=True):
    ASSETS.mkdir(parents=True, exist_ok=True)
    masses = {e["name"]: e for e in ST.mass_model()["elements"]}
    meshes = {}                     # part -> (name, bbox size mm, bbox)
    asset_lines = []
    for name, spec, _ in A.CHAIN:
        if isinstance(spec, G.GripperSpec):
            items = G.parts(0.0).items()
        else:
            st, mv = A.element_parts(name, spec)
            items = list(st.items()) + list(mv.items())
        for k, w in items:
            m = tess(w, 0.25 if "gripper" in k else 0.35, 0.5)
            bb = m.bounds
            meshes[k] = (k, bb[1] - bb[0], bb)
            for side in ("r", "l"):
                save(m, f"{k}_{side}", mirror=side == "l")
                asset_lines.append(f'    <mesh name="{k}_{side}" file="{k}_{side}.stl"/>')
            print(f"  {k:24s} {len(m.faces):6d} tris")
    torso_lines = []
    if torso:
        for nm, m, col in torso_meshes():
            save(m, nm)
            asset_lines.append(f'    <mesh name="{nm}" file="{nm}.stl"/>')
            torso_lines.append(f'    <geom class="visual" mesh="{nm}" rgba="{COLORS[col]}"/>')
            print(f"  {nm:24s} {len(m.faces):6d} tris")
    bodies = []
    for side in ("R", "L"):
        a = Arm(side, meshes, masses)
        a.build()
        # the servo's integral term (and the balance springs) hold the arm against gravity:
        # the position loop only has to move it, so the arm bodies get gravity compensation
        bodies.extend(l.replace("<body name=", '<body gravcomp="1" name=') for l in a.lines)

    p_act, eqs, sensors = [], [], []
    for side, p in SIDES.items():
        for j in A.DRIVE_ORDER[:-1]:
            sp = A.joint_spec(j)
            p_act.append(f'    <position name="{p}_{j}" joint="{p}_{j}" kp="{JOINT_KP[j]}" kv="{JOINT_KP[j] * 0.08:.3g}" '
                         f'forcerange="{-JOINT_FORCE[j]} {JOINT_FORCE[j]}" ctrlrange="{math.radians(sp.lo):.5f} {math.radians(sp.hi):.5f}"/>')
        p_act.append(f'    <position name="{p}_gripper" joint="{p}_finger_a" kp="3" kv="0.05" '
                     f'forcerange="{-FINGER_FORCE} {FINGER_FORCE}" ctrlrange="0 {math.radians(G.THETA_MAX):.5f}"/>')
        eqs.append(f'    <joint joint1="{p}_finger_b" joint2="{p}_finger_a" polycoef="0 1 0 0 0" solref="0.005 1"/>')
        p_ex = f'{p}_finger_a'
        sensors.append(f'    <jointpos name="{p}_grip_pos" joint="{p_ex}"/>')
    xml = f"""<!-- generated by sim/build_model.py from the CAD in arm/ - do not edit by hand -->
<mujoco model="tendon-torso">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="64M"/>
  <default>
    <mesh scale="0.001 0.001 0.001"/>
    <joint damping="0.3" armature="0.01" frictionloss="0.02"/>
    <default class="hinge"><joint damping="1.0" armature="0.05"/></default>
    <default class="roll"><joint damping="0.5" armature="0.02"/></default>
    <default class="finger"><joint damping="0.02" armature="0.001" frictionloss="0.002"/></default>
    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>
    <default class="hull"><geom type="mesh" contype="2" conaffinity="0" group="3" density="0" rgba="1 1 1 0.2"/></default>
    <default class="pad"><geom contype="1" conaffinity="1" group="3" density="0" friction="1.2 0.02 0.001"
        condim="4" solref="0.004 1" solimp="0.95 0.99 0.001" rgba="0.1 0.1 0.1 0.4"/></default>
  </default>
  <asset>
{chr(10).join(asset_lines)}
  </asset>
  <worldbody>
    <body name="torso">
{chr(10).join(torso_lines)}
    </body>
{chr(10).join(bodies)}
  </worldbody>
  <contact>
    <exclude body1="r_finger_a" body2="r_finger_b"/>
    <exclude body1="l_finger_a" body2="l_finger_b"/>
  </contact>
  <equality>
{chr(10).join(eqs)}
  </equality>
  <actuator>
{chr(10).join(p_act)}
  </actuator>
</mujoco>
"""
    (OUT / "robot.xml").write_text(xml)
    print("sim/robot.xml written")


if __name__ == "__main__":
    build(torso="--no-torso" not in sys.argv)
