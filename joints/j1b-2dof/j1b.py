"""
J1b - two degrees of freedom: J1-S (elbow) + J2 (wrist), both motors remote
===========================================================================

    python joints/j1b-2dof/j1b.py        # -> joints/j1b-2dof/stl/

Architecture
------------
* J1 is the J1-S hinge (NEMA17 in the base, 6:1 bare-tendon reduction).
  Two variants are used: the proximal cap has THROUGH holes for tubes and the
  distal link ends in a MOUNT FLANGE (4x M3 heat-set inserts).
* J2 is a smaller hinge module that bolts onto that flange. The square bolt
  pattern is 90-deg symmetric, so J2 can be mounted with its axis parallel to
  J1 (planar arm) or perpendicular (default here: wrist yaw).
* J2's tendons are BOWDEN cables: Dyneema inside PTFE tubes (2/4 mm). The tubes
  are anchored in the J2 mount and in a separate BOWDEN DRIVE module (NEMA17 +
  helical spool) that can sit anywhere - next to the base now, in the torso
  later. The tubes pass through J1's open centre, so J1 motion should not
  move J2's tendons (that's the experiment).
* J2 ratio: drum r 20 / spool r 6 = 3.33:1 (wrist ~0.7 N*m usable after
  Bowden friction).

J2 local frame: axis = X', distal = +Z', origin on the J2 axis.
Its mount plate bottom sits on the J1 flange top, J2_H below the J2 axis.
"""
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "common"))
sys.path.insert(0, str(HERE.parent / "j1s-hinge"))
from cadlib import (box, chord_anchor, cyl_x, groove_ring, hex_prism_x,  # noqa: E402
                    hole_along, polar_yz)
import j1s_hinge as S  # noqa: E402

OUT = HERE / "stl"

# ---------------------------------------------------------------- J2 parameters
CORD_D = S.CORD_D
R2_CORD = 20.0
R2_GROOVE = R2_CORD - CORD_D / 2
R2_HUB = R2_CORD + 2.0
G2_X = 9.5
GROOVE_W = 2.4
RATIO2 = R2_CORD / S.R_CORD_SPOOL

BRG_D, BRG_B, FIT = 22.0, 7.0, 0.15
CAV2_HALF = 5.0
WALL2 = 1.0
HUB2_HALF = CAV2_HALF + WALL2 + BRG_B          # 13
R2_CAV = 14.0
R2_LEDGE = 9.5
RANGE2 = 100.0

LUG2_GAP, LUG2_T, LUG2_R = 1.0, 12.0, 13.0
J2_H = 45.0                       # J2 axis above the mount plate bottom
PLATE_T = 5.0
PLATE_X, PLATE_Y = 26.0, 22.0
BOSS_X, BOSS_Y, BOSS_TOP = 12.5, 11.0, -30.0
ANCHOR_Y = 6.0                    # tube exit (cord leaves the tube here)
SOCKET_DEPTH = 8.0

STUB_OD, STUB_ID = 36.0, 30.0
STUB_START = R2_HUB + 4.0         # 26
STUB_LEN = 50.0
STUB_END_T = 5.0
TAB2_Z = (40.0, 68.0)

TUBE_OD, TUBE_ID = S.TUBE_OD, S.TUBE_ID
M3_CLR, M3_HEAD_D, M3_NUT_AF = S.M3_CLR, S.M3_HEAD_D, S.M3_NUT_AF
M8_CLR, M8_NUT_AF, M8_NUT_T = S.M8_CLR, S.M8_NUT_AF, S.M8_NUT_T

J2_MOUNT_ROT = 90.0               # 90 = J2 axis perpendicular to J1, 0 = parallel


def anchor_and_tangent():
    """Flexor tube exit A (y,z) and the drum tangent point T (y,z, phi_deg) in the J2 frame."""
    ay, az = ANCHOR_Y, BOSS_TOP
    d = math.hypot(ay, az)
    alpha = math.atan2(az, ay)
    beta = math.acos(R2_CORD / d)
    phi = alpha + beta                       # tangent point on the +Y side
    return (ay, az), (R2_CORD * math.cos(phi), R2_CORD * math.sin(phi), math.degrees(phi))


# ---------------------------------------------------------------- J2 distal half
def j2_distal_half():
    _, (_, _, phi_t) = anchor_and_tangent()
    hub = cyl_x(R2_HUB, -HUB2_HALF, 0)
    neck_x = -(G2_X - GROOVE_W / 2 - 0.8)
    neck = box(neck_x, 0, -12, 12, 10, STUB_START + 4)
    top = STUB_START + STUB_LEN
    stub = (cq.Workplane("XY").workplane(offset=STUB_START).circle(STUB_OD / 2).extrude(STUB_LEN)
            .intersect(box(-STUB_OD, 0, -STUB_OD, STUB_OD, STUB_START, top)))
    body = hub.union(neck).union(stub)
    tab_y = STUB_OD / 2 + 4.5
    for z in TAB2_Z:
        body = body.union(box(-6, 0, STUB_OD / 2 - 3, tab_y + 4.5, z - 5, z + 5))
        body = body.union(box(-6, 0, -tab_y - 4.5, -(STUB_OD / 2 - 3), z - 5, z + 5))
    cuts = [
        cq.Workplane("XY").workplane(offset=STUB_START + 3).circle(STUB_ID / 2).extrude(top - STUB_END_T - STUB_START - 3),
        cyl_x(R2_CAV, -CAV2_HALF, 0.1),
        cyl_x(R2_LEDGE, -CAV2_HALF - WALL2 - 0.1, -CAV2_HALF + 0.1),
        cyl_x(BRG_D / 2 + FIT, -HUB2_HALF - 0.1, -CAV2_HALF - WALL2),
        box(-CAV2_HALF + 0.5, 0.1, -6, 6, 5, STUB_START + 4),
    ]
    cuts.append(cq.Workplane("YZ").workplane(offset=-CAV2_HALF).moveTo(0, 0)
                .lineTo(*polar_yz(R2_HUB + 3, -90 - RANGE2))
                .threePointArc(polar_yz(R2_HUB + 3, -90), polar_yz(R2_HUB + 3, -90 + RANGE2))
                .close().extrude(CAV2_HALF + 0.1))
    cuts.append(groove_ring(-G2_X, R2_GROOVE, R2_HUB, GROOVE_W))
    # tendon arrives at phi_t (not 0 like J1) -> anchor 135 deg further round
    cuts.append(chord_anchor(-G2_X, R2_GROOVE, R2_HUB, phi_t + 135.0, +1, CORD_D))
    for z in TAB2_Z:
        for s in (1, -1):
            y = s * tab_y
            cuts.append(hole_along((-7, y, z), (1, y, z), M3_CLR))
            cuts.append(hole_along((-7, y, z), (-3.5, y, z), M3_HEAD_D) if s > 0
                        else hex_prism_x(-7, -3.5, y, z, M3_NUT_AF))
    # end plate: centre hole for a future gripper cord + 4x M3 (gripper mount)
    cuts.append(cq.Workplane("XY").workplane(offset=top - STUB_END_T - 1).circle(4.0).extrude(STUB_END_T + 2))
    for y in (-9.0, 9.0):
        cuts.append(cq.Workplane("XY").workplane(offset=top - STUB_END_T - 1).center(-9, y).circle(M3_CLR / 2).extrude(STUB_END_T + 2))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- J2 mount
def j2_mount():
    z_bot = -J2_H
    z_plate = z_bot + PLATE_T
    lug_in = HUB2_HALF + LUG2_GAP
    lug_out = lug_in + LUG2_T
    plate = box(-PLATE_X, PLATE_X, -PLATE_Y, PLATE_Y, z_bot, z_plate).edges("|Z").fillet(5)
    boss = box(-BOSS_X, BOSS_X, -BOSS_Y, BOSS_Y, z_plate - 0.5, BOSS_TOP)
    body = plate.union(boss)
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        body = body.union(box(x0, x1, -LUG2_R, LUG2_R, z_plate - 0.5, 0).union(cyl_x(LUG2_R, x0, x1)))

    (ay, az), (ty, tz, _) = anchor_and_tangent()
    u = cq.Vector(0, ty - ay, tz - az).normalized()       # cord direction leaving the tube
    cuts = []
    for sx, sy in ((-1, 1), (1, -1)):                        # flexor at -X/+Y, extensor at +X/-Y
        x = sx * G2_X
        A = cq.Vector(x, sy * ay, az)
        d = cq.Vector(0, sy * u.y, u.z)
        B = A - d * SOCKET_DEPTH                             # tube end stop
        C = A - d * ((az - z_bot + 2) / u.z)                 # tube enters from below the plate
        cuts.append(hole_along(B.toTuple(), (A + d * 1).toTuple(), TUBE_ID + 0.4))
        cuts.append(hole_along(C.toTuple(), B.toTuple(), TUBE_OD + 0.3))
    # centre: spare tube / gripper cord
    cuts.append(cq.Workplane("XY").workplane(offset=z_bot - 1).circle(TUBE_OD / 2 + 0.25).extrude(BOSS_TOP - z_bot + 2))
    # mounting holes to the J1 flange (+ counterbore for the heads)
    for x in (-S.MOUNT_BOLT, S.MOUNT_BOLT):
        for y in (-S.MOUNT_BOLT, S.MOUNT_BOLT):
            cuts.append(cq.Workplane("XY").workplane(offset=z_bot - 1).center(x, y).circle(M3_CLR / 2).extrude(PLATE_T + 2))
    # M8 stub axles + captive nuts
    for s in (1, -1):
        x0, x1 = sorted((s * lug_in, s * lug_out))
        cuts.append(cyl_x(M8_CLR / 2, x0 - 1, x1 + 1))
        xm = (x0 + x1) / 2
        cuts.append(box(xm - M8_NUT_T / 2, xm + M8_NUT_T / 2, -M8_NUT_AF / 2, M8_NUT_AF / 2,
                        -M8_NUT_AF / 2 / math.cos(math.pi / 6), LUG2_R + 1))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- Bowden drive module
# local frame: spool axis = X, spool centre at origin, tubes arrive from +Z
BD_WALL_X = S.MOTOR_WALL_X             # same motor/spool spacing as the J1-S base
BD_WALL_T = S.MOTOR_WALL_T
BD_END_X = 21.0
BD_BAR_Z = (24.0, 34.0)
BD_FOOT_Z = -36.0
BD_Y = 26.0
BD_SOCK_X = 9.0                         # centre of each helical section on the spool


def bowden_drive():
    xm0 = BD_WALL_X - BD_WALL_T
    wall = box(xm0, BD_WALL_X, -BD_Y, BD_Y, BD_FOOT_Z, BD_BAR_Z[1])
    end = box(BD_END_X, BD_END_X + 5, -12, 12, BD_FOOT_Z, BD_BAR_Z[1])
    bar = box(BD_WALL_X - 0.5, BD_END_X + 0.5, -12, 12, *BD_BAR_Z)
    # foot runs under the motor too, so the module can't tip over
    foot = box(xm0 - 44, BD_END_X + 5, -BD_Y, BD_Y, BD_FOOT_Z, BD_FOOT_Z + 5)
    body = wall.union(end).union(bar).union(foot)
    cuts = []
    # motor: pilot slot + 4 slotted holes (slide DOWN = away from the tubes = tension)
    x0 = xm0 - 1
    cuts.append(cq.Workplane("YZ").workplane(offset=x0).center(0, -S.SLOT / 2)
                .slot2D(S.NEMA_PILOT + 1 + S.SLOT, S.NEMA_PILOT + 1, 90).extrude(BD_WALL_T + 2))
    for dy in (-S.NEMA_HOLE / 2, S.NEMA_HOLE / 2):
        for dz in (-S.NEMA_HOLE / 2, S.NEMA_HOLE / 2):
            cuts.append(cq.Workplane("YZ").workplane(offset=x0).center(dy, dz - S.SLOT / 2)
                        .slot2D(S.SLOT + 3.4, 3.4, 90).extrude(BD_WALL_T + 2))
    # tube sockets: flexor above the -X helix at +Y, extensor above the +X helix at -Y
    r = S.R_CORD_SPOOL
    for x, y in ((-BD_SOCK_X, r), (BD_SOCK_X, -r)):
        cuts.append(cq.Workplane("XY").workplane(offset=BD_BAR_Z[1] - 6).center(x, y).circle(TUBE_OD / 2 + 0.15).extrude(7))
        cuts.append(cq.Workplane("XY").workplane(offset=BD_BAR_Z[0] - 1).center(x, y).circle(TUBE_ID / 2 + 0.2).extrude(10))
    # foot screw holes
    for x in (xm0 - 38, BD_END_X - 2):
        for y in (-BD_Y + 6, BD_Y - 6):
            cuts.append(cq.Workplane("XY").workplane(offset=BD_FOOT_Z - 1).center(x, y).circle(M3_CLR / 2).extrude(7))
    for c in cuts:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- assembly placement
def j2_to_j1_distal(wp):
    """Place a J2-frame object in J1's distal frame (on the mount flange)."""
    top = S.DIST_START + S.DIST_LEN
    return wp.rotate((0, 0, 0), (0, 0, 1), J2_MOUNT_ROT).translate((0, 0, top + J2_H))


def assembly(a1=0.0, a2=0.0):
    tb = S.tube_bot()
    d1 = S.distal_half("mount")
    d2 = j2_distal_half()
    j2 = [j2_mount()] + [x.rotate((0, 0, 0), (1, 0, 0), a2) for x in (d2, d2.rotate((0, 0, 0), (0, 0, 1), 180))]
    dist1 = [d1, d1.rotate((0, 0, 0), (0, 0, 1), 180)] + [j2_to_j1_distal(x) for x in j2]
    dist1 = [x.rotate((0, 0, 0), (1, 0, 0), a1) for x in dist1]
    return {
        "j1_distal_a": dist1[0], "j1_distal_b": dist1[1],
        "j2_mount": dist1[2], "j2_distal_a": dist1[3], "j2_distal_b": dist1[4],
        "proximal": S.proximal("through"),
        "base": S.base().translate((0, 0, tb)),
        "spool1": S.spool().rotate((0, 0, 0), (1, 0, 0), a1 * S.RATIO).translate((0, 0, S.spool_z())),
        "bowden_drive": bowden_drive().translate((0, 130, tb - 40)),
        "spool2": S.spool().rotate((0, 0, 0), (1, 0, 0), a2 * RATIO2).translate((0, 130, tb - 40)),
    }


def export(shape, name):
    OUT.mkdir(exist_ok=True)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, str(OUT / f"{name}.step"))


if __name__ == "__main__":
    export(S.distal_half("mount").rotate((0, 0, 0), (0, 1, 0), 90), "j1_distal_half_mount")
    export(S.proximal("through"), "j1_proximal_through")
    export(j2_mount().rotate((0, 0, 0), (0, 0, 1), 0), "j2_mount")                    # plate down
    export(j2_distal_half().rotate((0, 0, 0), (0, 1, 0), 90), "j2_distal_half")       # split face down
    export(bowden_drive(), "bowden_drive")                                           # foot down
    asm = assembly(30, 40)
    cq.exporters.export(cq.Compound.makeCompound([v.val() for v in asm.values()]), str(OUT / "assembly_preview.step"))
    (ay, az), (ty, tz, phi) = anchor_and_tangent()
    print(f"J2 ratio {RATIO2:.2f}, tube exit A=({ay},{az}) tangent T=({ty:.1f},{tz:.1f}) phi={phi:.1f} -> {OUT}")
