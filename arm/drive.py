"""Motor drive unit family (one per joint, 8 per arm, 16 in the torso).

Grown out of the hand crank (crank.py): the same helical spool (r 8 mm, cord A on
the -X helix, cord B on the +X helix), the same two socket blocks with one M4
tension screw each, the same roof and tube clip. What changes:

* the spool rides on a 608 (wall A, M8 bolt + nut cup, as on the crank) and on a
  6805 thin-section bearing (wall B) around a 25 mm journal. All cord load goes
  into these two bearings, none into the motor;
* SPOOL HUB (common to every motor): outside wall B the journal ends in a flange
  with two M3 drive pins (M3x12 screws in heat-set inserts, 6 mm proud, at r 12);
* HORN ADAPTER (one per motor type): bolts to the motor's output and takes the
  two pins in radial slots, so it couples torque only (an Oldham-like dog: small
  misalignment between the bearings and the motor shaft does not load either);
* MOTOR CUP (one per motor type): an end plate that holds the motor body plus four
  hollow pillars; four M3 screws run through the pillars into heat-set inserts in
  wall B at (y, z) = (+-22, +-25). Same pattern for every motor = upgrade path:
  swap adapter + cup, nothing else;
* a guide on the beam for the joint's magnetic-encoder cable, so it runs with the
  joint's two tubes.

Local frame (as crank.py): spool axis = X (motor on +X), tubes leave along +Z
from the tube clip at z = 72, foot underside at z = FOOT_Z[0].

Motor data (checked 2026-11; every number below says where it comes from):

* STS3215 (C001 7.4 V / C018 12 V): body 45.2 x 24.7 x 35 mm, 55 g, 25T spline
  (OD 5.9). Measured from Feetech's STEP model as shipped in the SO-ARM100 repo
  (STEP/SO100/STS3215_03a.step, TheRobotStudio/SO-ARM100): case 45.4 x 24.8, output
  axis 12.5 mm from the case centre on the long axis, Ø20 x 2.5 horn with 4x M3
  (Ø2.5 tapped) on a 9.8 mm square, horn top 2.8 mm above the case top, horn top
  to case bottom 34.6 mm, 4x M2 tapped holes in the bottom cover at
  (x 4.2 / -20.3, y +-10.2) from the case centre, two 3-pin connectors in the
  bottom cover at x -1.6, a Ø20 idler horn 1.8 mm proud of the bottom.
  Sizes 45.2 x 24.7 x 35 / 25T also on servodatabase.com/servo/feetech/sts3215
  and core-electronics.com.au (feetech-sts3215-smart-servo).
* STS3250: 45.22 x 24.72 x 35 mm (openelab.io feetech-sts3250-c001-ttl-servo),
  the STS3215 housing; same layout assumed (verify the horn on yours).
* STS3095 (C001/C002): 65 x 30 x 48 mm, 194.5 g, 15T spline Ø7.6 (openelab.io
  feetech-sts3095-c001-ttl-servo, rcdrone.top feetech-sts3095-servo-motor). The
  hole pattern, output offset and horn height are NOT published in any source I
  could read: the cup locates the case in a 2 mm pocket and straps it with two
  cable ties, the adapter has 4 radial slots (fits M3 patterns on Ø12-26). The
  output offset (17.5 mm from the case centre) and 4 mm horn height are ESTIMATES.
* Damiao DM-J4310-2EC V1.1: Ø56-57 x 46 mm, ~300 g (aifitlab DM-J4310-2EC V1.1
  instruction manual, openelab listing). Front face 6x M3 (5 deep) on Ø50, rear
  face 4x M3 (4 deep) on Ø38 (rcdrone.top damiao-dm-j4310-2ec-v1-1 listing, a
  reseller, not the official drawing): the adapter bolts to the front (output)
  pattern, the cup end plate to the rear pattern. Verify both against the drawing
  of your motor revision before printing.
"""
import math

import cadquery as cq

from . import crank as C
from . import params as P
from .cad import box, cut_all, cyl_x, cyl_z, hex_x, polar, yz

# ------------------------------------------------------------------ frame
FOOT_Z = (-34.0, -28.0)        # foot (the 6805 pocket reaches z = -18.8)
WALL_Y = 29.0                  # half width of the walls / foot (local y)
B6805 = (25.0, 37.0, 7.0)      # thin-section bearing for the spool journal (already in the BOM)
JOURNAL_R = B6805[0] / 2       # 12.5
LIP_R = JOURNAL_R + 0.6        # hole through wall B's 2 mm lip
FLANGE_X = (C.WALL_OUT + 0.5, C.WALL_OUT + 7.5)    # spool hub flange (26 .. 33)
FLANGE_R = 18.0
PIN_R, PIN_PROUD = 12.0, 4.0   # drive pins: M3 screws at r 12, 4 mm proud of the flange
SLOT_D = 4.5                   # pin slot depth in the adapters
ADAPTER_T = 6.0
HORN_X = FLANGE_X[1] + 0.5 + ADAPTER_T     # 39.5: motor output face (adapter outer face)
CUP_PILLARS = [(sy * 22.0, sz * 25.0) for sy in (1, -1) for sz in (1, -1)]   # (y, z) on wall B
PILLAR_R = 3.6
PLATE_T = 4.0
FOOT_BOLTS = [(sx * 10.0, sy * 20.0) for sx in (1, -1) for sy in (1, -1)]  # M4 into the saddle
CABLE_GUIDE = (0.0, 19.5, 3.0)  # (x, y, r): encoder cable hole in a tab on the beam
TUBE_TOP = C.BEAM_Z[1] + C.CLIP_T + C.CLIP_SLEEVE_H   # 72: where the tubes leave the clip

# ------------------------------------------------------------------ motors
# Every motor frame: output axis = +X of the unit, output face at x = HORN_X.
# 'case' = (length along local z, width along local y, depth along x from the
# output face to the case bottom); 'zc' = case centre on local z (the case
# reaches towards the tube side, +z, away from the foot).
MOTORS = {
    "STS3215": dict(kind="sts", case=(45.4, 24.8, 34.6), zc=-12.5 + 22.7, mass=55.0,
                    holes=[(z, y) for z in (8.3, 32.8) for y in (10.2, -10.2)],
                    horn=[(4.9 * a, 4.9 * b) for a in (1, -1) for b in (1, -1)],
                    joints=("F", "W1", "W2", "G")),
    "STS3250": dict(kind="sts", case=(45.4, 24.8, 34.6), zc=-12.5 + 22.7, mass=72.0,
                    holes=[(z, y) for z in (8.3, 32.8) for y in (10.2, -10.2)],
                    horn=[(4.9 * a, 4.9 * b) for a in (1, -1) for b in (1, -1)],
                    joints=("S3", "E")),
    "STS3095": dict(kind="big", case=(65.4, 30.4, 4.0 + 48.0), zc=-17.5 + 32.7, mass=194.5,
                    joints=("S1", "S2")),
    "DM-J4310": dict(kind="dm", case=(57.0, 57.0, 46.0), zc=0.0, mass=300.0,
                     out_pcd=50.0, rear_pcd=38.0, joints=()),
}
PHASE1 = {j: m for m, d in MOTORS.items() for j in d["joints"]}   # joint -> motor (phase 1)


def motor_depth(m):
    return MOTORS[m]["case"][2]


def plate_x(m):
    """x of the motor cup end plate (its inner face)."""
    return HORN_X + motor_depth(m)


# ------------------------------------------------------------------ frame
def frame():
    """Drive frame, prints foot down. Wall A: 608 pocket (as the crank), wall B: 6805
    pocket + journal hole + 4 M3 inserts for the motor cup on its outer face."""
    WI, WO = C.WALL_IN, C.WALL_OUT
    z0, z1 = FOOT_Z[0], C.BEAM_Z[1]
    wall_b = box(WI, WO, -WALL_Y, WALL_Y, z0, z1)
    wall_a = box(-WO, -WI, -22, 22, z0, z1)
    foot = box(-WO, WO, -WALL_Y, WALL_Y, *FOOT_Z)
    beam = box(-WO, WO, -14, 14, *C.BEAM_Z)
    gx, gy, gr = CABLE_GUIDE
    tab = box(gx - 7, gx + 7, 13.9, gy + gr + 2.5, *C.BEAM_Z)
    body = wall_a.union(wall_b).union(foot).union(beam).union(C._roof()).union(tab)
    cuts = [C._teardrop_x(P.B608[1] / 2 + P.FIT, -WI - P.B608[2], -WI + 0.1),   # 608, wall A
            C._teardrop_x(9.5, -WO - 1, -WI + 1),                              # nut cup boss
            C._teardrop_x(B6805[1] / 2 + P.FIT, WI - 0.1, WI + B6805[2]),      # 6805, wall B
            C._teardrop_x(LIP_R, WI, WO + 1)]                                  # journal through the lip
    for y, z in CUP_PILLARS:                                                   # motor cup inserts
        cuts.append(cyl_x(P.INSERT_D / 2, WO - P.INSERT_DEPTH, WO + 1).translate((0, y, z)))
    for x, y in C.TENSION:                                                     # tension screws
        cuts.append(cyl_z(P.M4_CLR / 2, C.roof_z(x) - 5, C.BEAM_Z[1] + 1).translate((x, y, 0)))
    for x, y in ((C.SOCK_X, C.R_SPOOL), (C.SOCK_B, -C.R_SPOOL)):               # tubes
        cuts.append(cyl_z(P.TUBE_OD / 2 + 1.0, C.roof_z(x) - 5, C.BEAM_Z[1] + 1).translate((x, y, 0)))
    for x, y in FOOT_BOLTS:
        cuts.append(cyl_z(P.M4_CLR / 2, FOOT_Z[0] - 1, FOOT_Z[1] + 1).translate((x, y, 0)))
    cuts.append(cyl_z(gr, C.BEAM_Z[0] - 1, C.BEAM_Z[1] + 1).translate((gx, gy, 0)))   # encoder cable
    cuts.append(box(gx - 1.5, gx + 1.5, gy - gr - 4.5, gy - gr + 0.2, C.BEAM_Z[0] - 1, C.BEAM_Z[1] + 1))  # tie slot
    return cut_all(body, cuts)


# ------------------------------------------------------------------ spool hub
def _profile():
    """(x, r) outline: the crank spool up to its +X flange rim, then a 43 deg cone out to the
    6805 shoulder (r 13.3), the Ø25 journal and the hub flange."""
    pts = C._spool_profile(C.BOSS_R - 1.0)
    i = next(k for k, (x, r) in enumerate(pts) if abs(r - C.FLANGE_R) < 1e-9 and x > C.SEC_B[1] + C._CF + 0.1)
    xc = pts[i][0]
    WI = C.WALL_IN
    return pts[:i + 1] + [(15.0, 12.6), (15.0, 13.3), (WI - 0.1, 13.3), (WI - 0.1, JOURNAL_R - 0.05), (FLANGE_X[0], JOURNAL_R - 0.05),
                          (FLANGE_X[0], FLANGE_R), (FLANGE_X[1], FLANGE_R), (FLANGE_X[1], 0)]


def spool():
    """Drive spool = the crank spool's helices on a new body: a Ø25 journal for the 6805 and
    the hub flange with the two drive pins (M3 inserts). Prints standing on its -X boss
    (like the crank spool); every downward face is a <= 43 deg cone or the 5.5 mm flange ring."""
    r_rec = C.BOSS_R - 1.0
    body = C._revolve_x(_profile())
    for a, b in (C.SEC_A, C.SEC_B):              # same grooved cores as crank.spool()
        h = b - a - 1.0
        path = cq.Wire.makeHelix(C.PITCH, h, C.R_SPOOL - 3.0)
        g = C._groove_profile().sweep(cq.Workplane().add(path), isFrenet=True)
        ring = cyl_z(C.CORE_R, -0.5, h + 0.5).cut(cyl_z(r_rec - 0.5, -1.5, h + 1.5))
        core = ring.cut(g)
        body = body.union(core.rotate((0, 0, 0), (0, 1, 0), 90).translate((a + 0.5, 0, 0)))
    cuts = [cyl_x(P.M8_CLR / 2, -C.SPOOL_HALF - 1, FLANGE_X[1] + 1),
            hex_x(FLANGE_X[1] - P.M8_HEAD_T - 0.3, FLANGE_X[1] + 1, 0, 0, P.M8_HEAD_AF)]   # M8 head
    ra = C.R_SPOOL + 0.55                          # cord anchor holes, as the crank spool
    turns = (C.SEC_B[1] - C.SEC_B[0] - 1.0) / C.PITCH
    from .cad import rod
    for x0, x1, ang in ((-C.SPOOL_HALF, C.SEC_A[0] + 0.5, C._helix_angle(0)),
                        (C.SEC_B[1] - 0.5, C.WALL_IN - 0.5, C._helix_angle(turns))):
        y, z = yz(ra, ang)
        cuts.append(rod((x0 - 0.5, y, z), (x1 + 0.5, y, z), 1.5))
    for a in (90.0, 270.0):
        y, z = yz(PIN_R, a)
        cuts.append(cyl_x(P.INSERT_D / 2, FLANGE_X[1] - P.INSERT_DEPTH, FLANGE_X[1] + 1).translate((0, y, z)))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ horn adapters
def adapter(m):
    """Horn adapter for motor m: a disc between the hub flange and the motor output face
    (x = HORN_X - ADAPTER_T .. HORN_X). Two radial slots on the spool side take the drive
    pins. Prints with its motor face down (the slots and counterbores open upwards)."""
    d = MOTORS[m]
    x0, x1 = HORN_X - ADAPTER_T, HORN_X
    r = 28.6 if d["kind"] == "dm" else FLANGE_R      # DM: clears the cup pillars (r 29.7)
    body = cyl_x(r, x0, x1)
    cuts = []
    for sz in (1, -1):                            # pin slots (pins at y = 0, z = +-12), 5 deep
        cuts.append(box(x0 - 1, x0 + SLOT_D, -1.9, 1.9, *sorted((sz * (PIN_R - 3.5), sz * (PIN_R + 3.5)))))
    if d["kind"] == "sts":
        for y, z in d["horn"]:                    # 4x M3 into the horn, heads on the spool side
            cuts.append(cyl_x(P.M3_CLR / 2, x0 - 1, x1 + 1).translate((0, y, z)))
            cuts.append(cyl_x(P.M3_HEAD_D / 2 + 0.3, x0 - 1, x0 + 2.6).translate((0, y, z)))
        cuts.append(cyl_x(3.2, x1 - 2.0, x1 + 1))   # horn centre screw head
    elif d["kind"] == "big":                      # 4 radial M3 slots: patterns on Ø12 .. 26
        for a in (45.0, 135.0, 225.0, 315.0):
            for rr in (6.0, 7.5, 9.0, 10.5, 12.0, 13.0):
                y, z = yz(rr, a)
                cuts.append(cyl_x(P.M3_CLR / 2, x0 - 1, x1 + 1).translate((0, y, z)))
        cuts.append(cyl_x(5.0, x0 - 1, x1 + 1))
    else:                                         # Damiao: 6x M3 on the front pattern; the heads sit
        for k in range(6):                        # on the spool-side face, outside the hub flange
            y, z = yz(d["out_pcd"] / 2, 30.0 + 60.0 * k)
            cuts.append(cyl_x(P.M3_CLR / 2, x0 - 1, x1 + 1).translate((0, y, z)))
        cuts.append(cyl_x(6.0, x0 - 1, x1 + 1))
    return cut_all(body, cuts)


# ------------------------------------------------------------------ motor cup
def cup(m):
    """Motor cup for motor m: end plate on the motor's back + 4 hollow pillars to wall B.
    Prints end plate down (pillars vertical)."""
    d = MOTORS[m]
    xp = plate_x(m)
    L, W, _ = d["case"]
    zc = d["zc"]
    if d["kind"] == "dm":
        plate = cyl_x(31.0, xp, xp + PLATE_T)
    else:
        z_lo = min(zc - L / 2, -25.0 - PILLAR_R - 1.9) - 2.0
        z_hi = max(zc + L / 2, 25.0 + PILLAR_R + 1.9) + 2.0
        plate = box(xp, xp + PLATE_T, -27.5, 27.5, z_lo, z_hi)
    body = plate
    for y, z in CUP_PILLARS:
        body = body.union(cyl_x(PILLAR_R, C.WALL_OUT + 0.2, xp + 0.1).translate((0, y, z)))
    cuts = []
    for y, z in CUP_PILLARS:
        cuts.append(cyl_x(P.M3_CLR / 2, C.WALL_OUT - 1, xp + PLATE_T + 1).translate((0, y, z)))
        cuts.append(cyl_x(P.M3_HEAD_D / 2 + 0.3, xp + PLATE_T - 2.5, xp + PLATE_T + 1).translate((0, y, z)))
    if d["kind"] == "sts":
        for z, y in d["holes"]:                   # M2 screws into the bottom cover
            cuts.append(cyl_x(1.2, xp - 1, xp + PLATE_T + 1).translate((0, y, z)))
        cuts.append(cyl_x(11.5, xp - 1, xp + PLATE_T + 1))                     # idler horn
        cuts.append(box(xp - 1, xp + PLATE_T + 1, -9.0, 9.0, 11.0, 18.6))       # connectors
    elif d["kind"] == "big":
        cuts.append(box(xp - 0.01, xp + 2.0, -W / 2, W / 2, zc - L / 2, zc + L / 2))   # 2 mm locating pocket
        cuts.append(cyl_x(11.0, xp - 1, xp + PLATE_T + 1))
        for z in (zc - L / 2 + 12.0, zc + L / 2 - 12.0):                       # cable-tie slots
            for sy in (1, -1):
                cuts.append(box(xp - 1, xp + PLATE_T + 1, sy * (W / 2 + 1.0) - 1.5, sy * (W / 2 + 1.0) + 1.5, z - 2.5, z + 2.5))
    else:
        for k in range(4):
            y, z = yz(d["rear_pcd"] / 2, 45.0 + 90.0 * k)
            cuts.append(cyl_x(P.M3_CLR / 2, xp - 1, xp + PLATE_T + 1).translate((0, y, z)))
        cuts.append(cyl_x(12.0, xp - 1, xp + PLATE_T + 1))
    return cut_all(body, cuts)


def motor_body(m):
    """Simple stand-in solid of the motor (for collision checks and the viewer)."""
    d = MOTORS[m]
    L, W, D = d["case"]
    if d["kind"] == "dm":
        return cyl_x(L / 2, HORN_X, HORN_X + D)
    top = 2.8 if d["kind"] == "sts" else 4.0
    body = box(HORN_X + top, HORN_X + D, -W / 2, W / 2, d["zc"] - L / 2, d["zc"] + L / 2)
    return body.union(cyl_x(10.0, HORN_X, HORN_X + top))


# ------------------------------------------------------------------ assembly helpers
def tube_exits():
    """[(point, direction)] of the tubes leaving the unit: cord A, cord B (local frame)."""
    return [((C.SOCK_X, C.R_SPOOL, TUBE_TOP), (0, 0, 1)), ((C.SOCK_B, -C.R_SPOOL, TUBE_TOP), (0, 0, 1))]


def unit_parts(m, lift=0.0):
    """{name: workplane} of one complete drive unit with motor m, in the unit frame."""
    return {"frame": frame(), "spool": spool(), "socket_block": C.socket_block(lift), "tube_clip": C.tube_clip(),
            "nut_cup": C.nut_cup(), "adapter": adapter(m), "cup": cup(m), "motor": motor_body(m)}


def envelope(m):
    """(x0, x1, y0, y1, z0, z1) of a unit with motor m (local)."""
    d = MOTORS[m]
    L, W, _ = d["case"]
    if d["kind"] == "dm":
        yy, zz = (-31.0, 31.0), (-31.0, 31.0)
    else:
        yy, zz = (-25.6, 25.6), (min(d["zc"] - L / 2, -28.6) - 2, max(d["zc"] + L / 2, 28.6) + 2)
    return (-C.WALL_OUT - 0.5 - C.NUT_CUP_T, plate_x(m) + PLATE_T, min(yy[0], -WALL_Y), max(yy[1], WALL_Y),
            FOOT_Z[0], max(zz[1], TUBE_TOP))


def printed_parts():
    """name -> (workplane in PRINT orientation, qty, note) for export.py."""
    rotY = lambda w, a: w.rotate((0, 0, 0), (0, 1, 0), a)
    n = {m: 2 * len(d["joints"]) for m, d in MOTORS.items()}
    out = {
        "drive_frame": (frame(), 16, "drive unit frame - foot down (teardrop bearing pockets, 48 deg roof)"),
        "drive_spool": (rotY(spool(), -90), 16, "drive spool hub (crank helices + 6805 journal + pin flange) - "
                        "standing on its -X boss, 0.12 mm layers, brim"),
    }
    for m, d in MOTORS.items():
        k = m.lower().replace("-", "")
        q = n[m] if d["joints"] else 0
        tag = f"x{q} (phase 1: {', '.join(d['joints'])} on both arms)" if q else "upgrade path, print as needed"
        out[f"drive_adapter_{k}"] = (rotY(adapter(m), 90), q, f"horn adapter for {m} - motor face down; {tag}")
        out[f"drive_cup_{k}"] = (rotY(cup(m), 90), q, f"motor cup for {m} - end plate down; {tag}")
    return out
