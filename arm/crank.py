"""Hand-crank drive module (one per joint). Swap for a NEMA17 later.

Local frame: shaft along X through the origin, Z up, Bowden tubes arrive from above.

* FRAME: two walls with 608 pockets, foot, top beam. Wall B (+X) carries a ring of
  36 index holes: a pin through the crank arm locks the joint every 10 deg of crank.
  The beam underside is a 48 deg roof (only a 7 mm flat strip is bridged) and the
  bearing holes are steep teardrops, so the frame prints foot-down without supports.
* SPOOL: one solid, printed standing on its -X boss. Two helical grooves
  (cord A on -X, cord B on +X): rounded 90 deg V, 0.5 mm deep (cord half buried),
  pitch 1.9, ~3.4 turns each, between flanges whose undersides are 48 deg cones. Clamped between the two 608 inner races by the M8 shaft bolt.
* CRANK ARM: M8 bolt head sits in its hex pocket. Knob turns on an M4 bolt.
* NUT CUP: holds the M8 nut on the -X side and clamps the stack.
* SOCKET BLOCKS A and B: one per cord, so each cord's length / pretension is set
  on its own. The space between the walls is split at y=0: block A (y>0) holds the
  tube of cord A right above its helix tangent, block B (y<0) that of cord B. Each
  block is pulled up by ONE M4 screw through the beam into a captive nut, 10 mm of
  travel. Guiding: the block fills the gap between the two walls (0.3 mm each side),
  so it cannot turn about Z or tilt sideways; its screw sits at the SAME y as its
  cord, so cord tension makes no moment about X; the remaining moment about Y
  (screw 9 mm beside the cord, in X) is taken by the walls. The screw is within
  4 mm of the block centre, so the drawer effect cannot self-lock (that would need
  an offset > h / (2 mu) ~ 10 mm). The block top follows the roof (48 deg), so the
  blocks are tall in the middle where the nuts are.
* TUBE CLIP: sits on the beam top under both tension screw heads (they hold it) and
  guides both tubes vertically out of the beam for 15 mm.
"""
import math
import cadquery as cq

from . import params as P
from .cad import box, cut_all, cyl_x, cyl_z, hex_x, hex_z, polar, rod, yz

R_SPOOL = 8.0                  # cord centre radius on the spool (= effective drum radius)
PITCH = 1.9                    # 1.42 mm groove + 0.48 mm flat land
SLOPE = 1.1                    # rise per mm of radius on every downward-facing cone (48 deg)
SPOOL_HALF = 16.5              # bosses touch the inner races here
CORE_R = R_SPOOL               # groove core: the cord sits half buried (groove 0.5 mm deep)
GROOVE_BOTTOM = R_SPOOL - P.CORD_D / 2   # rounded 90 deg V: cord radius at the bottom, 45 deg flanks
FLANGE_R = 10.3
BOSS_R = 6.0                   # touches only the 608 inner race
BOSS_L = 0.5
FLAT = 1.3                     # flat rim of every flange
TOP_SLOPE = 0.8                # top (+X) outer cone faces up when printed: any slope prints
_C0 = (FLANGE_R - BOSS_R) * SLOPE       # bottom outer cone length
_CF = (FLANGE_R - CORE_R) * SLOPE       # core -> flange cone length
_CT = (FLANGE_R - BOSS_R) * TOP_SLOPE   # top outer cone length
_L = (2 * SPOOL_HALF - 2 * BOSS_L - _C0 - _CT - 3 * FLAT - 2 * _CF) / 2   # each helical section
_A0 = -SPOOL_HALF + BOSS_L + _C0 + FLAT
SEC_A = (_A0, _A0 + _L)                 # helical section for cord A (x range of the core)
SEC_B = (SEC_A[1] + _CF + FLAT, SEC_A[1] + _CF + FLAT + _L)
WALL_T = 9.0
WALL_IN = SPOOL_HALF           # inner faces of the walls
WALL_OUT = WALL_IN + WALL_T
FOOT_Z = (-78.0, -72.0)          # shaft high enough for the crank to clear the table
BEAM_Z = (48.0, 54.0)          # beam top plate; the roof below it starts at the walls
ROOF_FLAT = 3.5                # half width of the bridged strip at the top of the roof
SOCK_X = (SEC_A[0] + SEC_A[1]) / 2   # cord A socket (centre of its helix)
SOCK_B = (SEC_B[0] + SEC_B[1]) / 2   # cord B socket
BLOCK_TRAVEL = 10.0            # tension travel of each socket block
BLOCK_GAP = 0.5                # block top -> roof underside at the top of the travel
BLOCK_SIDE = 0.3               # clearance to each wall and between the two blocks
BLOCK_Y = 14.0                 # outer y end of the blocks (= roof edge)
BLOCK_Z = (16.0, 48.0 - BLOCK_GAP - BLOCK_TRAVEL)   # bottom (lowest position), top at the centre
TENSION_DX = 9.0               # tension screw beside its socket, towards the centre (x)
TENSION = ((SOCK_X + TENSION_DX, R_SPOOL), (SOCK_B - TENSION_DX, -R_SPOOL))   # (x, y) screw A, screw B
CLIP_T = 3.0                   # tube clip plate on the beam top (under the screw heads)
CLIP_SLEEVE_H = 15.0
CLIP_BORE_R = P.TUBE_OD / 2 + 0.3    # tubes slide freely
INDEX_R, INDEX_N = 32.0, 36
INDEX_DEPTH = 6.0
ARM_R = 62.0
ARM_X = (WALL_OUT + 0.5, WALL_OUT + 8.5)


def _teardrop_x(r, x0, x1, phi=50.0, alpha=52.0):
    """Horizontal hole along X that prints without support: the circle above +-phi
    (from the top) is replaced by a point whose sides rise at alpha (> 45 deg)."""
    c = cyl_x(r, x0, x1)
    s, co = math.sin(math.radians(phi)), math.cos(math.radians(phi))
    h = r * co + r * s * math.tan(math.radians(alpha))
    tri = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(-r * s, r * co), (0, h), (r * s, r * co), (0, 0)]).close()
           .extrude(x1 - x0))
    return c.union(tri)


def _revolve_x(pts):
    """Solid of revolution about the X axis from (x, r) profile points."""
    return cq.Workplane("XY").polyline(pts).close().revolve(360, (0, 0, 0), (1, 0, 0))


def _spool_profile(r_core=CORE_R):
    """(x, r) outline of the spool, -X end first. Every step that faces -X (= down
    when printed standing) is a cone with rise SLOPE per mm of radius. With r_core <
    CORE_R the helical sections are left as recesses (filled by the grooved cores)."""
    xa0, xa1 = SEC_A
    xb0, xb1 = SEC_B
    xc = xb1 + _CF + FLAT
    return [
        (-SPOOL_HALF, 0), (-SPOOL_HALF, BOSS_R), (-SPOOL_HALF + BOSS_L, BOSS_R), (xa0 - FLAT, FLANGE_R),
        (xa0, FLANGE_R), (xa0, r_core), (xa1, r_core), (xa1, CORE_R), (xa1 + _CF, FLANGE_R), (xb0, FLANGE_R),
        (xb0, r_core), (xb1, r_core), (xb1, CORE_R), (xb1 + _CF, FLANGE_R), (xc, FLANGE_R),
        (xc + _CT, BOSS_R), (SPOOL_HALF, BOSS_R), (SPOOL_HALF, 0),
    ]


def _helix_angle(t):
    """Global angle (from +Y towards +Z) of the groove at helix parameter t (turns)."""
    return -90.0 + 360.0 * t


def _groove_profile():
    """Rounded 90 deg V in the (radius, axial) plane of the helix start: a cord-radius
    arc at the bottom, 45 deg flanks tangent to it up to just past the core surface,
    then straight out (no overlap between neighbouring turns)."""
    rc = P.CORD_D / 2
    t = rc * math.sqrt(0.5)                      # tangent point offsets (45 deg)
    r_f = CORE_R + 0.05                          # flanks end just outside the core
    w = t + (r_f - (R_SPOOL - t))                # half width there
    r_out = CORE_R + 1.0
    return (cq.Workplane("XZ").moveTo(r_out, -w).lineTo(r_f, -w).lineTo(R_SPOOL - t, -t)
            .threePointArc((GROOVE_BOTTOM, 0), (R_SPOOL - t, t)).lineTo(r_f, w).lineTo(r_out, w).close())


def spool():
    # The grooved cores are cut as separate rings (built along Z, like the helix) and
    # fused in: cutting the helix straight out of the revolved body, or out of a ring
    # built along X, makes OCC return a broken shell or silently skip the cut.
    r_rec = BOSS_R - 1.0
    body = _revolve_x(_spool_profile(r_rec))
    for a, b in (SEC_A, SEC_B):
        h = b - a - 1.0
        path = cq.Wire.makeHelix(PITCH, h, R_SPOOL - 3.0)   # spine inside the core; profile stays at R_SPOOL
        g = _groove_profile().sweep(cq.Workplane().add(path), isFrenet=True)
        ring = cyl_z(CORE_R, -0.5, h + 0.5).cut(cyl_z(r_rec - 0.5, -1.5, h + 1.5))
        core = ring.cut(g)
        assert ring.val().Volume() - core.val().Volume() > 10, "groove cut failed"
        body = body.union(core.rotate((0, 0, 0), (0, 1, 0), 90).translate((a + 0.5, 0, 0)))
    cuts = [cyl_x(P.M8_CLR / 2, -SPOOL_HALF - 1, SPOOL_HALF + 1)]
    # cord anchor holes: axial, through the outer flange at the outer end of each
    # helix; the knot sits on the outer cone, clear of the bearing.
    ra = R_SPOOL + 0.55
    turns = (SEC_B[1] - SEC_B[0] - 1.0) / PITCH
    for x0, x1, ang in ((-SPOOL_HALF, SEC_A[0] + 0.5, _helix_angle(0)),
                        (SEC_B[1] - 0.5, SPOOL_HALF, _helix_angle(turns))):
        y, z = yz(ra, ang)
        cuts.append(rod((x0 - 0.5, y, z), (x1 + 0.5, y, z), 1.5))
    return cut_all(body, cuts)


def _roof():
    """Material under the beam plate between the walls: 48 deg slopes from each
    wall up to a narrow flat strip (the only bridge, 2*ROOF_FLAT wide)."""
    z_top = BEAM_Z[0]
    z_wall = z_top - (WALL_IN - ROOF_FLAT) * SLOPE
    return (cq.Workplane("XZ")
            .polyline([(-WALL_IN - 0.2, z_wall), (-ROOF_FLAT, z_top), (ROOF_FLAT, z_top),
                       (WALL_IN + 0.2, z_wall), (WALL_IN + 0.2, z_top + 0.2), (-WALL_IN - 0.2, z_top + 0.2)])
            .close().extrude(14.0, both=True))


def roof_z(x):
    """Underside of the roof at x (between the walls)."""
    return BEAM_Z[0] - max(0.0, abs(x) - ROOF_FLAT) * SLOPE


def frame():
    yw, zw = 40.0, 40.0
    wall_b = box(WALL_IN, WALL_OUT, -yw, yw, FOOT_Z[0], max(zw, BEAM_Z[1]))
    wall_a = box(-WALL_OUT, -WALL_IN, -22, 22, FOOT_Z[0], BEAM_Z[1])
    foot = box(-WALL_OUT, WALL_OUT, -yw, yw, *FOOT_Z)
    beam = box(-WALL_OUT, WALL_OUT, -14, 14, *BEAM_Z)
    body = wall_a.union(wall_b).union(foot).union(beam).union(_roof())
    cuts = []
    for sgn in (1, -1):
        x_in, x_out = (WALL_IN, WALL_OUT) if sgn > 0 else (-WALL_OUT, -WALL_IN)
        pocket = (_teardrop_x(P.B608[1] / 2 + P.FIT, WALL_IN - 0.1, WALL_IN + P.B608[2]) if sgn > 0
                  else _teardrop_x(P.B608[1] / 2 + P.FIT, -WALL_IN - P.B608[2], -WALL_IN + 0.1))
        cuts.append(pocket)
        cuts.append(_teardrop_x(9.5, x_in - 1, x_out + 1))
    # index holes (blind, INDEX_DEPTH deep from the outer face of wall B; 3 mm floor)
    for k in range(INDEX_N):
        y, z = yz(INDEX_R, 360.0 / INDEX_N * k)
        if z + 2.5 > max(zw, BEAM_Z[1]) or z - 2.5 < FOOT_Z[1]:
            continue
        cuts.append(cyl_x(2.2, WALL_OUT - INDEX_DEPTH, WALL_OUT + 1).translate((0, y, z)))
    # tension screws through the roof + beam (one per socket block)
    for x, y in TENSION:
        cuts.append(cyl_z(P.M4_CLR / 2, roof_z(x) - 5, BEAM_Z[1] + 1).translate((x, y, 0)))
    # tube pass holes in the beam (tubes go through to the block)
    for x, y in ((SOCK_X, R_SPOOL), (SOCK_B, -R_SPOOL)):
        cuts.append(cyl_z(P.TUBE_OD / 2 + 1.0, roof_z(x) - 5, BEAM_Z[1] + 1).translate((x, y, 0)))
    # foot screw holes
    for x in (-18.0, 18.0):
        for y in (-32.0, 32.0):
            cuts.append(cyl_z(P.M4_CLR / 2, FOOT_Z[0] - 1, FOOT_Z[1] + 1).translate((x, y, 0)))
    # lighten wall A: window with a pointed (48 deg) top
    zt = -16.0
    win = (cq.Workplane("YZ").workplane(offset=-WALL_OUT - 1)
           .polyline([(-14, FOOT_Z[1] + 4), (14, FOOT_Z[1] + 4), (14, zt - 11 * SLOPE),
                      (3.0, zt), (-3.0, zt), (-14, zt - 11 * SLOPE)]).close()
           .extrude(WALL_T + 2))
    cuts.append(win)
    return cut_all(body, cuts)


def _block_outline(lift=0.0):
    """XZ outline of a socket block (both have the same): flat bottom, sides along the
    walls, top parallel to the roof (BLOCK_GAP + BLOCK_TRAVEL below it)."""
    x1 = WALL_IN - BLOCK_SIDE
    top = lambda x: roof_z(x) - BLOCK_GAP - BLOCK_TRAVEL + lift
    z0 = BLOCK_Z[0] + lift
    return [(-x1, z0), (x1, z0), (x1, top(x1)), (ROOF_FLAT, top(ROOF_FLAT)),
            (-ROOF_FLAT, top(ROOF_FLAT)), (-x1, top(x1))]


def socket_block_ab(which, lift=0.0):
    """Socket block for cord 'A' (y > 0) or 'B' (y < 0), at height lift above its
    lowest position. Prints bottom down (nut trap open to the bed)."""
    sock, (tx, ty) = ((SOCK_X, R_SPOOL), TENSION[0]) if which == "A" else ((SOCK_B, -R_SPOOL), TENSION[1])
    ya, yb = (BLOCK_SIDE, BLOCK_Y) if which == "A" else (-BLOCK_Y, -BLOCK_SIDE)
    body = cq.Workplane("XZ").polyline(_block_outline(lift)).close().extrude(-(yb - ya)).translate((0, ya, 0))
    z0 = BLOCK_Z[0] + lift
    top = lambda x: roof_z(x) - BLOCK_GAP - BLOCK_TRAVEL + lift
    sx, sy = sock
    r_tube, r_scr = P.TUBE_OD / 2 + 0.15, P.M4_CLR / 2 - 0.3
    # flat spot faces round the holes in the sloped top (no knife edges at the rims)
    zt = top(abs(sx) + r_tube + 1.2)
    zs = top(abs(tx) + r_scr + 1.2)
    cuts = [cyl_z(r_tube + 1.2, zt, 60).translate((sx, sy, 0)),
            cyl_z(r_scr + 1.2, zs, 60).translate((tx, ty, 0)),
            cyl_z(r_tube, zt - 7, zt + 1).translate((sx, sy, 0)),                   # tube socket, 7 deep
            cyl_z(P.TUBE_ID / 2 + 0.2, z0 - 1, zt).translate((sx, sy, 0)),          # cord
            cyl_z(r_scr, z0 - 1, 60).translate((tx, ty, 0)),                        # tension screw (thread clearance)
            # captive nut from below, flats facing +-y (>= 2.3 mm walls all round)
            hex_z(z0 - 1, z0 + P.M4_NUT_T + 0.3, tx, ty, P.M4_NUT_AF + 0.2)]
    return cut_all(body, cuts)


def socket_block_a(lift=0.0):
    return socket_block_ab("A", lift)


def socket_block_b(lift=0.0):
    return socket_block_ab("B", lift)


def socket_block(lift=0.0):
    """Both blocks together, in place (for the assembly / viewer)."""
    return socket_block_a(lift).add(socket_block_b(lift)).combine(clean=False)


def tube_clip():
    """Plate on the beam top, held by both tension-screw heads (it is their washer),
    with a sleeve over each tube hole that keeps the tube vertical for 15 mm before
    it bends away (free bend, R >= 25 mm). Prints plate down."""
    z0, z1 = BEAM_Z[1], BEAM_Z[1] + CLIP_T
    pts = [(SOCK_X, R_SPOOL), (SOCK_B, -R_SPOOL)]
    xs = [p[0] for p in pts + list(TENSION)]
    body = box(min(xs) - 5.0, max(xs) + 5.0, -R_SPOOL - 5.0, R_SPOOL + 5.0, z0, z1).edges("|Z").fillet(3.0)
    sleeve_r = CLIP_BORE_R + 1.8
    for x, y in pts:
        body = body.union(cyl_z(sleeve_r, z1 - 0.1, z1 + CLIP_SLEEVE_H).translate((x, y, 0)))
    cuts = []
    for x, y in pts:
        cuts.append(cyl_z(CLIP_BORE_R, z0 - 1, z1 + CLIP_SLEEVE_H + 1).translate((x, y, 0)))
        # entry flare at the top (45 deg cone, faces up when printed)
        cuts.append(cq.Workplane("XY").workplane(offset=z1 + CLIP_SLEEVE_H - 0.5).center(x, y)
                    .circle(CLIP_BORE_R).workplane(offset=0.51).circle(CLIP_BORE_R + 0.51).loft())
    for x, y in TENSION:
        cuts.append(cyl_z(P.M4_CLR / 2, z0 - 1, z1 + 1).translate((x, y, 0)))
    return cut_all(body, cuts)


def crank_arm():
    x0, x1 = ARM_X
    hub = cyl_x(13.0, x0, x1)
    boss = cyl_x(6.0, WALL_IN + P.B608[2], x0 + 0.1)       # presses the 608-B inner race
    arm = box(x0, x1, -7, 7, 0, ARM_R).union(cyl_x(7.0, x0, x1).translate((0, 0, ARM_R)))
    body = hub.union(boss).union(arm)
    cuts = [cyl_x(P.M8_CLR / 2, WALL_IN, x1 + 1),
            hex_x(x1 - P.M8_HEAD_T - 0.3, x1 + 1, 0, 0, P.M8_HEAD_AF),
            cyl_x(2.25, x0 - 1, x1 + 1).translate((0, 0, INDEX_R)),          # lock pin
            cyl_x(P.M4_CLR / 2, x0 - 1, x1 + 1).translate((0, 0, ARM_R)),     # knob bolt
            hex_x(x0 - 1, x0 + P.M4_NUT_T + 0.3, 0, ARM_R, P.M4_NUT_AF)]
    return cut_all(body, cuts)


NUT_CUP_T = 10.0               # disc thickness outside wall A (nut pocket + 2.9 mm roof)


def nut_cup():
    x1 = -(WALL_IN + P.B608[2])
    x0 = -WALL_OUT - 0.5 - NUT_CUP_T
    body = cyl_x(6.0, -WALL_OUT - 0.5, x1).union(cyl_x(12.0, x0, -WALL_OUT - 0.5))
    cuts = [cyl_x(P.M8_CLR / 2, x0 - 1, x1 + 1), hex_x(x0 - 1, x0 + P.M8_NUT_T + 0.3, 0, 0, P.M8_NUT_AF)]
    return cut_all(body, cuts)


LOCK_PIN_L = (ARM_X[1] - ARM_X[0]) + 0.5 + INDEX_DEPTH - 0.5   # arm + gap + hole, 0.5 short


def lock_pin():
    return cyl_z(2.0, 0, LOCK_PIN_L).union(cyl_z(6.0, LOCK_PIN_L, LOCK_PIN_L + 4)).edges(">Z").fillet(1.0)


def knob():
    return cyl_z(7.0, 0, 26).cut(cyl_z(P.M4_CLR / 2 + 0.2, -1, 27)).edges().fillet(1.0)


def tangent_points():
    """Cord exits (bottom of the socket block) and spool tangents for cords A and B."""
    return [((SOCK_X, R_SPOOL, BLOCK_Z[0]), (SOCK_X, R_SPOOL, 0.0)),
            ((SOCK_B, -R_SPOOL, BLOCK_Z[0]), (SOCK_B, -R_SPOOL, 0.0))]
