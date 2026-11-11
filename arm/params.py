"""Shared parameters for the tendon arm (all mm).

Change CORD_D to the Dyneema you actually have; grooves and holes follow.
"""

# --- tendons
CORD_D = 1.0                 # Dyneema / UHMWPE braid
TUBE_OD, TUBE_ID = 4.0, 2.0  # PTFE Bowden liner (standard 3D-printer tube)

# --- bearings (d, D, B)
B608 = (8.0, 22.0, 7.0)      # shoulder hinges (M8 stub axles)
B625 = (5.0, 16.0, 5.0)      # elbow + wrist hinges (M5 stub axles): 4.5 g vs 12 g
B623 = (3.0, 10.0, 4.0)      # idler pulleys in the roll joints
FIT = 0.15                   # radial clearance for pressed bearings

# --- fasteners
M3_CLR, M3_HEAD_D, M3_HEAD_H, M3_NUT_AF, M3_NUT_T = 3.4, 6.0, 3.0, 5.6, 2.6
M4_CLR, M4_HEAD_D, M4_NUT_AF, M4_NUT_T = 4.5, 7.6, 7.1, 3.4
M5_CLR, M5_NUT_AF, M5_NUT_T = 5.4, 8.0, 4.0
M8_CLR, M8_NUT_AF, M8_NUT_T, M8_HEAD_AF, M8_HEAD_T = 8.4, 13.2, 6.8, 13.2, 5.5
INSERT_D, INSERT_DEPTH = 4.0, 5.5   # M3 heat-set insert (or 2.6 mm pilot for self-tapping M3)

PLATE_T = 7.0                # module mounting plates (hold the inserts)
LIP_T = 5.0                  # link bolting lips

# --- flange classes.  tube_r: link tube outer radius at that end,
#     bc: bolt circle radius, od: lip/plate radius, n: bolts.
#     Links bolt with heads on their OUTWARD lips; modules carry heat-set inserts.
FLANGES = {
    # forearm-twist stator: its link bore covers the two inclined Bowden socket exits
    # (r <= 32.2 + tube 2.15 + 1.5 mm + 2 mm wall). The upper-arm stator (S3, sockets at
    # r <= 34.7) uses "L".
    "ML": dict(tube_r=38.0, bc=44.0, od=50.0, n=8),
    "L":  dict(tube_r=40.0, bc=46.0, od=52.0, n=8),   # shoulder hinges
    "M":  dict(tube_r=31.0, bc=37.0, od=43.0, n=8),   # upper arm / elbow
    "S":  dict(tube_r=22.0, bc=27.0, od=32.0, n=4),   # forearm / wrist
}


def bolt_angles(n):
    """Bolt angles in degrees; never on the 0/90/180/270 split and lug planes."""
    step = 360.0 / n
    return [step / 2 + k * step for k in range(n)]


# stub axles of the hinge modules: clearance hole, nut across flats, nut thickness
AXLES = {"M8": dict(clr=M8_CLR, nut_af=M8_NUT_AF, nut_t=M8_NUT_T),
         "M5": dict(clr=M5_CLR, nut_af=M5_NUT_AF, nut_t=M5_NUT_T)}
