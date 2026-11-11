"""The arm's joints. Ranges are in degrees from the straight (hanging) pose.

tubes_through counts the Bowden tubes of the joints further out + the gripper tube
(it runs through every joint, S1..W2, and ends in the gripper palm)."""
from . import params as P
from .hinge import HingeSpec
from .roll import RollSpec

S1 = HingeSpec("S1", "L", R=42.0, cav=14.0, lo=-30, hi=120, lug_r=17, slot_y=6.5, base_style="torso", tubes_through=13, neck_extra=12)
S2 = HingeSpec("S2", "L", R=42.0, cav=12.0, lo=-10, hi=100, lug_r=17, slot_y=6.5, tubes_through=11)
# elbow + wrist: 625 bearings on M5 stub axles (they see <= 135 N; a 625 is rated 670 N static)
E = HingeSpec("E", "S", R=36.0, cav=8.0, lo=0, hi=115, lug_r=12, lug_t=10, slot_y=6.5, tubes_through=7, rotor_cls="S",
              bearing=P.B625, axle="M5")
W1 = HingeSpec("W1", "S", R=20.0, cav=5.0, lo=-60, hi=60, lug_r=10, lug_t=9, slot_y=6.5, tubes_through=3,
               bearing=P.B625, axle="M5")
W2 = HingeSpec("W2", "S", R=20.0, cav=4.0, lo=-30, hi=30, lug_r=10, lug_t=9, slot_y=4, tubes_through=1,
               bearing=P.B625, axle="M5")
HINGES = [S1, S2, E, W1, W2]

S3 = RollSpec("S3", "L", "M", bearing=(30.0, 42.0, 7.0), lo=-80, hi=80, tubes_through=9)   # 6806-2RS x2
F = RollSpec("F", "ML", "S", bearing=(25.0, 37.0, 7.0), lo=-85, hi=85, tubes_through=5)    # 6805-2RS x2
ROLLS = [S3, F]
