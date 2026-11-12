"""Generic creep cycle used by the motion study (NOT the firmware gait).  duty 0.75, order FL, RR, FR, RL.
2-DOF coupling: stance = femur fixed at q=0 (foot on ground), coxa sweeps the foot back; swing = femur lifts,
coxa swings forward."""
import math
from . import kin

DUTY = 0.75
ORDER = [0, 2, 1, 3]                 # FL, RR, FR, RL  (indices into kin.LEG_NAMES)
FWD_SIGN = [-1, +1, +1, -1]          # sign of coxa yaw that moves the foot toward body +X
STRIDE_DEG = 20.0                    # coxa half sweep
LIFT_DEG = 30.0                      # femur lift at mid swing (<= 60)


def leg_state(i, t):
    """t in [0,1) cycle phase -> (psi, q) for leg i"""
    off = ORDER.index(i) / 4.0
    u = (t + off) % 1.0
    sw = 1.0 - DUTY
    if u < sw:                       # swing
        s = u / sw
        psi = FWD_SIGN[i] * STRIDE_DEG * (-1 + 2 * s)       # back -> front
        q = LIFT_DEG * math.sin(math.pi * s)
    else:                            # stance
        s = (u - sw) / DUTY
        psi = FWD_SIGN[i] * STRIDE_DEG * (1 - 2 * s)        # front -> back
        q = 0.0
    return psi, q


def pose(t):
    st = [leg_state(i, t) for i in range(4)]
    return tuple(s[0] for s in st), tuple(s[1] for s in st)


def foot_xy(i, psi, q):
    s = kin.solve_leg(q)
    x, y, z = kin.leg_to_body(i, s["foot"], psi)
    return x, y, z
