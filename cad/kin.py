"""Derived geometry + leg kinematics (pure python, no CadQuery).  Leg plane = (r, z), t = 0."""
import math
from . import params as P

# ---- derived leg-plane points at neutral
A_Z = P.BRACKET_BOT_Z - P.SERVO_POCKET - P.MG996R["W"] / 2
A = (P.COXA_LEN, A_Z)
DVEC = (P.FOURBAR_D * math.cos(math.radians(P.FOURBAR_THETA)),
        P.FOURBAR_D * math.sin(math.radians(P.FOURBAR_THETA)))
B = (A[0] + DVEC[0], A[1] + DVEC[1])

HIPS = [(P.HIP, P.HIP), (P.HIP, -P.HIP), (-P.HIP, -P.HIP), (-P.HIP, P.HIP)]   # FL, FR, RR, RL
LEG_NAMES = ["FL", "FR", "RR", "RL"]
HEADINGS = [math.degrees(math.atan2(y, x)) for x, y in HIPS]


def phi(q):
    return P.PHI0 + q


def knee(q):
    a = math.radians(phi(q))
    return (A[0] + P.FEMUR_LEN * math.cos(a), A[1] + P.FEMUR_LEN * math.sin(a))


def circle_int(c0, r0, c1, r1, near):
    dx, dz = c1[0] - c0[0], c1[1] - c0[1]
    d = math.hypot(dx, dz)
    if d > r0 + r1 or d < abs(r0 - r1):
        raise ValueError("four-bar cannot close")
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    xm, zm = c0[0] + a * dx / d, c0[1] + a * dz / d
    sols = [(xm + h * dz / d, zm - h * dx / d), (xm - h * dz / d, zm + h * dx / d)]
    return min(sols, key=lambda s: math.hypot(s[0] - near[0], s[1] - near[1]))


def solve_leg(q, rocker_len=None, fixed=None):
    """Return dict with leg-plane points for femur servo angle q (deg).
    fourbar: K2 from the closed-loop constraint |B K2| = rocker_len, |K K2| = |AB|."""
    fixed = (P.TIBIA_MODE == "fixed") if fixed is None else fixed
    rocker_len = P.ROCKER_LEN if rocker_len is None else rocker_len
    K = knee(q)
    out = {"A": A, "B": B, "K": K, "q": q}
    if fixed:
        # tibia rigid with femur: whole thing rotates about A by q
        ang_tib = -90.0 + P.TIBIA_TILT + q
        out["tibia_dev"] = P.TIBIA_TILT + q
        out["K2"] = None
    else:
        K2_0 = (knee(0)[0] + DVEC[0], knee(0)[1] + DVEC[1])
        near = (K[0] + DVEC[0], K[1] + DVEC[1])
        K2 = circle_int(B, rocker_len, K, P.FOURBAR_D, near)
        cang = math.degrees(math.atan2(K2[1] - K[1], K2[0] - K[0]))
        delta = cang - P.FOURBAR_THETA
        ang_tib = -90.0 + P.TIBIA_TILT + delta
        out["K2"] = K2
        out["coupler_delta"] = delta
        out["tibia_dev"] = P.TIBIA_TILT + delta
    out["tibia_axis_deg"] = ang_tib
    a = math.radians(ang_tib)
    rc = P.TIBIA_LEN - P.FOOT_D / 2                      # pad sphere centre along the tibia axis
    out["foot_center"] = (K[0] + rc * math.cos(a), K[1] + rc * math.sin(a))
    flat = 0.0 if fixed else P.FOOT_FLAT              # the flat patch only faces down when the tibia stays near vertical
    out["foot"] = (out["foot_center"][0], out["foot_center"][1] - (P.FOOT_D / 2 - flat))   # lowest point = ground contact
    out["foot_axis_tip"] = (K[0] + P.TIBIA_LEN * math.cos(a), K[1] + P.TIBIA_LEN * math.sin(a))
    return out


def leg_to_body(idx, pt_rz, psi_deg=0.0, t=0.0):
    """(r, z) leg-plane point (+ tangent offset t) -> body-frame xyz for leg idx with coxa yaw psi"""
    h = math.radians(HEADINGS[idx] + psi_deg)
    r = pt_rz[0]
    x = HIPS[idx][0] + r * math.cos(h) - t * math.sin(h)
    y = HIPS[idx][1] + r * math.sin(h) + t * math.cos(h)
    return (x, y, pt_rz[1])


NEUTRAL = solve_leg(0.0)
RIDE_HEIGHT = -NEUTRAL["foot"][1]       # body underside -> ground at neutral pose (foot tip on ground)
