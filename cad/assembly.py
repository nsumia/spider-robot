"""Assembly: build all solids for a given pose (coxa yaw psi[i], femur angle q[i], i = FL,FR,RR,RL)."""
import functools
from . import params as P
from . import kin
from . import parts as pt
from .cqh import V, rot

FIXED = P.TIBIA_MODE == "fixed"


@functools.lru_cache(maxsize=1)
def base_parts():
    d = {}
    d["body_lower"] = pt.body_lower()
    d["body_upper"] = pt.body_upper()
    d["coxa_bracket"] = pt.coxa_bracket()
    d["femur_servo"] = pt.femur_servo_leg()
    d["coxa_servo"] = pt.coxa_servo_leg()
    if FIXED:
        d["femur_tibia"] = pt.femur_tibia_fixed()
        d["foot_pad"] = pt.foot_pad_leg()
    else:
        d["femur"] = pt.femur()
        d["rocker"] = pt.rocker()
        d["tibia"] = pt.tibia()
        d["foot_pad"] = pt.foot_pad_leg()
        d["hw_K"] = pt.hw_K()
        d["hw_K2"] = pt.hw_K2()
        d["hw_B"] = pt.hw_B()
    return d


@functools.lru_cache(maxsize=1)
def static_items():
    bp = base_parts()
    items = [("body_lower", "PLA", bp["body_lower"], None), ("body_upper", "PLA", bp["body_upper"], None)]
    for i in range(4):
        items.append((f"coxa_servo_{kin.LEG_NAMES[i]}", "servo", pt.leg_to_body_shape(bp["coxa_servo"], i), i))
    zb = P.DECK_T
    items.append(("battery", "bom", pt.board(P.BATTERY, P.BATTERY_POS, zb), None))
    items.append(("ubec", "bom", pt.board(P.UBEC, P.UBEC_POS, zb), None))
    items.append(("nano", "bom", pt.board(P.NANO, P.NANO_POS, P.BODY_H), None))
    items.append(("pca9685", "bom", pt.board(P.PCA, P.PCA_POS, P.BODY_H + 4.0), None))
    items.append(("imu", "bom", pt.board(P.IMU, P.IMU_POS, P.BODY_H - 1.0), None))
    return items


def leg_items(i, psi, q):
    """moving items of leg i; returns list of (name, kind, shape, leg)"""
    bp = base_parts()
    sol = kin.solve_leg(q)
    out = []
    ln = kin.LEG_NAMES[i]

    def place(shape):
        return pt.leg_to_body_shape(shape, i, psi)

    out.append((f"coxa_bracket_{ln}", "PETG", place(bp["coxa_bracket"]), i))
    out.append((f"femur_servo_{ln}", "servo", place(bp["femur_servo"]), i))
    A, K0 = kin.A, pt.K0
    if FIXED:
        s = rot(bp["femur_tibia"], (A[0], 0, A[1]), (0, 1, 0), -q)
        out.append((f"femur_tibia_{ln}", "PETG", place(s), i))
        out.append((f"foot_pad_{ln}", "TPU", place(rot(bp["foot_pad"], (A[0], 0, A[1]), (0, 1, 0), -q)), i))
        return out
    import math
    out.append((f"femur_{ln}", "PETG", place(rot(bp["femur"], (A[0], 0, A[1]), (0, 1, 0), -q)), i))
    # rocker rotation about B from the solved loop
    B = kin.B
    a0 = math.degrees(math.atan2(pt.K2_0[1] - B[1], pt.K2_0[0] - B[0]))
    a1 = math.degrees(math.atan2(sol["K2"][1] - B[1], sol["K2"][0] - B[0]))
    out.append((f"rocker_{ln}", "PETG", place(rot(bp["rocker"], (B[0], 0, B[1]), (0, 1, 0), -(a1 - a0))), i))
    K = sol["K"]
    delta = sol["coupler_delta"]

    def tib(shape):
        s = rot(shape, (K0[0], 0, K0[1]), (0, 1, 0), -delta)
        return place(s.translate(V(K[0] - K0[0], 0, K[1] - K0[1])))

    out.append((f"tibia_{ln}", "PETG", tib(bp["tibia"]), i))
    out.append((f"foot_pad_{ln}", "TPU", tib(bp["foot_pad"]), i))
    out.append((f"hw_K_{ln}", "hw", tib(bp["hw_K"]), i))
    out.append((f"hw_K2_{ln}", "hw", tib(bp["hw_K2"]), i))
    out.append((f"hw_B_{ln}", "hw", place(bp["hw_B"]), i))
    return out


def assemble(psi=(0, 0, 0, 0), q=(0, 0, 0, 0)):
    items = list(static_items())
    for i in range(4):
        items += leg_items(i, psi[i], q[i])
    return items
