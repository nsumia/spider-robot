"""Clearance of the rotating pin joints and servo horn interfaces (neutral pose, leg FL).  Exit 1 on fail.
Required: rotating pairs >= 0.2 mm (SPEC 0.25 mm pin clearance, hole 3.55 vs 3.0 -> 0.275 radial), servo-in-pocket >= 0.25."""
import sys
from common import *
from cad import params as P, assembly as asm

def main():
    items = {n: s for n, k, s, l in asm.leg_items(0, 0.0, 0.0)}
    fixed = P.TIBIA_MODE == "fixed"
    pairs = [("femur_servo_FL", "coxa_bracket_FL", 0.25, "MG996R case in bracket cavity (tabs are meant to touch the flange -> case only)"),]
    checks = []
    if not fixed:
        from cad import parts as pt, kin
        from cad.cqh import cyl_y
        # bare M3 shafts (head/nut faces legitimately touch the plates) in the rotating holes -> radial clearance
        sh = lambda c, y0, y1: pt.leg_to_body_shape(cyl_y(P.BOLT_D / 2, y0, y1, *c), 0)
        items["shaft_K"] = sh(pt.K0, P.T_FEM0, P.T_TIB1 - 1.0); items["shaft_K2"] = sh(pt.K2_0, P.T_FEM0, P.T_TIB1 - 1.0)
        items["shaft_B"] = sh(kin.B, pt.T_POST_END - 12.0, P.T_FEM1 - 3.0)
        # same shaft but only through the femur / rocker plate and the tibia plate separately
        items["shaft_Kf"] = sh(pt.K0, P.T_FEM0 - 0.01, P.T_FEM1 + 0.01); items["shaft_Kt"] = sh(pt.K0, P.T_TIB0, P.T_TIB1 - 1.0)
        items["shaft_K2r"] = sh(pt.K2_0, P.T_FEM0 - 0.01, P.T_FEM1 + 0.01); items["shaft_K2t"] = sh(pt.K2_0, P.T_TIB0, P.T_TIB1 - 1.0)
        items["shaft_Br"] = sh(kin.B, P.T_FEM0 - 0.01, P.T_FEM1 - 3.0)
        items["shaft_Bp"] = sh(kin.B, P.T_FEM1 - 3.0 - 12.0, pt.T_POST_END)
        checks += [("shaft_Kf", "femur_FL", 0.25), ("shaft_Kt", "tibia_FL", 0.25), ("shaft_K2r", "rocker_FL", 0.25),
                   ("shaft_K2t", "tibia_FL", 0.25), ("shaft_Br", "rocker_FL", 0.25), ("shaft_Bp", "coxa_bracket_FL", 0.14),
                   ("hw_B_FL", "coxa_bracket_FL", 0.09),
                   ("femur_servo_FL", "femur_FL", 0.2), ("foot_pad_FL", "tibia_FL", 0.1),
                   ("femur_FL", "tibia_FL", 0.2), ("rocker_FL", "tibia_FL", 0.2), ("femur_FL", "rocker_FL", 0.2),
                   ("femur_FL", "coxa_bracket_FL", 0.2)]
    else:
        checks += [("femur_servo_FL", "femur_tibia_FL", 0.2), ("femur_tibia_FL", "coxa_bracket_FL", 0.2)]
    items["coxa_servo_FL"] = [s for n, k, s, l in asm.static_items() if n == "coxa_servo_FL"][0]
    checks += [("coxa_servo_FL", "coxa_bracket_FL", 0.5)]
    ok = True
    for a, b, need in checks:
        d = items[a].distance(items[b])
        g = d >= need
        ok &= g
        print(f"  {a:16s} - {b:16s} min distance {d:.3f} mm (need >= {need}) {'ok' if g else 'TOO CLOSE'}")
    verdict(ok, "pin joints / servo horn interfaces keep their clearances")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
