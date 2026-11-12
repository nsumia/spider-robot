"""Worst-case torque margin, 3-leg stance, 800 g, full reach, for the femur (MG996R) and coxa (MG90S).  Exit 1 on fail."""
import sys, math
from common import *
from cad import params as P, kin

G = 9.81


def main():
    m = P.ROBOT_MASS_TORQUE / 1000.0
    N = P.LOAD_FRACTION * m * G                      # newton on the worst stance foot
    qs = [x * 0.5 for x in range(int(P.FEMUR_RANGE[0] * 2), int(P.FEMUR_RANGE[1] * 2) + 1)]
    # femur: virtual work, T = F * dz_foot/dq  (the foot translates with the knee in the four-bar)
    best = (0, None)
    for q in qs:
        h = 0.05
        z1 = kin.solve_leg(q + h)["foot"][1]; z0 = kin.solve_leg(q - h)["foot"][1]
        lever = abs(z1 - z0) / (2 * h * math.pi / 180) / 1000.0     # m per rad
        if lever > best[0]:
            best = (lever, q)
    lever, qw = best
    T_fem = N * lever * P.FEMUR_LOSS                             # N m
    kg = lambda nm: nm / (G / 100.0)                              # N m -> kg cm
    T_fem_kgcm = kg(T_fem)
    # coxa: tangential foot force = coeff * N at the largest radial reach
    reach = max(kin.solve_leg(q)["foot"][0] for q in qs) / 1000.0
    T_cox = P.COXA_FORCE_COEFF * N * reach
    T_cox_kgcm = kg(T_cox)
    T_cox_slip = kg(0.6 * N * reach)
    print(f"  load: {P.ROBOT_MASS_TORQUE:.0f} g, worst stance leg carries {P.LOAD_FRACTION*100:.0f}% = {N:.2f} N")
    print(f"  femur: lever arm dz/dq max {lever*1000:.1f} mm at q={qw:.1f} deg, x{P.FEMUR_LOSS} linkage loss -> {T_fem_kgcm:.2f} kg.cm")
    print(f"  coxa : foot force {P.COXA_FORCE_COEFF}*N at full radial reach {reach*1000:.1f} mm -> {T_cox_kgcm:.2f} kg.cm "
          f"(if the foot grips with mu=0.6: {T_cox_slip:.2f} kg.cm)")
    ok = True
    for nm, spec, T in (("femur MG996R", P.MG996R, T_fem_kgcm), ("coxa  MG90S ", P.MG90S, T_cox_kgcm)):
        m60 = spec["stall_kgcm_60"] / T
        m48 = spec["stall_kgcm_48"] / T
        good = m60 >= P.TORQUE_MARGIN_MIN
        ok &= good
        verdict(good, f"{nm} torque margin {m60:.2f}x at 6.0 V (stall {spec['stall_kgcm_60']} kg.cm), {m48:.2f}x at 4.8 V; required >= {P.TORQUE_MARGIN_MIN}x")
    print(f"  INFO coxa margin if the foot is friction-limited at mu=0.6: {P.MG90S['stall_kgcm_60']/T_cox_slip:.2f}x (6 V) -> see report")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
