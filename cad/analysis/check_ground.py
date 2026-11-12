"""Ride height 60..80 mm, body ground clearance >= 40 mm over the sweep and creep cycle, swing feet never below ground,
feet do not collide with each other.  Exit 1 on fail."""
import sys, math, itertools
from common import *
from cad import params as P, kin, assembly as asm, gait


def lowest(items, names=None, exclude=("foot_pad", "tibia", "hw_K")):
    lo = 1e9; who = None
    for name, kind, shape, leg in items:
        if any(name.startswith(e) for e in exclude):
            continue
        if names is not None and kind not in names:
            continue
        z = shape.BoundingBox().zmin
        if z < lo:
            lo, who = z, name
    return lo, who


def main():
    ok = True
    ride = kin.RIDE_HEIGHT
    good = P.RIDE_MIN <= ride <= P.RIDE_MAX
    ok &= good
    verdict(good, f"ride height (body underside -> ground at neutral) {ride:.1f} mm in [{P.RIDE_MIN:.0f},{P.RIDE_MAX:.0f}]")
    ground = -ride
    # body + servos + boards clearance (non-leg parts) and every other non-foot part, over sweep & gait
    min_body = min_any = 1e9; who_any = None
    poses = [((0,) * 4, (0,) * 4)]
    for c in (-45, 0, 45):
        for f in (-30, 0, 30, 60):
            poses.append(((c,) * 4, (f,) * 4))
    N = 24
    poses += [gait.pose(k / N) for k in range(N)]
    for psi, q in poses:
        items = asm.assemble(psi, q)
        zb, _ = lowest(items, names=("PLA", "bom"))
        min_body = min(min_body, zb - ground)
        if psi == (0,) * 4 and q != (0,) * 4 and min(q) < 0:
            continue                    # q<0 sweep poses push the feet through the floor: not a standing pose
        za, who = lowest(items)
        if za - ground < min_any:
            min_any, who_any = za - ground, who
    fwp = firmware_poses()
    fw_body = fw_serv = 1e9; fw_feet_below = 0.0
    for name, psi, q in fwp:
        items = asm.assemble(psi, q)
        if name != "sit":                       # sit deliberately lowers the body (informational below)
            fw_body = min(fw_body, lowest(items, names=("PLA", "bom"))[0] - ground)
            fw_serv = min(fw_serv, lowest(items, names=("servo",))[0] - ground)
        for i in range(4):
            z = kin.leg_to_body(i, kin.solve_leg(q[i])["foot"], psi[i])[2] - ground
            fw_feet_below = min(fw_feet_below, z)
    # sit: the body drops until the feet (raised by the femur) touch the ground: ground_sit = foot contact z of the sit pose
    sp = [p for p in fwp if p[0] == "sit"][0]
    sit_items = asm.assemble(sp[1], sp[2])
    ground_sit = min(kin.leg_to_body(i, kin.solve_leg(sp[2][i])["foot"], sp[1][i])[2] for i in range(4))
    sit_body = lowest(sit_items, names=("PLA", "bom"))[0] - ground_sit
    sit_serv = lowest(sit_items, names=("servo",))[0] - ground_sit
    good = fw_body >= P.GROUND_CLEARANCE_MIN and (sit_body >= P.GROUND_CLEARANCE_MIN or P.TIBIA_MODE == "fixed") and fw_serv >= P.GROUND_CLEARANCE_MIN and fw_feet_below >= -0.01
    ok &= good
    verdict(good, f"firmware-commanded poses ({len(fwp)}: stand, sit, creep/trot fwd/back/left/right): body {fw_body:.1f} mm, servos {fw_serv:.1f} mm "
                  f"clearance (>= {P.GROUND_CLEARANCE_MIN:.0f}); no foot below ground (min z {fw_feet_below:.2f} mm)")
    print(f"  INFO sit pose: body underside/electronics {sit_body:.1f} mm above ground (gated >= 40 in the main config; fallback fixed-tibia config: informational, its sit angle would need retuning), servos {sit_serv:.1f} mm (rest pose, not gated)")
    sv = [lowest(asm.assemble(psi, q), names=("servo",))[0] - ground for psi, q in poses]
    good = min_body >= P.GROUND_CLEARANCE_MIN and min(sv) >= P.GROUND_CLEARANCE_MIN
    ok &= good
    verdict(good, f"ground clearance >= {P.GROUND_CLEARANCE_MIN:.0f} mm: body/electronics {min_body:.1f} mm, servos {min(sv):.1f} mm (neutral, sweep, creep)")
    print(f"  INFO lowest leg/body part other than tibia+foot (bracket lug, rocker, femur): {who_any} at {min_any:.1f} mm above ground "
          f"({'meets' if min_any >= P.GROUND_CLEARANCE_MIN else 'BELOW'} {P.GROUND_CLEARANCE_MIN:.0f} mm; gate = body+servos, see report)")
    # creep: stance feet on ground, swing feet above
    zs_stance, zs_swing = [], []
    for k in range(N * 4):
        t = k / (N * 4)
        for i in range(4):
            psi, q = gait.leg_state(i, t)
            z = kin.leg_to_body(i, kin.solve_leg(q)["foot"], psi)[2]
            (zs_stance if q == 0 else zs_swing).append(z - ground)
    good = abs(max(zs_stance)) < 0.01 and min(zs_swing) >= -0.01
    ok &= good
    verdict(good, f"creep: stance feet on ground (|z|<0.01), swing feet above ground (min {min(zs_swing):.2f} mm, max lift {max(zs_swing):.1f} mm)")
    # feet vs feet (pad sphere centres) over creep + coxa extremes
    dmin = 1e9
    def centers(psis, qs):
        return [kin.leg_to_body(i, kin.solve_leg(qs[i])["foot_center"], psis[i]) for i in range(4)]
    cases = [gait.pose(k / 96) for k in range(96)]
    for c in itertools.product((-45, 0, 45), repeat=4):
        for f in (0, 60):
            cases.append((c, (f,) * 4))
    for psi, q in cases:
        cs = centers(psi, q)
        for a, b in itertools.combinations(range(4), 2):
            dmin = min(dmin, math.dist(cs[a], cs[b]) - P.FOOT_D)
    good = dmin > 5.0
    ok &= good
    verdict(good, f"feet do not collide: min gap between pads {dmin:.1f} mm (>5 mm) over creep + all coxa-extreme combos")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
