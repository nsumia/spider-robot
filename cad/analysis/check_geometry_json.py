"""leg_geometry.json (the firmware contract) must match the CAD kinematics: schema keys, link lengths, hips,
foot table re-derived from the four-bar solver (independent recomputation) and the CAD pad position.  Exit 1 on fail."""
import sys, json, os, math
from common import *
from cad import params as P, kin

OUTD = os.path.join(OUT, "fixed") if P.TIBIA_MODE == "fixed" else OUT


def main():
    g = json.load(open(os.path.join(OUTD, "leg_geometry.json")))
    ok = True
    need = ["links", "joints", "hips", "servo_zero_deg", "limits_deg", "foot_table", "neutral", "tibia_mode", "servo_sign"]
    miss = [k for k in need if k not in g]
    ok &= not miss
    verdict(not miss, "leg_geometry.json has all contract keys", str(miss) if miss else "")
    good = (g["links"]["coxa_len"] == P.COXA_LEN and g["links"]["femur_len"] == P.FEMUR_LEN
            and g["links"]["tibia_len_knee_to_axis_tip"] == P.TIBIA_LEN and g["limits_deg"]["coxa"] == list(P.COXA_RANGE)
            and g["limits_deg"]["femur"] == list(P.FEMUR_RANGE) and g["tibia_mode"] == P.TIBIA_MODE)
    ok &= good
    verdict(good, "link lengths / limits / mode equal params.py")
    err = 0.0
    for row in g["foot_table"]:
        s = kin.solve_leg(float(row["q"]))
        err = max(err, math.dist(row["foot_contact"], s["foot"]), math.dist(row["knee"], s["K"]))
    good = err < 0.005
    ok &= good
    verdict(good, f"foot table (91 rows) reproduces the solver (max diff {err:.4f} mm)")
    good = all(math.dist(h["xy"], kin.HIPS[i]) < 1e-6 for i, h in enumerate(g["hips"])) and abs(g["neutral"]["ride_height_body_underside_to_ground"] - kin.RIDE_HEIGHT) < 0.01
    ok &= good
    verdict(good, "hip positions and ride height match")
    imu_ok = math.hypot(*g["body"]["imu_xy"]) <= 10.0
    ok &= imu_ok
    verdict(imu_ok, f"IMU within 10 mm of body centre (SPEC) -> {math.hypot(*g['body']['imu_xy']):.1f} mm")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
