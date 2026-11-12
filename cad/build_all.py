"""Build all printable parts, export STEP+STL (print orientation) to cad/out/, write leg_geometry.json.
Usage: python cad/build_all.py   (SPIDER_TIBIA_MODE=fixed -> cad/out/fixed/)"""
import os, sys, json, math
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import cadquery as cq
from cad import params as P, kin, parts as pt
from cad.cqh import V, rot

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
if P.TIBIA_MODE == "fixed":
    OUT = os.path.join(OUT, "fixed")
os.makedirs(OUT, exist_ok=True)

# part -> (builder, material, qty, print-orientation rotation (axis, deg) or None)
def printable():
    d = {
        "body_lower": (pt.body_lower, "PLA", 1, None),
        "body_upper": (pt.body_upper, "PLA", 1, None),
        "coxa_bracket": (pt.coxa_bracket, "PETG", 4, ((1, 0, 0), 180)),
        "foot_pad": (pt.foot_pad, "TPU", 4, None),
    }
    if P.TIBIA_MODE == "fixed":
        d["femur_tibia_fixed"] = (pt.femur_tibia_fixed, "PETG", 4, ((1, 0, 0), 90))
    else:
        d["femur"] = (pt.femur, "PETG", 4, ((1, 0, 0), 90))
        d["rocker"] = (pt.rocker, "PETG", 4, ((1, 0, 0), 90))
        d["tibia"] = (pt.tibia, "PETG", 4, ((1, 0, 0), 90))
    return d


def oriented(builder, orient):
    s = builder()
    if orient:
        s = rot(s, (0, 0, 0), orient[0], orient[1])
    bb = s.BoundingBox()
    return s.translate(V(-bb.xmin, -bb.ymin, -bb.zmin))


def normalise_step(path):
    """Replace the FILE_NAME timestamp so re-exports are byte-identical (keeps git clean after test runs)."""
    import re
    t = open(path).read()
    t = re.sub(r"(FILE_NAME\('[^']*',')[^']*(')", r"\g<1>1970-01-01T00:00:00\g<2>", t, count=1)
    open(path, "w").write(t)


def export_parts():
    info = {}
    for name, (b, mat, qty, orient) in printable().items():
        s = oriented(b, orient)
        cq.exporters.export(cq.Workplane(obj=s), os.path.join(OUT, f"{name}.step"))
        normalise_step(os.path.join(OUT, f"{name}.step"))
        cq.exporters.export(cq.Workplane(obj=s), os.path.join(OUT, f"{name}.stl"), tolerance=0.03, angularTolerance=0.1)
        bb = s.BoundingBox()
        info[name] = dict(material=mat, qty=qty, volume_mm3=s.Volume(), area_mm2=s.Area(),
                          bbox=[bb.xlen, bb.ylen, bb.zlen])
        print(f"exported {name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {s.Volume()/1000:.2f} cm3")
    json.dump(info, open(os.path.join(OUT, "parts_info.json"), "w"), indent=1)
    return info


def r2(v):
    return [round(x, 3) for x in v]


def leg_geometry():
    fixed = P.TIBIA_MODE == "fixed"
    table = []
    for qi in range(int(P.FEMUR_RANGE[0]), int(P.FEMUR_RANGE[1]) + 1):
        s = kin.solve_leg(float(qi))
        table.append(dict(q=qi, knee=r2(s["K"]), foot_contact=r2(s["foot"]), foot_center=r2(s["foot_center"]),
                          tibia_dev_from_vertical_deg=round(s["tibia_dev"], 3)))
    n = kin.NEUTRAL
    g = {
        "units": "mm, deg",
        "version": "spider-cad-v1",
        "tibia_mode": P.TIBIA_MODE,
        "notes": [
            "Leg-local frame: origin on the coxa yaw axis at body-underside height (z=0 = body underside), +r radial outward, "
            "+t tangential (CCW from +r seen from above), +z up. The whole linkage lies in the plane t=0 (foot has no lateral offset).",
            "Body frame: +x forward, +y left, +z up. Leg heading h = atan2(hip_y, hip_x); yaw psi is CCW-positive seen from above, "
            "measured from the heading. Foot body-frame xy = hip + r*(cos(h+psi), sin(h+psi)).",
            "Femur angle q is CCW-positive in the (r,z) plane (positive = knee up). Femur elevation phi = phi0 + q.",
            "Servo convention (assumed, UNVERIFIED on hardware): servo_angle = 90 + sign*angle; sign +1 for both joints. "
            "Determine the real sign / trim per servo with the firmware calibration mode.",
            "foot_contact is the lowest point of the 12 mm pad sphere (ground contact). Ground plane z = -ride_height at neutral.",
            "Femur-lift IK is a lookup: foot (r,z) is a function of q only (2-DOF coupling). Use the foot_table below.",
        ],
        "leg_names": kin.LEG_NAMES,
        "hips": [dict(name=kin.LEG_NAMES[i], xy=r2(kin.HIPS[i]), heading_deg=round(kin.HEADINGS[i], 3)) for i in range(4)],
        "links": {
            "coxa_len": P.COXA_LEN, "femur_len": P.FEMUR_LEN, "tibia_len_knee_to_axis_tip": P.TIBIA_LEN,
            "foot_pad_diameter": P.FOOT_D, "foot_pad_center_from_knee": P.TIBIA_LEN - P.FOOT_D / 2,
            "tibia_tilt_from_vertical_deg": P.TIBIA_TILT if not fixed else P.TIBIA_TILT,
        },
        "joints": {
            "coxa": {"axis_leg_local": [0, 0, 1], "point": [0, 0, 0], "servo": "MG90S", "zero_angle_def": "psi = 0 -> leg along its hip heading (diagonal)"},
            "femur": {"axis_leg_local": [0, 1, 0], "point_rz": r2(kin.A), "servo": "MG996R",
                      "zero_angle_def": f"q = 0 -> femur elevation phi0 = {P.PHI0} deg above horizontal; q positive = knee up"},
            "knee_passive": {"point_rz_at_q0": r2(kin.knee(0)), "type": "fixed bent tibia (fallback)" if fixed else "parallelogram four-bar"},
        },
        "fourbar": None if fixed else {
            "fixed_pivot_B_rz": r2(kin.B), "offset_AB_equals_KK2_rz": r2(kin.DVEC), "rocker_len": P.ROCKER_LEN,
            "coupler_len_KK2": P.FOURBAR_D, "tibia_stays_at_deg_from_vertical": P.TIBIA_TILT,
        },
        "servo_zero_deg": {"coxa": 90.0, "femur": 90.0},
        "servo_sign": {"coxa": 1, "femur": 1},
        "limits_deg": {"coxa": list(P.COXA_RANGE), "femur": list(P.FEMUR_RANGE)},
        "servo_limits_deg_absolute": {"coxa": [90 + P.COXA_RANGE[0], 90 + P.COXA_RANGE[1]],
                                      "femur": [90 + P.FEMUR_RANGE[0], 90 + P.FEMUR_RANGE[1]]},
        "neutral": {"q": 0.0, "psi": 0.0, "foot_contact_rz": r2(n["foot"]), "ride_height_body_underside_to_ground": round(kin.RIDE_HEIGHT, 3)},
        "body": {"width": P.BODY_W, "hip_square": P.HIP_SQ, "height": P.BODY_H, "imu_xy": list(P.IMU_POS)},
        "foot_table": table,
    }
    return g


def main():
    export_parts()
    g = leg_geometry()
    json.dump(g, open(os.path.join(OUT, "leg_geometry.json"), "w"), indent=1)
    print("wrote leg_geometry.json (mode %s)" % P.TIBIA_MODE)


if __name__ == "__main__":
    main()
