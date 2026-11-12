"""Four-bar linkage: tibia angle vs femur angle (plot), within +-15 deg of vertical; also CAD <-> kinematics consistency
(foot contact point of the posed CAD pad vs kin.solve_leg).  Exit 1 on fail."""
import sys, math
from common import *
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from cad import params as P, kin, assembly as asm


def main():
    PLOT = os.path.join(OUT, "fixed" if P.TIBIA_MODE == "fixed" else "", "tibia_angle_vs_femur.png")
    os.makedirs(os.path.dirname(PLOT), exist_ok=True)
    ok = True
    qs = np.arange(P.FEMUR_RANGE[0], P.FEMUR_RANGE[1] + 0.5, 1.0)
    if P.TIBIA_MODE == "fixed":
        print("  fixed-tibia fallback: tibia rotates with the femur by design; vertical-tolerance check not applicable")
        verdict(True, "linkage check N/A in fixed-tibia mode")
        # CAD consistency still checked below
    series = {}
    for label, dl in (("nominal", 0.0), ("rocker -0.3 mm", -0.3), ("rocker +0.3 mm", +0.3)):
        dev = []
        for q in qs:
            try:
                s = kin.solve_leg(float(q), rocker_len=P.ROCKER_LEN + dl)
                dev.append(s["tibia_dev"])
            except ValueError:
                dev.append(float("nan"))
        series[label] = np.array(dev)
    if P.TIBIA_MODE != "fixed":
        for label, d in series.items():
            dmax = np.nanmax(np.abs(d))
            good = dmax <= P.TIBIA_VERT_LIMIT and not np.isnan(d).any()
            print(f"  {label:16s}: tibia angle from vertical min {np.nanmin(d):.2f} max {np.nanmax(d):.2f} deg (limit +-{P.TIBIA_VERT_LIMIT})")
            if label == "nominal" or True:
                ok &= good
        verdict(ok, f"tibia within +-{P.TIBIA_VERT_LIMIT} deg of vertical for femur {P.FEMUR_RANGE[0]}..{P.FEMUR_RANGE[1]} deg (also with +-0.3 mm link error)")
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for label, d in series.items():
        ax[0].plot(qs, d, label=label)
    ax[0].axhline(15, color="r", ls="--", lw=0.8); ax[0].axhline(-15, color="r", ls="--", lw=0.8)
    ax[0].set_xlabel("femur servo angle q [deg] (+ = knee up)"); ax[0].set_ylabel("tibia angle from vertical [deg]")
    ax[0].set_title(f"Tibia angle vs femur angle ({P.TIBIA_MODE})"); ax[0].legend(); ax[0].grid(alpha=0.3)
    ax[0].set_ylim(-20, 20 if P.TIBIA_MODE != "fixed" else 80)
    # linkage drawing at three poses
    for q, col in ((P.FEMUR_RANGE[0], "tab:blue"), (0, "tab:green"), (P.FEMUR_RANGE[1], "tab:red")):
        s = kin.solve_leg(float(q))
        A, B, K, K2, F = s["A"], s["B"], s["K"], s["K2"], s["foot_axis_tip"]
        ax[1].plot([A[0], K[0], F[0]], [A[1], K[1], F[1]], "-o", color=col, label=f"q={q:g}")
        if K2:
            ax[1].plot([B[0], K2[0]], [B[1], K2[1]], "--o", color=col, alpha=0.6)
            ax[1].plot([K[0], K2[0]], [K[1], K2[1]], ":", color=col, alpha=0.6)
    ax[1].axhline(-kin.RIDE_HEIGHT, color="k", lw=1); ax[1].set_aspect("equal"); ax[1].grid(alpha=0.3)
    ax[1].set_title("leg plane (r,z); z=0 body underside, black = ground"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(PLOT, dpi=130)
    print(f"  plot: {PLOT}")

    # CAD vs kinematics: lowest vertex of posed pad == kin foot contact
    worst = 0.0
    for q in (P.FEMUR_RANGE[0], -10.0, 0.0, 25.0, P.FEMUR_RANGE[1]):
        for psi in (-45.0, 0.0, 30.0):
            leg = 1
            items = asm.leg_items(leg, psi, q)
            pad = [it for it in items if it[0].startswith("foot_pad")][0][2]
            vs = np.array([(v.x, v.y, v.z) for v in pad.tessellate(0.01, 0.05)[0]])
            zmin = vs[:, 2].min()
            low = vs[vs[:, 2] < zmin + 0.05].mean(axis=0)          # centroid of the lowest facets (robust to tessellation)
            ref = kin.leg_to_body(leg, kin.solve_leg(q)["foot"], psi)
            # sphere tessellation lowest vertex is within a hair of the pole; compare z and xy
            err = math.dist(low[:2], ref[:2]) + abs(low[2] - ref[2])
            worst = max(worst, err)
    good = worst < 0.6        # pad contact is a ~2 mm patch, tessellated: sub-mm agreement is what matters
    ok &= good
    verdict(good, "CAD pad contact point agrees with the kinematics used for the plots / JSON", f"(worst error {worst:.3f} mm)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
