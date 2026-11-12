"""Mass estimate from CAD volumes (PETG 1.27, PLA 1.24 g/cm3) + BOM masses <= 800 g.  Exit 1 on fail.
Printed-part mass uses a shell+infill model: 4 perimeters (1.6 mm wall) solid, rest 15 % infill."""
import sys, json, os
from common import *
from cad import params as P

OUTD = os.path.join(OUT, "fixed") if P.TIBIA_MODE == "fixed" else OUT
SHELL = 1.6
INFILL = 0.15
RHO = {"PETG": P.RHO_PETG, "PLA": P.RHO_PLA, "TPU": 1.21}


def main():
    info = json.load(open(os.path.join(OUTD, "parts_info.json")))
    solid = est = 0.0
    print(f"  {'part':18s} {'qty':>3s} {'vol cm3':>8s} {'solid g':>8s} {'est g (4 perim+15% infill)':>26s}")
    for n, d in info.items():
        v = d["volume_mm3"] / 1000.0
        if n == "foot_pad":
            continue                                    # counted in BOM (TPU pads)
        shell_v = min(v, d["area_mm2"] * SHELL / 1000.0)
        ve = shell_v + INFILL * (v - shell_v)
        gs = v * RHO[d["material"]] * d["qty"]; ge = ve * RHO[d["material"]] * d["qty"]
        solid += gs; est += ge
        print(f"  {n:18s} {d['qty']:3d} {v:8.2f} {gs:8.1f} {ge:26.1f}")
    bom = sum(P.BOM.values())
    for k, v in P.BOM.items():
        print(f"  BOM {k:28s} {v:7.1f} g")
    t_est, t_solid = est + bom, solid + bom
    print(f"  printed: solid-fill {solid:.1f} g, estimated {est:.1f} g | BOM {bom:.1f} g")
    good = t_est <= P.MASS_LIMIT
    verdict(good, f"mass estimate {t_est:.0f} g <= {P.MASS_LIMIT:.0f} g (printed {est:.0f} + BOM {bom:.0f})")
    print(f"  INFO worst case with 100 % solid prints: {t_solid:.0f} g ({'over' if t_solid > P.MASS_LIMIT else 'within'} the limit)")
    sys.exit(0 if good else 1)


if __name__ == "__main__":
    main()
