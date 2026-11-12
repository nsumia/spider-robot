"""Printable parts: import the exported STEP, valid single solid, STL watertight, fits the bed. Exit 1 on fail."""
import sys, json, os
from common import *
import cadquery as cq
import trimesh
from cad import params as P

OUTD = os.path.join(OUT, "fixed") if P.TIBIA_MODE == "fixed" else OUT


def main():
    info = json.load(open(os.path.join(OUTD, "parts_info.json")))
    ok = True
    for name, d in info.items():
        step = os.path.join(OUTD, name + ".step"); stl = os.path.join(OUTD, name + ".stl")
        shp = cq.importers.importStep(step)
        sol = shp.val()
        nsol = len(shp.solids().vals())
        valid = sol.isValid() and nsol == 1 and sol.Volume() > 0
        m = trimesh.load(stl)
        wt = bool(m.is_watertight) and m.volume > 0
        bb = sol.BoundingBox()
        dims = (bb.xlen, bb.ylen, bb.zlen)
        fits = all(a <= b for a, b in zip(sorted(dims), sorted(P.BED)))
        fits_fixed = dims[0] <= P.BED[0] and dims[1] <= P.BED[1] and dims[2] <= P.BED[2]
        okp = valid and wt and fits_fixed
        ok &= okp
        print(f"  {name:18s} solids={nsol} valid={sol.isValid()} stl_watertight={wt} vol_step={sol.Volume()/1000:.2f} cm3 "
              f"vol_stl={m.volume/1000:.2f} cm3 bbox={dims[0]:.1f}x{dims[1]:.1f}x{dims[2]:.1f} fits_bed={fits_fixed} -> {'ok' if okp else 'BAD'}")
        if abs(sol.Volume() - m.volume) / sol.Volume() > 0.01:
            print(f"    WARN STEP/STL volume differ {abs(sol.Volume()-m.volume)/sol.Volume()*100:.2f}%")
    verdict(ok, f"all printable parts valid solids, watertight STL, fit bed {P.BED[0]:.0f}x{P.BED[1]:.0f}x{P.BED[2]:.0f}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
