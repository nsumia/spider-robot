"""Interference over (a) a full sweep of each leg (coxa +-45, femur -30..+60, others neutral) and (b) a creep cycle.
Overlap volume above OVERLAP_TOL_MM3 between any two solids = FAIL.  Exit 1 on fail."""
import sys, time
from multiprocessing import Pool
from common import *
from cad import params as P, assembly as asm, gait, kin

COXA = [-45, -22.5, 0, 22.5, 45]
FEMUR = [-30, -15, 0, 15, 30, 45, 60]


def job(args):
    kind, psi, q, moving = args
    items = asm.assemble(psi, q)
    bad, n = interferences(items, P.OVERLAP_TOL_MM3, moving=moving)
    return kind, psi, q, bad, n


def main():
    jobs = []
    legs = range(4)
    for leg in legs:
        for c in COXA:
            for f in FEMUR:
                psi = [0.0] * 4; q = [0.0] * 4
                psi[leg] = c; q[leg] = f
                jobs.append(("sweep", tuple(psi), tuple(q), {leg}))
    # neighbouring legs both at extremes (worst combos) for the front pair and side pair
    for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
        for ca in (-45, 45):
            for cb in (-45, 45):
                for f in (0, 60):
                    psi = [0.0] * 4; q = [0.0] * 4
                    psi[a], psi[b], q[a], q[b] = ca, cb, f, f
                    jobs.append(("pairs", tuple(psi), tuple(q), {a, b}))
    N = 24
    for k in range(N):
        psi, q = gait.pose(k / N)
        jobs.append(("gait", psi, q, {0, 1, 2, 3}))
    t0 = time.time()
    with Pool(10) as pool:
        res = pool.map(job, jobs, chunksize=2)
    summary = {}
    allbad = {}
    for kind, psi, q, bad, n in res:
        s = summary.setdefault(kind, [0, 0, 0])
        s[0] += 1; s[1] += n
        if bad:
            s[2] += 1
            for a, b, v in bad:
                key = (a, b)
                if key not in allbad or v > allbad[key][0]:
                    allbad[key] = (v, kind, psi, q)
    ok = True
    for kind, (np_, npairs, nbad) in summary.items():
        print(f"  {kind}: {np_} poses, {npairs} bbox-overlapping pairs tested, {nbad} poses with overlap > {P.OVERLAP_TOL_MM3} mm3")
        ok &= nbad == 0
    for (a, b), (v, kind, psi, q) in sorted(allbad.items(), key=lambda kv: -kv[1][0])[:25]:
        print(f"  OVERLAP {a} x {b}: {v:.1f} mm3 ({kind}, psi={tuple(round(x,1) for x in psi)}, q={tuple(round(x,1) for x in q)})")
    print(f"  ({time.time()-t0:.0f} s)")
    verdict(ok, "no part-to-part interference (sweep + neighbour extremes + creep cycle)",
            f"[{sum(s[0] for s in summary.values())} poses]")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
