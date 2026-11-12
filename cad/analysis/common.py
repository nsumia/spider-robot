"""Shared helpers for the analysis scripts."""
import sys, os, itertools, time
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
OUT = os.path.join(ROOT, "cad", "out")
os.makedirs(OUT, exist_ok=True)


def bbox(shape):
    b = shape.BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def bb_overlap(a, b, pad=0.0):
    return not (a[1] < b[0] - pad or b[1] < a[0] - pad or a[3] < b[2] - pad or b[3] < a[2] - pad
                or a[5] < b[4] - pad or b[5] < a[4] - pad)


def overlap_volume(sa, sb):
    try:
        r = sa.intersect(sb)
        return r.Volume() if r is not None else 0.0
    except Exception as e:      # boolean failure counts as a failure (reported)
        return float("nan")


def interferences(items, tol, moving=None):
    """items: [(name, kind, shape, leg)].  Returns list of (nameA, nameB, mm3) with overlap > tol.
    moving: set of leg indices that moved; static-static and unmoved-unmoved pairs are skipped when given."""
    bbs = [bbox(it[2]) for it in items]
    bad = []
    npairs = 0
    for i, j in itertools.combinations(range(len(items)), 2):
        a, b = items[i], items[j]
        if moving is not None:
            ma = a[3] in moving
            mb = b[3] in moving
            if not (ma or mb):
                continue
        if not bb_overlap(bbs[i], bbs[j]):
            continue
        npairs += 1
        v = overlap_volume(a[2], b[2])
        if v != v or v > tol:
            bad.append((a[0], b[0], v))
    return bad, npairs


def verdict(ok, label, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}: {label} {detail}".rstrip())
    return ok


def firmware_poses():
    """Poses commanded by the firmware gaits (cad/out/fw_poses.json, written by tools/dump_fw_poses.sh).
    Returns [(name, psi_tuple, q_tuple)]; exits 1 if the file is missing (the gate must not silently pass)."""
    import json
    path = os.path.join(OUT, "fw_poses.json")
    if not os.path.exists(path):
        print("FAIL: cad/out/fw_poses.json missing (run tools/dump_fw_poses.sh)")
        sys.exit(1)
    d = json.load(open(path))
    return [(p["name"], tuple(p["psi"]), tuple(p["q"])) for p in d["poses"]]
