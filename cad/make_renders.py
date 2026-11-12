"""Renders: iso.png (assembly), parts_sheet.png, leg_motion.gif, gait.gif, foot_path.png -> cad/out/"""
import os, sys, math
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
from cad import params as P, kin, gait, assembly as asm, parts as pt
from cad import render as R
from cad.build_all import oriented, printable, OUT


def iso():
    items = asm.assemble()
    R.render(items, os.path.join(OUT, "iso.png"), view=(1.0, -1.1, 0.75), size=(1100, 800), dist_scale=1.2)
    R.render(items, os.path.join(OUT, "side.png"), view=(0.0, -1.0, 0.12), size=(1100, 650), dist_scale=1.2)
    R.render(items, os.path.join(OUT, "bottom.png"), view=(0.6, -0.6, -0.9), size=(1100, 800), dist_scale=1.2, up=(0, 0, 1))
    print("iso/side/bottom done")


def sheet():
    tiles = []
    kind = {"PLA": "PLA", "PETG": "PETG", "TPU": "TPU"}
    for name, (b, mat, qty, orient) in printable().items():
        s = oriented(b, orient)
        im = R.render([(name, kind[mat], s, None)], None, view=(1, -1.2, 0.9), size=(520, 420), dist_scale=1.2)
        d = ImageDraw.Draw(im)
        bb = s.BoundingBox()
        d.text((8, 6), f"{name}  x{qty}  {mat}", fill=(0, 0, 0))
        d.text((8, 22), f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm (print orientation)", fill=(80, 80, 80))
        tiles.append(im)
    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    W, H = tiles[0].size
    out = Image.new("RGB", (W * cols, H * rows), (255, 255, 255))
    for k, t in enumerate(tiles):
        out.paste(t, ((k % cols) * W, (k // cols) * H))
    out.save(os.path.join(OUT, "parts_sheet.png"))
    print("parts sheet done")


def leg_items_only(psi, q, leg=0):
    items = [it for it in asm.static_items() if it[0] in ("body_lower", f"coxa_servo_{kin.LEG_NAMES[leg]}")]
    items += asm.leg_items(leg, psi, q)
    return items


def leg_gif():
    frames = []
    seq = []
    for k in range(16):
        seq.append((0.0, P.FEMUR_RANGE[0] + (P.FEMUR_RANGE[1] - P.FEMUR_RANGE[0]) * 0.5 * (1 - math.cos(2 * math.pi * k / 16))))
    for k in range(16):
        seq.append((P.COXA_RANGE[1] * math.sin(2 * math.pi * k / 16), 0.0))
    for k in range(24):
        seq.append(gait.leg_state(0, k / 24))
    bnds = (30, 130, 30, 130, -100, 30)
    for psi, q in seq:
        items = leg_items_only(psi, q)
        im = R.render(items, None, view=(-0.55, 0.85, 0.35), size=(640, 520), bounds=bnds, dist_scale=0.9,
                      opacity={"PLA": 0.35})
        ImageDraw.Draw(im).text((8, 6), f"coxa yaw {psi:+5.1f} deg   femur q {q:+5.1f} deg   ({P.TIBIA_MODE})", fill=(0, 0, 0))
        frames.append(im)
    frames[0].save(os.path.join(OUT, "leg_motion.gif"), save_all=True, append_images=frames[1:], duration=90, loop=0, optimize=True)
    print("leg gif done", len(frames))


def gait_gif():
    frames = []
    N = 24
    for k in range(N):
        psi, q = gait.pose(k / N)
        im = R.render(asm.assemble(psi, q), None, view=(1.0, -1.1, 0.75), size=(640, 480), dist_scale=1.2)
        ImageDraw.Draw(im).text((8, 6), "generic creep cycle (duty 0.75) - CAD study, not the firmware gait", fill=(0, 0, 0))
        frames.append(im)
    frames[0].save(os.path.join(OUT, "gait.gif"), save_all=True, append_images=frames[1:], duration=100, loop=0, optimize=True)
    print("gait gif done")


def foot_path():
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.2))
    qs = np.linspace(P.FEMUR_RANGE[0], P.FEMUR_RANGE[1], 91)
    pts = np.array([kin.solve_leg(q)["foot"] for q in qs])
    ax[0].plot(pts[:, 0], pts[:, 1], "-", color="tab:blue", label="foot contact vs femur q (-30..60)")
    for q in (-30, 0, 30, 60):
        s = kin.solve_leg(float(q))
        A, K, F = s["A"], s["K"], s["foot"]
        ax[0].plot([A[0], K[0], F[0]], [A[1], K[1], F[1]], "-o", ms=3, alpha=0.7, label=f"q={q}")
    gp = np.array([kin.solve_leg(gait.leg_state(0, k / 200)[1])["foot"] for k in range(200)])
    ax[0].plot(gp[:, 0], gp[:, 1], "k.", ms=2, label="creep cycle (lift 30 deg)")
    ax[0].axhline(-kin.RIDE_HEIGHT, color="k", lw=1); ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3)
    ax[0].set_xlabel("radial r from yaw axis [mm]"); ax[0].set_ylabel("z (body underside = 0) [mm]")
    ax[0].set_title("Leg plane: foot locus (2-DOF: height is a function of q only)"); ax[0].legend(fontsize=7, loc="lower left")
    cols = ["tab:red", "tab:green", "tab:purple", "tab:orange"]
    h = P.BODY_W / 2
    ax[1].plot([-h, h, h, -h, -h], [-h, -h, h, h, -h], "k-", lw=0.8)
    for i in range(4):
        xy = []; st = []
        for k in range(240):
            psi, q = gait.leg_state(i, k / 240)
            x, y, z = gait.foot_xy(i, psi, q)
            xy.append((x, y)); st.append(q == 0)
        xy = np.array(xy); st = np.array(st)
        ax[1].plot(xy[st, 0], xy[st, 1], ".", ms=3, color=cols[i], label=f"{kin.LEG_NAMES[i]} stance")
        ax[1].plot(xy[~st, 0], xy[~st, 1], "x", ms=3, color=cols[i])
        ax[1].plot(*kin.HIPS[i], "ks")
    ax[1].set_aspect("equal"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=7)
    ax[1].set_title("Top view: foot paths over the creep cycle (x = swing)"); ax[1].set_xlabel("x forward [mm]"); ax[1].set_ylabel("y left [mm]")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "foot_path.png"), dpi=130)
    print("foot path done")


if __name__ == "__main__":
    which = sys.argv[1:] or ["iso", "sheet", "foot", "leg", "gait"]
    if "foot" in which: foot_path()
    if "iso" in which: iso()
    if "sheet" in which: sheet()
    if "leg" in which: leg_gif()
    if "gait" in which: gait_gif()
