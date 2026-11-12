"""CadQuery part builders.  Leg parts are modelled at the NEUTRAL pose (femur servo angle 0,
coxa yaw 0) directly in the leg-local frame; the body in the body frame.  Units mm."""
import math
import cadquery as cq
from . import params as P
from . import kin
from .cqh import (V, box, cyl, cyl_y, hexprism_y, prism_xz, hull_plate, hull_pts, fuse_all, cut_all,
                  rot, single_solid)

A = kin.A
B = kin.B
K0 = kin.knee(0.0)
K2_0 = (K0[0] + kin.DVEC[0], K0[1] + kin.DVEC[1])
TIB_AXIS = (math.sin(math.radians(P.TIBIA_TILT)), -math.cos(math.radians(P.TIBIA_TILT)))   # (r, z) unit
TIP_S = 54.0                                  # tibia plate tip circle at s along axis
TIP_C = (K0[0] + TIP_S * TIB_AXIS[0], K0[1] + TIP_S * TIB_AXIS[1])
FOOT_TIP = (K0[0] + P.TIBIA_LEN * TIB_AXIS[0], K0[1] + P.TIBIA_LEN * TIB_AXIS[1])

# tangent-coordinate landmarks of the femur servo / bracket
SV = P.MG996R
T_TAB_FRONT = P.T_FACE - SV["tab_depth"]                 # tab slab face towards output side
T_RIM = T_TAB_FRONT - SV["tab_t"]                         # bracket flange front face
T_SERVO_BACK = P.T_FACE - SV["H"]
TAB_C = A[0] + SV["off"] - SV["L"] / 2                    # case centre (radial)
CAV_R0 = A[0] - (SV["L"] - SV["off"]) - P.SERVO_POCKET
CAV_R1 = A[0] + SV["off"] + P.SERVO_POCKET
CAV_Z0 = A[1] - SV["W"] / 2 - P.SERVO_POCKET
CAV_Z1 = A[1] + SV["W"] / 2 + P.SERVO_POCKET              # == bracket bottom
T_CAV_BACK = T_SERVO_BACK - P.SERVO_POCKET
T_POST_END = P.T_FEM0 - 0.5
TUBE_DEPTH = 14.0
T_NUT_B = (T_POST_END - 8.0, T_POST_END - 4.0)


def _fwd(u, s, base):
    return (base[0] + s * u[0], base[1] + s * u[1])


# ------------------------------------------------------------------ servo dummies
def servo_dummy(spec):
    """Servo envelope in servo-local frame: output face z=0 (boss +z), case -z, shaft axis at x=0,y=0."""
    L, W, H, off = spec["L"], spec["W"], spec["H"], spec["off"]
    case = box(-(L - off), off, -W / 2, W / 2, -H, 0)
    cx = off - L / 2
    tab = box(cx - spec["tab_len"] / 2, cx + spec["tab_len"] / 2, -W / 2, W / 2,
              -spec["tab_depth"] - spec["tab_t"], -spec["tab_depth"])
    boss = cyl(spec["boss_d"] / 2, spec["boss_h"], (0, 0, 0), (0, 0, 1))
    return fuse_all([case, tab, boss])


def femur_servo_leg():
    """MG996R placed in the leg-local frame: shaft along +Y at A, output face at t = T_FACE."""
    s = servo_dummy(SV)
    s = rot(s, (0, 0, 0), (1, 0, 0), -90)            # servo +z -> leg +y, servo y -> -z
    return s.translate(V(A[0], P.T_FACE, A[1]))


def coxa_servo_leg():
    """MG90S in leg-local frame at neutral: shaft down (-Z) at the yaw axis, face at z=0, case above."""
    s = servo_dummy(P.MG90S)
    s = rot(s, (0, 0, 0), (1, 0, 0), 180)            # servo +z -> -z, y -> -y
    return s


# ------------------------------------------------------------------ hardware dummies
def bolt_y(x, z, y_head_under, length, head_dir):
    """M3 socket-head bolt along Y. head_dir=-1: head on the -Y side of y_head_under, shaft toward +Y."""
    if head_dir < 0:
        head = cyl_y(P.BOLT_HEAD_D / 2, y_head_under - P.BOLT_HEAD_H, y_head_under, x, z)
        shaft = cyl_y(P.BOLT_D / 2, y_head_under, y_head_under + length, x, z)
    else:
        head = cyl_y(P.BOLT_HEAD_D / 2, y_head_under, y_head_under + P.BOLT_HEAD_H, x, z)
        shaft = cyl_y(P.BOLT_D / 2, y_head_under - length, y_head_under, x, z)
    return head.fuse(shaft)


def nut_y(x, z, y0):
    return hexprism_y(5.5, y0, y0 + P.NUT_H, x, z)


TRAP_DEPTH = 3.2
NUT_Y0_TIB = P.T_TIB1 - TRAP_DEPTH


def hw_K():
    return bolt_y(*K0, P.T_FEM0, 16.0, -1).fuse(nut_y(*K0, NUT_Y0_TIB))


def hw_K2():
    return bolt_y(*K2_0, P.T_FEM0, 16.0, -1).fuse(nut_y(*K2_0, NUT_Y0_TIB))



def hw_B():
    return bolt_y(B[0], B[1], P.T_FEM1 - P.BOLT_HEAD_H, 12.0, +1).fuse(nut_y(B[0], B[1], T_NUT_B[0]))


# ------------------------------------------------------------------ printed leg parts
def femur():
    plate = hull_plate(A, P.FEMUR_HUB_R, K0, P.FEMUR_END_R, P.T_FEM0, P.T_FEM1)
    hub = cyl_y(P.FEMUR_HUB_R, P.T_FEM1, P.T_HUB1, *A)
    s = plate.fuse(hub)
    cuts = [cyl_y(P.M3_ROT / 2, P.T_FEM0 - 1, P.T_FEM1 + 1, *K0)]
    # servo horn interface: horn disc pocket, spline/boss pocket, centre screw + counterbore
    cuts.append(cyl_y((SV["horn_d"] + 0.4) / 2, P.T_FEM0 - 1, P.T_FEM0 + P.HORN_POCKET_DEPTH, *A))
    boss_end = P.T_FACE + SV["boss_h"]
    cuts.append(cyl_y((SV["boss_d"] + 1.2) / 2, P.T_FEM0 - 1, boss_end + 0.3, *A))
    cuts.append(cyl_y(P.M3_CLEAR / 2, P.T_FEM0 - 1, P.T_HUB1 + 1, *A))
    cuts.append(cyl_y(6.5 / 2, P.T_HUB1 - 2.0, P.T_HUB1 + 1, *A))
    # 4 horn screw pilot holes (M2) on a 16 mm circle (unverified horn pattern)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        cuts.append(cyl_y(P.M2_TAP / 2, P.T_FEM0 + P.HORN_POCKET_DEPTH - 0.1, P.T_FEM0 + P.HORN_POCKET_DEPTH + 4,
                          A[0] + 8 * math.cos(a), A[1] + 8 * math.sin(a)))
    return single_solid(cut_all(s, cuts))


def rocker():
    s = hull_plate(B, P.ROCKER_HUB_R, K2_0, P.ROCKER_END_R, P.T_FEM0, P.T_FEM1)
    cuts = [cyl_y(P.M3_ROT / 2, P.T_FEM0 - 1, P.T_FEM1 + 1, *c) for c in (B, K2_0)]
    cuts.append(cyl_y(6.2 / 2, P.T_FEM1 - P.BOLT_HEAD_H, P.T_FEM1 + 1, *B))      # bolt head sunk flush at B
    return single_solid(cut_all(s, cuts))


def tibia(fixed=False):
    r1, r2 = P.FEMUR_END_R, P.ROCKER_END_R
    shapes = [hull_plate(K0, r1, TIP_C, 3.2, P.T_TIB0, P.T_TIB1)]
    if not fixed:
        for c in (K0, K2_0):
            shapes.append(cyl_y(P.TIB_BOSS_R, P.T_BOSS0, P.T_TIB0 + 0.01, *c))      # knuckle bosses bridge to the femur layer
        shapes.append(hull_plate(K2_0, r2, TIP_C, 3.2, P.T_TIB0, P.T_TIB1))
        shapes.append(hull_plate(K0, r1, K2_0, r2, P.T_TIB0, P.T_TIB1))
    # peg for the foot pad, along the tibia axis, centred on the plate
    d = (TIB_AXIS[0], 0, TIB_AXIS[1])
    peg_s0, peg_s1 = 50.0, FOOT_PAD_END_S - 0.5 - P.FOOT_HOLE_DEPTH + P.FOOT_HOLE_DEPTH  # ends 0.5 short of hole bottom
    base = (K0[0] + peg_s0 * TIB_AXIS[0], 0.0, K0[1] + peg_s0 * TIB_AXIS[1])
    shapes.append(cyl(P.PEG_D / 2, peg_s1 - peg_s0, base, d))
    s = fuse_all(shapes)
    if fixed:
        return s
    cuts = [cyl_y(P.M3_ROT / 2, P.T_BOSS0 - 1, P.T_TIB1 + 1, *c) for c in (K0, K2_0)]
    for c in (K0, K2_0):
        cuts.append(hexprism_y(P.NUT_AF, NUT_Y0_TIB, P.T_TIB1 + 1, *c))
    return single_solid(cut_all(s, cuts))


FOOT_PAD_END_S = P.FOOT_PAD_Z0 + P.FOOT_HOLE_DEPTH - 0.0   # hole bottom at s = 66


def foot_pad():
    """TPU/rubber pad in the TIBIA-AXIS frame (axis +Z, s measured from the knee): placed by foot_pad_leg()."""
    s0 = P.FOOT_PAD_Z0
    r = P.FOOT_D / 2
    c = cyl(r, (P.TIBIA_LEN - r) - s0, (0, 0, s0), (0, 0, 1))
    sph = cq.Solid.makeSphere(r, V(0, 0, P.TIBIA_LEN - r), V(0, 0, 1), 0, 90, 360)
    s = c.fuse(sph)
    s = s.cut(cyl(P.PEG_HOLE_D / 2, P.FOOT_HOLE_DEPTH, (0, 0, s0), (0, 0, 1)))
    s = s.cut(cyl(r + 1, 2.0, (0, 0, P.TIBIA_LEN - P.FOOT_FLAT), (0, 0, 1)))        # flat contact patch at the apex
    return single_solid(s.clean())


def foot_pad_leg():
    s = foot_pad()
    s = rot(s, (0, 0, 0), (0, 1, 0), 180 - P.TIBIA_TILT)
    return s.translate(V(K0[0], 0, K0[1]))


def femur_tibia_fixed():
    """fallback: femur + rigid bent tibia as ONE part (knee block joins the two layers)"""
    f = hull_plate(A, P.FEMUR_HUB_R, K0, P.FEMUR_END_R, P.T_FEM0, P.T_FEM1)
    hub = cyl_y(P.FEMUR_HUB_R, P.T_FEM1, P.T_HUB1, *A)
    knee = cyl_y(P.FEMUR_END_R, P.T_FEM1 - 0.1, P.T_TIB0 + 0.1, *K0)
    t = tibia(fixed=True)
    s = fuse_all([f, hub, knee, t])
    cuts = [cyl_y((SV["horn_d"] + 0.4) / 2, P.T_FEM0 - 1, P.T_FEM0 + P.HORN_POCKET_DEPTH, *A),
            cyl_y((SV["boss_d"] + 1.2) / 2, P.T_FEM0 - 1, P.T_FACE + SV["boss_h"] + 0.3, *A),
            cyl_y(P.M3_CLEAR / 2, P.T_FEM0 - 1, P.T_HUB1 + 1, *A),
            cyl_y(6.5 / 2, P.T_HUB1 - 2.0, P.T_HUB1 + 1, *A)]
    return single_solid(cut_all(s, cuts))


# ------------------------------------------------------------------ coxa bracket
def coxa_bracket():
    fixed = P.TIBIA_MODE == "fixed"
    top, bot = P.BRACKET_TOP_Z, P.BRACKET_BOT_Z
    ttop = CAV_Z1 + P.WALL                       # tube/flange top (3 mm roof); full height only at the yaw hub
    ear0, ear1 = TAB_C - (SV["tab_len"] / 2 + 1.5), TAB_C + (SV["tab_len"] / 2 + 1.5)
    tube_r0, tube_r1 = CAV_R0 - P.WALL, CAV_R1 + P.WALL
    t_back = T_RIM - TUBE_DEPTH                    # short collar: servo rear end sticks out (open back, cables)
    t_fl0 = T_RIM - P.FLANGE_T
    parts = [
        box(-P.BRACKET_DISC_R, P.BRACKET_DISC_R, T_RIM - 1, 14.0, bot, top),         # bridge to the yaw axis
        box(tube_r0, tube_r1, t_back, T_RIM, CAV_Z0 - P.WALL, ttop),                     # tube around the servo
        box(ear0, ear1, t_fl0, T_RIM, CAV_Z0 - P.WALL, ttop),                            # flange with screw ears
    ]
    if not fixed:
        parts.append(box(B[0] - P.POST_R - 1, B[0] + P.POST_R + 1, t_fl0, T_RIM, B[1] - P.POST_R - 1, A[1]))   # lug
        parts.append(cyl_y(P.POST_R, T_RIM - 0.5, T_POST_END, *B))                                 # pivot post
    s = fuse_all(parts)
    cuts = [box(CAV_R0, CAV_R1, t_back - 1, T_RIM + 0.01, CAV_Z0, CAV_Z1)]                  # servo cavity (open front)
    for dr in (-SV["tab_hole_sp"] / 2, SV["tab_hole_sp"] / 2):                              # tab screws M3 self-tap
        cuts.append(cyl_y(P.M3_TAP / 2, T_RIM - 6.0, T_RIM + 0.01, TAB_C + dr, A[1]))
    if not fixed:
        # M3 pin B: hole through the post + nut trap with a slot open downward
        cuts.append(cyl_y(P.M3_CLEAR / 2, T_NUT_B[0] - 3.5, T_POST_END + 0.01, *B))
        cuts.append(hexprism_y(P.NUT_AF, T_NUT_B[0] - 0.2, T_NUT_B[1] + 0.2, *B))
        cuts.append(box(B[0] - P.NUT_AF / 2, B[0] + P.NUT_AF / 2, T_NUT_B[0] - 0.2, T_NUT_B[1] + 0.2,
                        B[1] - P.POST_R - 3, B[1]))
    # coxa horn interface (MG90S): pocket from the top face, boss pocket, screw + counterbore from below
    hp = P.MG90S["horn_d"] + 0.4
    cuts.append(cyl(hp / 2, P.HORN_POCKET_DEPTH + 0.01, (0, 0, top - P.HORN_POCKET_DEPTH), (0, 0, 1)))
    cuts.append(cyl((P.MG90S["boss_d"] + 3) / 2, P.BOSS_POCKET_DEPTH + 0.01, (0, 0, top - P.BOSS_POCKET_DEPTH), (0, 0, 1)))
    cuts.append(cyl(P.M3_CLEAR / 2, P.BRACKET_T + 2, (0, 0, bot - 1), (0, 0, 1)))
    cuts.append(cyl(6.5 / 2, 2.5 + 1, (0, 0, bot - 1), (0, 0, 1)))
    return single_solid(cut_all(s, cuts))


# ------------------------------------------------------------------ body
def _outline_pts(w, c):
    h = w / 2
    return [(-h + c, -h), (h - c, -h), (h, -h + c), (h, h - c), (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c)]


def lightening(z0, z1, keep, pitch=13.0, d=10.0, margin=1.8, edge=7.0):
    """through holes on a grid, skipped when within margin of a keep-out ('c',x,y,r) / ('r',x0,x1,y0,y1)"""
    cut = []
    n = int(P.BODY_W / 2 / pitch) + 1
    lim = P.BODY_W / 2 - edge
    diag = 2 * P.BODY_W / 2 - P.CORNER_CHAMFER - edge * 1.4143
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = (i + 0.5) * pitch, (j + 0.5) * pitch
            if abs(x) > lim or abs(y) > lim or abs(x) + abs(y) > diag:
                continue
            ok = True
            for k in keep:
                if k[0] == "c":
                    ok &= math.hypot(x - k[1], y - k[2]) > k[3] + d / 2 + margin
                else:
                    dx = max(k[1] - x, 0, x - k[2]); dy = max(k[3] - y, 0, y - k[4])
                    ok &= math.hypot(dx, dy) > d / 2 + margin
            if ok:
                cut.append(cyl(d / 2, z1 - z0 + 2, (x, y, z0 - 1)))
    return cut


def _deck(z0, z1):
    wp = cq.Workplane("XY").workplane(offset=z0).polyline(_outline_pts(P.BODY_W, P.CORNER_CHAMFER)).close().extrude(z1 - z0)
    return wp.val()


def leg_to_body_shape(shape, idx, psi=0.0):
    s = rot(shape, (0, 0, 0), (0, 0, 1), kin.HEADINGS[idx] + psi)
    return s.translate(V(kin.HIPS[idx][0], kin.HIPS[idx][1], 0))


def body_lower():
    s = _deck(0, P.DECK_T)
    add = []
    for (x, y) in P.PILLAR_XY:
        add.append(cyl(P.PILLAR_D / 2, P.UPPER_DECK_Z0 - P.DECK_T + 0.01, (x, y, P.DECK_T - 0.01)))
    bt = P.BATTERY
    lx = bt["L"] / 2 + 0.5
    for sx in (-1, 1):                                              # battery end walls + side lips
        add.append(box(sx * lx - (0 if sx > 0 else 2), sx * lx + (2 if sx > 0 else 0), -12, 12, P.DECK_T - 0.01, P.DECK_T + 10))
        for sy in (-1, 1):
            xs = sx * 27
            ys = sy * (bt["W"] / 2 + 0.5)
            add.append(box(xs - 7, xs + 7, ys if sy > 0 else ys - 2, ys + 2 if sy > 0 else ys, P.DECK_T - 0.01, P.DECK_T + 9))
    ub = P.UBEC
    ux, uy = P.UBEC_POS
    outer = box(ux - ub["L"] / 2 - 0.3 - 2, ux + ub["L"] / 2 + 0.3 + 2, uy - ub["W"] / 2 - 0.3 - 2, uy + ub["W"] / 2 + 0.3 + 2,
                P.DECK_T - 0.01, P.DECK_T + 8)
    inner = box(ux - ub["L"] / 2 - 0.3, ux + ub["L"] / 2 + 0.3, uy - ub["W"] / 2 - 0.3, uy + ub["W"] / 2 + 0.3,
                P.DECK_T - 0.02, P.DECK_T + 9)
    ring = outer.cut(inner).cut(box(ux - 10, ux + 10, uy - 20, uy + 20, P.DECK_T + 3, P.DECK_T + 9))
    add.append(ring)
    s = fuse_all([s] + add)
    cuts = []
    for i in range(4):
        sv = P.MG90S
        pocket = box(-(sv["L"] - sv["off"]) - P.SERVO_POCKET, sv["off"] + P.SERVO_POCKET,
                     -sv["W"] / 2 - P.SERVO_POCKET, sv["W"] / 2 + P.SERVO_POCKET, -1, P.DECK_T + 1)
        cuts.append(leg_to_body_shape(pocket, i))
        cx = sv["off"] - sv["L"] / 2
        for dr in (-sv["tab_hole_sp"] / 2, sv["tab_hole_sp"] / 2):
            cuts.append(leg_to_body_shape(cyl(P.M2_TAP / 2, 3.2, (cx + dr, 0, P.DECK_T - 3.2)), i))
    for (x, y) in P.PILLAR_XY:
        cuts.append(cyl(P.M3_TAP / 2, 9, (x, y, P.UPPER_DECK_Z0 - 8)))
    keep = [("c", hx, hy, 21.0) for hx, hy in kin.HIPS] + [("c", x, y, 6.0) for x, y in P.PILLAR_XY]
    keep += [("r", -57, -50, -14, 14), ("r", 50, 57, -14, 14), ("r", -34, 34, -22, -16), ("r", -34, 34, 16, 22),
             ("r", ux - 23.5, ux + 23.5, uy - 11, uy + 11), ("r", -40, 40, -26, -19), ("r", -40, 40, 19, 26)]
    cuts += lightening(0, P.DECK_T, keep)
    for sx in (-1, 1):                                              # battery-bay windows (6.5 mm rails stay under the pack)
        cuts.append(box(sx * 3 if sx > 0 else -46, 46 if sx > 0 else -3, -11, 11, -1, P.DECK_T + 1))
    for sx in (-1, 1):                                              # strap slots
        for sy in (-1, 1):
            cuts.append(box(sx * 32 - 6, sx * 32 + 6, sy * 22.5 - 1.5, sy * 22.5 + 1.5, -1, P.DECK_T + 1))
    return single_solid(cut_all(s, cuts))


def body_upper():
    z0, z1 = P.UPPER_DECK_Z0, P.BODY_H
    s = _deck(z0, z1)
    add = []
    px, py = P.PCA_POS
    for sx in (-1, 1):                                             # PCA9685 standoffs
        for sy in (-1, 1):
            add.append(cyl(2.5, 4.0 + 0.01, (px + sx * P.PCA["hole_dx"] / 2, py + sy * P.PCA["hole_dy"] / 2, z1 - 0.01)))
    nx, ny = P.NANO_POS                                            # Nano Every cradle: two rails + end blocks
    nl, nw = P.NANO["L"] + 0.6, P.NANO["W"] + 0.6
    for sy in (-1, 1):
        add.append(box(nx - nl / 2, nx + nl / 2, ny + sy * (nw / 2) - (0 if sy > 0 else 2), ny + sy * (nw / 2) + (2 if sy > 0 else 0),
                       z1 - 0.01, z1 + 5))
    add.append(box(nx - nl / 2 - 2, nx - nl / 2 + 1.0, ny - nw / 2 - 2, ny + nw / 2 + 2, z1 - 0.01, z1 + 5))    # closed at -X, open at +X (USB)
    s = fuse_all([s] + add)
    cuts = []
    for (x, y) in P.PILLAR_XY:
        cuts.append(cyl(P.M3_CLEAR / 2, DECK_T_H, (x, y, z0 - 1)))
    ix, iy = P.IMU_POS
    cuts.append(box(ix - P.IMU["L"] / 2 - 0.3, ix + P.IMU["L"] / 2 + 0.3, iy - P.IMU["W"] / 2 - 0.3, iy + P.IMU["W"] / 2 + 0.3, z1 - 1.0, z1 + 1))
    for sx in (-1, 1):
        cuts.append(cyl(P.M2_TAP / 2, 3.5, (ix + sx * P.IMU["hole_dx"] / 2, iy, z1 - 3.5)))
    keep = [("c", x, y, 6.0) for x, y in P.PILLAR_XY] + [("c", px + sx * P.PCA["hole_dx"] / 2, py + sy * P.PCA["hole_dy"] / 2, 4.0) for sx in (-1, 1) for sy in (-1, 1)]
    keep += [("r", nx - nl / 2 - 4, nx + nl / 2 + 3, ny - nw / 2 - 5, ny + nw / 2 + 5), ("r", -14, 14, -12, 12), ("r", -48, -32, -14, -6), ("r", 32, 48, -14, -6), ("r", -48, -32, 6, 14), ("r", 32, 48, 6, 14)]
    cuts += lightening(z0, z1, keep)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(cyl(P.M2_TAP / 2, 5.0, (px + sx * P.PCA["hole_dx"] / 2, py + sy * P.PCA["hole_dy"] / 2, z1 - 1)))
            cuts.append(box(sx * 40 - 6, sx * 40 + 6, sy * 10 - 2, sy * 10 + 2, z0 - 1, z1 + 1))   # wire slots
    return single_solid(cut_all(s, cuts))


DECK_T_H = P.DECK_T + 2


def board(spec, pos, z0):
    return box(pos[0] - spec["L"] / 2, pos[0] + spec["L"] / 2, pos[1] - spec["W"] / 2, pos[1] + spec["W"] / 2, z0, z0 + spec["H"])
