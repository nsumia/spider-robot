"""All dimensions of the spider robot (mm, deg, g).  Single source of truth.

Leg-local frame (used for every leg part): +X = radial (outward from the yaw axis),
+Y = tangent (CCW from +X seen from above), +Z = up.  Yaw axis = Z axis through the origin.
Body frame: X forward, Y left, Z up, body underside at Z = 0.
Every numbered SPEC value is marked [SPEC]; [choice] = my decision; [UNVERIFIED] = servo/board
dimensions taken from commonly published numbers (see research brief), measure real parts!
"""
import os

# ---------------------------------------------------------------- config flags
# "fourbar" = passive parallelogram tibia (default), "fixed" = rigid bent tibia fallback  [SPEC]
TIBIA_MODE = os.environ.get("SPIDER_TIBIA_MODE", "fourbar")
assert TIBIA_MODE in ("fourbar", "fixed")

# ---------------------------------------------------------------- tolerances [SPEC]
M3_CLEAR = 3.3            # M3 clearance hole
PIN_EXTRA = 0.25          # rotating pins: +0.25 (interpreted: hole = M3_CLEAR + 0.25)
M3_ROT = M3_CLEAR + PIN_EXTRA
NUT_AF = 5.5 + 0.2        # M3 nut 5.5 AF + 0.2 -> 5.7 across flats
NUT_H = 4.0               # nylock M3 height
SERVO_POCKET = 0.3        # per side
M3_TAP = 2.6              # pilot hole for M3 self tapping into PETG/PLA [choice]
M2_TAP = 1.7              # pilot hole for M2 self tapping [choice]
BOLT_D = 3.0
BOLT_HEAD_D = 5.5
BOLT_HEAD_H = 3.0

# ---------------------------------------------------------------- printer [SPEC]
BED = (220.0, 220.0, 250.0)
RHO_PETG = 1.27           # g/cm3 [SPEC]
RHO_PLA = 1.24
FILL_FACTOR = 1.0         # 1.0 = solid volume (conservative); mass check uses CAD solid volume

# ---------------------------------------------------------------- body [SPEC]
BODY_W = 120.0            # footprint 120 x 120
HIP_SQ = 90.0             # hip pivots on 90 x 90 square
HIP = HIP_SQ / 2          # 45
BODY_H = 45.0             # lower deck underside to upper deck top
DECK_T = 4.0
CORNER_CHAMFER = 18.0     # [choice]
UPPER_T = 3.0
UPPER_DECK_Z0 = BODY_H - UPPER_T   # 42
PILLAR_XY = [(57.0, 22.0), (57.0, -22.0), (-57.0, 22.0), (-57.0, -22.0)]   # [choice]
PILLAR_D = 6.0

# ---------------------------------------------------------------- leg link lengths [SPEC]
COXA_LEN = 25.0           # yaw axis -> femur axis
FEMUR_LEN = 45.0
TIBIA_LEN = 70.0          # knee -> foot tip along tibia axis
TIBIA_TILT = 8.0          # deg from vertical, foot outward [choice]  (tibia stays +-15 deg of vertical)
PHI0 = 18.0               # femur elevation above horizontal at servo angle 0 (neutral) [choice]
COXA_RANGE = (-45.0, 45.0)   # [SPEC]
FEMUR_RANGE = (-30.0, 60.0)  # [SPEC]  positive = knee up
FOURBAR_D = 26.0          # parallelogram offset |AB| = |KK2|  [choice]
FOURBAR_THETA = -55.0     # direction of A->B (and K->K2) from +X, deg  [choice]
ROCKER_LEN = FEMUR_LEN    # equal -> true parallelogram (tolerance study varies it)

# ---------------------------------------------------------------- servos [UNVERIFIED published dims]
# local frame: output face z=0, boss to +z, case to -z, L along x with shaft at x=0 (case x in [-(L-off), off])
MG90S = dict(name="MG90S", L=22.8, W=12.2, H=22.5, off=5.9, tab_len=32.5, tab_t=2.5, tab_depth=4.0,
             tab_hole_sp=27.8, boss_d=5.0, boss_h=4.0, horn_d=20.0, mass=13.4,
             stall_kgcm_48=1.8, stall_kgcm_60=2.2)
MG996R = dict(name="MG996R", L=40.7, W=19.7, H=37.0, off=10.0, tab_len=55.0, tab_t=2.5, tab_depth=10.0,
              tab_hole_sp=49.5, boss_d=7.0, boss_h=5.9, horn_d=21.0, mass=55.0,
              stall_kgcm_48=9.4, stall_kgcm_60=11.0)

# ---------------------------------------------------------------- leg layers (tangent coordinate t = leg-local Y)
T_TIB0, T_TIB1 = -2.5, 2.5            # tibia plate (centred on the leg plane t=0 so IK has no lateral offset)
LAYER_GAP = 0.5
SPACER = 3.5                          # extra offset so the 12 mm foot pad (t=+-6) clears the femur/rocker layer
T_FEM1 = T_TIB0 - LAYER_GAP - SPACER  # femur / rocker plate  -6.5
T_FEM0 = T_FEM1 - 5.0                 # -11.5
T_HUB1 = T_FEM1 + 2.0                 # femur hub thickens to here
T_BOSS0 = T_FEM1 + LAYER_GAP          # tibia knuckle bosses (K, K2) reach out to here
TIB_BOSS_R = 5.0
T_FACE = T_FEM0 - 1.0                 # MG996R output face  -9.0
FEMUR_HUB_R = 12.5
FEMUR_END_R = 7.0
ROCKER_HUB_R = 6.0
ROCKER_END_R = 6.5

# coxa bracket
BRACKET_TOP_Z = -1.5                  # top face (gap to lower deck underside)
BRACKET_T = 8.0
BRACKET_BOT_Z = BRACKET_TOP_Z - BRACKET_T
BRACKET_DISC_R = 14.0
HORN_POCKET_DEPTH = 3.5
BOSS_POCKET_DEPTH = 5.5
WALL = 3.0
FLANGE_T = 4.5
POST_R = 5.0
HEX_SLOT_EXTRA = 0.4

# foot pad [SPEC] Ø12, TPU / rubber cap
FOOT_D = 12.0
FOOT_FLAT = 0.1           # tiny flat at the dome apex (no degenerate pole in the mesh; also the contact patch)
PEG_D = 6.0
PEG_HOLE_D = PEG_D + PIN_EXTRA
FOOT_PAD_Z0 = 59.0      # s along tibia axis: pad top face
FOOT_HOLE_DEPTH = 7.0

# ---------------------------------------------------------------- electronics envelopes [UNVERIFIED]
NANO = dict(L=45.0, W=18.0, H=8.0, mass=5.0)
PCA = dict(L=62.0, W=26.0, H=12.0, hole_dx=56.0, hole_dy=20.0, mass=10.0)
IMU = dict(L=21.0, W=16.0, H=3.0, hole_dx=15.5, mass=2.0)
UBEC = dict(L=42.0, W=16.0, H=7.0, mass=25.0)
BATTERY = dict(L=105.0, W=35.0, H=19.0, mass=110.0)     # 2S 2200 mAh upper bound
IMU_POS = (0.0, 0.0)
PCA_POS = (0.0, 30.0)
NANO_POS = (0.0, -30.0)
UBEC_POS = (0.0, 31.0)
BATTERY_POS = (0.0, 0.0)

# ---------------------------------------------------------------- BOM masses (g)  non-printed [UNVERIFIED]
BOM = {
    "MG996R x4": 4 * MG996R["mass"],
    "MG90S x4": 4 * MG90S["mass"],
    "battery 2S": BATTERY["mass"],
    "UBEC": UBEC["mass"],
    "Nano Every": NANO["mass"],
    "PCA9685": PCA["mass"],
    "MPU-6050": IMU["mass"],
    "fuse+holder+switch+cap": 18.0,
    "wiring": 25.0,
    "M3/M2 hardware (~60 pcs)": 35.0,
    "foot pads TPU x4": 4 * 1.5,
}
MASS_LIMIT = float(os.environ.get("SPIDER_MASS_LIMIT", 800.0))   # env override only used to prove the test harness fails

# ---------------------------------------------------------------- analysis limits [SPEC]
RIDE_MIN, RIDE_MAX = 60.0, 80.0
GROUND_CLEARANCE_MIN = 40.0
TIBIA_VERT_LIMIT = 15.0
TORQUE_MARGIN_MIN = 2.0
ROBOT_MASS_TORQUE = 800.0        # g used in the torque study
LOAD_FRACTION = 0.5              # worst leg share in a 3-leg stance (CoM on the diagonal edge)
FEMUR_LOSS = 1.25                # four-bar friction/backlash allowance (brief: +20..30 %)
COXA_FORCE_COEFF = 0.30          # tangential foot force / normal load at the coxa [choice, see report]
OVERLAP_TOL_MM3 = 2.0
