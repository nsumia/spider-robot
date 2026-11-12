# spider-robot
4-legged, 8-servo 3D-printed spider robot: Arduino Nano Every + PCA9685 + MPU-6050 (GY-521), 2S LiPo + 6 V UBEC.
Coxa yaw = MG90S, femur lift = MG996R, passive parallelogram four-bar tibia (fixed-bent-tibia fallback flag).
This branch holds the CAD only (CadQuery). Firmware comes later and must use `cad/out/leg_geometry.json`.

## Commands
    tools/run_tests.sh                 # THE test command: build + export + every check, exits non-zero on any failure
    ~/tools/cad-venv/bin/python cad/build_all.py           # STEP/STL (print orientation) + leg_geometry.json -> cad/out/
    ~/tools/cad-venv/bin/python cad/make_renders.py        # iso.png, parts_sheet.png, leg_motion.gif, gait.gif, foot_path.png
    SPIDER_TIBIA_MODE=fixed ...                            # fixed-bent-tibia fallback (outputs in cad/out/fixed/)

## Layout
- `cad/params.py`   every dimension (SPEC values, tolerances, servo envelopes, BOM masses, analysis limits)
- `cad/kin.py`      leg kinematics (four-bar solver) + derived geometry; `cad/gait.py` generic creep cycle for the studies
- `cad/parts.py`    CadQuery part builders; `cad/assembly.py` posed assembly; `cad/render.py`, `cad/make_renders.py`
- `cad/analysis/`   check_*.py (each prints PASS/FAIL, exit 1 on fail)
- `cad/out/`        exports + renders + `leg_geometry.json`

## Printed parts (qty)
body_lower (1, PLA), body_upper (1, PLA), coxa_bracket (4, PETG), femur (4), rocker (4), tibia (4) (PETG), foot_pad (4, TPU).
Servo/board envelopes are dummies from commonly published dimensions: UNVERIFIED, measure the real parts before printing
(MG996R tab hole spacing, horn bolt patterns, GY-521 holes, PCA9685 holes). Print one test leg first.
