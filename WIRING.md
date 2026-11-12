# Wiring

Mount the MPU-6050 flat within 10 mm of the body centre with **+X pointing forward, +Y left, +Z up** (the firmware assumes this).
Never connect the PCA9685 `VCC` (logic) to `V+` (servo power). Keep all grounds common.

```
 2S LiPo (7.4 V) ---[10 A blade fuse]---[rocker switch]---+--------------------> UBEC in (+)
        (-) -----------------------------------------------+--------------------> UBEC in (-)

 UBEC (set to 6 V, >= 5 A) out +  ---+---> PCA9685 V+ terminal  (+ 1000 uF/10 V cap across V+ / GND)
                           out -  ---+---> PCA9685 GND terminal
 UBEC 5 V pin / VIN feed:  Nano Every VIN <--- 6 V (UBEC out +)   (Nano regulates to 5 V; GND common)

 I2C bus (5 V logic):
   Nano 5V  --> PCA9685 VCC   Nano 5V --> GY-521 VCC
   Nano GND --> PCA9685 GND   Nano GND --> GY-521 GND
   Nano A4 (SDA) --> PCA9685 SDA, GY-521 SDA
   Nano A5 (SCL) --> PCA9685 SCL, GY-521 SCL
   PCA9685 addr 0x40 (default), MPU-6050 addr 0x68 (AD0 low)

 Servos on the PCA9685 (brown GND, red V+, orange PWM):
```

Channel map (`channel = leg * 2 + joint`, legs in the CAD order FL, FR, RR, RL; hips at (+-45, +-45) mm, +x forward):

| Channel | Leg | Joint | Servo |
|---|---|---|---|
| 0 | FL (front left) | coxa yaw | MG90S |
| 1 | FL | femur lift | MG996R |
| 2 | FR (front right) | coxa | MG90S |
| 3 | FR | femur | MG996R |
| 4 | RR (rear right) | coxa | MG90S |
| 5 | RR | femur | MG996R |
| 6 | RL (rear left) | coxa | MG90S |
| 7 | RL | femur | MG996R |

Notes
- Peak servo current can reach 3-5 A: use short, thick wires from the UBEC to the PCA9685 V+ terminal.
- The Nano is powered from the UBEC (VIN). Powering it from USB at the same time is NOT VERIFIED safe for the Nano Every: calibrate with
  the battery connected and either unplug USB or check the board's VIN/USB isolation first.
- Sources/ratings: brief sections 5 and 6 (fuse/switch/capacitor values are recommendations, UNVERIFIED).
