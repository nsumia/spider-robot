# Wiring

Mount the MPU-6050 flat within 10 mm of the body centre with **+X pointing forward, +Y left, +Z up** (the firmware assumes this).
Never connect the PCA9685 `VCC` (logic) to `V+` (servo power). Keep all grounds common.

```
 2S LiPo (7.4 V) --[10 A blade fuse]--[rocker switch]--+---------------------> UBEC (6 V, >=5 A) IN
        (-) --------------------------------------------+---------------------> UBEC IN (-), buck IN (-)
                                                        +---------------------> 5 V buck IN (+)   (small buck, >= 1 A)

 Servo power (high current, 3-5 A typical, >10 A at stall):
   UBEC OUT (+, set to 6 V) ---> terminal block / direct wires ---> PCA9685 V+ screw terminal
   UBEC OUT (-)             ---> same for GND
   1000 uF / 16 V electrolytic across V+ / GND at the PCA9685 terminal.

 Logic power (do NOT use the Nano VIN: it needs 7-21 V, 6 V is out of spec):
   5 V buck OUT (+) --->|--- (Schottky diode, e.g. 1N5819, anode to buck) ---> Nano Every 5V pin
   5 V buck OUT (-) ---------------------------------------------------------> Nano GND
   The diode stops 5 V backfeeding into the USB port when USB is plugged in (diode drop ~0.3 V: logic runs at ~4.7 V).
   (Alternative: a dedicated 5 V BEC.) Use USB only for the serial console; if in doubt, switch the robot off first.
   Common ground: UBEC GND, buck GND, Nano GND, PCA9685 GND, GY-521 GND.

 I2C bus (Nano supplies the logic rails):
   Nano 5V  --> PCA9685 VCC   Nano 5V  --> GY-521 VCC     (VCC pins only: never connect to V+)
   Nano A4 (SDA) --> PCA9685 SDA, GY-521 SDA
   Nano A5 (SCL) --> PCA9685 SCL, GY-521 SCL
   PCA9685 addr 0x40 (default), MPU-6050 addr 0x68 (AD0 low)

 Servos on the PCA9685 (brown GND, red V+, orange PWM).
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
- Stall current of 8 servos can exceed 10 A: distribute V+/GND with thick wire and a terminal block directly from the UBEC, **not** through
  the PCA9685 board traces/terminal alone, and keep the wires short. The 10 A fuse may blow at stall: this is intended (size it after measuring).
- Fit a LiPo low-voltage alarm (3.3 V/cell) on the balance lead; the robot has no battery-voltage sensing.
- Component ratings (fuse, diode, cap, buck) are recommendations, UNVERIFIED against datasheets. Nano Every VIN range 7-21 V is from the
  Arduino documentation as recalled by review, re-check before wiring.
- Sources: brief sections 5 and 6.
