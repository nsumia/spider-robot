# Bill of materials

Prices are approximate USD from the research brief (section 9): **UNVERIFIED, not from live listings**. Check sellers and
dimensions before buying. Servo, board and bolt sizes in the CAD are dummies from common datasheet values (measure first).

| Item | Qty | Approx price | Source (brief) | Notes |
|---|---|---|---|---|
| MG996R servo (femur lift) | 4 | $6 each | AliExpress / Amazon | clones often have lower stall torque than quoted |
| MG90S servo (coxa yaw) | 4 | $4 each | AliExpress / Amazon | torque margin is only ~2x, see LESSONS |
| Arduino Nano Every | 1 | $13 | arduino.cc store / Amazon | ATMega4809, 5 V logic |
| PCA9685 16-ch servo board | 1 | $3 | AliExpress / Amazon | hole spacing varies by vendor |
| MPU-6050 GY-521 | 1 | $3 | AliExpress / Amazon | I2C 0x68 |
| 2S LiPo 1000-2200 mAh (XT60) | 1 | $15 | HobbyKing / Amazon | 2200 mAh ~110 g; 1000 mAh ~60-66 g |
| UBEC 5-8 A, set to 6 V (Hobbywing) | 1 | $15-25 | hobbywingdirect.com / HobbyKing | never feed the PCA9685 VCC from V+ |
| 10 A blade fuse + holder, rocker switch | 1 | $3 | Amazon | ratings are our recommendation, UNVERIFIED |
| Electrolytic cap 1000 uF / 10 V | 1 | $1 | local | across PCA9685 V+/GND |
| LiPo low-voltage alarm | 1 | ~$3 | Amazon | cutoff 3.3 V per cell (recommendation) |
| M3 bolts 16-20 mm, nylock nuts, washers, M2 screws | assorted | $8 | AliExpress / Amazon | pin joints; M3 bushings where parts rotate |
| Filament PETG (legs) / PLA (body) 1 kg each | 1 | $20 | local | TPU or rubber caps for the 4 foot pads |
| **Total** | | **~$115-130** | | brief estimate |

Printed parts (qty): body_lower 1 (PLA), body_upper 1 (PLA), coxa_bracket 4, femur 4, rocker 4, tibia 4 (PETG), foot_pad 4 (TPU).
Mass budget 800 g; the BOM electronics alone are ~510 g (see `cad/analysis/check_mass.py`).
