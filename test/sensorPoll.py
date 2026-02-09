import lgpio
import time
import board
import busio
import math
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
PHYSICAL_TO_BCM = {
    7: 4,
    16: 23,
    18: 24,
    22: 25,
    31: 6,
    29: 5
}
PIN_NAMES = ["SW1", "SW2", "SW3", "SW4", "SW5", "SW6"]
h = lgpio.gpiochip_open(4)
for pin in PHYSICAL_TO_BCM.values():
    lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c, address=0x4A)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
ads = ADS1115(i2c)
ads.gain = 1
chan0 = AnalogIn(ads, 0)
chan1 = AnalogIn(ads, 1)
chan2 = AnalogIn(ads, 2)
chan3 = AnalogIn(ads, 3)
try:
    while True:
        switchStates = []
        for i, pin in enumerate(PHYSICAL_TO_BCM.values()):
            state = lgpio.gpio_read(h, pin)
            switchStates.append("PRESSED" if state == 0 else "released")
        print("Limit switches:", dict(zip(PIN_NAMES, switchStates)))

        quat = bno.quaternion
        roll, pitch, yaw = quaternion_to_euler(*quat)
        print(f"IMU -> Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        print(f"A0: {chan0.value} ({chan0.voltage*(82+20)/20:.4f} V)")
        print(f"A1: {chan1.value} ({chan1.voltage*(82 + 120)/82:.4f} V)")
        print(f"A2: {chan2.value} ({chan2.voltage*(82 + 120)/82:.4f} V)")
        print(f"A3: {chan3.value} ({chan3.voltage*(82 + 120)/82:.4f} V)")

        print("-")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Done")
finally:
    lgpio.gpiochip_close(h)
