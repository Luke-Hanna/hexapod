import time
import board
import busio
import digitalio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# Setup I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Optional hardware reset pin (GPIO20)
reset_pin = digitalio.DigitalInOut(board.D20)
reset_pin.direction = digitalio.Direction.OUTPUT

# Hardware reset
reset_pin.value = False
time.sleep(0.05)
reset_pin.value = True
time.sleep(0.1)

# Initialize IMU over I2C
bno = BNO08X_I2C(i2c)  # <-- correct, no address parameter

# Enable the feature you want
REPORT_INTERVAL = 100000  # e.g. 100Hz
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, REPORT_INTERVAL)

print("Rotation vector enabled!")

# Now read quaternion
quat_i, quat_j, quat_k, quat_real = bno.quaternion
print(f"Q: {quat_i:.6f}, {quat_j:.6f}, {quat_k:.6f}, {quat_real:.6f}")
