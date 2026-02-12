from leg import Leg, load_legs
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685
import time
import numpy as np

calFile = "servo_calibration_with_offset.json"

# Addresses and pins
PCA1Addr = 0x40
PCA2Addr = 0x41
OE = board.D16
freq = 50

# ---------- gait helpers ----------
def gait_foot(x_center, z_ground, stride, lift, phase, duty=0.65):
    """
    Smooth stance+swing trajectory in the (x,z) plane.
    phase: 0..1
    duty: stance fraction (0.6-0.75 typical)
    returns (x, z)
    """
    phase = phase % 1.0

    if phase < duty:
        # STANCE: foot on ground, moving back relative to body
        u = phase / duty  # 0..1
        # +stride/2 -> -stride/2, eased
        x = x_center + (stride / 2.0) * np.cos(np.pi * u)
        z = z_ground
    else:
        # SWING: lift and move forward to reset
        u = (phase - duty) / (1.0 - duty)  # 0..1
        # -stride/2 -> +stride/2, eased
        x = x_center - (stride / 2.0) * np.cos(np.pi * u)
        # 0 at ends, max at mid swing
        z = z_ground + lift * np.sin(np.pi * u)

    return x, z


# ---------- hardware init ----------
oe = digitalio.DigitalInOut(OE)
oe.direction = digitalio.Direction.OUTPUT
oe.value = False

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)

pca1 = PCA9685(i2c, address=PCA1Addr)
pca2 = PCA9685(i2c, address=PCA2Addr)
pca1.frequency = freq
pca2.frequency = freq

legs = load_legs(pca1, pca2, calFile)

# ---------- standing pose ----------
x_center = 160
y_center = 0
z_ground = -140
foot_pitch = -np.pi / 2

for leg in legs:
    leg.write_leg_pos(x_center, y_center, z_ground, foot_pitch)
time.sleep(1.0)

# ---------- tripod gait params ----------
# NOTE: adjust these if your leg indexing is different
# Common pattern if legs are ordered around the body: even/odd tripod split
tripodA = {0, 2, 4}
tripodB = {1, 3, 5}

stride = 40      # mm (increase slowly)
lift = 35        # mm
duty = 0.65      # fraction of cycle in stance
T = 1.0          # seconds per full gait cycle
dt = 0.01        # control/update step

# Optional per-leg Y offsets (keeps legs in their own lateral lanes).
# If you don't have these yet, leave them all 0.
y_offsets = [0, 0, 0, 0, 0, 0]

# ---------- main loop ----------
t0 = time.monotonic()

try:
    while True:
        t = time.monotonic() - t0
        base_phase = (t / T) % 1.0

        for i, leg in enumerate(legs):
            phase = base_phase if i in tripodA else (base_phase + 0.5)
            x, z = gait_foot(x_center, z_ground, stride, lift, phase, duty=duty)
            y = y_center + (y_offsets[i] if i < len(y_offsets) else 0)

            leg.write_leg_pos(x, y, z, foot_pitch)

        time.sleep(dt)

except KeyboardInterrupt:
    oe.value = True
    pca1.deinit()
    pca2.deinit()