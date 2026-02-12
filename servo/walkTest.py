from leg import Leg, load_legs
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685
import time
import numpy as np

calFile = "servo_calibration_with_offset.json"

PCA1Addr = 0x40
PCA2Addr = 0x41
OE = board.D16
freq = 50

# ---------- helpers ----------
def rot2(x, y, ang_rad):
    """Rotate (x,y) by ang_rad."""
    c = np.cos(ang_rad)
    s = np.sin(ang_rad)
    return c*x - s*y, s*x + c*y

def gait_dx_z(stride, lift, phase, duty=0.80):
    """
    Returns (dx, z_add) where dx is fore-aft offset in BODY frame.
    Smooth stance+swing. phase in [0,1).
    """
    phase = phase % 1.0

    if phase < duty:
        # stance: dx goes +stride/2 -> -stride/2 smoothly
        u = phase / duty
        dx = (stride / 2.0) * np.cos(np.pi * u)
        z_add = 0.0
    else:
        # swing: dx goes -stride/2 -> +stride/2, with lift
        u = (phase - duty) / (1.0 - duty)
        dx = -(stride / 2.0) * np.cos(np.pi * u)
        z_add = lift * np.sin(np.pi * u)

    return dx, z_add

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

# ---------- geometry / mounting ----------
# legs ordered anticlockwise from head: 0,1,2,3,4,5
# assumed evenly spaced around body:
mount_deg = [0, 60, 120, 180, 240, 300]
mount_rad = [np.deg2rad(a) for a in mount_deg]

# Nominal "stand" radius from body center to each foot in BODY frame
r0 = 160.0

# Nominal height
z_ground = -140.0

# desired foot pitch
foot_pitch = -np.pi / 2

# ---------- gait parameters (SLOW + obvious) ----------
tripodA = {0, 2, 4}
tripodB = {1, 3, 5}

stride = 35.0     # mm (increase if it still looks stationary)
lift = 25.0       # mm
duty = 0.85       # very stable/slow swing
T = 6.0           # seconds per cycle (slow)
dt = 0.02         # 50 Hz

# ---------- stand first ----------
for i, leg in enumerate(legs):
    a = mount_rad[i]
    # body-frame nominal foot location (radially outward)
    xb = r0 * np.cos(a)
    yb = r0 * np.sin(a)

    # convert BODY frame -> LEG local frame by rotating by -mount_angle
    xl, yl = rot2(xb, yb, -a)
    leg.write_leg_pos(xl, yl, z_ground, foot_pitch)

time.sleep(2.0)

# ---------- main loop ----------
t0 = time.monotonic()
try:
    while True:
        t = time.monotonic() - t0
        base_phase = (t / T) % 1.0

        for i, leg in enumerate(legs):
            a = mount_rad[i]

            phase = base_phase if i in tripodA else (base_phase + 0.5)
            dx, z_add = gait_dx_z(stride, lift, phase, duty=duty)

            # BODY-frame target: nominal radial position + fore/aft dx along BODY +X
            xb = r0 * np.cos(a) + dx
            yb = r0 * np.sin(a)

            # BODY -> LEG local
            xl, yl = rot2(xb, yb, -a)

            leg.write_leg_pos(xl, yl, z_ground + z_add, foot_pitch)

        time.sleep(dt)

except KeyboardInterrupt:
    oe.value = True
    pca1.deinit()
    pca2.deinit()