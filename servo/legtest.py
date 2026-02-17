"""
Leg index -> physical leg identification test

What it does:
1) Moves all legs to a neutral "nominal" stance pose slowly
2) Then, one commanded leg index at a time (0..5), it:
   - lifts the foot (z up)
   - swings it slightly forward/back (x +/-)
   - returns to nominal

Watch which *physical* leg moves for each printed index.
Record a table like: commanded index 0 -> physical leg __

SAFETY:
- Put the robot on a stand so feet are off the ground.
- Keep step sizes small at first.
- Ctrl+C to stop; outputs will be disabled in finally.
"""

import time
import numpy as np

import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

from leg import load_legs
from hexapod import Hexapod

# ----------------- USER SETTINGS -----------------
calFile = "servo_calibration_with_offset.json"

PCA1Addr = 0x40
PCA2Addr = 0x41
OE_PIN = board.D16
freq = 50

# These must match your current geometry assumptions
LEG_ANGLES_DEG = [120, 180, 240, 300, 0, 60]   # index 0..5
legRad = 32.25 + 93                             # from your Hexapod class
stanceRadius = 125 + 160                        # from your gait instantiation (=285)
stanceHeight = -160                             # mm
theta_tip = -np.pi / 2                          # end-effector pitch used in IK

# Motion magnitudes for the test (start conservative)
lift_mm = 30            # z raise (less negative = up)
swing_mm = 25           # x swing in body frame (small)
move_steps = 25         # interpolation steps per move
step_dt = 0.02          # seconds between interpolation steps

# -------------------------------------------------

def make_nominal_positions():
    """Nominal foot positions in BODY frame, matching your gait construction."""
    nominal = []
    for ang in LEG_ANGLES_DEG:
        x = stanceRadius * np.cos(np.deg2rad(ang))
        y = stanceRadius * np.sin(np.deg2rad(ang))
        z = stanceHeight
        nominal.append([x, y, z])
    return nominal

def lerp(a, b, s):
    return a + (b - a) * s

def move_leg_body(hexapod, leg_index, start_pos, target_pos, theta, steps=25, dt=0.02):
    """Interpolated move in BODY coordinates for one leg."""
    start = np.array(start_pos, dtype=float)
    targ = np.array(target_pos, dtype=float)
    for k in range(steps + 1):
        s = k / steps
        p = lerp(start, targ, s)
        hexapod.write_body_to_leg(leg_index, p.tolist(), theta)
        time.sleep(dt)

def move_all_to_nominal(hexapod, nominal, theta, steps=40, dt=0.02):
    """Move all legs to nominal smoothly (simple interpolation per leg)."""
    # Read current angles not available in body coords, so we just "ramp in" by repeated writes.
    # This is still gentler than a single jump.
    for k in range(steps):
        for i in range(6):
            hexapod.write_body_to_leg(i, nominal[i], theta)
        time.sleep(dt)

def main():
    oe = digitalio.DigitalInOut(OE_PIN)
    oe.direction = digitalio.Direction.OUTPUT
    oe.value = False  # enable outputs (common wiring: False enables, True disables)

    i2c = busio.I2C(board.SCL, board.SDA)
    time.sleep(0.1)

    pca1 = PCA9685(i2c, address=PCA1Addr)
    pca2 = PCA9685(i2c, address=PCA2Addr)
    pca1.frequency = freq
    pca2.frequency = freq

    legs = load_legs(pca1, pca2, calFile)
    hexapod = Hexapod(legs)

    nominal = make_nominal_positions()

    try:
        print("\nMoving all legs to nominal pose...")
        move_all_to_nominal(hexapod, nominal, theta_tip, steps=50, dt=0.02)
        time.sleep(0.5)

        print("\nStarting identification sequence.")
        print("Watch which PHYSICAL leg moves for each COMMANDED index.\n")

        for i in range(6):
            print(f"=== Commanded leg index {i} (angle {LEG_ANGLES_DEG[i]} deg) ===")

            base = np.array(nominal[i], dtype=float)

            # 1) Lift
            lifted = base.copy()
            lifted[2] = base[2] + lift_mm
            move_leg_body(hexapod, i, base, lifted, theta_tip, steps=move_steps, dt=step_dt)
            time.sleep(0.25)

            # 2) Swing forward (body +x)
            fwd = lifted.copy()
            fwd[0] = lifted[0] + swing_mm
            move_leg_body(hexapod, i, lifted, fwd, theta_tip, steps=move_steps, dt=step_dt)
            time.sleep(0.25)

            # 3) Swing backward (body -x)
            back = lifted.copy()
            back[0] = lifted[0] - swing_mm
            move_leg_body(hexapod, i, fwd, back, theta_tip, steps=2*move_steps, dt=step_dt)
            time.sleep(0.25)

            # 4) Return to lifted center then down to nominal
            move_leg_body(hexapod, i, back, lifted, theta_tip, steps=move_steps, dt=step_dt)
            time.sleep(0.15)
            move_leg_body(hexapod, i, lifted, base, theta_tip, steps=move_steps, dt=step_dt)
            time.sleep(0.6)

        print("\nDone. Build your mapping from what you observed.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # Disable outputs and deinit
        oe.value = True
        pca1.deinit()
        pca2.deinit()
        print("Outputs disabled, PCA deinitialized.")

if __name__ == "__main__":
    main()