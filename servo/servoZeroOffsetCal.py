#!/usr/bin/env python3

import json
import time
import os
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

# ================== CONFIG ==================
TOTAL_SERVOS = 24

BASE_JSON = "servo_calibration.json"
OFFSET_JSON = "servo_calibration_with_offset.json"

PCA1_ADDR = 0x40
PCA2_ADDR = 0x41

OE_PIN = board.D16
SERVO_FREQ = 50

# ================== OE ==================
oe = digitalio.DigitalInOut(OE_PIN)
oe.direction = digitalio.Direction.OUTPUT
oe.value = True

# ================== I2C ==================
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.2)

pca1 = PCA9685(i2c, address=PCA1_ADDR)
pca2 = PCA9685(i2c, address=PCA2_ADDR)

pca1.frequency = SERVO_FREQ
pca2.frequency = SERVO_FREQ


# ================== HELPERS ==================
def pulse_us_to_duty(pulse_us):
    return int(pulse_us * 0xFFFF / 20000)

def set_servo_pulse(servo, pulse_us):
    duty = pulse_us_to_duty(pulse_us)
    if servo < 16:
        pca1.channels[servo].duty_cycle = duty
    else:
        pca2.channels[servo - 16].duty_cycle = duty

def disengage_servo(servo):
    if servo < 16:
        pca1.channels[servo].duty_cycle = 0
    else:
        pca2.channels[servo - 16].duty_cycle = 0


# ================== LOAD FILE ==================
if os.path.exists(OFFSET_JSON):
    print(f"Loading existing offset file: {OFFSET_JSON}")
    with open(OFFSET_JSON, "r") as f:
        cal = json.load(f)
else:
    print(f"No offset file found. Loading base file: {BASE_JSON}")
    with open(BASE_JSON, "r") as f:
        cal = json.load(f)

print("\n=== Zero Offset Calibration Tool ===")
print("Existing offsets will NOT be changed unless recalibrated.\n")

try:
    servo = 0

    while servo < TOTAL_SERVOS:

        data = cal[str(servo)]
        existing_offset = data.get("zero_offset_deg", None)

        print(f"\n=== Servo {servo} ===")
        print(f"Current zero_offset_deg: {existing_offset}")

        cmd = input("Enter = recalibrate | k = skip | q = quit: ").strip()

        if cmd == "k":
            servo += 1
            continue
        if cmd == "q":
            break

        neutral = data["neutral"]
        plus_90 = data["plus_90"]
        minus_90 = data["minus_90"]

        slope = (plus_90 - minus_90) / 180.0

        pulse = neutral

        oe.value = False
        set_servo_pulse(servo, pulse)

        print("\nAdjust until robot is at TRUE 0° position")

        while True:
            print(f"Pulse = {pulse} us")
            cmd = input("[j/l ±1 | a/d ±10 | A/D ±50 | z/c ±100 | s save]: ").strip()

            if cmd == "j":
                pulse -= 1
            elif cmd == "l":
                pulse += 1
            elif cmd == "a":
                pulse -= 10
            elif cmd == "d":
                pulse += 10
            elif cmd == "A":
                pulse -= 50
            elif cmd == "D":
                pulse += 50
            elif cmd == "z":
                pulse -= 100
            elif cmd == "c":
                pulse += 100
            elif cmd == "s":
                break

            set_servo_pulse(servo, pulse)

        physical_angle_deg = (pulse - neutral) / slope

        print(f"\nMeasured angle at robot zero: {physical_angle_deg:.3f}°")

        data["zero_offset_deg"] = physical_angle_deg

        print(f"Updated zero_offset_deg = {physical_angle_deg:.3f}")

        set_servo_pulse(servo, neutral)
        time.sleep(0.3)

        disengage_servo(servo)
        oe.value = True

        servo += 1

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    with open(OFFSET_JSON, "w") as f:
        json.dump(cal, f, indent=2)

    oe.value = True
    pca1.deinit()
    pca2.deinit()

    print(f"\nOffsets saved to {OFFSET_JSON}")
    print("Unmodified offsets preserved.")
    print("Safe exit.")
