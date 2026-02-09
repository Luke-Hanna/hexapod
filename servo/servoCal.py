#!/usr/bin/env python3

import json
import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

# ================== CONFIG ==================
TOTAL_SERVOS = 24
JSON_FILE = "servo_calibration.json"

PCA1_ADDR = 0x40
PCA2_ADDR = 0x41

OE_PIN = board.D16   # Physical pin 36
SERVO_FREQ = 50
DEFAULT_PULSE = 1500  # us

# ================== OE ==================
oe = digitalio.DigitalInOut(OE_PIN)
oe.direction = digitalio.Direction.OUTPUT
oe.value = True  # outputs disabled at boot

# ================== I2C / PCA ==================
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.2)

pca1 = PCA9685(i2c, address=PCA1_ADDR)
pca2 = PCA9685(i2c, address=PCA2_ADDR)

pca1.frequency = SERVO_FREQ
pca2.frequency = SERVO_FREQ

# ================== HELPERS ==================
def pulse_us_to_duty(pulse_us):
    return int(pulse_us * 0xFFFF / 20000)  # 20 ms @ 50 Hz

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

def load_or_create():
    try:
        with open(JSON_FILE, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {
            str(i): {
                "neutral": None,
                "plus_90": None,
                "minus_90": None,
                "direction": 1
            } for i in range(TOTAL_SERVOS)
        }

# ================== MAIN ==================
cal = load_or_create()

print("\n=== PCA9685 Servo Calibration Tool ===")
print("Fast mode enabled. Servo resets to neutral when done.\n")

try:
    servo = 0
    while servo < TOTAL_SERVOS:
        data = cal[str(servo)]

        print(f"\n=== Servo {servo} ===")
        print(data)

        cmd = input("Enter = calibrate | k = skip | q = quit: ").strip()
        if cmd == "k":
            servo += 1
            continue
        if cmd == "q":
            break

        # Enable outputs
        oe.value = False
        time.sleep(0.02)

        # Direction
        cmd = input(f"Direction [{data['direction']}] (p=+1, m=-1, Enter=keep): ").strip()
        if cmd == "p":
            data["direction"] = 1
        elif cmd == "m":
            data["direction"] = -1

        pulse = data["neutral"] or DEFAULT_PULSE
        set_servo_pulse(servo, pulse)

        for label in ["neutral", "plus_90", "minus_90"]:
            print(f"\nAdjusting {label}")
            pulse = data[label] or pulse
            set_servo_pulse(servo, pulse)

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
                    data[label] = pulse
                    break

                set_servo_pulse(servo, pulse)

        # ---- RESET TO NEUTRAL (90°) ----
        neutral = data["neutral"]
        if neutral is not None:
            set_servo_pulse(servo, neutral)
            time.sleep(0.2)  # let it physically settle

        # Disengage
        disengage_servo(servo)
        oe.value = True

        # Save
        with open(JSON_FILE, "w") as f:
            json.dump(cal, f, indent=2)

        print("Reset to neutral. Servo disengaged.")
        servo += 1

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    oe.value = True
    pca1.deinit()
    pca2.deinit()
    print("Outputs disabled. Safe exit.")
