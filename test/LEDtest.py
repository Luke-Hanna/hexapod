import time
import math
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

# -------------------------
# Brightness control (0.0 to 1.0)
# -------------------------
BRIGHTNESS = 0.3  # 30% brightness

# -------------------------
# Set OE pin LOW (active low)
# -------------------------
OE_PIN = board.D16  # Physical pin 36 on Raspberry Pi
oe = digitalio.DigitalInOut(OE_PIN)
oe.direction = digitalio.Direction.OUTPUT
oe.value = False  # LOW = enable outputs

# -------------------------
# Setup I2C and PCA9685
# -------------------------
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x41)
pca.frequency = 1000  # 1kHz PWM

# RGB pins on PCA9685
R_PIN = 8
G_PIN = 9
B_PIN = 10

# Helper for common-anode (invert PWM + apply brightness)
def ca_pwm(value):
    scaled = int(value * BRIGHTNESS)  # scale brightness
    return 0xFFFF - int(scaled * 65535 / 255)

# Rainbow cycle function using sine waves
def rainbow_cycle(delay=0.02, steps=256):
    for i in range(steps):
        r = int((1 + math.sin(2*math.pi*(i/steps))) * 127.5)
        g = int((1 + math.sin(2*math.pi*(i/steps + 1/3))) * 127.5)
        b = int((1 + math.sin(2*math.pi*(i/steps + 2/3))) * 127.5)

        pca.channels[R_PIN].duty_cycle = ca_pwm(r)
        pca.channels[G_PIN].duty_cycle = ca_pwm(g)
        pca.channels[B_PIN].duty_cycle = ca_pwm(b)
        time.sleep(delay)

# -------------------------
# Main loop
# -------------------------
try:
    while True:
        rainbow_cycle()
except KeyboardInterrupt:
    # Turn off LED on exit
    pca.channels[R_PIN].duty_cycle = 0xFFFF
    pca.channels[G_PIN].duty_cycle = 0xFFFF
    pca.channels[B_PIN].duty_cycle = 0xFFFF
    print("Exiting...")
