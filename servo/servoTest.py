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
# Enable output of PCAs
oe = digitalio.DigitalInOut(OE)
oe.direction = digitalio.Direction.OUTPUT
oe.value = False
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
# Setup PCAs
pca1 = PCA9685(i2c, address=PCA1Addr)
pca2 = PCA9685(i2c, address=PCA2Addr)
pca1.frequency = freq
pca2.frequency = freq

legs = load_legs(pca1,pca2, calFile)
legs[1].write_leg_angles(0,np.deg2rad(80),-np.deg2rad(135),-np.deg2rad(50))
# for leg in legs:
#     leg.write_leg_angles(0,0,0,0)
try:
    while True:
        pass
except KeyboardInterrupt:
    oe.value = True
    pca1.deinit()
    pca2.deinit()