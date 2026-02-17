from leg import Leg, load_legs
from hexapod import Hexapod
from tripod import TripodGait
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
hexapod = Hexapod(legs)
t0 = time.time()
gait = TripodGait(30, -100,170,0.5,1)
vx = 50
vy = 0
updateRate = 100
dt = 1/updateRate
nextUpdate = time.time()
try:
    while True:
        for leg in legs:
            leg.write_leg_angles(0,np.deg2rad(32),np.deg2rad(-97),np.deg2rad(-24))
        # hexapod.write_body_to_leg(0, [300*np.cos(np.deg2rad(120)), 300*np.sin(np.deg2rad(120)), -100] , -np.pi/2)
        # hexapod.write_body_to_leg(1, [300*np.cos(np.deg2rad(180)), 300*np.sin(np.deg2rad(180)), -100] , -np.pi/2)
        # hexapod.write_body_to_leg(2, [300*np.cos(np.deg2rad(240)), 300*np.sin(np.deg2rad(240)), -100] , -np.pi/2)
        # hexapod.write_body_to_leg(3, [300*np.cos(np.deg2rad(300)), 300*np.sin(np.deg2rad(300)), -100] , -np.pi/2)
        # hexapod.write_body_to_leg(4, [300*np.cos(np.deg2rad(000)), 300*np.sin(np.deg2rad(000)), -100] , -np.pi/2)
        # hexapod.write_body_to_leg(5, [300*np.cos(np.deg2rad(60)), 300*np.sin(np.deg2rad(60)), -100] , -np.pi/2)
        time.sleep(0.5)

except KeyboardInterrupt:
    oe.value = True
    pca1.deinit()
    pca2.deinit()