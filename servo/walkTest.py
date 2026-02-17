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
gait = TripodGait(30, -160,125+160,0.5,2)
vx = 0
vy = 50
updateRate = 40
dt = 1/updateRate
nextUpdate = time.time()
try:
    while True:
        currTime = time.time()
        if currTime >= nextUpdate:

            t = currTime-t0
            footPos = []
            for i in range(6):
                pos = gait.get_foot_pos(i,t,vx,vy)
                footPos.append(pos)
            for legNo in range(6):
                hexapod.write_body_to_leg(legNo, footPos[legNo], -np.pi/2)
            nextUpdate = currTime + dt

except KeyboardInterrupt:
    oe.value = True
    pca1.deinit()
    pca2.deinit()