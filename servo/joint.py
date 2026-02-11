import numpy as np
class Joint:
    minAngle = np.deg2rad(-100)
    maxAngle = np.deg2rad(100)
    def __init__(self,driver,channel,minPWM,maxPWM,zeroPWM,direction=1):
        self.driver = driver
        self.channel = channel
        self.minPWM = minPWM
        self.maxPWM = maxPWM
        self.direction = direction
        self.zeroPWM = zeroPWM
        self.PWMrad = (self.maxPWM - self.minPWM)/(np.pi)
        self.angle = None
    def clamp(self,angle):
        return max(self.minAngle, min(self.maxAngle, angle))
    def angle_to_pwm(self,angle):
        angle = self.clamp(angle)
        return int(self.zeroPWM +self.direction*angle*self.PWMrad)
    def pwm_to_duty(self,pwm):
        return int(pwm*0xFFFF/20000)
    def set_angle(self,angle):
        self.angle = angle
        if self.driver is not None:
            self.driver.channels[self.channel].duty_cycle = self.pwm_to_duty(self.angle_to_pwm(self.clamp(angle)))


import json
def load_joints(driver1, driver2, path):
    with open(path, "r") as f:
        data = json.load(f)
    joints = []
    for channelString, config in data.items():
        channel = int(channelString)
        if channel < 16:
            driver = driver1
            driverChannel = channel
        else:
            driver = driver2
            driverChannel = channel - 16
        if config["plus_90"] > config["neutral"]:
            direction = 1
        else:
            direction = -1
        joints.append(Joint(driver, driverChannel, min(config["plus_90"], config["minus_90"]), max(config["plus_90"], config["minus_90"]), config["neutral"], direction))
    return joints
