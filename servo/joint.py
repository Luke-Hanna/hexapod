import numpy as np
class joint:
    minAngle = -np.pi/2
    maxAngle = np.pi/2
    def __init__(self,driver,channel,minPWM,maxPWM,zeroPWM,direction=1):
        self.driver = driver
        self.channel = channel
        self.minPWM = minPWM
        self.maxPWM = maxPWM
        self.direction = direction
        self.zeroPWM = zeroPWM
        self.PWMrad = (self.maxPWM - self.minPWM)/(self.maxAngle - self.minAngle)
        self.angle = None
    def clamp(self,angle):
        return max(self.minAngle, min(self.maxAngle, angle))
    def angle_to_pwm(self,angle):
        angle = self.clamp(angle)
        return int(np.clip(self.zeroPWM +self.direction*angle*self.PWMrad,self.minPWM,self.maxPWM))
    def pwm_to_duty(self,pwm):
        return int(pwm*0xFFFF/20000)
    def set_angle(self,angle):
        self.angle = angle
        if self.driver is not None:
            self.driver.channels[self.channel].duty_cycle = self.pwm_to_duty(self.angle_to_pwm(self.clamp(angle)))

    