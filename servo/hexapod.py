import numpy as np
class Hexapod:
    def __init__(self, legs):
        self.legs = legs
        self.legAngles = [120,180,240,300,0,60]
        self.legRad = 32.25 + 93
    def body_to_leg_transform(self, legNo, footBody):
        legCentrePos = np.array([self.legRad*np.cos(np.deg2rad(self.legAngles[legNo])),self.legRad*np.sin(np.deg2rad(self.legAngles[legNo])),0])
        legCentreToPos = np.array(footBody) - legCentrePos
        rot = np.array([[np.cos(-np.deg2rad(self.legAngles[legNo])), -np.sin(-np.deg2rad(self.legAngles[legNo])), 0],
                       [np.sin(-np.deg2rad(self.legAngles[legNo])), np.cos(-np.deg2rad(self.legAngles[legNo])), 0],
                       [0,0,1]])
        return rot @ legCentreToPos
    def write_body_to_leg(self, legNo, footBody, theta):
        x,y,z = self.body_to_leg_transform(legNo, footBody)
        self.legs[legNo].write_leg_pos(x,y,z, theta)
