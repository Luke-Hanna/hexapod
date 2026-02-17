from joint import Joint, load_joints
import numpy as np
class Leg:
    def __init__(self, joints):
        self.joints = joints
        self.L1 = 54.86
        self.L2 = 86
        self.L3 = 112
        self.L4 = 44
    def write_leg_angles(self, *angles):
        for joint, angle in zip(self.joints, angles):
            joint.set_angle(angle)
    def get_angles(self):
        return [joint.angle for joint in self.joints]
    # def solve_IK(self, x,y,z,theta):
    #     theta1 = np.arctan2(y,x)
    #     d = np.sqrt(x**2 + y**2)
    #     d1 = d - self.L4*np.cos(theta) - self.L1
    #     z1 = z - self.L4*np.sin(theta)
    #     dist = np.sqrt(d1**2 + z1**2)
    #     angle23 = np.arccos((self.L2**2 + self.L3**2 - dist**2)/(2*self.L2*self.L3))
    #     theta12 = np.arcsin((self.L3*np.sin(angle23))/dist)
    #     angleHoriToThing = np.arctan2(z1,d1)
        
    #     theta3 = -(np.pi-angle23)
    #     theta2 = theta2 = angleHoriToThing - np.arctan2(self.L3*np.sin(theta3), self.L2 + self.L3*np.cos(theta3))
    #     theta4 = theta + theta2 + theta3
    #     return theta1,theta2,theta3,theta4
    def solve_IK(self, x, y, z, theta, elbowUp=True):
        theta1 = np.arctan2(y, x)
        d = np.sqrt(x**2 + y**2)
        d1 = d - self.L4*np.cos(theta) - self.L1
        z1 = z - self.L4*np.sin(theta)
        dist = np.sqrt(d1**2 + z1**2)
        c_raw = (self.L2**2 + self.L3**2 - dist**2) / (2*self.L2*self.L3)
        if abs(c_raw) > 1.0:
            print("IK UNREACHABLE?", "c_raw=", c_raw, "dist=", dist, "d1=", d1, "z1=", z1)
        c = (self.L2**2 + self.L3**2 - dist**2) / (2*self.L2*self.L3)
        c = np.clip(c, -1.0, 1.0)
        angle23 = np.arccos(c)
        angleHoriToThing = np.arctan2(z1, d1)
        theta3 = (np.pi - angle23)
        if elbowUp:
            theta3 = -theta3
        theta2 = angleHoriToThing - np.arctan2(self.L3*np.sin(theta3), self.L2 + self.L3*np.cos(theta3))
        theta4 = theta - theta2 - theta3
        print(np.rad2deg(theta1))
        print(np.rad2deg(theta2))
        print(np.rad2deg(theta3))
        print(np.rad2deg(theta4))
        return theta1, theta2, theta3, theta4
    def write_leg_pos(self,x,y,z,theta):
        self.write_leg_angles(*self.solve_IK(x,y,z,theta))


def load_legs(pca1, pca2, calFile):
    joints = load_joints(pca1,pca2, calFile)
    legs = []
    for i in range(6):
        legs.append(Leg([joints[4*i],joints[4*i+1],joints[4*i+2],joints[4*i+3]]))
    return legs
