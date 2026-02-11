from joint import Joint, load_joints
class Leg:
    def __init__(self, joints):
        self.joints = joints
        self.angles = []
    def write_leg_angles(self, *angles):
        for joint, angle in zip(self.joints, angles):
            joint.set_angle(angle)
    def get_angles(self):
        return [joint.angle for joint in self.joints]


def load_legs(pca1, pca2, calFile):
    joints = load_joints(pca1,pca2, calFile)
    legs = []
    for i in range(6):
        legs.append(Leg([joints[4*i],joints[4*i+1],joints[4*i+2],joints[4*i+3]]))
    return legs
