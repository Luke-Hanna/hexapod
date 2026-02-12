import numpy as np
class TripodGait:
    def __init__(self,stepHeight, stanceHeight,stanceRadius, stanceDuty, cyclePeriod):
        self.stepHeight = stepHeight
        self.stanceHeight = stanceHeight
        self.stanceRadius = stanceRadius
        self.stanceDuty = stanceDuty
        self.cyclePeriod = cyclePeriod
        self.legAngles = [120,180,240,300,0,60]
        self.nominalPos = []
        for angle in self.legAngles:
            x = self.stanceRadius*np.cos(np.deg2rad(angle))
            y = self.stanceRadius*np.sin(np.deg2rad(angle))
            z = self.stanceHeight
            self.nominalPos.append([x,y,z])
        self.phaseOffsets = [0,0.5,0,0.5,0,0.5]
    def phase(self,t,legNo):
        return (((t % self.cyclePeriod)/self.cyclePeriod) + self.phaseOffsets[legNo]) % 1.0
    def get_foot_pos(self,legNo,t,vx,vy):
        phase = self.phase(t,legNo)
        stepLenX = vx*self.cyclePeriod
        stepLenY = vy*self.cyclePeriod
        if phase < self.stanceDuty:
            # On the ground
            ratio = phase/self.stanceDuty
            xFromNominal = -stepLenX*(ratio - 0.5) # depending on ratio goes from +stepLen to -stepLen
            yFromNominal = -stepLenY*(ratio - 0.5)
            zFromNominal = 0 # on the ground
        else:
            # swinging in the air
            ratio = (phase - self.stanceDuty)/(1-self.stanceDuty)
            xFromNominal = stepLenX*(ratio - 0.5) # same but in the air and going the other way
            yFromNominal = stepLenY*(ratio - 0.5)
            zFromNominal = self.stepHeight*np.sin(np.pi*ratio)

        return [self.nominalPos[legNo][0] + xFromNominal,self.nominalPos[legNo][1] + yFromNominal,self.nominalPos[legNo][2] + zFromNominal]

