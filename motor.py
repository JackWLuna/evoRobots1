import math
import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:

    def __init__(self,jointName):

        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):

        self.amplitude = c.AMPLITUDE1
        self.frequency = c.FREQUENCY1
        self.offset = c.PHASE1

        if(self.jointName == b"Torso_BackLeg"):
            self.frequency = self.frequency/2.0

        self.domain = np.linspace(0,2*math.pi,num=c.NUM_ITERATIONS)
        self.domain = self.domain * self.frequency
        self.domain = self.domain + self.offset
        self.motorValues = np.sin(self.domain)
        self.motorValues *= self.amplitude

    def Set_Value(self, robot, desiredAngle):
        pyrosim.Set_Motor_For_Joint(

            bodyIndex = robot,

            jointName = self.jointName,

            controlMode = p.POSITION_CONTROL,

            targetPosition = desiredAngle,

            maxForce = 500)
        
    def Save_Values(self):
        np.save("data/motor_" + self.jointName + ".npy", self.motorValues)