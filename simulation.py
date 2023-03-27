from world import WORLD
from robot import ROBOT
import constants as c
import time as t

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim


class SIMULATION:

    def __init__(self, direct_or_GUI):
        
        if(direct_or_GUI == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
            self.sleep_val = 0
        else:
            self.physicsClient = p.connect(p.GUI)
            self.sleep_val = c.SLEEP_VAL
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8*c.GRAV)

        self.world = WORLD()
        self.robot = ROBOT()

    def __del__(self):

        p.disconnect()
       
    def Run(self):
        for i in range(c.NUM_ITERATIONS):
            #print(i)
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            t.sleep(self.sleep_val)

    def Get_Fitness(self):
        self.robot.Get_Fitness()

        