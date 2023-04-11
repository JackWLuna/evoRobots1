from world import WORLD
from robot import ROBOT
import constants as c
import time as t

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim


class SIMULATION:

    def __init__(self, direct_or_GUI,id):
        self.run_type = direct_or_GUI
        if(direct_or_GUI == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
    
        else:
            self.physicsClient = p.connect(p.GUI)
    
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8*c.GRAV)

        self.world = WORLD(id)
        self.robot = ROBOT(id)

    def __del__(self):

        p.disconnect()
       
    def Run(self):
        for i in range(c.NUM_ITERATIONS):
            #print(i)
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)

            if(self.run_type == "GUI"):
                t.sleep(c.SLEEP_VAL)

    def Get_Fitness(self):
        self.robot.Get_Fitness()

        