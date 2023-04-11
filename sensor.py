import numpy as np
import constants as c
import pyrosim.pyrosim as pyrosim
import math
class SENSOR:

    def __init__(self, linkName):

        self.linkName = linkName
        self.Prepare_To_Sense()
        

    def Prepare_To_Sense(self):
        self.values = np.zeros(c.NUM_ITERATIONS)

    def Get_Value(self,t):

        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        
        if(self.linkName=="Head"):
            self.values[t] = math.sin(t*c.SIN_FACTOR)

    def Save_Values(self):
        np.save("data/sensor_" + self.linkName + ".npy", self.values)