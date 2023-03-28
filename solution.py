import time
import numpy as np
import random
import constants
import os
import pyrosim.pyrosim as pyrosim

length0 = 1
width0 = 1
height0 = 1

x0 = 0
y0 = 0
z0 = 1.5


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.my_id = nextAvailableID
        self.weights = np.random.rand(3,2)
        self.weights = (self.weights * 2)  - 1

    def Set_ID(self,id):
        self.my_id = id

    def Evaluate(self, run_type):      
        pass

       

    def Start_Simulation(self, run_type):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()

        while not os.path.exists("brain" + str(self.my_id)+ ".nndf"):
            time.sleep(0.01)
        os.system("start /B python3 simulate.py " + run_type + " " + str(self.my_id))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.my_id)+ ".txt"):
            time.sleep(0.01)
        fitness_record = open("fitness" + str(self.my_id)+ ".txt",'r')
        self.fitness = float(fitness_record.read())
        fitness_record.close()
        os.system("del fitness" + str(self.my_id)+ ".txt")
        #print("Fitness for id " + str(self.my_id)+ " is: " + str(self.fitness))

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")

        pyrosim.Send_Cube(name="Box", pos=[x0-5,y0+10,z0] , size=[length0,width0,height0])

        pyrosim.End()




    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[x0,y0,z0] , size=[length0,width0,height0])

        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x0-.5,y0,z0-.5])
        pyrosim.Send_Cube(name="BackLeg", pos=[-.5,0,-.5] , size=[length0,width0,height0])

        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x0+.5,y0,z0-.5])
        pyrosim.Send_Cube(name="FrontLeg", pos=[+.5,0,-.5] , size=[length0,width0,height0])


        pyrosim.End()


    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.my_id)+".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for current_row in range(3):
            for current_column in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = current_row , targetNeuronName = current_column + 3 , weight = self.weights[current_row][current_column])
        

        pyrosim.End()

    def Mutate(self):
        row = random.randint(0,2)
        col = random.randint(0,1)
        self.weights[row,col] = random.random() * 2 - 1