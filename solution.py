import time
import numpy as np
import random
import constants as c
import os
import pyrosim.pyrosim as pyrosim

length0 = 1
width0 = 1
height0 = 1

x0 = 0
y0 = 0
z0 = 1


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.my_id = nextAvailableID
        self.weights = np.random.rand(c.num_sensor_neurons,c.num_motor_neurons)
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

        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-.5,1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-.5,0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,.5,1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,.5,0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-.5,0,0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[.5,0,0] , size=[1,.2,0.2])




        pyrosim.Send_Joint( name = "BackLeg_LowBackLeg" , parent= "BackLeg" , child = "LowBackLeg" , type = "revolute", position = [0,-1,0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowBackLeg", pos=[0,-.5,0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "FrontLeg_LowFrontLeg" , parent= "FrontLeg" , child = "LowFrontLeg" , type = "revolute", position = [0,1,0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowFrontLeg", pos=[0,.5,0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "LeftLeg_LowLeftLeg" , parent= "LeftLeg" , child = "LowLeftLeg" , type = "revolute", position = [-1,0,0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowLeftLeg", pos=[-.5,0,0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "RightLeg_LowRightLeg" , parent= "RightLeg" , child = "LowRightLeg" , type = "revolute", position = [1,0,0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowRightLeg", pos=[.5,0,0] , size=[1,.2,0.2])


        pyrosim.End()


    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.my_id)+".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LowBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LowFrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "LowLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "LowRightLeg")

        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "BackLeg_LowBackLeg")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "FrontLeg_LowFrontLeg")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "LeftLeg_LowLeftLeg")
        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "RightLeg_LowRightLeg")

        for current_row in range(c.num_sensor_neurons):
            for current_column in range(c.num_motor_neurons):
                pyrosim.Send_Synapse( sourceNeuronName = current_row , targetNeuronName = current_column + c.num_sensor_neurons , weight = self.weights[current_row][current_column])
        

        pyrosim.End()

    def Mutate(self):
        row = random.randint(0,c.num_sensor_neurons-1)
        col = random.randint(0,c.num_motor_neurons-1)
        self.weights[row,col] = random.random() * 2 - 1