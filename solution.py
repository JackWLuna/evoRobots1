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
z0 = 3.5


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
        open_flag = False
        attempts = 0
        while(not open_flag):
            try:
                fitness_record = open("fitness" + str(self.my_id)+ ".txt",'r')
            except:
                print("Can't open fit ",str(self.my_id),", retrying...",str(attempts))
                attempts+=1
            else:
                open_flag=True

        self.fitness = float(fitness_record.read())
        fitness_record.close()
        os.system("del fitness" + str(self.my_id)+ ".txt")
        #print("Fitness for id " + str(self.my_id)+ " is: " + str(self.fitness))

    def Create_World(self):
        pyrosim.Start_SDF("world"+str(self.my_id)+".sdf")

        pyrosim.Send_Cube(name="Box", pos=[x0-5,y0+10,z0] , size=[length0,width0,height0])

        pyrosim.End()




    def Generate_Body(self):
        pyrosim.Start_URDF("body" + str(self.my_id)+ ".urdf")

        pyrosim.Send_Cube(name="Torso", pos=[x0,y0,z0] , size=[1,.5,2])

        pyrosim.Send_Joint( name = "Torso_Head" , parent= "Torso" , child = "Head" , type = "revolute", position = [x0,y0,z0+1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="Head", pos=[0,0,.3] , size=[.6,.6,.6])

        pyrosim.Send_Joint( name = "Torso_LeftHip" , parent= "Torso" , child = "LeftHip" , type = "revolute", position = [x0+0.4,y0,z0-1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftHip", pos=[0,0,-.125] , size=[.25,.25,.25])

        pyrosim.Send_Joint( name = "Torso_RightHip" , parent= "Torso" , child = "RightHip" , type = "revolute", position = [x0-.4,y0,z0-1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightHip", pos=[0,0,-.125] , size=[.25,.25,.25])

        pyrosim.Send_Joint( name = "LeftHip_LeftThigh" , parent= "LeftHip" , child = "LeftThigh" , type = "revolute", position = [0,0,-.25], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LeftThigh", pos=[0,0,-.5] , size=[.25,.25,1])

        pyrosim.Send_Joint( name = "RightHip_RightThigh" , parent= "RightHip" , child = "RightThigh" , type = "revolute", position = [0,0,-.25], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="RightThigh", pos=[0,0,-.5] , size=[.25,.25,1])

        pyrosim.Send_Joint( name = "LeftThigh_LeftCalf" , parent= "LeftThigh" , child = "LeftCalf" , type = "revolute", position = [0,0,-1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LeftCalf", pos=[0,0,-.5] , size=[.75,.75,1])

        pyrosim.Send_Joint( name = "RightThigh_RightCalf" , parent= "RightThigh" , child = "RightCalf" , type = "revolute", position = [0,0,-1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="RightCalf", pos=[0,0,-.5] , size=[.75,.75,1])

        # pyrosim.Send_Joint( name = "Torso_LeftShoulder" , parent= "Torso" , child = "LeftShoulder" , type = "revolute", position = [x0-0.5,y0,z0+1], jointAxis="0 0 1")
        # pyrosim.Send_Cube(name="LeftShoulder", pos=[0,0,0] , size=[.25,.25,.25])

        # pyrosim.Send_Joint( name = "Torso_RightShoulder" , parent= "Torso" , child = "RightShoulder" , type = "revolute", position = [x0+.5,y0,z0+1], jointAxis="0 0 1")
        # pyrosim.Send_Cube(name="RightShoulder", pos=[0,0,0] , size=[.25,.25,.25])

        # pyrosim.Send_Joint( name = "LeftShoulder_LeftBicep" , parent= "LeftShoulder" , child = "LeftBicep" , type = "revolute", position = [0,0,0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftBicep", pos=[-.5,0,0] , size=[1,.25,.25])

        # pyrosim.Send_Joint( name = "RightShoulder_RightBicep" , parent= "RightShoulder" , child = "RightBicep" , type = "revolute", position = [0,0,0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightBicep", pos=[.5,0,0] , size=[1,.25,.25])

        # pyrosim.Send_Joint( name = "LeftBicep_LeftForearm" , parent= "LeftBicep" , child = "LeftForearm" , type = "revolute", position = [-1,0,0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftForearm", pos=[-.5,0,0] , size=[1,.25,.25])

        # pyrosim.Send_Joint( name = "RightBicep_RightForearm" , parent= "RightBicep" , child = "RightForearm" , type = "revolute", position = [1,0,0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightForearm", pos=[.5,0,0] , size=[1,.25,.25])

       


        pyrosim.End()


    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.my_id)+".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LeftHip")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "RightHip")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftThigh")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightThigh")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LeftCalf")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "RightCalf")
        # pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "LeftShoulder")
        # pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "RightShoulder")
        # pyrosim.Send_Sensor_Neuron(name = 9 , linkName = "LeftBicep")
        # pyrosim.Send_Sensor_Neuron(name = 10 , linkName = "RightBicep")
        # pyrosim.Send_Sensor_Neuron(name = 11 , linkName = "LeftForearm")
        # pyrosim.Send_Sensor_Neuron(name = 12 , linkName = "RightForearm")
        # pyrosim.Send_Sensor_Neuron(name = 13 , linkName = "RightForearm")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "Head")
        # pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LowBackLeg")
        # pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LowFrontLeg")
        # pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "LowLeftLeg")
        # pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "LowRightLeg")

        pyrosim.Send_Motor_Neuron( name = 8 , jointName = "Torso_LeftHip")
        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_RightHip")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "LeftHip_LeftThigh")
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "RightHip_RightThigh")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "LeftThigh_LeftCalf")
        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "RightThigh_RightCalf")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "Torso_Head")

        

        # pyrosim.Send_Motor_Neuron( name = 11 , jointName = "LeftLeg_LowLeftLeg")
        # pyrosim.Send_Motor_Neuron( name = 12 , jointName = "RightLeg_LowRightLeg")

        for current_row in range(c.num_sensor_neurons):
            for current_column in range(c.num_motor_neurons):
                pyrosim.Send_Synapse( sourceNeuronName = current_row , targetNeuronName = current_column + c.num_sensor_neurons , weight = self.weights[current_row][current_column])
        

        pyrosim.End()

    def Mutate(self):
        row = random.randint(0,c.num_sensor_neurons-1)
        col = random.randint(0,c.num_motor_neurons-1)
        self.weights[row,col] = random.random() * 2 - 1