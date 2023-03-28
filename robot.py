import pybullet as p
from sensor import SENSOR
from motor import MOTOR
import os
import constants as c
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK

class ROBOT:

    def __init__(self,id):
        self.my_id = id
        self.robot = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robot)
        self.Prepare_To_Sense()
        self.nn = NEURAL_NETWORK("brain"+str(id)+".nndf")
        self.Prepare_To_Act()
        os.system("del " + "brain"+str(id)+".nndf")
        
        

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:

            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self,t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)


    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:

            self.motors[jointName] = MOTOR(jointName)

    def Act(self,t):
        
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[bytes(jointName,"utf-8")].Set_Value(self.robot,desiredAngle)
                
            
        #for motor in self.motors.values():
            #motor.Set_Value(self.robot,t)

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robot,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        #fitness = open("fitness" + str(self.my_id)+ ".txt",'w')
        fitness = open("tmp" + str(self.my_id)+ ".txt",'w')
        fitness.write(str(xCoordinateOfLinkZero))
        fitness.close()
        os.rename("tmp"+str(self.my_id)+".txt" , "fitness"+str(self.my_id)+".txt")
        #os.system("rename tmp" + str()+ ".txt fitness" + str(self.my_id)+ ".txt")
        