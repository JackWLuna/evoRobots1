
import pyrosim.pyrosim as pyrosim
from pathlib import Path
import math

print(Path.cwd())

length0 = 1
width0 = 1
height0 = 1

x0 = 0
y0 = 0
z0 = 1.5




def Create_World():
    pyrosim.Start_SDF("world.sdf")

    pyrosim.Send_Cube(name="Box", pos=[x0-5,y0+10,z0] , size=[length0,width0,height0])

    pyrosim.End()




def Generate_Body():
    pyrosim.Start_URDF("body.urdf")

    pyrosim.Send_Cube(name="Torso", pos=[x0,y0,z0] , size=[length0,width0,height0])

    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [x0-.5,y0,z0-.5])
    pyrosim.Send_Cube(name="BackLeg", pos=[-.5,0,-.5] , size=[length0,width0,height0])

    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [x0+.5,y0,z0-.5])
    pyrosim.Send_Cube(name="FrontLeg", pos=[+.5,0,-.5] , size=[length0,width0,height0])


    pyrosim.End()


def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")

    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

    pyrosim.End()

Generate_Body()
Generate_Brain()
#Create_Robot()
