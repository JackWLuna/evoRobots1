import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import time as t
import math
import random

GRAV = 5
NUM_ITERATIONS = 10000

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8*GRAV)
planeId = p.loadURDF("plane.urdf")
robotID = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotID)

backLegSensorValues = np.zeros(NUM_ITERATIONS)
frontLegSensorValues = np.zeros(NUM_ITERATIONS)

AMPLITUDE1 = math.pi/2
FREQUENCY1 = NUM_ITERATIONS/100
PHASE1 = 0


AMPLITUDE2 = math.pi/2
FREQUENCY2 = NUM_ITERATIONS/100
PHASE2 = math.pi/4




domain = np.linspace(0,2*math.pi,num=NUM_ITERATIONS)
domain = domain * FREQUENCY1
domain = domain + PHASE1
vals1 = np.sin(domain)
vals1 *= AMPLITUDE1


domain = np.linspace(0,2*math.pi,num=NUM_ITERATIONS)
domain = domain * FREQUENCY2
domain = domain + PHASE2
vals2 = np.sin(domain)
vals2 *= AMPLITUDE2

for i in range(NUM_ITERATIONS):

    
    p.stepSimulation()
    backLegSensorValues[i]  = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i]  = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(

    bodyIndex = robotID,

    jointName = b"Torso_BackLeg",

    controlMode = p.POSITION_CONTROL,

    targetPosition = vals1[i],

    maxForce = 500)

    pyrosim.Set_Motor_For_Joint(

    bodyIndex = robotID,

    jointName = b"Torso_FrontLeg",

    controlMode = p.POSITION_CONTROL,

    targetPosition = vals2[i],

    maxForce = 500)

    t.sleep(1.0/144.0)



np.save("data/backleg.npy",backLegSensorValues)
np.save("data/frontleg.npy",frontLegSensorValues)

p.disconnect()