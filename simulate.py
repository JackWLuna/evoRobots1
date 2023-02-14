import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import time as t

GRAV = 10
NUM_ITERATIONS = 500

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8*GRAV)
planeId = p.loadURDF("plane.urdf")
robotID = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotID)
backLegSensorValues = np.zeros(NUM_ITERATIONS)
frontLegSensorValues = np.zeros(NUM_ITERATIONS)
for i in range(NUM_ITERATIONS):
    p.stepSimulation()
    backLegSensorValues[i]  = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i]  = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    t.sleep(1.0/144.0)



np.save("data/backleg.npy",backLegSensorValues)
np.save("data/frontleg.npy",frontLegSensorValues)

p.disconnect()