import pybullet as p
import pybullet_data
import time as t

GRAV = 10

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8*GRAV)
planeId = p.loadURDF("plane.urdf")
robotID = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

while(True):
    p.stepSimulation()
    t.sleep(1.0/144.0)
    

p.disconnect()