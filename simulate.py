import pybullet as p
import time as t

physicsClient = p.connect(p.GUI)

for i in range(1000):
    p.stepSimulation()
    t.sleep(1.0/60.0)
    print(i)

p.disconnect()