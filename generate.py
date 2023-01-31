
import pyrosim.pyrosim as pyrosim
from pathlib import Path
import math

print(Path.cwd())

length0 = 1
width0 = 1
height0 = 1

x0 = 0
y0 = 0
z0 = .5


pyrosim.Start_SDF("boxes.sdf")

def tower(x,y,z):
    for i in range(10):
        pyrosim.Send_Cube(name="Box", pos=[x,y,z+(i)] , size=[length0*(.9**i),width0*(.9**i),height0*(.9**i)])

for i in range(25):
    x_index = i%5
    y_index = i//5

    tower(x0+x_index,y0+y_index,z0)


pyrosim.End()