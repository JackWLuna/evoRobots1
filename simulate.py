
from simulation import SIMULATION
import sys

direct_or_GUI = sys.argv[1]
id = sys.argv[2]
simulation = SIMULATION(direct_or_GUI,id)
simulation.Run()
simulation.Get_Fitness()
