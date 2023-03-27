
from simulation import SIMULATION
import sys

direct_or_GUI = sys.argv[1]

simulation = SIMULATION(direct_or_GUI)
simulation.Run()
simulation.Get_Fitness()
