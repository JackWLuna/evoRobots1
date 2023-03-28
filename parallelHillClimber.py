import os
from solution import SOLUTION
import copy
import constants as c

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.POPULATION_SIZE):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
    
    def Spawn(self):

        self.children = {}
        for parent_key in self.parents:
            self.children[parent_key] = copy.deepcopy(self.parents[parent_key])
            self.children[parent_key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

        # self.child = copy.deepcopy(self.parent)
        # self.child[0].Set_ID(self.nextAvailableID)
        # self.nextAvailableID += 1
    def Mutate(self):
        for child in self.children:
            self.children[child].Mutate()
    def Select(self):

        for key in self.parents:
            if(self.parents[key].fitness>self.children[key].fitness):
                self.parents[key] = self.children[key]

    def Print(self):
        print()
        for key in self.parents:
            print("Parent " + str(key) + " fitness: " + str(self.parents[key].fitness) + ", corresponding child fitness: " + str(self.children[key].fitness))
        print()


    def Evaluate(self,solutions):
        for solution in solutions:
            solutions[solution].Start_Simulation("DIRECT")
        for solution in solutions:
            solutions[solution].Wait_For_Simulation_To_End()


    def Evolve_For_One_Generation(self):
        
        self.Spawn()

        self.Mutate()

        self.Evaluate(self.children)
        
        self.Print()
        self.Select()

    def Show_Best(self):
        max_fit_key = 0
        max_fit = 0
        for key in self.parents:
            if(self.parents[key].fitness < max_fit):
                max_fit_key = key
                max_fit = self.parents[key].fitness
        self.parents[max_fit_key].Start_Simulation("GUI")
        

    def Evolve(self):
        self.Evaluate(self.parents)
        
        for current_generation in range(c.NUM_GENERATIONS-1):
            self.Evolve_For_One_Generation()
        
        self.Show_Best()

    

        

