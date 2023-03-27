from solution import SOLUTION
import copy
import constants as c

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()
    
    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
    def Mutate(self):
        self.child.Mutate()
    def Select(self):
        if(self.parent.fitness>self.child.fitness):
            self.parent = self.child

    def Print(self):
        print("Parent fitness: ",self.parent.fitness,"  Child fitness: ",self.child.fitness)

    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()

        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()

    def Show_Best(self):
        self.parent.Evaluate("GUI")

    def Evolve(self):
        self.parent.Evaluate("GUI")
        for current_generation in range(c.NUM_GENERATIONS-1):
            self.Evolve_For_One_Generation()
        
        self.Show_Best()

    

        

