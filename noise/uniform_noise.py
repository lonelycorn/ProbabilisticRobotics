from numpy import random

class UniformNoise:
    def __init__(self, args):
        self.lb = args[0] * 1.0 # lower bound
        self.ub = args[1] * 1.0 # upper bound

    def __str__(self):
        return 'UniformNoise: lb = ' + str(self.lb) + ', ub = ' + str(self.ub)

    def GetValue(self):
        return random.uniform(self.lb, self.ub)
        
if __name__ == '__main__':
    pass
