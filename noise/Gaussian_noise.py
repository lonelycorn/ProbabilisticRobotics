from numpy import random

class GaussianNoise:
    def __init__(self, args):
        self.mu = args[0] * 1.0    # mean
        self.sigma = args[1] * 1.0 # standard deviation

    def __str__(self):
        return 'GaussianNoise: mu = ' + str(self.mu) + ', sigma = ' + str(self.sigma)

    def GetValue(self):
        return random.normal(self.mu, self.sigma)

if __name__ == '__main__':
    pass
