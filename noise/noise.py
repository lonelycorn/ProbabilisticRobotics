import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise

class Noise:
    def __init__(self):
        self.items = [] # a list of dictionary
        self.noise_type_list = [GaussianNoise, UniformNoise]

    def __str__(self):
        content = ''
        if (len(self.items) > 0):
            for item in self.items:
                content = content + '\n' + str(item['noise']) + ', weight = ' + str(item['weight'])
        else:
            content = 'empty'

        return 'Noise: ' + content

    def LoadFromFile(self, fd):
        '''
        load noise from current location of the given file
        '''
        count = [int(x) for x in fd.readline().split()][0]
        #~ count = fd.readline().split()
        #~ print(count)
        #~ count = count[0]
        for i in range(count):
            tmp = fd.readline().split()
            weight = float(tmp[0])
            noise_type = int(tmp[1])
            noise_args = [float(x) for x in tmp[2:]]
            self.RegisterNoiseFromArgs(weight, noise_type, noise_args)

    def RegisterNoiseFromArgs(self, weight, noise_type, noise_args):
        n = self.noise_type_list[noise_type](noise_args)
        self.RegisterNoise(weight, n)

    def RegisterNoise(self, weight, noise):
        self.items.append({'noise' : noise, 'weight' : weight})

    def GetValue(self):
        value = 0.0
        for item in self.items:
            value = value + item['noise'].GetValue() * item['weight']

        return value
        
if __name__ == '__main__':
    pass
