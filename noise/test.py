#!/usr/bin/env python3
import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

# from <file_name> import <ClassName>
from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise
from noise.noise_builder import NoiseBuilder

print('testing noises')

print('Gaussian noise')
g = GaussianNoise([0, 2.0])
print(g)
print(g.GetValue())

print('\nUniform noise')
u = UniformNoise([10, 11])
print(u)
print(u.GetValue())

print('\nRegister noise')
n = NoiseBuilder()
n.RegisterNoise(0.2, g)
n.RegisterNoise(0.8, u)
print(n)
print(n.GetValue())

print('\nRegister noise from args')
n.RegisterNoiseFromArgs(1, 0, [0, 1])
print(n)
