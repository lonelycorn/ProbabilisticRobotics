#!/usr/bin/env python

import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np

# from <file_name> import <ClassName>
from noise.noise import Noise
from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise

from range_sensor import RangeSensor
from measurement import Measurement


print('testing Measurement')
m = Measurement(1.0, np.deg2rad(90), 0)
print(m)

# prepare noises
ng = GaussianNoise([0, 1])
nu = UniformNoise([-3, 3])
ndist = Noise()
ndist.RegisterNoise(0.5, ng)
ndist.RegisterNoise(0.5, nu)
nangle = Noise()
nangle.RegisterNoise(0.8, ng)
nangle.RegisterNoise(0.2, nu)

print('\ntesting RangeSensor\n')
rs = RangeSensor(ndist, [0.1, 4], nangle, [0,0]) # no limit on angle 
print(rs)

m = Measurement(1.0, np.deg2rad(90), 0)
visible, actual_m = rs.GetActualReading(m)
print('true ' + str(m))
print('got ' + str(actual_m) + ', visible = ' + str(visible))

m = Measurement(5.0, np.deg2rad(-180), 0)
visible, actual_m = rs.GetActualReading(m)
print('true ' + str(m))
print('got ' + str(actual_m) + ', visible = ' + str(visible))


print('')

rs = RangeSensor(ndist, [0, 0], nangle, [0, np.pi]) # no limit on dist 
print(rs)

m = Measurement(1.0, np.deg2rad(90), 0)
visible, actual_m = rs.GetActualReading(m)
print('true ' + str(m))
print('got ' + str(actual_m) + ', visible = ' + str(visible))

m = Measurement(50.0, np.deg2rad(10), 0)
visible, actual_m = rs.GetActualReading(m)
print('true ' + str(m))
print('got ' + str(actual_m) + ', visible = ' + str(visible))
