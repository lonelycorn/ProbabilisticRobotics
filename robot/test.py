#!/usr/bin/env python

import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

# from <path.file> import <class>
from robot_action import RobotAction
from robot_pose import RobotPose
from robot import Robot

from sensor.range_sensor import RangeSensor

from noise.noise import Noise
from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise

# initialize noises
ng = GaussianNoise([0, 1]) 
nu = UniformNoise([-1, 1])

nd = Noise() # distance noise for sensor
nd.RegisterNoise(0.5, ng)
nd.RegisterNoise(0.5, nu)

na = Noise() # angle noise for sensor
na.RegisterNoise(0.4, ng)
na.RegisterNoise(0.1, nu)

nt = Noise() # translational noise for robot
nt.RegisterNoise(0.4, ng)
nt.RegisterNoise(0.6, nu)

nr = Noise() # rotational noise for robot
nr.RegisterNoise(0.3, ng)
nr.RegisterNoise(0.2, nu)

# initialize a robot
rs = RangeSensor([0.1, 2], nd, [0, 0], na)
r = Robot()
r.Initialize(rs, nt, nr)
print(str(r))


#####################
print('')
ra = RobotAction(1.0, 0)
print(str(ra))

print('')
rp = RobotPose(0,0,0)
print(str(rp))

print('\nAction apply to pose:')
print(str(ra.ApplyToPose(rp, 1)))

print('\nPose apply action:')
print(str(rp.ApplyAction(ra, 1)))

############
print('')
ra = RobotAction(0.0, 1)
print(str(ra))

print('')
rp = RobotPose(0,0,0)
print(str(rp))

print('\nAction apply to pose:')
print(str(ra.ApplyToPose(rp, 1)))

print('\nPose apply action:')
print(str(rp.ApplyAction(ra, 1)))

############
print('')
ra = RobotAction(1, 1)
print(str(ra))

print('')
rp = RobotPose(0,0,0)
print(str(rp))

print('\nAction apply to pose:')
print(str(ra.ApplyToPose(rp, 1)))

print('\nPose apply action:')
print(str(rp.ApplyAction(ra, 1)))

###########
print('\nRobot apply action:')
print(str(r.ApplyAction(ra, 1)))

###########
print('\nRobot load from file:')
r.LoadFromFile('../testcase/robot1.txt')
print(str(r))
