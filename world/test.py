#!/usr/bin/env python

import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

# from <file_name> import <ClassName>
from feature_state import FeatureState
from feature_based_world import FeatureBasedWorld
from robot.robot_pose import RobotPose

f1 = FeatureState(1,0,1)
print('f1: ')
print(str(f1))

f2 = FeatureState(0,1,2)
print('\nf2: ')
print(str(f2))

w = FeatureBasedWorld()
w.AppendFeature(f1)
print('\nworld append f1:');
print(str(w))

w.AppendFeature(f2)
print('\nworld append f2:');
print(str(w))

rp = RobotPose(0,0,0)
print('\nRobot at: ')
print(str(rp))

m = w.GetTrueMeasurements(rp)
print('\nNoise-free measurements:')
for i in m:
    print(str(i))

print('\nLoad from file')
w.LoadFromFile("../testcase/map1.txt")
print(str(w))
