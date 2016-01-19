import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np

from robot_pose import RobotPose
from robot_action import RobotAction
from noise.noise import Noise
from sensor.range_sensor import RangeSensor

class RobotException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class Robot:
    def __init__(self):
        # These fields will be filled by either LoadFromFile() or Initialize()
        self.sensor = None
        self.translation_noise = None
        self.rotation_noise = None
        self.true_pose = None;

    def __str__(self):
        return 'Robot: \n' + str(self.sensor) + '\ntranslation ' + \
                str(self.translation_noise) + '\nrotation ' + \
                str(self.rotation_noise) + '\ntrue ' + str(self.true_pose)

    def LoadFromFile(self, filename):
        '''
        Load robot configuration from the given file
        ===INPUT===
        filename: the filename to load the configuration from.
        ===OUTPUT===
        None.
        '''
        print('Loading robot configurations from: ' + filename)
        
        fd = open(filename, "r")

        # translational noise
        self.translation_noise = Noise()
        self.translation_noise.LoadFromFile(fd)
        fd.readline()

        # rotational noise
        self.rotation_noise = Noise()
        self.rotation_noise.LoadFromFile(fd)
        fd.readline()
        
        # sensor
        ## dist ranges
        dist_ranges = [float(x) for x in fd.readline().split()]
        fd.readline()
        
        ## dist noise
        dist_noise = Noise()
        dist_noise.LoadFromFile(fd)
        fd.readline()
        
        ## angle ranges
        angle_ranges = [float(x) for x in fd.readline().split()]
        fd.readline()
        
        ## angle noise
        angle_noise = Noise()
        angle_noise.LoadFromFile(fd)
        fd.readline()
        
        self.sensor = RangeSensor(dist_ranges, dist_noise, angle_ranges,  angle_noise)

        # default robot pose
        self.true_pose = RobotPose(0, 0, 0)

    def SetPose(self, pose):
        '''
        Set the robot's true pose
        ===INPUT===
        pose: the pose to set to
        ===OUTPUT===
        None.
        '''
        if (not (isinstance(pose, RobotPose))):
            raise RobotException('pose should be an instance of RobotPose')
        self.true_pose = pose

    def GetPose(self):
        return self.true_pose

    def Initialize(self, range_sensor, translation_noise, rotation_noise):
        if (not (isinstance(range_sensor, RangeSensor))):
            raise RobotException('range_sensor should be an instance of RangeSensor')
        self.sensor = range_sensor
        
        if (not (isinstance(translation_noise, Noise))):
            raise RobotException('translation_noise should be an instance of Noise')
        self.translation_noise = translation_noise
        
        if (not (isinstance(rotation_noise, Noise))):
            raise RobotException('rotation_noise should be an instance of Noise')
        self.rotation_noise = rotation_noise
        
        self.true_pose = RobotPose(0,0,0) # always start at origin
        
    def ApplyAction(self, robot_action, dt):
        '''
        A robot applies robot_action
        ===INPUT===
        robot_action: of class RobotAction, the commanded action to take
        dt: duration of the aciton
        ===OUTPUT===
        the true robot pose after the action
        '''
        if (not (isinstance(robot_action, RobotAction))):
            raise RobotException('robot_action should be an instance of RobotAction')

        real_action = RobotAction(self.translation_noise.GetValue(), \
                                  self.rotation_noise.GetValue())
        real_action = real_action + robot_action

        self.true_pose = real_action.ApplyToPose(self.true_pose, dt)

        return self.true_pose

    def MeasureWorld(self, world):
        '''
        Let the robot measure the environment
        ===INPUT===
        world: of class FeatureBasedWorld, the world to measure
        ===OUTPUT===
        A list of Measurement
        '''
        from world.feature_based_world import FeatureBasedWorld
        if (not (isinstance(world, FeatureBasedWorld))):
            raise RobotException('world should be an instance or FeatureBasedWorld')

        true_measurements = world.GetTrueMeasurements(self.true_pose)
        m = []
        for measurement in true_measurements:
            visible, actual_reading = self.sensor.GetActualReading(measurement)

            if (visible):
                m.append(actual_reading)
                
        return m

if __name__ == "__main__":
    pass
    
