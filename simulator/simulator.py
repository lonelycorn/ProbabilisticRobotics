import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from robot.robot import Robot
from robot.robot_action import RobotAction
from world.feature_based_world import FeatureBasedWorld

class SimulatorException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class Simulator:
    # This class simulates a robot moving in a world. It updates the robot's
    # state and measurements every tick.
    def __init__(self, map_filename, robot_filename):
        self.robot = Robot()
        self.world = FeatureBasedWorld()
        self.tick = 0.0
        self.pose_history = [] # ground truth trajectory

        self.robot.LoadFromFile(robot_filename)
        self.world.LoadFromFile(map_filename)

    def Update(self, t, action):
        '''
        Update the simulator.
        ===INPUT===
        t: the time of update.
        action: the active action from last Update() to t.
        ===OUTPUT===
        A list of Measurement, which the robot observes at tick t.
        '''
        dt = t - self.tick
        self.tick = t
        rp = self.robot.ApplyAction(action, dt)
        self.pose_history.append(rp)
        return self.robot.MeasureWorld(self.world)

    def GetTrajectory(self):
        '''
        Get the ground truth trajectory.
        '''
        return self.pose_history

    def GetMap(self):
        '''
        Get the ground truth map.
        '''
        return self.world

if __name__ == '__main__':
    pass
