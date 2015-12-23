import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from SLAM_algorithm import SLAM_Algorithm
from robot.robot import Robot
from robot.robot_pose import RobotPose
from robot.robot_action import RobotAction
from noise.noise import Noise
from world.feature_based_world import FeatureBasedWorld
from world.feature_state import FeatureState

class SimulatorException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class Simulator:
    def __init__(self, SLAM_algorithm, map_filename, robot_filename, action_history_filename):
        self.SLAM_algorithm = SLAM_algorithm
        self.robot = Robot()
        self.world = FeatureBasedWorld()
        self.ClearHistory()

        self.robot.LoadFromFile(robot_filename)
        self.world.LoadFromFile(map_filename)
        self.LoadActionHistory(action_history_filename)

    def ClearHistory(self):
        self.history_length = 0
        self.time_history = []
        self.action_history = []
        self.true_pose_history = []

    def LoadActionHistory(self, filename):
        '''
        Load action history from the given file
        '''
        fd = open(filename, "r")

        # robot initial pose
        x, y, theta = [float(x) for x in fd.readline().split()]
        self.robot.SetInitialPose(RobotPose(x, y, theta))

        # number of steps
        self.history_length = [int(x) for x in fd.readline().split()][0]

        # initialize time_history and action_history
        #~ self.time_history.append(0)
        #~ self.action_history.append(RobotAction(0,0))
        
        for i in range(self.history_length):
            t, v, w = [float(x) for x in fd.readline().split()]
            self.time_history.append(t)
            self.action_history.append(RobotAction(v, w))

    def RobotExecuteAction(self, action, dt):
        '''
        update the actual robot pose using the given action
        ===INPUT===
        action: of class RobotAction, the action to execute
        dt: duration of the action
        ===OUTPUT===
        The true pose of the robot after the action
        '''
        return self.robot.ApplyAction(action, dt)

    def RobotGetMeasurements(self):
        '''
        get the measurements from the robot
        ===OUTPUT===
        A list of Measurement, namely what the robot observes with its latest
        pose
        '''
        return self.robot.MeasureWorld(self.world)

    def Run(self):
        '''
        Run the simulation
        '''
        print('Running Simulation')

        R = np.diag([1.0, 1.0, 1.57]) # process noise; (x, y, theta). should be over-estimate
        Q = np.diag([0.01, 0.01, 0.01]) # measurement noise; (dist, bearing, signature),

        # start at t = 0
        self.SLAM_algorithm.Initialize(0, \
                                       self.robot.GetTruePose(), \
                                       Q,
                                       R,
                                       self.world.GetFeatureCount())
                                       
        self.true_pose_history.append(self.robot.GetTruePose())
        
        # number of steps
        for k in range(self.history_length - 1):
            
            dt = self.time_history[k+1] - self.time_history[k] # duration
            t = self.time_history[k+1] # timestamp

            print("k = " + str(k) + ", t = " + str(t))

            a = self.action_history[k] # action

            # robot applies action
            p = self.RobotExecuteAction(a, dt) 
            self.true_pose_history.append(p) # save history
            print("true " + str(p))

            # robot measures world
            z = self.RobotGetMeasurements()
            print("robot measures :")
            for m in z:
                print(str(m))

            # call SLAM update
            self.SLAM_algorithm.Update(t, a, z)
            
            #~ raw_input("Press Enter to continue...")

        # visualize results
        self.PlotResults(self.true_pose_history, \
                         self.world.features, \
                         self.SLAM_algorithm.GetTrajectory(), \
                         self.SLAM_algorithm.GetMap())

    def PlotResults(self, true_trajectory, true_map, SLAM_trajectory, SLAM_map):
        import matplotlib.pyplot as plt
        import numpy as np
        dash_size = 0.5
        map_size = 5
        
        plt.figure(1)
        print("True trajectory")
        for rp in true_trajectory:
            x = rp.x
            y = rp.y
            theta = rp.theta
            plt.plot([x,x+np.cos(theta)*dash_size], [y,y+np.sin(theta)*dash_size], \
                     'g-', linewidth=2)
            plt.plot(x,y,'go',markersize=10)
            print(x,y,theta)
            
        for f in true_map:
            x = f.x
            y = f.y
            plt.plot(x,y,'gp',markersize=20)

        plt.axis([-map_size,map_size,-map_size,map_size])
        plt.title("real trajectory")

        plt.figure(2)
        print("Estimated trajectory")
        for rp in SLAM_trajectory:
            x = rp.x
            y = rp.y
            theta = rp.theta
            plt.plot([x,x+np.cos(theta)*dash_size], [y,y+np.sin(theta)*dash_size], \
                     'r-', linewidth=2)
            plt.plot(x,y,'ro',markersize=10)
            print(x,y,theta)

        for f in SLAM_map.features:
            x = f.x
            y = f.y
            plt.plot(x,y,'rp',markersize=20)

        plt.axis([-map_size,map_size,-map_size,map_size])
        plt.title("SLAM trajectory")

        plt.show(1)
        plt.show(2)
        
        
        
if __name__ == '__main__':
    pass
