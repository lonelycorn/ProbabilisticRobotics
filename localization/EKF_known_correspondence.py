import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from robot.robot_pose import RobotPose
from world.feature_state import FeatureState
from world.feature_based_world import FeatureBasedWorld
from utility.math import wrap_angle
from localization_interface import LocalizationInterface


class EKF_Localization(LocalizationInterface):
    def get_pose(self):
        '''
        get estimated robot pose
        '''
        return RobotPose(self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]) # x, y, theta

    def set_pose(self, robot_pose):
        '''
        set estimated robot pose
        '''
        self.mu[0, 0] = robot_pose.x
        self.mu[1, 0] = robot_pose.y
        self.mu[2, 0] = robot_pose.theta
        
    def motion_update(self, action, dt):
        '''
        update the state and covar estimate using the action
        c.f page 204, Probabilistic Robotics
        '''

        # delta of pose
        rp = self.get_pose()
        rp_new = rp.ApplyAction(action, dt)
        rp_delta = np.matrix(np.zeros((3,1)))
        rp_delta[0, 0] = rp_new.x - rp.x
        rp_delta[1, 0] = rp_new.y - rp.y
        rp_delta[2, 0] = rp_new.theta - rp.theta

        

        # line 2: the linearized model: y[t+1] ~= g(mu[t], u) + G_t * (y[t] - mu[t])
        G_t = np.matrix(np.eye(3 * self.N + 3))
        G_t[0, 2] = -rp_delta[1, 0]
        G_t[1, 2] =  rp_delta[0, 0]


        # line 6
        self.mu = self.mu + rp_delta
        
        # line 7
        self.sigma = G_t * self.sigma * G_t.T + F_x.T * self.Q * F_x

    
    #======================#
    #     Public API's     #
    #======================#
    def __init__(self):
        self.mu = np.zeros(1)       # state estimate
        self.sigma = np.zeros(1)    # covar estimate
        self.tick = 0               # last time of run
        self.R = np.zeros(1)        # measurement noise covar
        self.Q = np.zeros(1)        # process noise covar
        self.true_map = None        # true map
        self.N = 0                  # number of features
        self.trajectory = []        # trajectory recorded

    def Initialize(self, Q, R, true_map):
        self.tick = 0
        
        self.true_map = true_map
        self.N = true_map.GetFeatureCount()
        self.Q = np.matrix(Q)
        self.R = np.matrix(R)

        # initialize the state estimate.
        # 0, 1, 2: robot (x, y, theta)
        self.mu = np.matrix(np.zeros((3, 1)))
        
        # initialize the covar
        self.sigma = np.matrix(np.eye(3) * 1e4 # know nothing about initial pose

        # initialize trajectory
        self.trajectory = [self.GetPose()]

    def Update(self, t, action, measurements):
        dt = (t - self.tick) * 1.0
        self.tick = t

        self.motion_update(action, dt)

        for m in measurements:
            self.measurement_update(m)

        # save trajectory
        self.trajectory.append(self.GetPose())

        return self.mu

    def Finalize(self):
        # nothing to do
        pass

    def GetTrajectory(self):
        '''
        EKF localization doesn't keep the trajectory in the states, but we saved
        the pose estimates during updates.
        '''
        return self.trajectory
