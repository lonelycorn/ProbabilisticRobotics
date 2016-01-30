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
        G_t = np.matrix(np.eye(3))
        G_t[0, 2] = -rp_delta[1, 0]
        G_t[1, 2] =  rp_delta[0, 0]

        # line 6
        self.mu = self.mu + rp_delta
        
        # line 7, note our Q is in pose space, rather than control space
        self.sigma = G_t * self.sigma * G_t.T + self.Q


    def measurement_update(self, measurement):
        rp = self.get_pose()

        # convert measurement into a matrix
        z = np.matrix([[measurement.r], \
                       [measurement.phi], \
                       [measurement.s]])
                       
        # line 10
        j = measurement.s # assuming the signature gives the correspondence

        # line 11
        fs = self.true_map.GetFeature(j)
        delta_x = fs.x - rp.x
        delta_y = fs.y - rp.y
        q = delta_x * delta_x + delta_y * delta_y

        # line 12, predicted measurement
        z_hat = np.matrix([[np.sqrt(q)], \
                           [wrap_angle(np.arctan2(delta_y, delta_x) - rp.theta)], \
                           [fs.s]])

        # line 13
        H_t = np.matrix(np.eye(3))
        H_t[0, 0] = -delta_x / np.sqrt(q)
        H_t[0, 1] = -delta_y / np.sqrt(q)
        H_t[1, 0] =  delta_y / q
        H_t[1, 1] =  delta_x / q
        H_t[1, 2] = -1.0

        # line 14
        S = H_t * self.sigma * H_t.T + self.R

        # line 15
        K_t = self.sigma * H_t.T * np.linalg.inv(S)
        
        # line 16
        self.mu = self.mu + K_t * (z - z_hat)

        # line 17
        self.sigma = self.sigma - K_t * H_t * self.sigma
    
    #======================#
    #     Public API's     #
    #======================#
    def __init__(self):
        self.mu = None          # state estimate
        self.sigma = None       # covar estimate
        self.tick = None        # last time of run
        self.R = None           # measurement noise covar
        self.Q = None           # process noise covar
        self.true_map = None    # true map
        #self.N = None           # number of features
        self.trajectory = None  # trajectory recorded

    def Initialize(self, Q, R, true_map):
        self.tick = 0
        
        self.true_map = true_map
        #self.N = true_map.GetFeatureCount()
        self.Q = np.matrix(Q)
        self.R = np.matrix(R)

        # initialize the state estimate.
        # 0, 1, 2: robot (x, y, theta)
        self.mu = np.matrix(np.zeros((3, 1)))
        
        # initialize the covar
        self.sigma = np.matrix(np.eye(3)) * 1e4 # know nothing about initial pose

        # initialize trajectory
        self.trajectory = [self.get_pose()]

    def Update(self, t, action, measurements):
        dt = (t - self.tick) * 1.0
        self.tick = t

        self.motion_update(action, dt)

        for m in measurements:
            self.measurement_update(m)

        # save trajectory
        self.trajectory.append(self.get_pose())

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
