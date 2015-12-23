import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from robot.robot_pose import RobotPose
from world.feature_state import FeatureState
from world.feature_based_world import FeatureBasedWorld
from SLAM_algorithm import SLAM_Algorithm
from utility.math import wrap_angle


class EKF_SLAM(SLAM_Algorithm):
    def get_feature_selection_matrix(self, k):
        '''
        get the selection matrix for feature k
        '''
        m = np.matrix(np.zeros((6, 3*self.N+3)))
        # robot pose
        m[0, 0] = 1
        m[1, 1] = 1
        m[2, 2] = 1
        # feature k
        m[3, 3*k+3] = 1
        m[4, 3*k+4] = 1
        m[5, 3*k+5] = 1
        
        return m

    def get_pose_selection_matrix(self):
        '''
        get the selection matrix for robot pose
        '''
        return np.matrix(np.eye(3, 3 * self.N + 3))

    def motion_update(self, action, dt):
        '''
        update the state and covar estimate using the action
        c.f page 314, Probabilistic Robotics
        '''
        # delta of pose
        rp = self.GetPose()
        rp_new = rp.ApplyAction(action, dt)
        rp_delta = np.matrix(np.zeros((3,1)))
        rp_delta[0, 0] = rp_new.x - rp.x
        rp_delta[1, 0] = rp_new.y - rp.y
        rp_delta[2, 0] = rp_new.theta - rp.theta
        
        # line 2
        F_x = self.get_pose_selection_matrix()

        # line 3
        self.mu = self.mu + F_x.T * rp_delta

        # line 4: the linearized model: y[t+1] ~= g(mu[t], u) + G_t * (y[t] - mu[t])
        rp_delta_Jacobian = np.matrix(np.zeros((3,3)))
        rp_delta_Jacobian[0, 2] =-rp_delta[1, 0]
        rp_delta_Jacobian[1, 2] = rp_delta[0, 0]
        G_t = np.matrix(np.eye(3 * self.N + 3)) + F_x.T * rp_delta_Jacobian * F_x

        # line 5
        self.sigma = G_t * self.sigma * G_t.T + F_x.T * self.R * F_x

    def measurement_update(self, measurement):
        '''
        update the state and covar estimate using the measurement
        c.f page 314, Probabilistic Robotics
        '''
        rp = self.GetPose()

        # convert measurement into a matrix
        z = np.matrix([[measurement.r], \
                       [measurement.phi], \
                       [measurement.s]])
        
        # line 8
        j = measurement.s # assuming the signature gives the correspondence
        print("correspondence = " + str(j))

        # line 9
        if (not self.feature_seen[j]):
            f = FeatureState(rp.x + measurement.r * np.cos(measurement.phi + rp.theta), \
                             rp.y + measurement.r * np.sin(measurement.phi + rp.theta), \
                             measurement.s)
            # line 10, initialize feature state
            self.SetFeature(j, f)
            self.feature_seen[j] = True
            
            print("newly observed " + str(f))
            #~ print("mu = ")
            #~ print(self.mu)

        fs = self.GetFeature(j)

        # line 12
        delta_x = fs.x - rp.x
        delta_y = fs.y - rp.y

        # line 13
        q = delta_x * delta_x + delta_y * delta_y
        print("delta_x = " + str(delta_x) + " delta_y = " + str(delta_y) + " q= " + str(q))

        # line 14, predicted measurement
        z_hat = np.matrix([[np.sqrt(q)], \
                           [wrap_angle(np.arctan2(delta_y, delta_x) - rp.theta)], \
                           [fs.s]])
        #FIXME: this np.arctan2 - rp.theta is creating problems
        print("z_hat = ")
        print(z_hat)

        # line 15
        F_xj = self.get_feature_selection_matrix(j)

        # line 16
        tmp = np.matrix(np.zeros((3, 6)))
        tmp[0, 0] = -delta_x * np.sqrt(q)
        tmp[0, 1] = -delta_y * np.sqrt(q)
        tmp[0, 3] =  delta_x * np.sqrt(q)
        tmp[0, 4] =  delta_y * np.sqrt(q)
        tmp[1, 0] = -delta_y 
        tmp[1, 1] = -delta_x 
        tmp[1, 2] = -q
        tmp[1, 3] = -delta_y
        tmp[1, 4] =  delta_x
        tmp[2, 5] =  q
        #~ print("tmp=")
        #~ print(tmp)
        H_t = tmp * F_xj / q
        #~ print("H_t=")
        #~ print(H_t)
        
        
        # line 17
        #~ print("Innovation=")
        #~ print(H_t * self.sigma * H_t.T + self.Q)
        K_t = self.sigma * H_t.T * np.linalg.inv(H_t * self.sigma * H_t.T + self.Q)
        #~ print("K_t=")
        #~ print(K_t)
        
        # line 18
        #~ print("K_t*(z-z_hat)")
        #~ print(K_t*(z-z_hat))
        
        self.mu = self.mu + K_t * (z - z_hat)

        print("mu=")
        print(self.mu)

        # line 19
        self.sigma = self.sigma - K_t * H_t * self.sigma

        

    #======================#
    #     Public API's     #
    #======================#
    def __init__(self):
        self.mu = np.zeros(1) # state estimate
        self.sigma = np.zeros(1) # covar estimate
        self.t_last = 0 # last time of run
        self.Q = np.zeros(1) # process noise covar
        self.R = np.zeros(1) # sensor noise covar
        self.N = 0 # number of features
        self.trajectory = []

    def Initialize(self, t0, rp, Q, R, N):
        '''
        ===INPUT===
        t0: initial time stamp
        rp: initial robot pose
        Q: process noise covar
        R: sensor noise covar
        N: number of features
        ===OUTPUT===
        none
        '''
        self.t_last = t0
        self.N = N
        self.Q = np.matrix(Q)
        self.R = np.matrix(R)

        # initialize the state estimate.
        # 0, 1, 2: robot x, y, theta
        # 2*k+3, 2*k+4: x, y of feature k (0..N-1)
        self.mu = np.matrix(np.zeros((3 * N + 3, 1)))
        self.SetPose(rp)
        
        # initialize the covar
        self.sigma = np.matrix(np.eye(3 * N + 3)) * 1e4 # know nothing about map features. how about signatures?
        self.sigma[0,0] = 0.0 # we don't want the origin to shift
        self.sigma[1,1] = 0.0
        self.sigma[2,2] = 0.0

        # mark all features as never seen
        self.feature_seen = [False for i in range(N)]

        # empty trajectory
        self.trajectory = []
        self.trajectory.append(rp)

    def Update(self, t, action, measurements):
        print("EKF Update, t = " + str(t) + ", t_last = " + str(self.t_last))
        
        dt = (t - self.t_last) * 1.0
        self.t_last = t
        
        self.motion_update(action, dt)

        for m in measurements:
            self.measurement_update(m)

        # save trajectory
        self.trajectory.append(self.GetPose())

        return self.mu
        
    def Finalize(self):
        # nothing to do
        pass

    def GetFeature(self, i):
        return FeatureState(self.mu[3+3*i, 0], \
                            self.mu[4+3*i, 0], \
                            self.mu[5+3*i, 0]) # x, y, s

    def SetFeature(self, i, feature_state):
        self.mu[3+3*i, 0] = feature_state.x
        self.mu[4+3*i, 0] = feature_state.y
        self.mu[5+3*i, 0] = feature_state.s
        
    def GetMap(self):
        w = FeatureBasedWorld()
        for i in range(self.N):
            w.AppendFeature(self.GetFeature(i))
        return w

    def GetPose(self):
        return RobotPose(self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]) # x, y, theta

    def SetPose(self, robot_pose):
        self.mu[0, 0] = robot_pose.x
        self.mu[1, 0] = robot_pose.y
        self.mu[2, 0] = robot_pose.theta

    def GetTrajectory(self):
        '''
        EKF SLAM doesn't keep the trajectory in the states, but we saved the 
        pose estimates during updates.
        '''
        return self.trajectory

if __name__ == '__main__':
    pass
