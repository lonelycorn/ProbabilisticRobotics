import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from robot.robot_pose import RobotPose
from world.feature_state import FeatureState
from world.feature_based_world import FeatureBasedWorld
from SLAM_interface import SLAM_Interface
from utility.math import wrap_angle
from utility.math import INFINITY
from utility.io import print_matrix

class Packet:
    def __init__(self, t = None, dt = None, action = None, measurements = None):
        self.t = t
        self.dt = dt
        self.action = action
        self.measurements = measurements

class GraphSLAM(SLAM_Interface):
    #===================#
    #     Utilities     #
    #===================#
    def get_action(self, u_1t, dt_1t, k):
        '''
        Get the action at step k.
        === INPUT ===
        k: in range 1..T
        === OUTPUT ===
        a: RobotAction
        dt: duration of the action.
        '''
        return u_1t[k-1], dt_1t[k-1]

    def get_measurements(self, z_1t, k):
        '''
        Get the measurements at step k.
        === INPUT ===
        k: in range 1..T
        === OUTPUT ===
        a list of Measurement
        '''
        return z_1t[k-1]

    def get_correspondence(self, c_1t, k):
        '''
        Get the measurement correspondences at step k.
        === INPUT ===
        k: in range 1..T
        === OUTPUT ===
        a list of integers.
        '''
        return c_1t[k-1]

    def get_pose(self, mu_0t, k):
        '''
        Get the robot pose at step k.
        === INPUT ===
        k: in range 0..T
        === OUTPUT ===
        a 3-by-1 matrix
        '''
        return mu_0t[(3*k):(3*k+3), :]

    def set_pose(self, mu_0t, k, pose):
        '''
        Set the robot pose at step k
        '''
        pass

    def get_feature(self, mu_0t, k):
        '''
        Get map feature k.
        === INPUT ===
        k: in range 0..N-1
        === OUTPUT ===
        A 3-by-1 matrix
        '''
        return mu_0t[(3*self.T+3+3*k):(3*self.T+3+3*k+3), :]

    def set_feature(self, mu_0t, k, feature):
        '''
        Set map feature k.
        NOTE: tested
        === INPUT ===
        k: in range 0..N-1
        feature: a FeatureState
        '''
        mu_0t[(3*self.T+3+3*k+0), 0] = feature.x
        mu_0t[(3*self.T+3+3*k+1), 0] = feature.y
        mu_0t[(3*self.T+3+3*k+2), 0] = feature.s

    #=========================#
    #     Private Methods     #
    #=========================#
    def get_pose_selection_matrix(self, k):
        ''' 
        Get the selection matrix for robot pose at step k.
        Note: tested.
        === INPUT ===
        k: an integer 0..T
        === OUTPUT ===
        A 3-by-(3*T+3+3*N) matrix F.
        To Map a 3-by-3 matrix M_33 to a (3*T+3+3*N)-by-(3*T+3+3*N) matrix, use
            F.T() * M_33 * F
        '''
        m = np.matrix(np.zeros((3, 3*self.T+3+3*self.N)))
        # robot pose at step k
        m[0, 3*k+0] = 1.0 
        m[1, 3*k+1] = 1.0 
        m[2, 3*k+2] = 1.0 

        return m

    def get_feature_selection_matrix(self, k):
        '''
        Get the seletion matrix for map feature k.
        Note: tested.
        === INPUT ===
        k: an integer 0..N-1
        === OUTPUT ===
        A 3-by-(3*T+3+3*N) matrix F.
        To Map a 3-by-3 matrix M_33 to a (3*T+3+3*N)-by-(3*T+3+3*N) matrix, use
            F.T() * M_33 * F
        '''
        m = np.matrix(np.zeros((3, 3*self.T+3+3*self.N)))
        # map feature k
        m[0, 3*self.T+3+3*k+0] = 1.0
        m[1, 3*self.T+3+3*k+1] = 1.0 
        m[2, 3*self.T+3+3*k+2] = 1.0 

        return m
        
    def get_motion_update_selection_matrix(self, k):
        '''
        Get the selection matrix for motion update from step (k-1) to step k.
        '''
        m = np.matrix(np.zeros((6, 3*self.T+3+3*self.N)))
        # robot pose at step k-1
        m[0:3, :] = self.get_pose_selection_matrix(k-1)
        # robot pose at  step k
        m[3:6, :] = self.get_pose_selection_matrix(k)

        return m

    def get_measurement_update_selection_matrix(self, t, j):
        '''
        Get the selection matrix for measurement update at step t on feature j.
        '''
        m = np.matrix(np.zeros((6, 3*self.T+3+3*self.N)))
        # robot pose at step t 
        m[0:3, :] = self.get_pose_selection_matrix(t)
        # map feature j
        m[3:6, :] = self.get_feature_selection_matrix(j)

        return m


    def initialize(self, u_1t, dt_1t):
        '''
        Create initial-guess for the full slame state.
        The robot poses are predicted open-loop with actions.
        The map features are left blank.
        c.f. p347, Probabilistic Robotics
        === INPUT ===
        u_1t: a list of RobotAction. The k-th item is the action at step k+1.
        dt_1t: a list of floats. The k-th item is the duration for the action at step k+1.
        === OUTPUT ===
        mu_0t: a (3*T+3+3*N)-by-1 matrix, i.e. the full slam state. 
              Rows 0:(3*T+3) are robot poses, while rows (3*T+3):(3*T+3+3*N) are map features.
        '''
        result = np.matrix(np.zeros((3*self.T+3+3*self.N, 1)))

        # line 2. 
        mu = RobotPose(0.0, 0.0, 0.0) # Always start at the origin.
        result[0:3, :] = mu.ToMatrix()

        # line 3, robot poses
        for t in range(1, self.T+1):
            a, dt = self.get_action(u_1t, dt_1t, t)

            # line 4.
            mu = mu.ApplyAction(a, dt)
            result[3*t:3*t+3, :] =  mu.ToMatrix()

        # rows (3*T+3):(3*T+3+3*N) are empty because no estimates 
        # map features are available right now.

        return result 

    def linearize(self, u_1t, z_1t, c_1t, mu_0t):
        '''
        Linearize the constraints (neg log likelihoods) around the given state estimate.
        c.f. p347 ~ p348, Probabilistic Robotics
        === INPUT ===
        u_1t: a list of RobotAction. The k-th item is the action at step k+1.
        z_1t: a list of (a list of Measurement). The k-th item contains the observations at step k+1.
        c_1t: a list of (a list of integers). The k-th item contains the correspondence for the observations at step k+1.
        mu_0t: a (3*T+3+3*N)-by-1 matrix, i.e. the full SLAM state.
              Rows 0:(3*T+3) are robot poses, while rows (3*T+3):(3*T+3+3*N) are map features.
        === OUTPUT ===
        omega: a (3*T+3+3*N)-by-(3*T+3+3*N) matrix, i.e. the informatio matrix.
        ksi: a (3*T+3+3*N)-by-1 matrix, i.e. the information vector.
        '''
        num_states = self.T*3 + 3 + self.N*3

        # line 2
        omega = np.matrix(np.zeros((num_states, num_states))) 
        ksi = np.matrix(np.zeros((num_states, 1))) 

        # line 3. Fix the origin.
        omega[0:3, 0:3] = np.diag([INFINITY, INFINITY, INFINITY])
        rp = RobotPose(0.0, 0.0, 0.0) # change this

        # line 4
        for t in range(1, self.T+1):
            mu_last = self.get_pose(mu_0t, t-1) # robot pose at previous step

            # line 5
            x_t_hat = self.get_pose(mu_0t, t)
            rp_delta = x_t_hat - mu_last

            # line 6
            G_t = np.matrix(np.eye(3))
            G_t[0, 2] = -rp_delta[1, 0]
            G_t[1, 2] =  rp_delta[0, 0]

            # line 7
            tmp = np.matrix(np.zeros((3, 6)))
            tmp[0:3, 0:3] = -G_t
            tmp[0:3, 3:6] = np.eye(3)
            F_t = self.get_motion_update_selection_matrix(t)
            omega += F_t.T * tmp.T * self.R_inv * tmp * F_t # TODO: find a way to do local update

            # line 8
            ksi += F_t.T * tmp.T * self.R_inv * (x_t_hat - G_t * mu_last)

        # line 10
        for t in range(1, self.T+1):
            z_t = self.get_measurements(z_1t, t) # measurements at step t
            c_t = self.get_correspondence(c_1t, t) # correspondences at step t 
            mu_t = self.get_pose(mu_0t, t) # robot pose at step t in matrix form

            # line 12
            for i in range(len(z_t)):
                z_t_i = z_t[i].ToMatrix()

                # line 13
                j = c_t[i]

                # if first seen then add feature to state. NOTE: this part is missing in the book
                if (not self.feature_seen[j]):
                    f = FeatureState(mu_t[0, 0] + z_t_i[0, 0] * np.cos(z_t_i[1, 0] + mu_t[2, 0]), \
                                     mu_t[1, 0] + z_t_i[0, 0] * np.sin(z_t_i[1, 0] + mu_t[2, 0]), \
                                     z_t_i[2, 0])

                    self.set_feature(mu_0t, j, f)
                    self.feature_seen[j] = True

                    print("newly observed " + str(f))

                mu_j = self.get_feature(mu_0t, j) # map feature in matrix form

                # line 14
                delta_x = mu_j[0, 0] - mu_t[0, 0]
                delta_y = mu_j[1, 0] - mu_t[1, 0]

                # line 15
                q = delta_x * delta_x + delta_y * delta_y

                # line 16
                z_t_i_hat = np.matrix([[np.sqrt(q)], \
                                       [wrap_angle(np.arctan2(delta_y, delta_x) - mu_t[2, 0])], \
                                       [z_t_i[2, 0]]])
                # line 17
                tmp = np.matrix(np.zeros((3, 6)))
                tmp[0, 0] = -delta_x * np.sqrt(q)
                tmp[0, 1] = -delta_y * np.sqrt(q)
                tmp[0, 3] =  delta_x * np.sqrt(q)
                tmp[0, 4] =  delta_y * np.sqrt(q)
                tmp[1, 0] =  delta_y 
                tmp[1, 1] = -delta_x 
                tmp[1, 2] = -q
                tmp[1, 3] = -delta_y
                tmp[1, 4] =  delta_x
                tmp[2, 5] =  q
                H_t_i = tmp / q

                # line 18
                F_xj = self.get_measurement_update_selection_matrix(t, j)
                omega += F_xj.T * H_t_i.T * self.Q_inv * H_t_i * F_xj # TODO: find a way to do local update

                # line 19
                tmp = np.matrix(np.zeros((6, 1)))
                tmp[0:3, :] = mu_t
                tmp[3:6, :] = mu_j
                ksi += F_xj.T * H_t_i.T * self.Q_inv * (z_t_i - z_t_i_hat + H_t_i * tmp)

        return omega, ksi

    def reduce(self, omega, ksi):
        '''
        Reduce the size of the graph, i.e. marginalize out all map feature nodes.
        === INPUT ===
        omega, ksi: the information matrix and information vector for the original
                    graph.
        === OUTPUT ===
        omega_tilda, ksi_tilda: the information matrix and information vector for 
                                the reduced graph.
        '''
        # for now we don't marginalize out the landmark nodes.
        return omega, ksi

    def solve(self, omega_tilda, ksi_tilda, omega, ksi):
        '''
        Solve the linearized reduced graph for robot pose history.
        === INPUT ===
        omega_tilda, ksi_tilda: the information matrix and information vector for
                                the reduced graph.
        omega, ksi: the information matrix and information vector for the original
                    graph.
        === OUTPUT ===
        mu_0t: the full slame state estimate
        sigma_0t: the covar for the estimate
        '''
        # since we didn't marginalize out the map features, we could solve for the
        # state estimate and estimate covar easily.
        sigma = np.linalg.inv(omega_tilda)
        mu = sigma * ksi_tilda
        return mu, sigma


    #======================#
    #     Public API'S     #
    #======================#
    def __init__(self):
        self.tick = None
        self.packets = None
        self.mu = None

    def Initialize(self, Q, R, T, N):
        '''
        ===INPUT===
        Q: process noise covar.
        R: sensor noise covar.
        T: number of simulation steps. (0..T)
        N: number of map features.
        '''
        
        self.tick = 0
        self.packets = []

        self.R = R # is this necessary?
        self.R_inv = np.linalg.inv(R)

        self.Q = Q # is this necessary?
        self.Q_inv = np.linalg.inv(Q)

        self.T = T
        self.N = N
        self.feature_seen = [False for i in range(N)] # whether a feature has been seen

    def Update(self, t, action, measurements):
        '''
        Just save the measurements and actions for later processing.
        '''
        if (t <= self.tick):
            raise SLAM_Exception("Time must be monotonically increasing.")

        if (action is None):
            raise SLAM_Exception("Missing action.")

        if (measurements is None):
            # we allow empty measurements
            raise SLAM_Exception("Missing measurements.")

        if (len(self.packets) >= self.T):
            raise SLAM_Exception("Simulation steps exceeds limit.")

        dt = t - self.tick
        self.tick = t
        
        self.packets.append(Packet(t, dt, action, measurements))

        # FIXME: can we get online pose estimates?
        return None

    def Finalize(self):
        '''
        Iteratively solve for the pose history and map features.
        c.f. p350, Probabilistic Robotics.
        '''
        # construct the 
        u_1t = []
        dt_1t = []
        z_1t = []
        c_1t = []
        for p in self.packets:
            u_1t.append(p.action)
            dt_1t.append(p.dt)
            z_1t.append(p.measurements)
            c = [m.s for m in p.measurements] # correspondence is known
            c_1t.append(c)

        print("=====actions:=====")
        print("length = %d" % len(u_1t))
        for u in u_1t:
            print(u)
        print("=====dt's:=====")
        print("length = %d" % len(dt_1t))
        print(dt_1t)
        # print("=====measurements:=====")
        # for z in z_1t:
        #     for m in z:
        #         print(m)
        #     print("---")
        # print("=====correspondences:=====")
        # print(c_1t)

        mu_0t = self.initialize(u_1t, dt_1t) 
        
        for i in range(10): # TODO: repeat until convergence
            print("<----- iteration " +str(i) + "----->")
            omega, ksi = self.linearize(u_1t, z_1t, c_1t, mu_0t)
            print("before reduce()")
            print_matrix(omega, "omega")
            print_matrix(ksi, "ksi")
            omega_tilda, ksi_tilda = self.reduce(omega, ksi)
            mu_0t, sigma_0t = self.solve(omega_tilda, ksi_tilda, omega, ksi)

        self.mu = mu_0t

    def GetMap(self):
        if (self.mu is None):
            raise SLAM_Exception("No estimates available.")

        w = FeatureBasedWorld()
        for i in range(self.N):
            f = self.get_feature(self.mu, i)
            fs = FeatureState(f[0, 0], f[1, 0], f[2, 0])
            w.AppendFeature(fs)
        return w

    def GetTrajectory(self):
        if (self.mu is None):
            raise SLAM_Exception("No estimates available.")

        t = []
        for i in range(self.T+1):
            p = self.get_pose(self.mu, i)
            rp = RobotPose(p[0, 0], p[1, 0], p[2, 0])
            t.append(rp)

        return t
        
if __name__ == '__main__':
    pass
