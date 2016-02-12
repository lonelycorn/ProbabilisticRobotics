import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from scipy.stats import norm
import numpy as np
from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise
from robot.robot_action import RobotAction
from robot.robot_pose import RobotPose
from world.feature_state import FeatureState
from world.feature_based_world import FeatureBasedWorld
from utility.math import wrap_angle
from localization.localization_interface import LocalizationInterface

class PF_Localization(LocalizationInterface):
    class ProcessNoise:
        '''
        '''
        def __init__(self, alpha):
            self.alpha = alpha

        def GetSample(self, pose, action):
            random.normal(self.mu, self.sigma)
            
            return RobotPose(self.noise_x.GetValue(),   \
                             self.noise_y.GetValue(),   \
                             self.noise_theta.GetValue())

    class MeasurementNoise:
        def __init__(self, R):
            self.R = R

        def GetProbability(self, deviation):
            '''
            Get the probability of the given measurement deviation
            === INPUT ===
            deviation: the difference between the true measurement and the \
                       actual measurement.
            === OUTPUT ===
            p: the probability.
            '''
            w = 1.0
            for i in range(3):
                if (self.R[i, i] > 0):
                    w = w * norm.pdf(deviation[i], 0, np.sqrt(self.R[i, i]))

            return w

    class RandomPose:
        def __init__(self, x_range, y_range):
            self.noise_x = UniformNoise(x_range)
            self.noise_y = UniformNoise(y_range)
            self.noise_theta = UniformNoise((-np.pi, np.pi))

        def GetSample(self):
            return RobotPose(self.noise_x.GetValue(),   \
                             self.noise_y.GetValue(),   \
                             self.noise_theta.GetValue())

    class Particle:
        def __str__(self):
            return str(self.pose)+", weight="+str(self.weight)
            
        def __init__(self, pose, weight):
            self.pose = pose
            self.weight = weight

        def GetPose(self):
            return self.pose
            
        def GetState(self):
            '''
            Return the state carried by the particle.
            ===OUTPUT===
            a matrix representing the state.
            '''
            return np.matrix([[self.pose.x], [self.pose.y], [self.pose.theta]])

    def get_random_particle(self):
        '''
        Generate a particle that is sampled random from the pose space.
        ===OUTPUT===
        A Particle with random pose, and weight 1/M.
        '''
        return self.Particle(self.random_pose.GetSample(), 1.0 / self.M)

    def prepare_resampling(self):
        # normalize particles' weights
        weights = [p.weight for p in self.particles]
        scale = 1.0 / sum(weights)

        for p in self.particles:
            p.weight = p.weight * scale

        #cumulative distribution function. cdv[k] = sum_{i=0}^k weight_i
        self.cdf = [0]
        for i in range(0, self.M):
            self.cdf.append(self.cdf[i] + self.particles[i].weight)
        
    def get_resampling(self):
        '''
        Resample the particle population based on particles' weights.
        === OUTPUT ===
        p: the particle resampled.
        '''
        w = np.random.uniform()

        # search for such an index m that w falls into [cdf[m], cdf[m+1])
        l = 0
        r = self.M - 1
        while (l < r):
            m = (l + r) / 2

            if ((self.cdf[m] <= w) and (self.cdf[m + 1] > w)):
                r = m
                break
                
            if (self.cdf[m] < w):
                l = m + 1
            else:
                r = m

        return self.particles[r]
        
        
    def sample_motion_model(self, a, dt, p):
        '''
        Find the robot's pose, given the action and duration, and the particle.
        Using the Velocity Motion Model on page 124 of Probabilistic Robotics.
        ===INPUT===
        a:  of RobotAction.
        dt: duration of a.
        p:  the particle of interest.
        ===OUTPUT===
        a RobotPose.
        '''
        def sample(cov):
            '''
            sample from a zero-mean Gaussian distribution with then given covariance.
            '''
            if (cov > 0):
                return np.random.normal(0, np.sqrt(cov))
            else:
                return 0

        # c.f. page 124
        cov_v = self.alpha[0]*a.v*a.v + self.alpha[1]*a.w*a.w
        cov_w = self.alpha[2]*a.v*a.v + self.alpha[3]*a.w*a.w
        cov_gamma = self.alpha[4]*a.v*a.v + self.alpha[5]*a.w*a.w

        action_noise = RobotAction(sample(cov_v), sample(cov_w))

        a = a + RobotAction(sample(cov_v), sample(cov_w))
        
        rp = p.pose.ApplyAction(a, dt)
        rp.theta = rp.theta + sample(cov_gamma)*dt

        return rp

    def measurement_model(self, measurements, pose):
        '''
        Compute the probability of the particle and the measurements given
        ===INPUT===
        measurement: a list of Measurement
        pose: the proposed pose
        ===OUTPUT===
        a decimal representing the probability
        '''
        w = 1.0

        for m in measurements:
            j = m.s # assuming the signature gives the correspondence
            
            fs = self.true_map.GetFeature(j)
            delta_x = fs.x - pose.x
            delta_y = fs.y - pose.y
            
            d = np.sqrt(delta_x * delta_x + delta_y * delta_y)
            b = np.arctan2(delta_y, delta_x) - pose.theta
            
            # deviation from expected measurement
            n_r = d - m.r
            n_phi = wrap_angle(b - m.phi)
            n_s = fs.s - m.s

            # compute probability of measurement. 
            w = w * self.measurement_noise.GetProbability([n_r, n_phi, n_s])

        return w
        
    def get_pose(self):
        '''
        Calculate the mean pose from the particles.
        '''
        sum_pose = np.matrix(np.zeros((3, 1)))
        sum_weight = 0.0
        for p in self.particles:
            sum_pose = sum_pose + p.GetState() * p.weight
            sum_weight = sum_weight + p.weight
        avg = sum_pose / sum_weight
        return RobotPose(avg[0, 0], avg[1, 0], avg[2, 0])

    def plot_particles(self, fig_title, color='r'):
        '''
        For debugging purposes only.
        '''
        import matplotlib.pyplot as plt
        fig_id = 1
        self.dash_size = 0.05
        self.line_width = 1
        self.map_size = 5
        self.pose_size = 3
        self.feature_size= 12
        
        plt.figure(fig_id)

        if (self.particles is not None):
            for p in self.particles:
                rp = p.GetPose()
                
                # position
                plt.plot(rp.x, rp.y, color+'o', markersize = self.pose_size)
                # heading dash
                plt.plot([rp.x, rp.x + np.cos(rp.theta) * self.dash_size], \
                         [rp.y, rp.y + np.sin(rp.theta) * self.dash_size], \
                         color+'-', linewidth = self.line_width)
        rp = self.true_pose
        # position
        plt.plot(rp.x, rp.y, 'rx', markersize = self.pose_size)
        # heading dash
        plt.plot([rp.x, rp.x + np.cos(rp.theta) * self.dash_size], \
                 [rp.y, rp.y + np.sin(rp.theta) * self.dash_size], \
                 'r-', linewidth = self.line_width)

        if (self.true_map is not None):
            for f in self.true_map.features:
                plt.plot(f.x, f.y, color+'p', markersize = self.feature_size)

        plt.axis([-self.map_size, self.map_size, -self.map_size, self.map_size])
        plt.title(fig_title)

        plt.show()

    def SetTruePose(self, p):
        self.true_pose = p


    #======================#
    #     Public API's     #
    #======================#
    def __init__(self, M):
        self.M = M                      # number of particles
        self.particles = None           # particles in state space
        self.tick = None                # last time of run
        self.measurement_noise = None   # measurement noise generator
        self.alpha = None               # process noise parameters
        self.random_pose = None         # random pose generator
        self.true_map = None            # true map
        self.trajectory = None          # trajectory recorded

    def Initialize(self, alpha, R, true_map):
        self.tick = 0

        self.alpha = alpha
        self.measurement_noise = self.MeasurementNoise(R)
        self.true_map = true_map

        x_range, y_range = true_map.GetBounds()
        self.random_pose = self.RandomPose(x_range, y_range)
        
        # initialize the particles
        self.particles = list()
        for i in range(self.M):
            self.particles.append(self.Particle(RobotPose(0, 0, 0), 1.0 / self.M))
            #self.particles.append(self.get_random_particle())
        
        # initialize trajectory
        self.trajectory = [self.get_pose()]
       

    def Update(self, t, action, measurements):
        '''
        c.f. p252 of Probabilistic Robotics
        '''
        dt = (t - self.tick) * 1.0
        self.tick = t

        print("=== PF update at t = " + str(t) + " ===")
        print("dt = " + str(dt))
        print(str(action))

        # line 2
        new_particles = list()
        for p in self.particles:
            # line 4
            pose = self.sample_motion_model(action, dt, p)
            
            # line 5
            weight = self.measurement_model(measurements, pose)
            
            # line 6
            new_particles.append(self.Particle(pose, weight))

        self.particles = new_particles
        #self.plot_particles("t="+str(self.tick)+", before resampling",'k')

        # line 8 ~ 11, resampling
        new_particles = list()
        self.prepare_resampling()
        for i in range(self.M):
            p = self.get_resampling()
            new_particles.append(p)

        self.particles = new_particles
        #self.plot_particles("t="+str(self.tick)+", after resampling",'b')

        # save trajectory
        rp = self.get_pose()
        self.trajectory.append(rp)

        return rp

    def Finalize(self):
        # nothing to do
        pass

    def GetTrajectory(self):
        '''
        EKF localization doesn't keep the trajectory in the states, but we saved
        the pose estimates during updates.
        '''
        return self.trajectory

    def TestProcessModel(self):
        M = 100
        dt = 1.0
        weight = 1.0
        p = self.Particle(RobotPose(0.0, 0.0, 0.0), weight)
        new_particles = list()
        action = RobotAction(1.0, 0.0)
        
        self.alpha = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        
        for i in range(M):
            pose = self.sample_motion_model(action, dt, p)
            new_particles.append(self.Particle(pose, weight))

        self.particles = new_particles
        self.true_pose = p.pose
        self.plot_particles("process model")


if __name__ == '__main__':
    pass
