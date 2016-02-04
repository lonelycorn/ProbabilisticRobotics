import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from scipy.stats import norm
import numpy as np
from noise.Gaussian_noise import GaussianNoise
from noise.uniform_noise import UniformNoise
from robot.robot_pose import RobotPose
from world.feature_state import FeatureState
from world.feature_based_world import FeatureBasedWorld
from utility.math import wrap_angle
from localization_interface import LocalizationInterface

class PF_Localization(LocalizationInterface):
    class ProcessNoise:
        def __init__(self, Q):
            self.noise_x = GaussianNoise((0, np.sqrt(Q[0, 0])))
            self.noise_y = GaussianNoise((0, np.sqrt(Q[1, 1])))
            self.noise_theta = GaussianNoise((0, np.sqrt(Q[2, 2])))

            print(str(self.noise_x))
            print(str(self.noise_y))
            print(str(self.noise_theta))

        def GetSample(self):
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
                    w = w * norm.pdf(deviation[i], 0, self.R[i, i])

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
        def __init__(self, pose, weight):
            self.pose = pose
            self.weight = weight
            
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
        scale = sum(weights) * 1.0 / self.M
        for p in self.particles:
            p.weight = p.weight * scale

        # cumulative distribution function
        self.cdf = [self.particles[0].weight]
        for i in range(1, self.M):
            self.cdf.append(self.cdf[i - 1] + self.particles[i].weight)
        
    def get_resampling(self):
        '''
        Resample the particle population based on particles' weights.
        === OUTPUT ===
        p: the particle resampled.
        '''
        w = np.random.uniform()

        # search for the smallest index whose cdf that is larger than or equal
        # to w. by the end of the search r would be the result.
        l = 0
        r = self.M - 1
        while (l < r):
            m = (l + r) / 2
            if (self.cdf[m] == w):
                break;
            if (self.cdf[m] > w):
                r = m
            else:
                l = m + 1

        print(r)

        return self.particles[r]

    
    def sample_motion_model(self, a, dt, p):
        '''
        Find the robot's pose, given the action and duration, and the particle
        ===INPUT===
        a:  of RobotAction
        dt: duration of a
        p:  the particle of interest
        ===OUTPUT===
        a RobotPose
        '''
        pose_noise = self.process_noise.GetSample()
        #print("pose noise:" + str(pose_noise))
        rp = p.pose.ApplyAction(a, dt)
        #print("after action (no noise)" + str(rp))
        rp = rp + pose_noise
        return rp

    def measurement_model(self, measurements, p):
        '''
        Compute the probability of the particle and the measurements given
        ===INPUT===
        measurement: a list of Measurement
        p: the particle of interest
        ===OUTPUT===
        a decimal representing the probability
        '''
        w = 1.0

        for m in measurements:
            j = m.s # assuming the signature gives the correspondence
            
            fs = self.true_map.GetFeature(j)
            delta_x = fs.x - p.pose.x
            delta_y = fs.y - p.pose.y
            q = delta_x * delta_x + delta_y * delta_y
            
            # actual measurement noise
            n_r = np.sqrt(q) - m.r
            n_phi = wrap_angle(np.arctan2(delta_y, delta_x) - m.phi)
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
        

    #======================#
    #     Public API's     #
    #======================#
    def __init__(self, M):
        self.M = M                      # number of particles
        self.particles = None           # particles in state space
        self.tick = None                # last time of run
        self.measurement_noise = None   # measurement noise generator
        self.process_noise = None       # process noise generator
        self.random_pose = None         # random pose generator
        self.true_map = None            # true map
        self.trajectory = None          # trajectory recorded

    def Initialize(self, Q, R, true_map):
        
        self.tick = 0
        
        self.process_noise = self.ProcessNoise(Q)
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
            #print("before sample " + str(p.pose))
            
            # line 4
            pose = self.sample_motion_model(action, dt, p)
            print("after sample " + str(pose))
            
            # line 5
            weight = self.measurement_model(measurements, p)
            print("weight = " + str(weight))
            #a=input('press any key to continue')
            
            # line 6
            new_particles.append(self.Particle(pose, weight))

        self.particles = new_particles

        # line 8 ~ 11, resampling
        new_particles = list()
        self.prepare_resampling()
        for i in range(self.M):
            p = self.get_resampling()
            new_particles.append(p)

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
