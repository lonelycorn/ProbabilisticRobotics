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
from localization.localization_interface import LocalizationInterface

class PF_Localization(LocalizationInterface):
    class ProcessNoise:
        def __init__(self, Q):
            self.noise_x = GaussianNoise((0, np.sqrt(Q[0, 0])))
            self.noise_y = GaussianNoise((0, np.sqrt(Q[1, 1])))
            self.noise_theta = GaussianNoise((0, np.sqrt(Q[2, 2])))

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
        print('particles weights')
        print(weights)
        print(sum(weights))
        scale = 1.0 / sum(weights)
        print(scale)
        for p in self.particles:
            p.weight = p.weight * scale
        print('after scaling')
        print([p.weight for p in self.particles])

        #cumulative distribution function. cdv[k] = sum_{i=0}^k weight_i
        self.cdf = [0]
        for i in range(0, self.M):
            self.cdf.append(self.cdf[i] + self.particles[i].weight)

        print("cdf is")
        print(self.cdf)
        
    def get_resampling(self):
        '''
        Resample the particle population based on particles' weights.
        === OUTPUT ===
        p: the particle resampled.
        '''
        w = np.random.uniform()

        # search for the smallest index whose cdf that is larger than or equal
        # to w. by the end of the search r would be the result.
        #l = 0
        #r = self.M - 1
        #while (l < r):
            #m = (l + r) / 2
            #if (self.cdf[m] == w):
                #break;
            #if (self.cdf[m] > w):
                #r = m
            #else:
                #l = m + 1
        #return self.particles[r]
        
        for r in range(self.M):
            if ((self.cdf[r]<w) and (self.cdf[r+1]>=w)):
                print("w="+str(w)+", r="+str(r))
                return self.particles[r]
        print("FUCK WE ARE IN TROUBLE")

        

    
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
        rp = p.pose.ApplyAction(a, dt)
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

        return w # minimal probability to avoid extinction.
        
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
        self.dash_size = 0.2
        self.line_width = 2
        self.map_size = 5
        self.pose_size = 8
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
            #print("after sample " + str(pose))
            
            # line 5
            weight = self.measurement_model(measurements, p)
            #print("weight = " + str(weight))
            
            # line 6
            new_particles.append(self.Particle(pose, weight))

        self.particles = new_particles
        self.plot_particles("t="+str(self.tick)+", before resampling",'k')

        # line 8 ~ 11, resampling
        new_particles = list()
        self.prepare_resampling()
        for i in range(self.M):
        #for i in range(self.M * 9 / 10):
            p = self.get_resampling()
            new_particles.append(p)

        #for i in range(self.M - len(new_particles)):
            #p = self.get_random_particle()
            #new_particles.append(p)

        self.particles = new_particles
        self.plot_particles("t="+str(self.tick)+", after resampling",'b')

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
