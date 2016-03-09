import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import numpy as np

#from <file_name> import <ClassName>
from noise.noise_builder import NoiseBuilder
from sensor.measurement import Measurement

class RangeSensorException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class RangeSensor:
    def __init__(self, dist_ranges, dist_noise, angle_ranges, angle_noise):
        '''
        if dist_ranges contains two same value then we are not constraining the distance
        if angle_ranges contains two same value then we are not constraining the angle
        '''
        if (not (isinstance(dist_noise, NoiseBuilder))):
            raise RangeSensorException('dist_noise must be an instance of NoiseBuilder.')
        self.dist_noise = dist_noise
        
        if (not (isinstance(dist_ranges, list) and (2 == len(dist_ranges)))):
            raise RangeSensorException('dist_ranges must be a list with two items.')
        self.dist_ranges = [min(dist_ranges), max(dist_ranges)]

        if (not (isinstance(angle_noise, NoiseBuilder))):
            raise RangeSensorException('angle_noise must be an instance of Noise.')
        self.angle_noise = angle_noise

        if (not (isinstance(angle_ranges, list) and (2 == len(angle_ranges)))):
            raise RangeSensorException('angle_ranges must be a list with two items')
        self.angle_ranges = [min(angle_ranges), max(angle_ranges)]

    def __str__(self):
        return 'RangeSensor: \ndist range = ' + str(self.dist_ranges) + ' m\ndist ' + str(self.dist_noise) + \
               '\nangle range = ' + str(np.rad2deg(self.angle_ranges)) + ' deg\nangle ' + str(self.angle_noise)
            
    def DistInRange(self, dist):
        '''
        check if dist is in the allowed range
        '''
        return (self.dist_ranges[0] == self.dist_ranges[1]) or \
               ((self.dist_ranges[0] <= dist) and (dist <= self.dist_ranges[1]))

    def AngleInRange(self, angle):
        '''
        check if angle is in the allowed range
        angle should be within [-pi, pi)
        '''
        return (self.angle_ranges[0] == self.angle_ranges[1]) or \
               ((self.angle_ranges[0] <= angle) and (angle <= self.angle_ranges[1]))

    def GetActualReading(self, ground_truth):
        '''
        get the actual reading that corresponds to the groud truth provided
        ===INPUT===
        ground_truth: the actual distance and bearing, i.e. a measurement from an ideal noise-free sensor.
        ==OUTPUT=== 
        visible represents whether the sensor could sense it.
        then a Measurement, the actual reading of the sensor
        '''
        if (not (isinstance(ground_truth, Measurement))):
            raise RangeSensorException('ground_truth must be an instance of Measurement')

        dist = ground_truth.r
        phi = ground_truth.phi
        
        # add noise
        dist = dist + self.dist_noise.GetValue()
        phi = phi + self.angle_noise.GetValue()

        # wrap bearing between [-pi, pi)
        phi = np.arctan2(np.sin(phi), np.cos(phi))

        visible = ((self.DistInRange(dist)) and (self.AngleInRange(phi)))
        
        return visible, Measurement(dist, phi, ground_truth.s)
