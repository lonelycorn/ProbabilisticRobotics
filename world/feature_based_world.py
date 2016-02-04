import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from robot.robot_pose import RobotPose
from sensor.measurement import Measurement
from feature_state import FeatureState
import numpy as np
from utility.math import wrap_angle

class FeatureBasedWorldException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class FeatureBasedWorld:
    '''
    This class models feature-based worlds.
    '''
    def __init__(self):
        self.features = []

    def __str__(self):
        content = 'FeatureBasedWorld: ';
        for f in self.features:
            content = content + '\n' + str(f)

        return content

    def AppendFeature(self, feature):
        '''
        add a feature to the collection
        ===INPUT===
        feature: of class Feature
        ===OUTPUT===
        none
        '''
        if (not (isinstance(feature, FeatureState))):
            raise FeatureBasedWorldException('feature should be an instance of FeatureState')
        self.features.append(feature);

    def GetFeatureCount(self):
        return len(self.features)

    def GetFeature(self, k):
        if (0 <= k < len(self.features)):
            return self.features[k]
        else:
            raise FeatureBasedWorldException('Index out of bounds.')

    def GetBounds(self):
        '''
        Get the boundary for the map.
        ===OUTPUT===
        two tuples, (x_min, x_max) and (y_min, y_max)
        '''
        if (0 == len(self.features)):
            raise FeatureBasedWorldException('Empty map.')
            
        x = []
        y = []
        for f in self.features:
            x.append(f.x)
            y.append(f.y)

        return (min(x), max(x)), (min(y), max(y))

    def GetTrueMeasurements(self, robot_pose):
        '''
        Get the true measurements for a robot at robot_pose using a noise-free
        range sensor.
        ===INPUT===
        robot_pose: of class RobotPose, the pose at which the robot measures
        ===OUTPUT===
        a list of Measurement
        '''
        
        if (not (isinstance(robot_pose, RobotPose))):
            raise FeatureBasedWorldException('robot_pose should be an instance of RobotPose')

        m = []

        for feature in self.features:
            dx = feature.x - robot_pose.x 
            dy = feature.y - robot_pose.y
            
            d = np.sqrt(dx * dx + dy * dy)  # true distance
            phi = wrap_angle(np.arctan2(dy, dx) - robot_pose.theta) # true relative bearing
            
            m.append(Measurement(d, phi, feature.s))
            
        return m

    def LoadFromFile(self, filename):
        print('Loading world features from: '+ filename)
        
        self.features = []
        
        fd = open(filename, "r")
        
        # number of features
        N = [int(x) for x in fd.readline().split()][0]

        # map features: x, y, signature
        for i in range(N):
            x, y, s = fd.readline().split()
            self.AppendFeature(FeatureState(float(x), float(y), int(s)))

if __name__ == "__main__":
    pass
