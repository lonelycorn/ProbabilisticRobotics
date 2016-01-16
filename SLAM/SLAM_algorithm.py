import numpy as np

class SLAM_Exception(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class SLAM_Algorithm:
    # This class defines an interface for various SLAM algorithms.
    # This API's shall be overriden to accommodate for different SLAM implementations.
    def __init__(self):
        raise SLAM_Exception('Must implement the constructor')

    def Initialize(self, Q, R):
        '''
        Initialize the algorithm.  This should be called only once, in the
        very beginning.
        ===INPUT===
        Q: the process noise covariance matrix
        R: the measurement noise covariance matrix
        ===OUTPUT===
        none
        '''
        raise SLAM_Exception('Must implement Initialize()')

    def Update(self, t, action, measurements):
        '''
        Update the state estimate. Should be called periodically.
        ===INPUT===
        t: the time of update.
        action: the robot action from last Update() to t,
        measurement: a list of measurements the robot gets at t.
        ===OUTPUT===
        the latest estimate of robot pose, if any
        '''
        raise SLAM_Exception('Must implement Update()')

    def Finalize(self):
        '''
        Finalization of the algorithm. This should be called only once, in the
        very end.
        ===INPUT===
        none
        ===OUTPUT===
        none
        '''
        raise SLAM_Exception('Must implement Finalize()')

    def GetMap(self):
        '''
        Get the map estimated by the algorithm.
        ===INPUT===
        none
        ===OUTPUT===
        an instance of FeatureBasedWorld
        '''
        raise SLAM_Exception('Must implement GetMap()')

    def GetTrajectory(self):
        '''
        Get the robot's trajectory estimated by the algorithm.
        ===INPUT===
        none
        ===OUTPUT===
        a list of RobotPose
        '''
        raise SLAM_Exception('Must implement GetTrajectory()')

if __name__ == '__main__':
    pass
