import numpy as np

class SLAM_Algorithm:
    def __init__(self):
        pass

    def Initialize(self, t0, x0):
        '''
        Initialize the algorithm. should be called only once in the very beginning
        ===INPUT===
        x0: the inistial state
        t0: the initial time stamp
        ===OUTPUT===
        none
        '''
        pass

    def Update(self, t, action, measurements):
        '''
        Execute the algorithm  for one iteration. Should be called periodically.
        ===INPUT===
        t: the new time stamp
        action: the robot action since last run
        measurement: a list of measurements the robot gets
        ===OUTPUT===
        the latest estimate of robot pose, if any
        '''
        pass

    def Finalize(self):
        '''
        Finalization of the algorithm
        ===INPUT===
        none
        ===OUTPUT===
        none
        '''
        pass

    def GetMap(self):
        '''
        Get the estimated map
        ===INPUT===
        none
        ===OUTPUT===
        an instance of FeatureBasedWorld
        '''
        pass

    def GetTrajectory(self):
        '''
        Get the robot's trajectory, if any
        ===INPUT===
        none
        ===OUTPUT===
        a list of RobotPose
        '''
        pass

    

if __name__ == '__main__':
    pass
