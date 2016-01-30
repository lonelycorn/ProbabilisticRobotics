class LocalizationException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class LocalizationInterface:
    # This class defines an interface for various localization algorithms.
    # These API's shall be overriden to accommodate for different algorithms.
    def __init__(self):
        raise LocalizationException("Must implement the constructor.")

    def Initialize(self, Q, R, true_map):
        '''
        Initialize the algorithm.  This should be called only once, in the
        very beginning.
        ===INPUT===
        Q: the process noise covariance matrix (in pose space)
        R: the measurement noise covariance matrix (in measurement space)
        true_map: the ground truth map
        ===OUTPUT===
        none
        '''
        raise LocalizationException("Must implement Initialize().")

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
        raise LocalizationException("Must implement Update().")

    def Finalize(self):
        '''
        Finalization of the algorithm. This should be called only once, in the
        very end.
        ===INPUT===
        none
        ===OUTPUT===
        none
        '''
        raise LocalizationException("Must implement Finalize().")

    def GetTrajectory(self):
        '''
        Get the robot's trajectory estimated by the algorithm.
        ===INPUT===
        none
        ===OUTPUT===
        a list of RobotPose
        '''
        raise LocalizationException("Must implement GetTrajectory().")

if __name__ == '__main__':
    pass
