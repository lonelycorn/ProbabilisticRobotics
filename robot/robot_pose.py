import numpy as np
import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

class RobotPoseException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class RobotPose:
    def __init__(self, x, y, theta):
        self.x = 1.0 * x
        self.y = 1.0 * y
        self.theta = np.arctan2(np.sin(theta), np.cos(theta)) # wrap between [-pi, pi)

    def __str__(self):
        return 'RobotPose: pos = (' + str(self.x) + ', ' + str(self.y) + '), theta = ' + str(self.theta)

    def __add__(self, other):
        '''
        overload operator +
        '''
        if (not (isinstance(other, RobotPose))):
            raise RobotActionException('unsupported operator + for ' + str(type(self) + ' and ' + str(type(other))))
        return RobotPose(self.x + other.x, \
                         self.y + other.y, \
                         self.theta + other.theta)

    def __sub__(self, other):
        '''
        overload operator -
        '''
        if (not (isinstance(other, RobotPose))):
            raise RobotActionException('unsupported operator + for ' + str(type(self) + ' and ' + str(type(other))))
        return RobotPose(self.x - other.x, \
                         self.y - other.y, \
                         self.theta - other.theta)
    
    def ApplyAction(self, robot_action, dt):
        '''
        update current pose with given action and duration
        ===INPUT===
        robot_action: of class RobotAction, the action to apply
        dt: duration of rthe action
        ===OUTPUT===
        the robot pose after the update
        '''
        from robot.robot_action import RobotAction
        if (not (isinstance(robot_action, RobotAction))):
            raise RobotPoseException('robot_action should be an instance of RobotAction');

        # Note: we don't write down the update equations explicitly, so that we
        # could play with different RobotAction models
        self = robot_action.ApplyToPose(self, dt)
        return self

    def ToMatrix(self):
        '''
        Get a matrix representation of the robot pose
        === OUTPUT ===
        A 3-by-1 matrix [[x], [y], [theta]]
        '''
        m = np.matrix(np.zeros((3, 1)))
        m[0, 0] = self.x
        m[1, 0] = self.y
        m[2, 0] = self.theta

        return m

if __name__ == "__main__":
    pass
    
