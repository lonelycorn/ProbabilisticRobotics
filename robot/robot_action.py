import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
from utility.math import angular_velocity_is_trivial

class RobotActionException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class RobotAction:
    # This is the velocity motion model, which assumes that a robot could be
    # controlled through two velocities, i.e. a translational and a rotational
    # velocity.
    def __init__(self, v, w):
        self.v = 1.0 * v # linear velocity
        self.w = 1.0 * w # angular velocity

    def __str__(self):
        return 'RobotAction: v = ' + str(self.v) + ', w = ' + str(self.w)

    def __add__(self, other):
        '''
        overload operator +
        '''
        if (not (isinstance(other, RobotAction))):
            raise RobotActionException('unsupported operator + for ' + str(type(self) + ' and ' + str(type(other))))
            
        return RobotAction(self.v + other.v, \
                           self.w + other.w)

    def ApplyToPose(self, robot_pose, dt):
        '''
        Find out the outcome of applying this action to a given pose with a
        specified duration.
        ===INPUT===
        robot_pose: of class RobotPose, the pose before the action
        dt: duration of action
        ===OUTPUT===
        the pose after the action
        '''
        from robot_pose import RobotPose
        if (not (isinstance(robot_pose, RobotPose))):
            raise RobotActionException('robot_pose should be an instance of RobotPose');
         
        new_pose = RobotPose(0,0,0)
        if (angular_velocity_is_trivial(self.w)): # straight line
            new_pose.theta = robot_pose.theta
            new_pose.x = robot_pose.x + self.v * dt * np.cos(robot_pose.theta)
            new_pose.y = robot_pose.y + self.v * dt * np.sin(robot_pose.theta)
        else: # arc
            r = self.v / self.w
            new_pose.theta = robot_pose.theta + self.w * dt
            new_pose.x = robot_pose.x - r * np.sin(robot_pose.theta) + r * np.sin(new_pose.theta)
            new_pose.y = robot_pose.y + r * np.cos(robot_pose.theta) - r * np.cos(new_pose.theta)

        return new_pose

if __name__ == "__main__":
    pass
    
