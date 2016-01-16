import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from robot.robot_action import RobotAction

class ActionHistory:
    def __init__(self, dt):
        self.history_length = 0
        self.time_history = []
        self.action_history = []
        self.dt = dt * 1.0
        self.tick = 0.0 # tick of time
        self.idx = 0 # index of action

    def LoadFromFile(self, filename):
        '''
        Load action history from the given file.
        ===INPUT===
        filename: string name of the file.
        '''
        fd = open(filename, "r")

        # number of steps
        self.history_length = [int(x) for x in fd.readline().split()][0]

        for i in range(self.history_length):
            t, v, w = [float(x) for x in fd.readline().split()]
            self.time_history.append(t)
            self.action_history.append(RobotAction(v, w))

        # create an artificial terminal
        self.time_history.append(-1.0)
        self.action_history.append(RobotAction(0,0))

        #~ print self.history_length
        #~ print self.time_history
        #~ print self.action_history

    def Rewind(self):
        '''
        Rewind the time history.
        '''
        self.tick = 0.0
        self.idx = 0

    def EOF(self):
        '''
        Check if reached end of history.
        '''
        return (self.idx + 1 >= self.history_length)

    def GetNext(self):
        '''
        Advance time and return the active action.
        ===OUTPUT===
        A tuple (t, a), where t is the time and a is the action.
        a is active from last GetNext() to t.
        '''
        if (self.EOF()):
            return (None, None)

        self.tick = self.tick + self.dt
            
        if (self.tick > self.time_history[self.idx]):
            self.idx = self.idx + 1

        #~ if (self.EOF()):
            #~ return (None, None)

        return (self.tick, self.action_history[self.idx])
