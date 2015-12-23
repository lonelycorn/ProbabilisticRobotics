#!/usr/bin/env python
from simulator import Simulator
from EKF_SLAM import EKF_SLAM

if __name__ == '__main__':
    algo = EKF_SLAM()
    s = Simulator(algo, \
                 '../testcase/map1.txt', \
                 '../testcase/robot2.txt', \
                 '../testcase/action_history_loop.txt')

    print(str(s.robot))
    print('\n')
    print(str(s.world))
    print('\n')
    for i in range(s.history_length):
        print('t = ' + str(s.time_history[i]) + ', '+str(s.action_history[i]))
        
    s.Run()
