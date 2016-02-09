#!/usr/bin/env python
import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from simulator.action_history import ActionHistory
from simulator.plotter import Plotter
from simulator.simulator import Simulator
from EKF_known_correspondence import EKF_SLAM

import numpy as np

if __name__ == '__main__':
    dt = 1.0

    # load testcase
    s = Simulator('../testcase/map1.txt', '../testcase/robot1.txt')
    ah = ActionHistory(dt)
    ah.LoadFromFile('../testcase/action_history_loop.txt')

    # set up the SLAM algorithm
    Q = np.diag([0.1, 0.1, 0.1]) # process noise; (x, y, theta). should be over-estimates
    R = np.diag([0.2, 0.2, 0.01]) # measurement noise; (dist, bearing, signature),
    algo = EKF_SLAM()
    algo.Initialize(Q, R, s.world.GetFeatureCount())
    
    # test the SLAM algorithm with the simulator
    ah.Rewind()
    while (not ah.EOF()):
        t, ra = ah.GetNext()
        m = s.Update(t, ra)
        algo.Update(t, ra, m)

    algo.Finalize()

    # visualize results
    p = Plotter(5.0, True)
    p.SetTrueMap(s.GetMap())
    p.SetTrueTrajectory(s.GetTrajectory())
    p.SetMap(algo.GetMap())
    p.SetTrajectory(algo.GetTrajectory())
    p.PlotResults()
