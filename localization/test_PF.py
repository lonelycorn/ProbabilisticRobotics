#!/usr/bin/env python
import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from simulator.action_history import ActionHistory
from simulator.plotter import Plotter
from simulator.simulator import Simulator
from localization.PF_known_correspondence import PF_Localization

import numpy as np



if __name__ == '__main__':
    dt = 1.0

    # load testcase
    s = Simulator('../testcase/map1.txt', '../testcase/robot1.txt')
    ah = ActionHistory(dt)
    ah.LoadFromFile('../testcase/action_history_loop.txt')

    # set up the localization algorithm
    Q = np.diag([0.01, 0.01, 0.01]) # process noise; (x, y, theta). should be over-estimates
    R = np.diag([0.2, 0.2, 0]) # measurement noise; (dist, bearing, signature),
    #R = np.diag([0.02, 0.02, 0]) # measurement noise; (dist, bearing, signature),
    algo = PF_Localization(10)
    algo.Initialize(Q, R, s.world)
    
    # test the SLAM algorithm with the simulator
    ah.Rewind()
    while (not ah.EOF()):
        t, ra = ah.GetNext()
        m = s.Update(t, ra)
        algo.SetTruePose(s.robot.true_pose)
        algo.Update(t, ra, m)

    algo.Finalize()

    # visualize results
    p = Plotter(5.0, False)
    p.SetTrueMap(s.GetMap())
    p.SetTrueTrajectory(s.GetTrajectory())
    p.SetTrajectory(algo.GetTrajectory())
    p.PlotResults()
