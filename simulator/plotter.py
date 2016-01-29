import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self, map_size):
        self.dash_size = 0.5
        self.line_width = 2
        self.pose_size = 10
        self.feature_size = 20
        
        self.map_size = map_size

        
    def plot_feature_pose(self, features, poses, fig_id, fig_title):
        plt.figure(fig_id)
        
        for rp in poses:
            plt.plot([rp.x, rp.x + np.cos(rp.theta) * self.dash_size], \
                     [rp.y, rp.y + np.sin(rp.theta) * self.dash_size], \
                     'r-', linewidth = self.line_width)

            plt.plot(rp.x, rp.y, 'ro', markersize = self.pose_size)
            
        for f in features:
            x = f.x
            y = f.y
            plt.plot(f.x, f.y, 'rp', markersize = self.feature_size)

        plt.axis([-self.map_size, self.map_size, -self.map_size, self.map_size])
        plt.title(fig_title)

    def SetTrueTrajectory(self, true_trajectory):
        self.true_trajectory = true_trajectory

    def setTrueMap(self, true_map):
        self.true_map = true_map

    def setTrajectory(estimated_trajectory):
        self.estimated_trajectory = estimated_trajectory

    def setMap(estimated_map):
        self.estimated_map = estimated_map

    def PlotResults(self, true_trajectory, true_map, SLAM_trajectory, SLAM_map):

        self.plot_feature_pose(true_map.features, true_trajectory, 1, "True trajectory")
        
        self.plot_feature_pose(SLAM_map.features, SLAM_trajectory, 2, "SLAM trajectory")

        plt.show(1)
        plt.show(2)

if __name__ == '__main__':
    pass
