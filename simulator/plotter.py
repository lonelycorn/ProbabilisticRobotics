import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self, map_size, separate_fig = True):
        self.dash_size = 0.5
        self.line_width = 2
        self.pose_size = 10
        self.feature_size = 20

        self.map_size = map_size
        self.separate_fig = separate_fig

        self.true_map = None
        self.true_trajectory = None
        self.estimated_map = None
        self.estimated_trajectory = None

        
    def plot_map_trajectory(self, feature_map, trajectory, fig_id, fig_title, color):
        show_figure = False
            
        plt.figure(fig_id)

        if (trajectory is not None):
            for rp in trajectory:
                # position
                plt.plot(rp.x, rp.y, color+'o', markersize = self.pose_size)
                # heading dash
                plt.plot([rp.x, rp.x + np.cos(rp.theta) * self.dash_size], \
                         [rp.y, rp.y + np.sin(rp.theta) * self.dash_size], \
                         color+'-', linewidth = self.line_width)
            show_figure = True
            

        if (feature_map is not None):
            for f in feature_map.features:
                plt.plot(f.x, f.y, color+'p', markersize = self.feature_size)
            show_figure = True

        if (show_figure):
            plt.axis([-self.map_size, self.map_size, -self.map_size, self.map_size])
            plt.title(fig_title)

        return show_figure

    def SetTrueTrajectory(self, true_trajectory):
        self.true_trajectory = true_trajectory

    def SetTrueMap(self, true_map):
        self.true_map = true_map

    def SetTrajectory(self, estimated_trajectory):
        self.estimated_trajectory = estimated_trajectory

    def SetMap(self, estimated_map):
        self.estimated_map = estimated_map

    def PlotResults(self):
        #self.separate_fig = True
        if (self.separate_fig):
            fig_id_truth = 1
            fig_id_estimate = 2
            fig_title_truth = "Ground Truth"
            fig_title_estimate = "Estimated"
            color_truth = 'r'
            color_estimate = 'r'
        else:
            fig_id_truth = 1
            fig_id_estimate = 1
            fig_title_truth = "Truth" # will be overwritten by fig_title_estimate
            fig_title_estimate = "Estimated vs. Truth"
            color_truth = 'r'
            color_estimate = 'b'

        success_truth = self.plot_map_trajectory(self.true_map,         \
                                                 self.true_trajectory,  \
                                                 fig_id_truth,          \
                                                 fig_title_truth,       \
                                                 color_truth)          

        success_estimate = self.plot_map_trajectory(self.estimated_map,         \
                                                    self.estimated_trajectory,  \
                                                    fig_id_estimate,            \
                                                    fig_title_estimate,         \
                                                    color_estimate)           

        if (success_truth):
            plt.show(fig_id_truth)
        if (success_estimate):
            plt.show(fig_id_estimate)

if __name__ == '__main__':
    pass
