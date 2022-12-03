# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import plotly as py
from plotly import graph_objs as go
import os.path
import scipy.io as sio
from obs_checking.OptimalModulationDS.python_scripts.parse_matlab_network import Net
import torch
import numpy as np

colors = ['darkblue', 'teal']


class Plot(object):
    def __init__(self, filename, X_limits):
        """
        Create a plot
        :param filename: filename
        """
        self.X_limits = X_limits
        self.filename = "../../output/visualizations/" + filename + ".html"
        self.data = []
        self.layout = {'title': 'Plot',
                       'showlegend': False
                       }

        self.fig = {'data': self.data,
                    'layout': self.layout}

    def plot_tree(self, X, trees):
        """
        Plot tree
        :param X: Search Space
        :param trees: list of trees
        """
        if X.dimensions == 2:  # plot in 2D
            self.plot_tree_2d(trees)
        elif X.dimensions == 3:  # plot in 3D
            self.plot_tree_3d(trees)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_tree_2d(self, trees):
        """
        Plot 2D trees
        :param trees: trees to plot
        """
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        line=dict(
                            color=colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

    def plot_tree_3d(self, trees):
        """
        Plot 3D trees
        :param trees: trees to plot
        """
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter3d(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        z=[start[2], end[2]],
                        line=dict(
                            color=colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

    def plot_obstacles(self, X, O):
        """
        Plot obstacles
        :param X: Search Space
        :param O: list of obstacles
        """
        if X.dimensions == 2:  # plot in 2D
            self.layout['shapes'] = []
            for O_i in O:
                # noinspection PyUnresolvedReferences
                self.layout['shapes'].append(
                    {
                        'type': 'rect',
                        'x0': O_i[0],
                        'y0': O_i[1],
                        'x1': O_i[2],
                        'y1': O_i[3],
                        'line': {
                            'color': 'purple',
                            'width': 4,
                        },
                        'fillcolor': 'purple',
                        'opacity': 0.70
                    },
                )
        elif X.dimensions == 3:  # plot in 3D
            for O_i in O:
                obs = go.Mesh3d(
                    x=[O_i[0], O_i[0], O_i[3], O_i[3], O_i[0], O_i[0], O_i[3], O_i[3]],
                    y=[O_i[1], O_i[4], O_i[4], O_i[1], O_i[1], O_i[4], O_i[4], O_i[1]],
                    z=[O_i[2], O_i[2], O_i[2], O_i[2], O_i[5], O_i[5], O_i[5], O_i[5]],
                    i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                    j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                    k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                    color='purple',
                    opacity=0.70
                )
                self.data.append(obs)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_path(self, X, path):
        """
        Plot path through Search Space
        :param X: Search Space
        :param path: path through space given as a sequence of points
        """
        if X.dimensions == 2:  # plot in 2D
            x, y = [], []
            for i in path:
                x.append(i[0])
                y.append(i[1])
            trace = go.Scatter(
                x=x,
                y=y,
                line=dict(
                    color="red",
                    width=4
                ),
                mode="lines"
            )

            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            x, y, z = [], [], []
            for i in path:
                x.append(i[0])
                y.append(i[1])
                z.append(i[2])
            trace = go.Scatter3d(
                x=x,
                y=y,
                z=z,
                line=dict(
                    color="red",
                    width=4
                ),
                mode="lines"
            )

            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_start(self, X, x_init):
        """
        Plot starting point
        :param X: Search Space
        :param x_init: starting location
        """
        if X.dimensions == 2:  # plot in 2D
            trace = go.Scatter(
                x=[x_init[0]],
                y=[x_init[1]],
                line=dict(
                    color="orange",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            trace = go.Scatter3d(
                x=[x_init[0]],
                y=[x_init[1]],
                z=[x_init[2]],
                line=dict(
                    color="orange",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_goal(self, X, x_goal):
        """
        Plot goal point
        :param X: Search Space
        :param x_goal: goal location
        """
        if X.dimensions == 2:  # plot in 2D
            trace = go.Scatter(
                x=[x_goal[0]],
                y=[x_goal[1]],
                line=dict(
                    color="green",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)
        elif X.dimensions == 3:  # plot in 3D
            trace = go.Scatter3d(
                x=[x_goal[0]],
                y=[x_goal[1]],
                z=[x_goal[2]],
                line=dict(
                    color="green",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def plot_obstacles_circle(self, X, O, resolution):
        """
        Plot obstacles
        :param X: Search Space
        :param O: list of obstacles
        """
        if X.dimensions == 2:  # plot in 2D
            if os.path.isfile("cspace_2_NN_circle.npy"):
                cspace = np.load("cspace_2_NN_circle.npy")
            else:
                # load weights
                mat_contents = sio.loadmat('../../obs_checking/OptimalModulationDS/matlab_scripts/planar_robot_2d/data/net_parsed.mat')
                W = mat_contents['W'][0]
                b = mat_contents['b'][0]

                # create net
                net = Net()
                net.setWeights(W, b)

                cspace = np.ones([np.size(np.arange(self.X_limits[0][0], self.X_limits[0][1], resolution)), np.size(np.arange(self.X_limits[1][0], self.X_limits[1][1], resolution))])
                ii = 0
                for i in np.arange(self.X_limits[0][0], self.X_limits[0][1], resolution):
                    jj = 0
                    for j in np.arange(self.X_limits[1][0], self.X_limits[1][1], resolution):
                        inp = np.concatenate([np.tile([i, j],[len(O),1]),O[:,0:2]],axis=1)
                        val = net.forward(torch.Tensor(inp))
                        if (val.detach().numpy().reshape([1,-1]) > O[:,-1]).all():
                            cspace[ii][jj] = 0
                        jj = jj + 1
                    ii = ii + 1
                    print(ii)
                np.save("cspace_2_NN_circle.npy", cspace)

            self.layout['shapes'] = []
            ii = 0
            for i in np.arange(self.X_limits[0][0], self.X_limits[0][1], resolution):
                jj = 0
                for j in np.arange(self.X_limits[1][0], self.X_limits[1][1], resolution):
                    if cspace[ii][jj]:
                        # noinspection PyUnresolvedReferences
                        self.layout['shapes'].append(
                            {
                                'type': 'rect',
                                'x0': i,
                                'y0': j,
                                'x1': i + resolution,
                                'y1': j + resolution,
                                'line': {
                                    'color': 'purple',
                                    'width': 4,
                                },
                                'fillcolor': 'purple',
                                'opacity': 0.70
                            },
                        )
                    jj = jj + 1
                ii = ii + 1
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")

    def draw(self, auto_open=True):
        """
        Render the plot to a file
        """
        py.offline.plot(self.fig, filename=self.filename, auto_open=auto_open)
