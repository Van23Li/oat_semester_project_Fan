# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import plotly as py
import os.path
from obs_checking.OptimalModulationDS.python_scripts.parse_matlab_network import Net
from plotly import graph_objs as go
import scipy.io as sio
import torch

colors = ['darkblue', 'teal']


class Plot_kd(object):
    def __init__(self, filename, X_limits, V_limits, A_limits):
        """
        Create a plot
        :param filename: filename
        """
        self.X_limits = X_limits
        self.V_limits = V_limits
        self.A_limits = A_limits
        self.filename = "../../output/visualizations/" + filename + ".html"
        self.data = []
        self.layout = {'title': 'Plot',
                       'showlegend': False
                       }

        self.fig = {'data': self.data,
                    'layout': self.layout}

    def plot_tree(self, X, trees, trees_v, dim1 = 0, dim2 = 1):
        """
        Plot tree
        :param X: Search Space
        :param trees: list of trees
        """
        if X.dimensions == 2:  # plot in 2D
            self.plot_tree_2d(trees, trees_v)
        elif X.dimensions == 3:  # plot in 3D
            self.plot_tree_3d(trees)
        else:  # can't plot in higher dimensions
            self.plot_tree_7D(trees, trees_v, dim1, dim2)

    def dist_kd(self, p_1, v_1, p, v):
        """ from p_1 to p
        Return the minimum time to steer from each node in tree to q
        :param tree: int, tree being searched
        :param q: the target node
        :return: vertex, nearest vertex in tree to new vertex
        """

        v_min = self.V_limits[:, 0]
        v_max = self.V_limits[:, 1]
        a_min = self.A_limits[:, 0]
        a_max = self.A_limits[:, 1]

        p_1 = np.array(p_1)
        p_2 = np.array(p)
        v_1 = np.array(v_1)
        v_2 = np.array(v)

        # judge if a1 is positive or negative
        p_acc = 1/2 * (v_1 + v_2) * (np.abs(v_2 - v_1)) / a_max
        sigma = np.sign(p_2 - p_1 - p_acc)
        a_1 = sigma * a_max
        a_2 = -sigma * a_max
        v_limit = sigma * v_max

        # calculate minmum time
        a = a_1
        b = 2 * v_1
        c = (np.square(v_2) - np.square(v_1)) / (2 * a_2) - (p_2 - p_1)
        q = -1 / 2 * (b + np.sign(b) * np.sqrt(np.square(b) - 4 * a * c))
        q[np.where(q == 0)] = 1 # incase q = 0. Note that the elements of 0 will be replaced in following line
        t_1 = (np.sign(a) != np.sign(b)) * q / a + (np.sign(a) == np.sign(b)) * c / q
        # t_1[np.where(np.sign(b) == 0)] = np.sign(a[np.where(np.sign(b)==0)]) * np.sqrt(-4 * a[np.where(np.sign(b)==0)] * c[np.where(np.sign(b)==0)]) / (2*a[np.where(np.sign(b)==0)])
        t_1[np.sign(b) == 0] = np.sign(a[np.sign(b)==0]) * np.sqrt(-4 * a[np.sign(b)==0] * c[np.sign(b)==0]) / \
                               (2*a[np.sign(b)==0])
        t_2 = (v_2 - v_1) / a_2 + t_1

        # check whether the solution satisfies the velocity limits
        velocity_valided = (v_1 + a_1 * t_1 <= v_max) * (v_1 + a_1 * t_1 >= v_min)
        t_1 = velocity_valided * t_1 + ~velocity_valided * (v_limit - v_1) / a_1
        t_v = ~velocity_valided * ((np.square(v_1) + np.square(v_2) - 2 * np.square(v_limit)) / (2 * v_limit * a_1) +
                                   (p_2 - p_1) / v_limit)
        t_2 = velocity_valided * t_2 + ~velocity_valided * (v_2 - v_limit) / a_2

        t = t_1 + t_2 + t_v

        return np.max(t)

    def Steer(self, closestNode_x, closestNode_v, sample_x, sample_v, T):
        a_min = self.A_limits[:, 0]
        a_max = self.A_limits[:, 1]
        v_min = self.V_limits[:, 0]
        v_max = self.V_limits[:, 1]

        p_1 = np.array(closestNode_x)
        p_2 = np.array(sample_x)
        v_1 = np.array(closestNode_v)
        v_2 = np.array(sample_v)

        # calculate the acceleration
        a = np.square(T)
        b = 2 * T * (v_1 + v_2) - 4 * (p_2 - p_1)
        c = -np.square(v_2 - v_1)
        q = -1 / 2 * (b + np.sign(b) * np.sqrt(np.square(b) - 4 * a * c))
        a_1 = q / a
        a_2 = -a_1

        t_1 = 1 / 2 * ((v_2 - v_1) / a_1 + T)
        t_v = 0 * t_1
        t_2 = T - t_1
        v_limit = 0 * t_1

        # check whether the solution satisfies the velocity limits
        velocity_valided = (v_1 + a_1 * t_1 <= v_max) * (v_1 + a_1 * t_1 >= v_min)
        if np.sum(velocity_valided) < len(velocity_valided):
            v_limit = np.sign(a_1) * v_max
            a_1 = velocity_valided * a_1 + ~velocity_valided * (np.square(v_limit - v_1) + np.square(v_limit - v_2)) / \
                        (2 * (v_limit * T - (p_2 - p_1)))
            a_2 = -a_1
            t_1 = velocity_valided * t_1 + ~velocity_valided * (v_limit - v_1) / a_1
            t_v = velocity_valided * t_v + ~velocity_valided * ((np.square(v_1) + np.square(v_2) - 2 * np.square(v_limit)) /
                                        (2 * v_limit * a_1) + (p_2 - p_1) / v_limit)
            t_2 = velocity_valided * t_2 + ~velocity_valided * (v_2 - v_limit) / a_2

        return a_1, v_limit, a_2, t_1, t_v, t_2

    def plot_tree_2d(self, trees, trees_v):
        """
        Plot 2D trees
        :param trees: trees to plot
        """
        for Tree_x, Tree_v in zip(enumerate(trees), enumerate(trees_v)):
            i_x, tree_x = Tree_x
            i_v, tree_v = Tree_v
            for value_x, value_v in zip(tree_x.E.items(), tree_v.E.items()):
                start_x, end_x = value_x
                start_v, end_v = value_v
                if end_x is not None and end_v is not None:
                    Time = self.dist_kd(end_x, end_v, start_x, start_v)
                    a_1, v_limit, a_2, t_1, t_v, t_2 = self.Steer(end_x, end_v, start_x, start_v, Time)

                    q_1_end = end_x + end_v * t_1 + 1 / 2 * a_1 * np.square(t_1)
                    v_1_end = end_v + a_1 * t_1
                    q_v_end = q_1_end + v_1_end * t_v
                    v_v_end = v_1_end
                    q_2_end = q_v_end + v_v_end * t_2 + 1 / 2 * a_2 * np.square(t_2)
                    v_2_end = v_v_end + a_2 * t_2

                    q_list = np.array(end_x)
                    # q_list = np.empty([2, 1])
                    for t in np.linspace(0, t_1[0] + t_v[0] + t_2[0], 100):
                        if t != 0:
                            q_check = np.array(end_x)
                            for j in range(len(end_x)):
                                if t < t_1[j]:
                                    q_1 = end_x[j] + end_v[j] * t + 1 / 2 * a_1[j] * np.square(t)
                                elif t < t_1[j] + t_v[j]:
                                    q_1 = q_1_end[j] + v_1_end[j] * (t - t_1[j])
                                else:
                                    q_1 = q_v_end[j] + v_v_end[j] * (t - t_v[j] - t_1[j]) + 1 / 2 * a_2[j] * np.square(t - t_v[j] - t_1[j])
                                q_check[j] = q_1
                            q_list = np.append(q_list, q_check).reshape([-1,2])
                    for i in range(len(q_list)-1):
                        trace = go.Scatter(
                            x=[q_list[i][0], q_list[i+1][0]],
                            y=[q_list[i][1], q_list[i+1][1]],
                            line=dict(
                                color=colors[i_x]
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

    def plot_tree_7D(self, trees, trees_v, dim1, dim2):
        """
        Plot 2D trees
        :param trees: trees to plot
        """
        for Tree_x, Tree_v in zip(enumerate(trees), enumerate(trees_v)):
            i_x, tree_x = Tree_x
            i_v, tree_v = Tree_v
            for value_x, value_v in zip(tree_x.E.items(), tree_v.E.items()):
                start_x, end_x = value_x
                start_v, end_v = value_v
                if end_x is not None and end_v is not None:
                    Time = self.dist_kd(end_x, end_v, start_x, start_v)
                    a_1, v_limit, a_2, t_1, t_v, t_2 = self.Steer(end_x, end_v, start_x, start_v, Time)

                    q_1_end = end_x + end_v * t_1 + 1 / 2 * a_1 * np.square(t_1)
                    v_1_end = end_v + a_1 * t_1
                    q_v_end = q_1_end + v_1_end * t_v
                    v_v_end = v_1_end
                    q_2_end = q_v_end + v_v_end * t_2 + 1 / 2 * a_2 * np.square(t_2)
                    v_2_end = v_v_end + a_2 * t_2

                    q_list = np.array(end_x)
                    # q_list = np.empty([2, 1])
                    for t in np.linspace(0, t_1[0] + t_v[0] + t_2[0], 100):
                        if t != 0:
                            q_check = np.array(end_x)
                            for j in range(len(end_x)):
                                if t < t_1[j]:
                                    q_1 = end_x[j] + end_v[j] * t + 1 / 2 * a_1[j] * np.square(t)
                                elif t < t_1[j] + t_v[j]:
                                    q_1 = q_1_end[j] + v_1_end[j] * (t - t_1[j])
                                else:
                                    q_1 = q_v_end[j] + v_v_end[j] * (t - t_v[j] - t_1[j]) + 1 / 2 * a_2[j] * np.square(t - t_v[j] - t_1[j])
                                q_check[j] = q_1
                            q_list = np.append(q_list, q_check).reshape([-1,7])
                    for i in range(len(q_list)-1):
                        trace = go.Scatter(
                            x=[q_list[i][dim1], q_list[i+1][dim1]],
                            y=[q_list[i][dim2], q_list[i+1][dim2]],
                            line=dict(
                                color=colors[i_x]
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

    def plot_obstacles_circle(self, X, O, resolution, dim1 = 0, dim2 = 1):
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
        else:  # can't plot in higher dimensions # plot in 2D
            if os.path.isfile("cspace_7_NN_circle.npy"):
                cspace = np.load("cspace_7_NN_circle.npy")
            else:
                # load weights
                mat_contents = sio.loadmat('../../obs_checking/OptimalModulationDS/matlab_scripts/planar_robot_2d/data/net_parsed.mat')
                W = mat_contents['W'][0]
                b = mat_contents['b'][0]

                # create net
                net = Net()
                net.setWeights(W, b)

                cspace = np.ones([np.size(np.arange(self.X_limits[dim1][0], self.X_limits[dim1][1], resolution)), np.size(np.arange(self.X_limits[dim2][0], self.X_limits[dim2][1], resolution))])
                ii = 0
                for i in np.arange(self.X_limits[dim1][0], self.X_limits[dim1][1], resolution):
                    jj = 0
                    for j in np.arange(self.X_limits[dim2][0], self.X_limits[dim2][1], resolution):
                        inp = np.concatenate([np.tile([i, j],[len(O),1]),O[:,0:2]],axis=1)
                        val = net.forward(torch.Tensor(inp))
                        if (val.detach().numpy().reshape([1,-1]) > O[:,-1]).all():
                            cspace[ii][jj] = 0
                        jj = jj + 1
                    ii = ii + 1
                    print(ii)
                np.save("cspace_7_NN_circle.npy", cspace)

            self.layout['shapes'] = []
            ii = 0
            for i in np.arange(self.X_limits[dim1][0], self.X_limits[dim1][1], resolution):
                jj = 0
                for j in np.arange(self.X_limits[dim2][0], self.X_limits[dim2][1], resolution):
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

    def plot_path(self, X, path_x, path_v):
        """
        Plot path through Search Space
        :param X: Search Space
        :param path: path through space given as a sequence of points
        """
        if X.dimensions == 2:  # plot in 2D
            x_1, x_2, v_1, v_2 = [], [], [], []
            for i, j in zip(path_x, path_v):
                x_1.append(i[0])
                x_2.append(i[1])
                v_1.append(j[0])
                v_2.append(j[1])

            for i in range(len(path_x) - 1):
                start_x, end_x = path_x[i + 1], path_x[i]
                start_v, end_v = path_v[i + 1], path_v[i]
                if end_x is not None and end_v is not None:
                    Time = self.dist_kd(end_x, end_v, start_x, start_v)
                    a_1, v_limit, a_2, t_1, t_v, t_2 = self.Steer(end_x, end_v, start_x, start_v, Time)

                    q_1_end = end_x + end_v * t_1 + 1 / 2 * a_1 * np.square(t_1)
                    v_1_end = end_v + a_1 * t_1
                    q_v_end = q_1_end + v_1_end * t_v
                    v_v_end = v_1_end
                    q_2_end = q_v_end + v_v_end * t_2 + 1 / 2 * a_2 * np.square(t_2)
                    v_2_end = v_v_end + a_2 * t_2

                    q_list = np.array(end_x)
                    # q_list = np.empty([2, 1])
                    for t in np.linspace(0, t_1[0] + t_v[0] + t_2[0], 100):
                        if t != 0:
                            q_check = np.array(end_x)
                            for j in range(len(end_x)):
                                if t < t_1[j]:
                                    q_1 = end_x[j] + end_v[j] * t + 1 / 2 * a_1[j] * np.square(t)
                                elif t < t_1[j] + t_v[j]:
                                    q_1 = q_1_end[j] + v_1_end[j] * (t - t_1[j])
                                else:
                                    q_1 = q_v_end[j] + v_v_end[j] * (t - t_v[j] - t_1[j]) + 1 / 2 * a_2[j] * np.square(t - t_v[j] - t_1[j])
                                q_check[j] = q_1
                            q_list = np.append(q_list, q_check).reshape([-1,2])
                    for i in range(len(q_list)-1):
                        trace = go.Scatter(
                            x=[q_list[i][0], q_list[i+1][0]],
                            y=[q_list[i][1], q_list[i+1][1]],
                            line=dict(
                                color="red",
                                width=4
                            ),
                            mode="lines"
                        )
                        self.data.append(trace)
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 2 dimensions")

    def plot_start(self, X, x_init, dim1=0, dim2=1):
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
            trace = go.Scatter(
                x=[x_init[dim1]],
                y=[x_init[dim2]],
                line=dict(
                    color="orange",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)

    def plot_goal(self, X, x_goal, dim1=0, dim2=0):
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
            trace = go.Scatter(
                x=[x_goal[dim1]],
                y=[x_goal[dim2]],
                line=dict(
                    color="green",
                    width=10
                ),
                mode="markers"
            )

            self.data.append(trace)

    def draw(self, auto_open=True):
        """
        Render the plot to a file
        """
        py.offline.plot(self.fig, filename=self.filename, auto_open=auto_open)
