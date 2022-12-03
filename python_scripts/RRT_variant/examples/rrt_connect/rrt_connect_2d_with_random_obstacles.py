# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from src.rrt.rrt_connect import RRTConnect
from src.search_space.search_space import SearchSpace
from src.utilities.obstacle_generation import generate_random_obstacles
from src.utilities.plotting import Plot

# X_dimensions = np.array([(0, 100), (0, 100)])  # dimensions of Search Space
X_dimensions = np.array([(-0.95 * np.pi, 0.95 * np.pi), (-0.95 * np.pi, 0.95 * np.pi)])  # dimensions of Search Space
# x_init = (0, 0)  # starting location
# x_goal = (100, 100)  # goal location  # dimensions of Search Space
x_init = (-2.5, -1)  # starting location
x_goal = (2.7, 0)  # goal location

Q = np.array([0.1 * np.pi])  # length of tree edges
r = 0.05  # length of smallest edge to check for intersection with obstacles
max_samples = 2048  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_dimensions)
n = 50
Obstacles = generate_random_obstacles(X, x_init, x_goal, n)

# create rrt_search
rrt_connect = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)
path = rrt_connect.rrt_connect()
# plot
plot = Plot("rrt_connect_2d_with_random_obstacles", X_dimensions)
plot.plot_tree(X, rrt_connect.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
