import numpy as np
import time

from src.rrt.rrt import RRT
from src.search_space.search_space import SearchSpace
from src.utilities.obstacle_generation import generate_random_obstacles
from src.utilities.plotting import Plot

# define the robot
r = (4, 4, 0)
d = (0, 0, 0)
alpha = (0, 0, 0)
base = np.eye(4)

k1 = 0.95
k2 = 0.95
q_min = (-k1 * np.pi, -k2 * np.pi, -10, -10)
q_max = (k1 * np.pi, k2 * np.pi, 10, 10)

X_dimensions = np.array([(q_min[0], q_max[1]), (q_min[0], q_max[1])])  # dimensions of Search Space
x_init = (-2.5, -1)  # starting location
x_goal = (2.7, 0)  # goal location
X_limits = np.array([(q_min[0], q_max[1]), (q_min[0], q_max[1])])  # dimensions of Acceleration

Q = np.array([(0.1 * np.pi, 10)])  # (length of tree edges, times of randomly sampling)
r = 0.04  # length of smallest edge to check for intersection with obstacles
max_samples = 2048  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_dimensions)
# n = 50
# Obstacles = generate_random_obstacles(X, x_init, x_goal, n)
Obstacles = np.array([[ 4.5,  3.5,  0.5],
                      [-6.0,  6.0,  1.0],
                      [-6.0, -7.0,  2.0],
                      [-3.0, -7.0,  2.0],
                      [ 0.0, -7.0,  2.0]])  # [centers,radiuses]

Start_time = time.perf_counter()
# create rrt_search
rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc, Obstacles, CheckNN=True)
path = rrt.rrt_search()

End_time = time.perf_counter()
print("Running time: ")
print(End_time - Start_time)

# plot
plot = Plot("rrt_2d_with_random_obstacles", X_limits)
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)
plot.plot_obstacles_circle(X, Obstacles, 0.01*np.pi)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
