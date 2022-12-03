import numpy as np
import time

from src.rrt.rrt_kd import RRTKd
from src.search_space.search_space import SearchSpace
# from src.utilities.obstacle_generation import generate_random_obstacles
from src.utilities.obstacle_circle_generation import generate_random_obstacles
from src.utilities.plotting import Plot
from src.utilities.plotting_kd import Plot_kd

# define the robot
r = (4, 4, 0)
d = (0, 0, 0)
alpha = (0, 0, 0)
base = np.eye(4)

q_max = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
q_min = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
v_max = (2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100)
v_min = (-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100)
a_max = (15, 7.5, 10, 12.5, 15, 20, 20)
a_min = (-15, -7.5, -10, -12.5, -15, -20, -20)

dim = 2
x_init = (0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0)  # starting location
x_goal = (1.0, 1.0, 1.0, -2.0, 1.0, 1.0, 1.0)  # goal location
v_init = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)  # starting location
v_goal = (-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0)  # goal location
X_limits = np.array([(q_min[0], q_max[0]), (q_min[1], q_max[1]), (q_min[2], q_max[2]), (q_min[3], q_max[3]), (q_min[4], q_max[4]), (q_min[5], q_max[5]), (q_min[6], q_max[6])])  # dimensions of Position
V_limits = np.array([(v_min[0], v_max[0]), (v_min[1], v_max[1]), (v_min[2], v_max[2]), (v_min[3], v_max[3]), (v_min[4], v_max[4]), (v_min[5], v_max[5]), (v_min[6], v_max[6])])  # dimensions of Velocity
A_limits = np.array([(a_min[0], a_max[0]), (a_min[1], a_max[1]), (a_min[2], a_max[2]), (a_min[3], a_max[3]), (a_min[4], a_max[4]), (a_min[5], a_max[5]), (a_min[6], a_max[6])])  # dimensions of Acceleration

Q = np.array([(0.1 * np.pi, 10)])  # (length of tree edges, times of randomly sampling)
r = 0.001 * np.pi  # length of smallest edge to check for intersection with obstacles
max_samples = 2048  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_limits)
V = SearchSpace(V_limits)
n = 15
# Obstacles = generate_random_obstacles(X, x_init, x_goal, n)
Obstacles = np.array([[ 4.5,  3.5,  0.5],
                      [-6.0,  6.0,  1.0],
                      [-6.0, -7.0,  2.0],
                      [-3.0, -7.0,  2.0],
                      [ 0.0, -7.0,  2.0]])  # [centers,radiuses]

Start_time = time.perf_counter()

# create rrt_sear
rrt = RRTKd(X, V, X_limits, V_limits, A_limits, Q, x_init, x_goal, v_init, v_goal, max_samples, r, prc, Obstacles, CheckNN=True, Model = "Panda")
# path = rrt.rrt_search()
path_x, path_v = rrt.rrt_kd()

End_time = time.perf_counter()
print("Running time: ")
print(End_time - Start_time)

# plot
# plot = Plot_kd("rrt_2d_with_random_obstacles", X_limits, V_limits, A_limits)
# plot.plot_tree(X, rrt.trees, rrt.trees_v)
# if path_x is not None and path_v is not None:
#     plot.plot_path(X, path_x, path_v)
# # plot.plot_obstacles(X, Obstacles)
# plot.plot_obstacles_circle(X, Obstacles, 0.01*np.pi)
# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)