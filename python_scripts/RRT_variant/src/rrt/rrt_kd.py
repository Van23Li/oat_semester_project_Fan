import numpy as np
import random
from src.rrt.rrt import RRT
from src.rrt.tree import Tree
from obs_checking.OptimalModulationDS.python_scripts.parse_matlab_network import Net
import scipy.io as sio
import torch
from obs_checking.Neural_JSDF.learning.nn_learning.sdf.robot_sdf import RobotSdfCollisionNet

class RRTKd(RRT):
    def __init__(self, X, V, X_limits, V_limits, A_limits, Q, x_init, x_goal, v_init, v_goal, max_samples, r, prc=0.01, Obstacles = None, CheckNN=False, Model=None):
        """
        RRT* Search
        :param X: Search Space
        :param V_limits: Velocity Limits
        :param A_limits: Acceleration Limits
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param dim: the dimension of the position (for kinodynamic planner, it equals to the dimension of X, otherwise,
                        it equals to the half of the dimension of X)
        :param prc: probability of checking whether there is a solution
        :param rewire_count: number of nearby vertices to rewire
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc, Obstacles, CheckNN, Model)
        self.V = V
        self.X_limits = X_limits
        self.V_limits = V_limits
        self.A_limits = A_limits
        # self.Obstacles = Obstacles
        # self.CheckNN = CheckNN
        # self.Model = Model
        # if CheckNN:
        #     if Model == "Panda":
        #         device = torch.device('cpu')
        #
        #         s = 256
        #         n_layers = 5
        #         skips = []
        #         if skips == []:
        #             n_layers -= 1
        #         tensor_args = {'device': device, 'dtype': torch.float32}
        #         nn_model = RobotSdfCollisionNet(in_channels=10, out_channels=9, layers=[s] * n_layers, skips=skips)
        #         nn_model.load_weights(
        #             '../../obs_checking/Neural_JSDF/learning/nn_learning/sdf_256x5_mesh_origin.pt',
        #             tensor_args)
        #
        #         nn_model.model.to(**tensor_args)
        #         self.net = nn_model.model
        #     else:
        #         mat_contents = sio.loadmat(
        #             '../../obs_checking/OptimalModulationDS/matlab_scripts/planar_robot_2d/data/net_parsed.mat')
        #         W = mat_contents['W'][0]
        #         b = mat_contents['b'][0]
        #
        #         # create net
        #         self.net = Net()
        #         self.net.setWeights(W, b)

        self.v_init = v_init
        self.v_goal = v_goal
        self.trees_v = []  # list of all trees
        self.add_tree_v()  # add initial tree



    def add_tree_v(self):
        """
        Create an empty tree and add to trees
        """
        self.trees_v.append(Tree(self.V))

    def dist_kd(self, tree, p, v):
        """
        Return the minimum time to steer from each node in tree to q
        :param tree: int, tree being searched
        :param q: the target node
        :return: vertex, nearest vertex in tree to new vertex
        """

        v_min = self.V_limits[:, 0]
        v_max = self.V_limits[:, 1]
        a_min = self.A_limits[:, 0]
        a_max = self.A_limits[:, 1]

        p_1 = np.zeros([self.trees[0].V_count, len(p)])
        p_2 = np.array(p)
        v_1 = np.zeros([self.trees[0].V_count, len(p)])
        v_2 = np.array(v)

        # for i, tree in enumerate(self.trees):
        j = 0
        for start, end in self.trees[tree].E.items():
            p_1[j, :] = np.array(start)
            j = j + 1
        j = 0
        for start, end in self.trees_v[tree].E.items():
            v_1[j, :] = np.array(start)
            j = j + 1

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

        return np.max(t, axis=1)

    def near_kd(self, tree, q, v):
        """
        Return the vertex in tree that is nearest to q based on kinodynamic steering method
        :param tree: int, tree being searched
        :param q: the target node
        :return: vertex, nearest vertex in tree to new vertex
        """

        # calculate the minimum time to steer from each node in tree to q
        distance = self.dist_kd(tree, q, v)
        M = min(distance)
        I = np.where(distance == M)

        j = 0
        for start, end in self.trees[tree].E.items():
            if j == I[0]:
                x_nearest = start
                break
            j = j + 1
        j = 0
        for start, end in self.trees_v[tree].E.items():
            if j == I[0]:
                v_nearest = start
                break
            j = j + 1

        return M, x_nearest, v_nearest


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

    def checkPath_kd(self, n_x, n_v, newPos):
        # newPos include [a_1, v_limit, a_2, t_1, t_v, t_2]
        a_1 = newPos[0]
        v_limit = newPos[1]
        a_2 = newPos[2]
        t_1 = newPos[3]
        t_v = newPos[4]
        t_2 = newPos[5]

        q_1_end = n_x + n_v * t_1 + 1 / 2 * a_1 * np.square(t_1)
        v_1_end = n_v + a_1 * t_1
        q_v_end = q_1_end + v_1_end * t_v
        v_v_end = v_1_end
        q_2_end = q_v_end + v_v_end * t_2 + 1 / 2 * a_2 * np.square(t_2)
        v_2_end = v_v_end + a_2 * t_2

        for t in np.arange(0, t_1[0] + t_v[0] + t_2[0], 0.05):
            q_check = np.array(n_x)
            for j in range(len(n_x)):
                if t < t_1[j]:
                    q_1 = n_x[j] + n_v[j] * t + 1 / 2 * a_1[j] * np.square(t)
                elif t < t_1[j] + t_v[j]:
                    q_1 = q_1_end[j] + v_1_end[j] * (t - t_1[j])
                else:
                    q_1 = q_v_end[j] + v_v_end[j] * (t - t_v[j] - t_1[j]) + 1 / 2 * a_2[j] * np.square(t - t_v[j] - t_1[j])
                q_check[j] = q_1
            if self.CheckNN:
                collied_free = self.collied_check(q_check)
            else:
                collied_free = self.X.obstacle_free(q_check)  # equals to True if free

            if not collied_free or sum(q_check < self.X_limits[:,0]) or sum(q_check > self.X_limits[:,1]):
                return True, q_check

        return False, q_check

    def connect_to_point_kd(self, tree, x_a, v_a, x_b, v_b):
        """
        Connect vertex x_a in tree to vertex x_b
        :param tree: int, tree to which to add edge
        :param x_a: tuple, vertex
        :param x_b: tuple, vertex
        :return: bool, True if able to add edge, False if prohibited by an obstacle
        """
        if self.trees[tree].V.count(x_b) == 0 and self.trees_v[tree].V.count(v_b) == 0:
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            self.add_vertex_kd(tree, v_b)
            self.add_edge_kd(tree, v_b, v_a)
            return True
        return False

    def new(self):
        """
        Return a new steered vertex
        :return: vertex, new steered vertex
        """
        x_rand = self.X.sample_free()
        v_rand = self.V.sample_free()
        # check if new point is in X_free and not already in V
        if self.CheckNN:
            collied_free = self.collied_check(x_rand)
        else:
            collied_free = self.X.obstacle_free(x_rand) # equals to True if free
        if (not self.trees[0].V.count(x_rand) == 0 and not self.trees_v[0].V.count(v_rand) == 0) or not collied_free:
            return None, None
        self.samples_taken += 1
        return x_rand, v_rand

    def add_vertex_kd(self, tree, v):
        """
        Add vertex to corresponding tree
        :param tree: int, tree to which to add vertex
        :param v: tuple, vertex to add
        """
        self.trees_v[tree].V.insert(0, v + v, v)
        self.trees_v[tree].V_count += 1  # increment number of vertices in tree
        self.samples_taken += 1  # increment number of samples taken

    def add_edge_kd(self, tree, child, parent):
        """
        Add edge to corresponding tree
        :param tree: int, tree to which to add vertex
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.trees_v[tree].E[child] = parent

    def check_solution_kd(self):
        # probabilistically check if solution found
        if self.prc and random.random() < self.prc:
            print("Checking if can connect to goal at", str(self.samples_taken), "samples")
            path_x, path_v = self.get_path_kd()
            if path_x is not None and path_v is not None:
                return True, path_x, path_v
        # check if can connect to goal after generating max_samples
        if self.samples_taken >= self.max_samples:
            path_x, path_v = self.get_path_kd()
            return True, path_x, path_v
        return False, None

    def get_path_kd(self):
        """
        Return path through tree from start to goal
        :return: path if possible, None otherwise
        """
        can_connect, x_nearest, v_nearest = self.can_connect_to_goal_kd(0)
        if can_connect:
            print("Can connect to goal")
            self.trees[0].E[self.x_goal] = x_nearest
            self.trees_v[0].E[self.v_goal] = v_nearest
            return self.reconstruct_path(0, self.x_init, self.x_goal), self.reconstruct_path_kd(0, self.v_init, self.v_goal)
        print("Could not connect to goal")
        return None, None

    def reconstruct_path_kd(self, tree, v_init, v_goal):
        """
        Reconstruct path from start to goal
        :param tree: int, tree in which to find path
        :param x_init: tuple, starting vertex
        :param x_goal: tuple, ending vertex
        :return: sequence of vertices from start to goal
        """
        path = [v_goal]
        current = v_goal
        if v_init == v_goal:
            return path
        while not self.trees_v[tree].E[current] == v_init:
            path.append(self.trees_v[tree].E[current])
            current = self.trees_v[tree].E[current]
        path.append(v_init)
        path.reverse()
        return path

    def can_connect_to_goal_kd(self, tree):
        """
        Check if the goal can be connected to the graph
        :param tree: rtree of all Vertices
        :return: True if can be added, False otherwise
        """
        minDistance, x_nearest, v_nearest = self.near_kd(tree, self.x_goal, self.v_goal)
        # x_nearest = self.get_nearest(tree, self.x_goal)
        if self.x_goal in self.trees[tree].E and x_nearest in self.trees[tree].E[self.x_goal] and \
            self.v_goal in self.trees_v[tree].E and v_nearest in self.trees_v[tree].E[self.v_goal]:
            # tree is already connected to goal using nearest vertex
            return True, x_nearest, v_nearest

        newPoint = self.Steer(x_nearest, v_nearest, self.x_goal, self.v_goal, minDistance)
        # a_1, v_limit, a_2, t_1, t_v, t_2 = self.Steer(x_nearest, x_new, minDistance)

        # Chech wheter the path from closest node to new node is available
        collided, collidedPoint = self.checkPath_kd(x_nearest, v_nearest, newPoint)
        if not collided:
            return True, x_nearest, v_nearest
        return False, None, None

    def rrt_kd(self):
        """
        Based on algorithm: Probabilistically Complete Kinodynamic Planning for Robot Manipulators with Acceleration Limits
        https://ieeexplore.ieee.org/document/6943083
        :return: TODO
        """
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)
        self.add_vertex_kd(0, self.v_init)
        self.add_edge_kd(0, self.v_init, None)

        while True:
            for q in self.Q:  # iterate over different edge lengths until solution found or time out
                for i in np.arange(0, q[1], 1):  # iterate over number of edges of given length to add

                    # obtain [q, q_dot] randomly
                    x_new, v_new = self.new()
                    if x_new is None or v_new is None:
                        continue

                    # find the nearest node in tree
                    minDistance, x_nearest, v_nearest = self.near_kd(0, x_new, v_new)

                    # Calculate the steering parameters from [x_nearest, v_nearest] to [x_new, v_new]
                    newPoint = self.Steer(x_nearest, v_nearest, x_new, v_new, minDistance)
                    # a_1, v_limit, a_2, t_1, t_v, t_2 = self.Steer(x_nearest, x_new, minDistance)

                    # Chech wheter the path from closest node to new node is available
                    collided, collidedPoint = self.checkPath_kd(x_nearest, v_nearest, newPoint)

                    if collided:
                        continue

                    # connect shortest valid edge
                    self.connect_to_point_kd(0, x_nearest, v_nearest, x_new, v_new)

                    solution = self.check_solution_kd()
                    if solution[0]:
                        return solution[1], solution[2]

