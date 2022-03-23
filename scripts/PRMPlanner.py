#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib 
matplotlib.use('TkAgg')  #  workaround for pycharm
from scipy.spatial import cKDTree


class Node:
    """
    None for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


class PRMPlanner:
    """
    Implementation of the probablistic roadmap planner.
    Paper Link: https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/PRM/prmbasic_01.pdf
    """

    def __init__(self, N_SAMPLE, N_KNN, MAX_EDGE_LEN, ROBOT_RADIUS):
        self.N_SAMPLE = N_SAMPLE  # Max number of sample points
        self.N_KNN = N_KNN  # Max Edges Per Node
        self.MAX_EDGE_LEN = MAX_EDGE_LEN  # Maximum allowed edge length
        self.ROBOT_RADIUS = ROBOT_RADIUS  # robot footprint diameter
        self.obstacle_kd_tree = None  # initialize KD tree for obstacles
        self.sample_kd_tree = None  # initialize KD tree for prm samples
        self.prm_samples = None  # N x 2 numpy array containing distributed samples based on the grid
        self.roadmap = None  # To be used as list that will consist the roadmap edges for the graph
        self.path = None  # List of 2D-Pose vectors for the trajectory
        self.obstacles = None  # N x 2 array containing locations of the obstacles

    def setup_obstacles(self, obstacles):
        """
        Loads the obstacles for the planner.

        :param obstacles: N x 2 numpy array of obstacle locations
        """
        self.obstacles = obstacles
        self.obstacle_kd_tree = cKDTree(self.obstacles)

    def __get_map_bounds(self):
        """
        Helper function to compute the map/grid bounds

        :return: 4 valued tuple representing x_max, x_min, y_max, y_min
        """
        x_max = np.max(self.obstacles[:, 0])
        x_min = np.min(self.obstacles[:, 0])
        y_max = np.max(self.obstacles[:, 1])
        y_min = np.min(self.obstacles[:, 1])
        return x_max, x_min, y_max, y_min

    def compute_prm(self, start, goal):
        """
        Computes the full Probabilistic Roadmap

        :param start: Tuple of x,y coordinates for start location
        :param goal: Tuple of x,y coordinates for goal location
        """
        self.compute_samples(start, goal)
        self.compute_roadmap()

    def compute_path(self, start, goal):
        """
        Computes the path using Djikstra's Algorithm based on the PRM.
        This method can be reused with varying start and goal locations on the same PRM.

        :param start: Tuple of x,y coordinates for start location
        :param goal: Tuple of x,y coordinates for goal location
        :return: Return a list of 2D pose vectors representing the trajectory
        """
        self.path = self.compute_djikstra(start, goal)
        return self.path

    def compute_roadmap(self):
        """
        Computes the roadmap using collision checking and generated samples.

        :return: None
        """
        print("Generating Roadmap")
        self.roadmap = [None] * len(self.prm_samples)
        q_out = self.sample_kd_tree.query(self.prm_samples, k=len(self.prm_samples))
        n_dists, n_indices = q_out[0], q_out[1]

        print("Scanning Neighbours for Collisions...", end="")
        for i in range(len(n_indices)):
            indices = n_indices[i]
            edges_idx = []
            _i = 1
            sample_point = self.prm_samples[i]

            while len(edges_idx) < self.N_KNN and _i < len(self.prm_samples):
                neighbour_point = self.prm_samples[indices[_i]]
                if not self.check_valid_neighbours(sample_point, neighbour_point):
                    edges_idx.append(indices[_i])
                _i += 1
            self.roadmap[i] = edges_idx
        print("Done!")
        return None

    def compute_samples(self, start, end):
        """
        A numpy-ized implementation of a uniform dist sampler based on the map/grid size.

        :param start: Tuple of x,y coordinates for start location
        :param end: Tuple of x,y coordinates for end location
        """

        print("Computing samples....", end="")
        x_max, x_min, y_max, y_min = self.__get_map_bounds()
        random_samples = np.random.uniform(size=(self.N_SAMPLE, 2))
        random_samples = (random_samples * ((x_max - x_min), (y_max - y_min))) + (x_min, y_min)
        result = self.obstacle_kd_tree.query(random_samples)
        dists, indices = result[0], result[1]
        free_dists = np.where(dists >= self.ROBOT_RADIUS)
        self.prm_samples = np.vstack((random_samples[free_dists], start, end))
        self.sample_kd_tree = cKDTree(self.prm_samples)
        print("Done!")

    def check_valid_neighbours(self, point_a, point_b):
        """
        Checks for free and valid path between two points.

        :param point_a: Tuple of x,y coordinates representing the point of interest
        :param point_b: Tuple of x,y coordinates representing the neighbour point
        :return: Boolean. If the path is free; True. otw False.
        """
        x = point_a[0]
        y = point_a[1]
        dx = point_b[0] - point_a[0]
        dy = point_b[1] - point_a[1]
        yaw = math.atan2(dy, dx)
        d = math.hypot(dx, dy)

        if d >= self.MAX_EDGE_LEN:
            return True

        D = self.ROBOT_RADIUS
        n_step = round(d / D)

        # Vectorized Implementation of Line Tracing
        # xy = (x, y) + np.linspace((1, 1), (n_step, n_step), n_step) * (D * math.cos(yaw), D * math.sin(yaw))
        # q = obstacle_kd_tree.query(xy)
        # dists = q[0]

        for i in range(n_step):
            dist, _ = self.obstacle_kd_tree.query([x, y])
            if dist <= self.ROBOT_RADIUS:
                return True
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        dist, _ = self.obstacle_kd_tree.query(point_b)
        if dist <= self.ROBOT_RADIUS:
            return True

        return False

    def compute_djikstra(self, start, goal):
        """
        Runs Djikstra's Algorithm to compute the final path between the start and goal.

        :param start: Tuple of x,y coordinates representing start
        :param goal:  Tuple of x,y coordinates representing goal
        :return:
        """
        start_node = Node(start[0], start[1], 0.0, -1)
        goal_node = Node(goal[0], goal[1], 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(self.roadmap) - 2] = start_node

        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            if c_id == (len(self.roadmap) - 1):
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i in range(len(self.roadmap[c_id])):
                n_id = self.roadmap[c_id][i]
                dx = self.prm_samples[:, 0][n_id] - current.x
                dy = self.prm_samples[:, 1][n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(self.prm_samples[:, 0][n_id], self.prm_samples[:, 1][n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue

                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry

#  Test code
# def main():
#     # load test obstacles
#     arr = np.load("scaled_obs.npy")
#     ox = arr[:, 0]
#     oy = arr[:, 1]

#     # start and goal position
#     start = (31, 30)
#     goal = (36.2, 22.2)

#     robot_size = 0.08  # [m]

#     planner = PRMPlanner(N_SAMPLE=2500, N_KNN=10, MAX_EDGE_LEN=2.5, ROBOT_RADIUS=robot_size)
#     obs = np.swapaxes(np.vstack([ox, oy]), 0, 1)
#     planner.setup_obstacles(obs)
#     planner.compute_prm(start, goal)
#     rx, ry = planner.compute_path(start, goal)
#     plt.plot(ox, oy, ",")

#     for pt in zip(rx, ry):
#         plt.plot(pt[0], pt[1], "o")

#     plt.show()


# if __name__ == "__main__":
#     main()
