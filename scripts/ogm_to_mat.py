#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import time

class OgmToMat:
    """
    Helper class to manage occupancy grid data obtained from map_server.
    """

    def __init__(self):
        rospy.init_node("ogm_to_mat_node")
        self.counter = 0
        self.ogm_data = None  # 1-D array with occupancies
        self.ogm_dims = (None, None)  # Height x Width
        self.ogm_resolution = None  # Grid resolution for px/meter
        self.ogm_mat = None  # Matrix structure for holding the occupancy grid
        self.ogm_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.ogm_callback)
        self.ogm_origin = None  # Map origin from map.yaml
        time.sleep(1)  # Wait for subscriber thread to spawn

    def ogm_callback(self, ogm_msg):
        self.ogm_data = ogm_msg.data
        self.ogm_dims = (ogm_msg.info.height, ogm_msg.info.width)
        self.ogm_resolution = ogm_msg.info.resolution
        self.ogm_origin = (ogm_msg.info.origin.position.x, ogm_msg.info.origin.position.y)

    def __parse_ogm(self):
        """
        Helper function to build and reshape a numpy array to hole the ogm data
        """
        self.ogm_mat = np.array(self.ogm_data, dtype=np.uint8)
        self.ogm_mat = self.ogm_mat.reshape(self.ogm_dims)
        

    def __scale_ogm(self, obstacles):
        """
        Converts map coordinates to world coordinates
        """
        #  Slice obstacle locations
        map_x = obstacles[:, 0]
        map_y = obstacles[:, 1]
        #  Convert grid points to world points in meters
        world_x = map_x * self.ogm_resolution + self.ogm_origin[1]
        world_y = map_y * self.ogm_resolution + self.ogm_origin[0]
        return np.swapaxes(np.vstack([world_x, world_y]), 0, 1)
    
    def to_numpy(self):
        """
        Returns the final numpy-ized occupancy grid obtained from map_server
        """
        obstacle_loc = np.where(self.ogm_mat > 0)
        obstacles = np.array(list(zip(obstacle_loc[0], obstacle_loc[1])))
        return self.__scale_ogm(obstacles)

    def run(self):
        """
        Runs the helper.
        """
        self.__parse_ogm()
        return self.to_numpy()

