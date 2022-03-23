#!/usr/bin/env python3

import rospy
from PRMPlanner import PRMPlanner
from ogm_to_mat import OgmToMat
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time


class PlanningNode:
    """Implementation for the solution to the assignment"""
    
    def __init__(self):
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.goal_subscriber = rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        self.start_subscriber = rospy.Subscriber("/start", PoseStamped, self.start_callback)
        self.goal_state = None
        self.start_state = None
        self.planner = None  # prm planner goes here
        self.path = None  # final path 
        self.START_PLANNING = False 
        self.prm_exists = False  # flag for re-using the prm
        time.sleep(1)  # wait for subscriber threads to spawn 

    def goal_callback(self, goal_msg):
        self.goal_state = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.run_planner()  # run as soon as goal is recieved  

    def start_callback(self, start_msg):
        self.start_state = (start_msg.pose.position.x, start_msg.pose.position.y)
        

    def init_planner(self, planner):
        """
        Initilizes the planner and obstacles from the occupancy grid.
        """
        self.planner = planner
        ogm2mat = OgmToMat()
        obstacles = ogm2mat.run()
        if obstacles is not None:        
            self.planner.setup_obstacles(obstacles)
            print("detected ", len(obstacles), " occupancies in the grid map...")
            self.START_PLANNING = True

    def run_planner(self):
        """
        Runs the planner once start and goal pose are recieved
        """
        if self.start_state and self.goal_state:
            
            print("Planning Path Between")
            print(self.start_state," and ",self.goal_state)
            
            #if self.prm_exists:
            self.planner.compute_prm(self.start_state, self.goal_state)
            self.prm_exists = True
            
            self.path = self.planner.compute_path(self.start_state, self.goal_state)
            self.publish_path()

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        
        for pt in zip(self.path[0], self.path[1]):
            pose = PoseStamped()
            pose.pose.position.x = pt[1]
            pose.pose.position.y = pt[0]
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0 
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def run(self):
        rospy.spin()

def main():
    p_node = PlanningNode()
    prm = PRMPlanner(N_SAMPLE=2500, N_KNN=10, MAX_EDGE_LEN=2.5, ROBOT_RADIUS=0.08)
    p_node.init_planner(prm)  # initilize node with the prm planner
    p_node.run()  # runs the planner

if __name__ == '__main__':
    main()

