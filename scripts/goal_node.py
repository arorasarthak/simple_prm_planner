#!/usr/bin/env python3

"""
Topic Mapping Node
/move_base_simple/goal: geometry_msgs/PoseStamped -> /goal: geometry_msgs/PoseStamped
/initialpose: geometry_msgs/PoseWithCovarianceStamped -> /start: geometry_msgs/PoseStamped


Note: The pose values are swapped in the callbacks as rviz treats x and y in an opposite fashion!
"""


import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


def goal_cb(msg):

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = msg.pose.position.y
    goal_pose.pose.position.y = msg.pose.position.x
    goal_pub.publish(goal_pose)

def start_cb(msg):
    
    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.header.stamp = rospy.Time.now()
    start_pose.pose.position.x = msg.pose.pose.position.y
    start_pose.pose.position.y = msg.pose.pose.position.x
    start_pub.publish(start_pose)


rospy.init_node('goal_node')

goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
start_pub = rospy.Publisher("/start", PoseStamped, queue_size=10)

goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb)
start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, start_cb)

rospy.spin()
