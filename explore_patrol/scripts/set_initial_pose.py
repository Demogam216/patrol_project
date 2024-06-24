#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tft

def set_initial_pose(x, y, theta):
    rospy.init_node('set_initial_pose')

    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0

    quaternion = tft.quaternion_from_euler(0, 0, theta)
    pose.pose.pose.orientation.x = quaternion[0]
    pose.pose.pose.orientation.y = quaternion[1]
    pose.pose.pose.orientation.z = quaternion[2]
    pose.pose.pose.orientation.w = quaternion[3]

    rospy.sleep(1)  # Give time to the publisher to register
    initial_pose_pub.publish(pose)
    rospy.loginfo("Initial pose set to x: {}, y: {}, theta: {}".format(x, y, theta))

if __name__ == "__main__":
    # Define the initial position and orientation (yaw) here
    x_initial = 1.0
    y_initial = 3.0
    theta_initial = 0.0  # Change this to your desired orientation

    set_initial_pose(x_initial, y_initial, theta_initial)
