#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import time

# Координаты 14 точек (x, y, yaw в радианах) для клиники
waypoints = [
    (1.2, 3.2, 1.57),    # 0.0: 0 или 360градусов -- вверх
    (-1.5, 3.0, 1.57),   # 1.57: 90 градусов -- влево
    (-4.0, 4.5, 3.14),   # 3.14: 180 градусов -- вниз
    (-6.5, -1.5, 1.57),  # -1.57 : 270 градусов -- вправо
    (-1.5, 0.2, 3.14),   
    (-2.0, -3.0, 3.14),  
    (-7.0, -5.5, 0.0), 
    (-4.0, 7.5, 0.0), 
    (7.0, 7.0, 0.0),  
    (7.0, 5.0, 3.14),
    (7.0, 1.5, 3.14),
    (1.0, -3.0, 0.0),
    (6.0, -7.0, 1.57),
    (0.0, -6.2, 0.0)   
]

goal_reached = False

def quaternion_from_yaw(yaw):
    from tf.transformations import quaternion_from_euler
    return quaternion_from_euler(0, 0, yaw)

def create_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = quaternion_from_yaw(yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def status_callback(msg):
    global goal_reached
    if len(msg.status_list) > 0:
        status = msg.status_list[-1].status
        if status == 3:  # Goal reached
            goal_reached = True

def patrol():
    global goal_reached

    rospy.init_node('clinic_patrol')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        for waypoint in waypoints:
            if rospy.is_shutdown():
                break
            x, y, yaw = waypoint
            goal = create_pose(x, y, yaw)
            rospy.loginfo(f"Going to: {x}, {y}, {yaw}")
            goal_reached = False
            pub.publish(goal)

            while not goal_reached and not rospy.is_shutdown():
                rospy.loginfo("Waiting for goal to be reached...")
                rate.sleep()

        rospy.loginfo("Patrol cycle completed. Restarting...")
        rate.sleep()

if __name__ == '__main__':
    try:
        patrol()
    except rospy.ROSInterruptException:
        pass