#!/usr/bin/env python3

import rospy
import time

if __name__ == '__main__':
    rospy.init_node('delay_launch', anonymous=True)
    delay = rospy.get_param('~delay', 10)
    rospy.loginfo("Delaying launch by %d seconds...", delay)
    time.sleep(delay)
    rospy.loginfo("Delay complete.")
