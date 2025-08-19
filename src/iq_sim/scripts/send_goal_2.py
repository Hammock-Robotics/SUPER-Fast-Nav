#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

def send_goal():
    rospy.init_node("send_goal_node")
    pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)

    # Read parameters, with defaults if not set
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 0.7)

    rospy.sleep(1.0)  # give ROS time to connect

    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.w = 1.0

    rospy.loginfo(f"Publishing goal at x={x}, y={y}, z={z}")
    pub.publish(goal)
    rospy.loginfo("Goal published!")

if __name__ == "__main__":
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass
