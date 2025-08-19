#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

def send_goal():
    rospy.init_node("send_goal_node")
    pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
    
    rospy.sleep(1.0)  

    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = 1
    goal.pose.position.y = 1
    goal.pose.position.z = 5

    desired_yaw = math.radians(180)  

    qx, qy, qz, qw = quaternion_from_euler(0, 0, desired_yaw)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    rospy.loginfo("Publication du goal...")
    pub.publish(goal)
    rospy.loginfo("Goal publie !")

if __name__ == "__main__":
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass


