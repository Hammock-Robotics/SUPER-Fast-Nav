#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from quadrotor_msgs.msg import PositionCommand  # Replace with the actual package name

class SuperToMavrosBridge:
    def __init__(self):
        rospy.init_node('super_to_mavros_bridge')

        # Subscriber for SUPER output
        self.pos_cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.callback, queue_size=1)

        # Publisher for MAVROS velocity command
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

    def callback(self, msg):
        # Transform the PositionCommand into MAVROS-compatible TwistStamped
        vel_msg = TwistStamped()
        # vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.twist.linear.x = msg.velocity.x
        vel_msg.twist.linear.y = msg.velocity.y
        vel_msg.twist.linear.z = msg.velocity.z
        vel_msg.twist.angular.z = msg.yaw_dot  # Yaw rate
        self.vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        bridge = SuperToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
