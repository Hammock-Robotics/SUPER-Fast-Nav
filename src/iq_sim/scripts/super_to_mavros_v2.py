#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget

class SuperToRawBridge:
    def __init__(self):
        rospy.init_node('super_to_raw_bridge')

        # Publisher: MAVROS will subscribe to this for offboard control
        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=1
        )

        # Subscriber: SUPER’s trajectory commands
        self.cmd_sub = rospy.Subscriber(
            '/planning/pos_cmd',
            PositionCommand,
            self.callback,
            queue_size=1
        )

    def callback(self, cmd: PositionCommand):
        pt = PositionTarget()

        # Header
        # pt.header.stamp = rospy.Time.now()
        # Use the timestamp from SUPER for tight sync
        pt.header.stamp = cmd.header.stamp
        pt.header.frame_id = ''  # usually ignored for local NED

        # Use LOCAL_NED frame
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # type_mask: bitmask of fields to IGNORE.
        # 0 means “use all fields”: position, velocity, accel, yaw, yaw_rate
        pt.type_mask = 0

        # Fill in position
        pt.position.x = cmd.position.x
        pt.position.y = cmd.position.y
        pt.position.z = cmd.position.z

        # Fill in velocity
        pt.velocity.x = cmd.velocity.x
        pt.velocity.y = cmd.velocity.y
        pt.velocity.z = cmd.velocity.z

        # Fill in acceleration (or force)
        pt.acceleration_or_force.x = cmd.acceleration.x
        pt.acceleration_or_force.y = cmd.acceleration.y
        pt.acceleration_or_force.z = cmd.acceleration.z

        # Fill in yaw (rad) and yaw_rate (rad/s)
        pt.yaw      = cmd.yaw
        pt.yaw_rate = cmd.yaw_dot

        # Publish the combined setpoint
        self.raw_pub.publish(pt)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    bridge = SuperToRawBridge()
    bridge.run()


