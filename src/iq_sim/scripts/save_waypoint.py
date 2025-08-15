#!/usr/bin/env python3
import os, sys
import rospy
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('save_odom_to_file', anonymous=True)

    # You can override these via ROS params if you want
    odom_topic  = rospy.get_param('~odom_topic', '/mavros/local_position/odom')
    output_file = rospy.get_param('~output_file', os.path.expanduser('~/Desktop/waypoints.csv'))
    timeout_s   = rospy.get_param('~timeout', 5.0)

    try:
        odom = rospy.wait_for_message(odom_topic, Odometry, timeout=timeout_s)
    except rospy.ROSException:
        rospy.logerr(f"Timed out waiting for {odom_topic}")
        sys.exit(1)

    p = odom.pose.pose.position
    q = odom.pose.pose.orientation

    # Ensure Desktop exists and append a CSV line: x,y,z,qx,qy,qz,qw
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'a') as f:
        f.write(f"{p.x:.6f},{p.y:.6f},{p.z:.6f},{q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f}\n")

    print("waypoint saved")
    rospy.loginfo(f"Saved to {output_file}: "
                  f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, "
                  f"qx={q.x:.5f}, qy={q.y:.5f}, qz={q.z:.5f}, qw={q.w:.5f}")

if __name__ == '__main__':
    main()
