#!/usr/bin/env python
import sys
import rospy
from mavros_msgs.srv import CommandLong

def turn_to_yaw(target_yaw_deg, yaw_rate=10.0, clockwise=True, relative=False):
    """
    Sends MAV_CMD_CONDITION_YAW to MAVROS to rotate the vehicle in place.
    - target_yaw_deg: desired heading in degrees (0–360).
    - yaw_rate: degrees per second (suggest 5–30).
    - clockwise: True for CW, False for CCW.
    - relative: True to yaw relative to current heading, False for absolute heading.
    """
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        resp = cmd(
            command=115,                # MAV_CMD_CONDITION_YAW
            confirmation=0,
            param1=target_yaw_deg,      # yaw angle
            param2=yaw_rate,            # yaw speed
            param3=1 if clockwise else -1,  # direction: 1=CW, -1=CCW
            param4=1 if relative else 0,    # relative(1) or absolute(0)
            param5=0, param6=0, param7=0
        )
        if resp.success:
            rospy.loginfo(f"Yaw command sent: {target_yaw_deg}°, rate={yaw_rate}°/s")
        else:
            rospy.logerr("Yaw command rejected by FCU")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('turn_to_yaw')
    if len(sys.argv) < 2:
        print("Usage: rosrun <pkg> turn_to_yaw.py <yaw_deg> [yaw_rate] [cw|ccw] [abs|rel]")
        sys.exit(1)

    # parse arguments
    yaw = float(sys.argv[1])
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    cw = True if len(sys.argv) < 4 or sys.argv[3].lower() == 'cw' else False
    rel = True if len(sys.argv) > 4 and sys.argv[4].lower() == 'rel' else False

    turn_to_yaw(yaw, yaw_rate=rate, clockwise=cw, relative=rel)
