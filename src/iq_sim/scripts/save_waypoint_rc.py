#!/usr/bin/env python3
import os
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn

class RCWaypointSaver:
    def __init__(self):
        rospy.init_node('rc_waypoint_saver', anonymous=True)

        # --- Config ---
        self.file_path = os.path.expanduser("~/Desktop/waypoints.csv")
        #self.channel_idx = rospy.get_param("~channel_idx", 6)  # sitl   ch7 => index 6
        self.channel_idx = rospy.get_param("~channel_idx", 10)  # drone ch11 => index 10
        # Hysteresis thresholds (µs)
        self.top_max = rospy.get_param("~top_max", 1300)        # < 1300 -> TOP
        self.bottom_min = rospy.get_param("~bottom_min", 1700)  # > 1700 -> BOTTOM
        self.debounce_sec = rospy.get_param("~debounce_sec", 0.6)

        # --- State ---
        self.last_pose = None  # (x,y,z,qx,qy,qz,qw)
        self.prev_rc_state = None  # 'TOP' | 'MID' | 'BOTTOM' | None
        self.last_trigger_time = rospy.Time(0)

        # --- Subs ---
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)
        

        rospy.loginfo("RC waypoint saver ready: ch%d mid->top edge saves waypoint to %s",
                      self.channel_idx+1, self.file_path)

    # --- Helpers ---
    def rc_state_from_pwm(self, pwm):
        if pwm is None:
            return None
        if pwm < self.top_max:
            return "TOP"
        elif pwm > self.bottom_min:
            return "BOTTOM"
        else:
            return "MID"

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def rc_cb(self, msg: RCIn):
        # Ensure channel exists
        if self.channel_idx >= len(msg.channels):
            return
        pwm = msg.channels[self.channel_idx]
        cur_state = self.rc_state_from_pwm(pwm)

        # Detect MID -> TOP edge
        if self.prev_rc_state == "MID" and cur_state == "TOP":
            # Debounce
            now = rospy.Time.now()
            if (now - self.last_trigger_time).to_sec() >= self.debounce_sec:
                self.save_waypoint()
                self.last_trigger_time = now

        self.prev_rc_state = cur_state

    def save_waypoint(self):
        if self.last_pose is None:
            rospy.logwarn("No odometry yet; cannot save waypoint.")
            return
        x, y, z, qx, qy, qz, qw = self.last_pose
        line = f"{x:.6f},{y:.6f},{z:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n"
        try:
            with open(self.file_path, "a") as f:
                f.write(line)
            rospy.loginfo("Waypoint saved: %s", line.strip())
            print("waypoint saved")  # plain print as you asked
        except Exception as e:
            rospy.logerr("Failed to write waypoint: %s", e)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    RCWaypointSaver().run()
