#!/usr/bin/env python3
import os
import rospy
# from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from arducam.srv import CaptureStill   # add this import at the top


class CameraStillRCTrigger:
    def __init__(self):
        rospy.init_node('rc_waypoint_saver', anonymous=True)

        # --- Config ---
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
        self.capture_srv = rospy.ServiceProxy("/camera/still_capture", CaptureStill)
        # rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)

        rospy.loginfo("Still Capture via RC ready: ch%d mid->top edge",
                      self.channel_idx+1)



    def take_picture(self):
        try:
            resp = self.capture_srv()
            if resp.success:
                rospy.loginfo("✅ Picture taken: %s" % resp.message)
            else:
                rospy.logwarn("⚠️ Capture failed: %s" % resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("❌ Service call failed: %s" % str(e))

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

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    CameraStillRCTrigger().run()
