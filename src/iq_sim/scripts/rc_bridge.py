#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from mavros_msgs.msg import RCIn

class RCNextBridge:
    def __init__(self):
        rospy.init_node("rc_next_bridge", anonymous=True)

        # Params
        self.channel_idx  = rospy.get_param("~channel_idx", 10)   # ch11 -> idx 10
        self.top_max      = rospy.get_param("~top_max", 1300)     # <1300 => TOP
        self.bottom_min   = rospy.get_param("~bottom_min", 1700)  # >1700 => BOTTOM
        self.debounce_sec = rospy.get_param("~debounce_sec", 0.6)

        # Pub
        self.pub_next = rospy.Publisher("/rc/next", Empty, queue_size=1)

        # State
        self.prev_state = None   # "MID" | "TOP" | "BOTTOM" | None
        self.last_fire  = rospy.Time(0)

        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=1)
        rospy.loginfo("rc_next_bridge up (ch=%d, top<%d, bottom>%d, debounce=%.2fs)",
                      self.channel_idx+1, self.top_max, self.bottom_min, self.debounce_sec)

    def rc_state(self, pwm):
        if pwm < self.top_max: return "TOP"
        if pwm > self.bottom_min: return "BOTTOM"
        return "MID"

    def rc_cb(self, msg: RCIn):
        if self.channel_idx >= len(msg.channels):
            return
        cur = self.rc_state(msg.channels[self.channel_idx])

        # Fire on MID -> TOP (debounced)
        if self.prev_state == "MID" and cur == "TOP":
            now = rospy.Time.now()
            if (now - self.last_fire).to_sec() >= self.debounce_sec:
                self.pub_next.publish(Empty())
                self.last_fire = now
        self.prev_state = cur

if __name__ == "__main__":
    RCNextBridge()
    rospy.spin()
