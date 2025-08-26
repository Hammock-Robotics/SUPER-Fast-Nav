#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCIn

class RCSwitchState:
    def __init__(self):
        rospy.init_node("rc_switch_state", anonymous=True)

        # Params
        # self.channel_idx = rospy.get_param("~channel_idx", 6)  # sitl   ch7 => index 6
        self.channel_idx = rospy.get_param("~channel_idx", 10)   # ch11 -> idx 10
        self.top_max     = rospy.get_param("~top_max", 1300)     # <1300 => TOP
        self.bottom_min  = rospy.get_param("~bottom_min", 1700)  # >1700 => BOTTOM

        # Publisher (latched so late subscribers get current state immediately)
        self.pub_state = rospy.Publisher("/rc/switch_state", String, queue_size=1, latch=True)

        self.prev_state = None

        rospy.Subscriber(
            "/mavros/rc/in",
            RCIn,
            self.rc_cb,
            queue_size=10
        )

        rospy.loginfo("rc_switch_state up (ch=%d, top<%d, bottom>%d)",
                      self.channel_idx + 1, self.top_max, self.bottom_min)

    def pwm_to_state(self, pwm: int) -> str:
        if pwm < self.top_max:
            return "TOP"
        elif pwm > self.bottom_min:
            return "BOTTOM"
        else:
            return "MID"

    def rc_cb(self, msg: RCIn):
        if self.channel_idx >= len(msg.channels):
            return
        pwm = msg.channels[self.channel_idx]
        cur_state = self.pwm_to_state(pwm)

        if cur_state != self.prev_state:
            self.pub_state.publish(String(data=cur_state))
            rospy.loginfo("[RC] switch_state -> %s (pwm=%d)", cur_state, pwm)
            self.prev_state = cur_state

if __name__ == "__main__":
    RCSwitchState()
    rospy.spin()
