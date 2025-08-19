#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import RCIn

class WaypointLooper:
    def __init__(self):
        # ---------------- Params ----------------
        self.csv_path     = rospy.get_param("~csv_path", os.path.expanduser("~/Desktop/waypoints.csv"))
        self.frame_id     = rospy.get_param("~frame_id", "world")
        self.rc_channel   = int(rospy.get_param("~rc_channel", 7))   # 1-based
        self.debounce_sec = float(rospy.get_param("~debounce_sec", 0.5))

        # PWM thresholds for 3-pos switch
        self.top_max  = float(rospy.get_param("~top_max", 1200.0))   # <= TOP (~1000)
        self.mid_min  = float(rospy.get_param("~mid_min", 1400.0))   # MID window
        self.mid_max  = float(rospy.get_param("~mid_max", 1600.0))
        self.bot_min  = float(rospy.get_param("~bot_min", 1800.0))   # >= BOT (~2000)

        # NEW: pause between waypoints (seconds)
        self.pause_sec = float(rospy.get_param("~pause_sec", 3.0))

        # ---------------- State ----------------
        self.waypoints    = []       # loaded only on trigger
        self.run_active   = False
        self.idx          = 0
        self.started_send = False
        self.fsm_state    = None
        self.last_fsm     = None

        self.prev_rc_state     = None
        self.last_trigger_time = rospy.Time(0)

        # One-shot timer used to delay the next send
        self.pause_timer = None

        # ---------------- Pub/Sub ----------------
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.fsm_sub  = rospy.Subscriber("/super_fsm_state", String, self.fsm_cb, queue_size=20)
        self.rc_sub   = rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)

        rospy.loginfo("Waypoint looper ready. Waiting for RC ch%d MID→BOT to start.", self.rc_channel)

    # ---------- CSV loader (called only on trigger) ----------
    def _load_csv(self, path):
        wps = []
        if not os.path.isfile(path):
            rospy.logerr("Waypoints CSV not found: %s", path)
            return wps
        with open(path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                parts = [p for p in (line.split(",") if "," in line else line.split()) if p != ""]
                if len(parts) != 7:
                    rospy.logwarn("Skipping malformed row (need 7 values): %s", line)
                    continue
                try:
                    x, y, z, qx, qy, qz, qw = map(float, parts)
                except Exception as e:
                    rospy.logwarn("Skipping unparsable row: %s (err: %s)", line, e)
                    continue
                wps.append((x, y, z, qx, qy, qz, qw))
        return wps

    # ---------- RC state classifier ----------
    def rc_state_from_pwm(self, pwm):
        if pwm <= self.top_max:
            return "TOP"
        if self.mid_min <= pwm <= self.mid_max:
            return "MID"
        if pwm >= self.bot_min:
            return "BOT"
        # Treat unknown as MID to avoid noise triggers
        return "MID"

    # ---------- RC callback with debounce (MID→BOT) ----------
    def rc_cb(self, msg: RCIn):
        idx = self.rc_channel - 1
        if idx < 0 or idx >= len(msg.channels):
            return
        pwm = msg.channels[idx]
        cur_state = self.rc_state_from_pwm(pwm)

        # Detect MID -> BOT edge
        if self.prev_rc_state == "MID" and cur_state == "BOT" and not self.run_active:
            now = rospy.Time.now()
            if (now - self.last_trigger_time).to_sec() >= self.debounce_sec:
                self.last_trigger_time = now
                rospy.loginfo("RC ch%d edge MID→BOT (pwm=%d). Attempting to start waypoint run…",
                              self.rc_channel, pwm)
                self.start_run()

        self.prev_rc_state = cur_state

    # ---------- FSM handling ----------
    def fsm_cb(self, msg: String):
        state = (msg.data or "").strip().upper()
        self.fsm_state = state

        if not self.run_active:
            self.last_fsm = state
            return

        # First waypoint: send immediately when planner is INIT/WAIT_GOAL
        if not self.started_send and state in ("INIT", "WAIT_GOAL"):
            self.started_send = True
            self._maybe_send_next()

        # Subsequent waypoints: schedule after pause when entering WAIT_GOAL
        elif self.last_fsm != "WAIT_GOAL" and state == "WAIT_GOAL":
            self._schedule_next_after_pause()

        self.last_fsm = state

    # ---------- Pause scheduling ----------
    def _schedule_next_after_pause(self):
        # Cancel any existing timer to avoid stacking
        if self.pause_timer is not None:
            try:
                self.pause_timer.shutdown()
            except Exception:
                pass
            self.pause_timer = None

        if self.idx >= len(self.waypoints):
            self.finish_run()
            return

        if self.pause_sec <= 0.0:
            # No pause requested
            self._maybe_send_next()
            return

        rospy.loginfo("Reached waypoint; pausing for %.2f s before next goal…", self.pause_sec)
        self.pause_timer = rospy.Timer(
            rospy.Duration(self.pause_sec),
            self._pause_timer_cb,
            oneshot=True
        )

    def _pause_timer_cb(self, _event):
        if not self.run_active:
            return
        self._maybe_send_next()

    # ---------- Run control ----------
    def start_run(self):
        # (Re)load CSV on each trigger
        self.waypoints = self._load_csv(self.csv_path)
        if not self.waypoints:
            rospy.logerr("No waypoints loaded; staying idle. Ensure file exists and has lines: x y z qx qy qz qw")
            return

        rospy.loginfo("Loaded %d waypoints from %s", len(self.waypoints), self.csv_path)

        self.run_active   = True
        self.idx          = 0
        self.started_send = False
        self.last_fsm     = None

        # If planner is already ready, send the first goal immediately
        if self.fsm_state in ("INIT", "WAIT_GOAL"):
            self.started_send = True
            self._maybe_send_next()
        else:
            rospy.loginfo("Waiting for FSM to be INIT/WAIT_GOAL before sending first goal…")

    def finish_run(self):
        rospy.loginfo("Waypoint run complete (%d waypoints). Waiting for next RC toggle.", len(self.waypoints))
        self.run_active   = False
        self.started_send = False
        self.idx          = 0
        # Clean up any pending timer
        if self.pause_timer is not None:
            try:
                self.pause_timer.shutdown()
            except Exception:
                pass
            self.pause_timer = None

    # ---------- Goal sending ----------
    def _maybe_send_next(self):
        if self.idx >= len(self.waypoints):
            self.finish_run()
            return
        x, y, z, qx, qy, qz, qw = self.waypoints[self.idx]
        self.idx += 1
        self._send_goal(x, y, z, qx, qy, qz, qw)

    def _send_goal(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # Optional: wait briefly for a subscriber
        start = rospy.Time.now()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.goal_pub.get_num_connections() > 0:
                break
            if (rospy.Time.now() - start).to_sec() > 5.0:
                break
            r.sleep()

        self.goal_pub.publish(msg)
        rospy.loginfo("Sent goal %d/%d → (%.3f, %.3f, %.3f)",
                      self.idx, len(self.waypoints), x, y, z)

def main():
    rospy.init_node("waypoint_looper", anonymous=False)
    looper = WaypointLooper()
    rospy.spin()

if __name__ == "__main__":
    main()

