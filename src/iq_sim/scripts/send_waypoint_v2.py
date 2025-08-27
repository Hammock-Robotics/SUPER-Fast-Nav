#!/usr/bin/env python3
import os, math
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from mavros_msgs.msg import StatusText

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def ang_norm(a): return (a + math.pi) % (2.0 * math.pi) - math.pi
def ang_diff(a, b): return ang_norm(a - b)
def vec_norm3(x, y, z): return math.sqrt(x*x + y*y + z*z)

SEV_INFO = 6
BYTE_LIMIT = 49

def sanitize_for_gcs(text, byte_limit=BYTE_LIMIT):
    s = str(text)
    reps = {"—":"-","–":"-","’":"'", "“":'"',"”":'"',"→":"->","…":"...","°":"deg","×":"x","≥":">=","≤":"<="}
    for k,v in reps.items(): s = s.replace(k,v)
    s = " ".join(s.split())
    b = s.encode("ascii", "ignore")
    if len(b) > byte_limit: b = b[:byte_limit]
    return b.decode("ascii","ignore")

class WaypointLooper:
    def __init__(self):
        # ---------------- Params ----------------
        self.csv_path     = rospy.get_param("~csv_path", os.path.expanduser("~/Desktop/waypoints.csv"))
        self.frame_id     = rospy.get_param("~frame_id", "world")
        self.rc_channel   = int(rospy.get_param("~rc_channel", 11))
        self.debounce_sec = float(rospy.get_param("~debounce_sec", 0.5))
        self.pause_sec    = max(0.0, float(rospy.get_param("~pause_sec", 2.0)))

        # RC thresholds
        self.top_max  = float(rospy.get_param("~top_max", 1200.0))
        self.mid_min  = float(rospy.get_param("~mid_min", 1400.0))
        self.mid_max  = float(rospy.get_param("~mid_max", 1600.0))
        self.bot_min  = float(rospy.get_param("~bot_min", 1800.0))

        # Pose settle
        self.pos_hold_mode        = rospy.get_param("~pos_hold_mode", "target").strip().lower()
        self.pos_tol_m            = float(rospy.get_param("~pos_tol_m", 0.4))
        self.vel_tol_m_s          = float(rospy.get_param("~vel_tol_m_s", 0.15))
        self.pos_rate_max_dist_m  = float(rospy.get_param("~pos_rate_max_dist_m", 1.0))

        # Yaw settle
        self.yaw_hold_mode        = rospy.get_param("~yaw_hold_mode", "target").strip().lower()
        self.yaw_tol_rad          = math.radians(float(rospy.get_param("~yaw_tol_deg", 5.0)))
        self.yaw_rate_tol_rad_s   = math.radians(float(rospy.get_param("~yaw_rate_tol_deg_s", 5.0)))
        self.yaw_rate_max_err_rad = math.radians(float(rospy.get_param("~yaw_rate_max_err_deg", 25.0)))

        # Common settle
        self.settle_dwell_sec     = float(rospy.get_param("~settle_dwell_sec", 0.5))
        self.settle_timeout_sec   = float(rospy.get_param("~settle_timeout_sec", 5.0))
        self.odom_topic           = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.check_hz             = float(rospy.get_param("~check_hz", 20.0))

        # Guided-mode resume policy
        self.resume_policy        = rospy.get_param("~resume_policy", "repeat").strip().lower()  # 'repeat' | 'skip'
        self.guided_modes         = rospy.get_param("~guided_modes", ["GUIDED"])
        if isinstance(self.guided_modes, str):
            self.guided_modes = [m.strip() for m in self.guided_modes.split(",") if m.strip()]
        self.guided_modes = [m.upper() for m in (self.guided_modes or ["GUIDED"])]

        # ---------------- State ----------------
        self.waypoints = []
        self.run_active = False
        self.idx = 0
        self.started_send = False

        # FSM states
        self.fsm_state = None
        self.last_fsm = None

        # Mode states
        self.mode = None
        self.mode_paused = False

        # RC edge detection
        self.prev_rc_state = None
        self.last_trigger_time = rospy.Time(0)

        # Per-goal flags
        self.enroute_active = False
        self.seen_follow_traj = False

        # Pause / countdown
        self.pause_timer = None
        self.countdown_timer = None
        self.countdown_remaining = 0

        # Odom
        self.cur_pos = None
        self.cur_yaw = None
        self.cur_vel = None
        self.cur_yaw_rate = None

        # Goal (also keep quaternion for resend)
        self.goal_pos  = None
        self.goal_yaw  = None
        self.goal_quat = None  # (qx,qy,qz,qw)

        # Settle gate
        self.waiting_settle = False
        self.settle_start_time = None
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None

        # Pub/Sub
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.fsm_sub  = rospy.Subscriber("/super_fsm_state", String, self.fsm_cb, queue_size=20)
        self.rc_sub   = rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=30)
        self.state_sub= rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=20)
        self.gcs_pub  = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10)

        self.check_timer = rospy.Timer(rospy.Duration(1.0/max(1.0,self.check_hz)),
                                       self._settle_check_cb, oneshot=False)

        rospy.loginfo("send_waypoint ready. RC ch%d MID→BOT starts the run. Pose=%s, Yaw=%s",
                      self.rc_channel, self.pos_hold_mode, self.yaw_hold_mode)

    # ---------------- Utils ----------------
    def gcs(self, text, severity=SEV_INFO):
        msg = StatusText()
        msg.header.stamp = rospy.Time.now()
        msg.severity = severity
        msg.text = sanitize_for_gcs(text, BYTE_LIMIT)
        self.gcs_pub.publish(msg)

    def _is_guided(self, mode_str):
        return (mode_str or "").upper() in self.guided_modes

    def _cancel_timers(self):
        if self.pause_timer is not None:
            try: self.pause_timer.shutdown()
            except Exception: pass
            self.pause_timer = None
        if self.countdown_timer is not None:
            try: self.countdown_timer.shutdown()
            except Exception: pass
            self.countdown_timer = None

    # ---------------- CSV loader ----------------
    def _load_csv(self, path):
        wps = []
        if not os.path.isfile(path):
            rospy.logerr("Waypoints CSV not found: %s", path); return wps
        with open(path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line: continue
                parts = [p for p in (line.split(",") if "," in line else line.split()) if p!=""]
                if len(parts)!=7:
                    rospy.logwarn("Skipping malformed row (need 7 values): %s", line); continue
                try:
                    x,y,z,qx,qy,qz,qw = map(float, parts)
                except Exception as e:
                    rospy.logwarn("Skipping unparsable row: %s (err: %s)", line, e); continue
                wps.append((x,y,z,qx,qy,qz,qw))
        return wps

    # ---------------- RC ----------------
    def rc_state_from_pwm(self, pwm):
        if pwm <= self.top_max: return "TOP"
        if self.mid_min <= pwm <= self.mid_max: return "MID"
        if pwm >= self.bot_min: return "BOT"
        return "MID"

    def rc_cb(self, msg: RCIn):
        idx = self.rc_channel - 1
        if idx<0 or idx>=len(msg.channels): return
        pwm = msg.channels[idx]
        cur_state = self.rc_state_from_pwm(pwm)

        # NEW: reset when switching to MID while running
        if self.run_active and cur_state == "MID" and self.prev_rc_state != "MID":
            self._reset_to_start()
            # Allow immediate re-trigger MID->BOT without debounce penalty
            self.last_trigger_time = rospy.Time(0)

        # Start on MID->BOT when idle
        if self.prev_rc_state=="MID" and cur_state=="BOT" and not self.run_active:
            now = rospy.Time.now()
            if (now - self.last_trigger_time).to_sec() >= self.debounce_sec:
                self.last_trigger_time = now
                rospy.loginfo("RC ch%d edge MID→BOT (pwm=%d). Attempting to start waypoint run…", self.rc_channel, pwm)
                self.start_run()

        self.prev_rc_state = cur_state

    # ---------------- Odom ----------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self.cur_pos = (p.x, p.y, p.z)
        self.cur_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.cur_vel = (t.linear.x, t.linear.y, t.linear.z)
        self.cur_yaw_rate = t.angular.z

    # ---------------- Flight mode ----------------
    def state_cb(self, msg: State):
        mode = (msg.mode or "").upper()
        if mode != self.mode:
            prev = self.mode if self.mode is not None else "(none)"
            rospy.loginfo("MODE changed from %s to %s", prev, mode)
            if self.mode is not None:
                self.gcs(f"MODE {prev} to {mode}")

        prev_guided = self._is_guided(self.mode)
        now_guided  = self._is_guided(mode)

        # Leaving GUIDED -> pause
        if self.run_active and prev_guided and not now_guided and not self.mode_paused:
            self._enter_mode_pause(mode)

        # Return to GUIDED -> resume
        if self.run_active and self.mode_paused and now_guided:
            self._resume_after_mode_pause()

        self.mode = mode

    def _enter_mode_pause(self, cur_mode):
        self.mode_paused = True
        self._cancel_timers()
        # Stop active settle checks, but KEEP context of current leg
        self.waiting_settle = False
        rospy.logwarn("Left GUIDED (now %s) — pausing run; waiting to re-enter GUIDED…", cur_mode)
        self.gcs("Paused: left GUIDED")

    def _resume_after_mode_pause(self):
        policy = self.resume_policy
        if policy not in ("repeat", "skip"):
            policy = "repeat"
        rospy.loginfo("GUIDED restored — resume policy: %s", policy)
        self.gcs("GUIDED restored - %s" % policy)

        self.mode_paused = False

        # If no goal ever sent yet, act like first start but respect FSM readiness
        if self.idx == 0 and not self.goal_pos:
            if (self.fsm_state or "") in ("INIT","WAIT_GOAL"):
                self._maybe_send_next()
            else:
                rospy.loginfo("Waiting for FSM to be INIT/WAIT_GOAL before sending first goal…")
            return

        # Otherwise continue current leg
        self.enroute_active = True

        if policy == "repeat":
            if self.goal_pos and self.goal_quat:
                self._resend_current_goal()
            else:
                self._maybe_send_next()
        else:  # skip
            self._maybe_send_next()

        # If we resumed and FSM is already WAIT_GOAL, kick settle gate
        if (self.fsm_state or "").upper() == "WAIT_GOAL" and not self.waiting_settle:
            self._begin_settle_gate()

    # ---------------- FSM ----------------
    def fsm_cb(self, msg: String):
        state = (msg.data or "").strip().upper()

        if state != self.last_fsm:
            prev = self.last_fsm if self.last_fsm is not None else "(none)"
            rospy.loginfo("SUPER FSM state changed from %s to %s", prev, state)
            if self.last_fsm is not None:
                self.gcs(f"{prev} to {state}")

        self.fsm_state = state

        if not self.run_active:
            self.last_fsm = state; return

        # First goal: if planner already idle/ready, send immediately
        if not self.started_send and state in ("INIT","WAIT_GOAL"):
            self.started_send = True
            # Only send if guided; else wait for GUIDED via state_cb
            if self._is_guided(self.mode):
                self._maybe_send_next()
            else:
                rospy.loginfo("Start armed but not in GUIDED (%s). Will send WP1 after GUIDED.", self.mode or "unknown")
        else:
            # Only start settle gate when we've actually been en-route (FOLLOW_TRAJ seen)
            if self.enroute_active:
                if state == "FOLLOW_TRAJ":
                    self.seen_follow_traj = True
                if state == "WAIT_GOAL" and (self.seen_follow_traj or self.mode_paused) and not self.waiting_settle:
                    self._begin_settle_gate()

        self.last_fsm = state

    def _begin_settle_gate(self):
        self.waiting_settle = True
        self.settle_start_time = rospy.Time.now()
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None
        rospy.loginfo("WAIT_GOAL — checking pose & yaw before next goal…")
        self.gcs("WAIT_GOAL - odom check.")

    # ---------------- Settle checker ----------------
    def _settle_check_cb(self, _evt):
        if not (self.run_active and self.waiting_settle): return
        if self.mode_paused: return
        if None in (self.cur_pos, self.cur_yaw, self.cur_vel, self.cur_yaw_rate): return
        if None in (self.goal_pos, self.goal_yaw): return

        now = rospy.Time.now()

        dx = self.cur_pos[0]-self.goal_pos[0]
        dy = self.cur_pos[1]-self.goal_pos[1]
        dz = self.cur_pos[2]-self.goal_pos[2]
        dist = vec_norm3(dx,dy,dz)
        speed = vec_norm3(*self.cur_vel)

        if self.pos_hold_mode=="target":
            pose_ok = (dist <= self.pos_tol_m)
        else:
            pose_ok = (speed <= self.vel_tol_m_s) and (dist <= self.pos_rate_max_dist_m)

        yaw_err = abs(ang_diff(self.cur_yaw, self.goal_yaw))
        if self.yaw_hold_mode=="target":
            yaw_ok = (yaw_err <= self.yaw_tol_rad)
        else:
            yaw_ok = (abs(self.cur_yaw_rate) <= self.yaw_rate_tol_rad_s) and (yaw_err <= self.yaw_rate_max_err_rad)

        if pose_ok and yaw_ok:
            if self.last_pose_ok_since is None or self.last_yaw_ok_since is None:
                self.last_pose_ok_since = now; self.last_yaw_ok_since = now
            else:
                if (now - self.last_pose_ok_since).to_sec() >= self.settle_dwell_sec and \
                   (now - self.last_yaw_ok_since).to_sec() >= self.settle_dwell_sec:
                    rospy.loginfo("Settle OK — dist=%.2fm | yaw_err=%.1f°", dist, math.degrees(yaw_err))
                    self.gcs("Settle OK, d=%.2fm | y=%.1fdeg" % (dist, math.degrees(yaw_err)))
                    self.gcs("Goal reached confirmed.")

                    self.waiting_settle = False
                    self.enroute_active = False
                    self.seen_follow_traj = False

                    if self.idx >= len(self.waypoints):
                        rospy.loginfo("Goal reached confirmed — mission complete.")
                        self.finish_run()
                        return

                    self._schedule_next_after_pause()
                    return
        else:
            self.last_pose_ok_since = None
            self.last_yaw_ok_since  = None

        if (now - self.settle_start_time).to_sec() > self.settle_timeout_sec:
            rospy.logwarn("Settle timeout (%.1fs). Proceeding.", self.settle_timeout_sec)
            self.waiting_settle = False
            self.enroute_active = False
            self.seen_follow_traj = False
            self._schedule_next_after_pause()

    # ---------------- Pause + countdown ----------------
    def _schedule_next_after_pause(self):
        self._cancel_timers()

        if self.idx >= len(self.waypoints):
            self.finish_run(); return

        if self.pause_sec <= 0.0:
            self._announce_heading_next_and_send(); return

        self.pause_timer = rospy.Timer(rospy.Duration(self.pause_sec), self._pause_timer_cb, oneshot=True)
        secs = int(math.ceil(self.pause_sec))
        if secs > 0:
            self.countdown_remaining = secs
            rospy.loginfo("Pausing for %d s…", secs)
            self.gcs("Pausing for %d s..." % secs)
            rospy.loginfo("%d", self.countdown_remaining)
            self.gcs(str(self.countdown_remaining))
            self.countdown_remaining -= 1
            if self.countdown_remaining > 0:
                self.countdown_timer = rospy.Timer(rospy.Duration(1.0), self._countdown_cb, oneshot=False)

    def _countdown_cb(self, _evt):
        if self.countdown_remaining <= 0:
            if self.countdown_timer is not None:
                try: self.countdown_timer.shutdown()
                except Exception: pass
                self.countdown_timer = None
            return
        rospy.loginfo("%d", self.countdown_remaining)
        self.gcs(str(self.countdown_remaining))
        self.countdown_remaining -= 1
        if self.countdown_remaining <= 0:
            if self.countdown_timer is not None:
                try: self.countdown_timer.shutdown()
                except Exception: pass
                self.countdown_timer = None

    def _pause_timer_cb(self, _event):
        if not self.run_active or self.mode_paused:
            return
        self._announce_heading_next_and_send()

    def _announce_heading_next_and_send(self):
        next_one_based = self.idx + 1
        total = len(self.waypoints)
        rospy.loginfo("Heading to waypoint %d/%d", next_one_based, total)
        self._maybe_send_next()

    # ---------------- Run control ----------------
    def start_run(self):
        self.waypoints = self._load_csv(self.csv_path)
        if not self.waypoints:
            rospy.logerr("No waypoints loaded; staying idle. Ensure lines: x y z qx qy qz qw")
            return

        rospy.loginfo("Loaded %d waypoints from %s", len(self.waypoints), self.csv_path)
        self.run_active = True
        self.idx = 0
        self.started_send = False
        self.last_fsm = None
        self.enroute_active = False
        self.seen_follow_traj = False
        self.waiting_settle = False

        # Only start sending if guided; otherwise wait for GUIDED
        if not self._is_guided(self.mode):
            self.mode_paused = True
            rospy.logwarn("Start requested but not in GUIDED (%s). Waiting to enter GUIDED…", self.mode or "unknown")
            self.gcs("Waiting GUIDED to start")
            return

        if (self.fsm_state or "") in ("INIT","WAIT_GOAL"):
            self.started_send = True
            self._maybe_send_next()
        else:
            rospy.loginfo("Waiting for FSM to be INIT/WAIT_GOAL before sending first goal…")

    def _reset_to_start(self):
        """Reset the mission to the beginning while running."""
        rospy.logwarn("RC reset: returning to start. Waiting for MID→BOT to send WP1.")
        self.gcs("Run reset; waiting trigger")

        # Stop everything
        self._cancel_timers()
        self.run_active = False
        self.started_send = False
        self.idx = 0

        # Clear gate/goal context
        self.waiting_settle = False
        self.enroute_active = False
        self.seen_follow_traj = False
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None
        self.goal_pos = None
        self.goal_quat = None
        self.goal_yaw = None

        # Do not modify self.fsm_state or self.mode; we just wait for a fresh trigger

    def finish_run(self):
        rospy.loginfo("Waypoint run complete (%d waypoints). Waiting for next RC toggle.", len(self.waypoints))
        self.run_active = False
        self.started_send = False
        self.idx = 0
        self._cancel_timers()
        self.waiting_settle = False
        self.enroute_active = False
        self.seen_follow_traj = False
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None

    # ---------------- Goal sending ----------------
    def _maybe_send_next(self):
        if self.idx >= len(self.waypoints):
            self.finish_run(); return
        x, y, z, qx, qy, qz, qw = self.waypoints[self.idx]
        self.idx += 1
        self._send_goal(x, y, z, qx, qy, qz, qw)

    def _send_goal(self, x, y, z, qx, qy, qz, qw):
        # store goal (for settle + possible re-send)
        self.goal_pos = (x, y, z)
        self.goal_yaw = quat_to_yaw(qx, qy, qz, qw)
        self.goal_quat = (qx, qy, qz, qw)

        # mark enroute phase for this goal
        self.enroute_active = True
        self.waiting_settle = False

        # publish PoseStamped to /goal
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z
        msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz; msg.pose.orientation.w = qw

        start = rospy.Time.now(); r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.goal_pub.get_num_connections() > 0: break
            if (rospy.Time.now() - start).to_sec() > 5.0: break
            r.sleep()

        self.goal_pub.publish(msg)
        rospy.loginfo("Sent goal %d/%d → (%.3f, %.3f, %.3f), yaw=%.1f°",
                      self.idx, len(self.waypoints), x, y, z, math.degrees(self.goal_yaw))

        # GCS short lines
        self.gcs("Sent waypoint %d/%d" % (self.idx, len(self.waypoints)))
        yaw_deg = math.degrees(self.goal_yaw)
        self.gcs("Sent goal %d/%d -> (%.3f, %.3f, %.3f), yaw=%.0f." %
                 (self.idx, len(self.waypoints), x, y, z, yaw_deg))

    def _resend_current_goal(self):
        """Re-publish the last goal without changing idx."""
        if not (self.goal_pos and self.goal_quat):
            rospy.logwarn("No current goal to resend; sending next instead.")
            self._maybe_send_next()
            return

        x,y,z = self.goal_pos
        qx,qy,qz,qw = self.goal_quat
        yaw_deg = math.degrees(self.goal_yaw) if self.goal_yaw is not None else 0.0

        # stay enroute
        self.enroute_active = True

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z
        msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz; msg.pose.orientation.w = qw

        self.goal_pub.publish(msg)
        rospy.loginfo("Resent goal %d/%d → (%.3f, %.3f, %.3f), yaw=%.1f°",
                      self.idx, len(self.waypoints), x, y, z, yaw_deg)
        self.gcs("Resent goal %d/%d -> (%.3f, %.3f, %.3f), yaw=%.0f." %
                 (self.idx, len(self.waypoints), x, y, z, yaw_deg))

        # If we're already in WAIT_GOAL, kick the settle gate immediately.
        if (self.fsm_state or "").upper() == "WAIT_GOAL" and not self.waiting_settle:
            self._begin_settle_gate()

def main():
    rospy.init_node("send_waypoint", anonymous=False)
    looper = WaypointLooper()
    rospy.spin()

if __name__ == "__main__":
    main()
