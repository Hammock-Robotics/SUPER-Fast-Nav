#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import RCIn, State, WaypointList, Waypoint, HomePosition
from mavros_msgs.srv import WaypointPull
from nav_msgs.msg import Odometry
from mavros_msgs.msg import StatusText
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic

# -------------- math helpers --------------
def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))

def ang_norm(a): return (a + math.pi) % (2.0 * math.pi) - math.pi
def ang_diff(a, b): return ang_norm(a - b)
def vec_norm3(x, y, z): return math.sqrt(x*x + y*y + z*z)
def hypot2(x, y): return math.sqrt(x*x + y*y)

def rot2d(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    return c*x - s*y, s*x + c*y

# MAVLink command IDs we care about
MAV_CMD_NAV_WAYPOINT       = 16
MAV_CMD_NAV_SPLINE_WAYPOINT= 82
MAV_CMD_CONDITION_YAW      = 115

# MAVLink frames we care about
MAV_FRAME_GLOBAL = 0                  # lat,lon,alt (AMSL)
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3     # lat,lon,alt (rel home)

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

class WaypointLooperMP:
    def __init__(self):
        # ---------------- Params ----------------
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

        # Mission->local conversion
        self.use_condition_yaw    = rospy.get_param("~use_condition_yaw", True)
        self.yaw_source           = rospy.get_param("~yaw_source", "wp").strip().lower()  # 'wp' | 'path' | 'current'
        self.alt_mode             = rospy.get_param("~alt_mode", "mission").strip().lower()  # 'mission' | 'current' | 'fixed'
        self.fixed_alt_m          = float(rospy.get_param("~fixed_alt_m", 2.0))
        self.z_offset_m           = float(rospy.get_param("~z_offset_m", 0.0))  # optional bias
        self.global_alt_strategy  = rospy.get_param("~global_alt_strategy", "home_rel").strip().lower()  # 'home_rel' | 'current'

        # Live re-targeting while enroute (default 10 m)
        self.update_dist_m        = float(rospy.get_param("~update_dist_m", 10.0))

        # ENU->local auto-alignment (for ExternalNav yaw)
        self.auto_align_enu       = rospy.get_param("~auto_align_enu", True)
        self.align_min_move_m     = float(rospy.get_param("~align_min_move_m", 15.0))
        self.align_theta          = None         # radians (ENU->local rotation)
        self.align_ref_gps        = None         # (lat, lon)
        self.align_ref_xy         = None         # (x, y)

        # ---------------- State ----------------
        self.raw_all_wps = []     # entire mission list from FCU (all commands)
        self.mission_wps = []     # list of NAV waypoints we will fly (first NAV skipped)
        self.yaw_overrides = []   # same length as mission_wps; dict or None

        self.run_active = False
        self.idx = 0
        self.started_send = False

        # SUPER FSM
        self.fsm_state = None
        self.last_fsm = None

        # Mode
        self.mode = None
        self.mode_paused = False

        # RC
        self.prev_rc_state = None
        self.last_trigger_time = rospy.Time(0)

        # Per-goal flags
        self.enroute_active = False
        self.seen_follow_traj = False

        # Pause / countdown
        self.pause_timer = None
        self.countdown_timer = None
        self.countdown_remaining = 0

        # Odom + GPS + Home
        self.cur_pos = None      # (x,y,z)
        self.cur_yaw = None
        self.cur_vel = None
        self.cur_yaw_rate = None
        self.cur_lat = None
        self.cur_lon = None
        self.home_alt_amsl = None

        # Current goal & mission WP
        self.goal_pos  = None
        self.goal_yaw  = None
        self.goal_quat = None
        self.cur_wp    = None
        self.cur_wp_idx= None

        # Settle gate
        self.waiting_settle = False
        self.settle_start_time = None
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None

        # Live re-targeting bookkeeping
        self.last_update_xy = None  # (x,y) where we last recomputed

        # ---------------- Pub/Sub ----------------
        self.goal_pub  = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.fsm_sub   = rospy.Subscriber("/super_fsm_state", String, self.fsm_cb, queue_size=20)
        self.rc_sub    = rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)
        self.odom_sub  = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=30)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=20)
        self.gcs_pub   = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10)
        self.gps_sub   = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb, queue_size=20)
        self.wp_sub    = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.wp_list_cb, queue_size=2)
        self.home_sub  = rospy.Subscriber("/mavros/home_position/home", HomePosition, self.home_cb, queue_size=1)

        self.check_timer = rospy.Timer(rospy.Duration(1.0/max(1.0,self.check_hz)),
                                       self._periodic_cb, oneshot=False)

        rospy.loginfo("send_waypoint_mp ready. RC ch%d MID→BOT starts. Pose=%s, Yaw=%s, yaw_src=%s, alt=%s",
                      self.rc_channel, self.pos_hold_mode, self.yaw_hold_mode,
                      self.yaw_source, self.alt_mode)

    # ---------------- GCS helper ----------------
    def gcs(self, text, severity=SEV_INFO):
        msg = StatusText()
        msg.header.stamp = rospy.Time.now()
        msg.severity = severity
        msg.text = sanitize_for_gcs(text, BYTE_LIMIT)
        self.gcs_pub.publish(msg)

    # ---------------- Simple utils ----------------
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

    # ---------------- Subscribers ----------------
    def home_cb(self, msg: HomePosition):
        self.home_alt_amsl = msg.geo.altitude

    def gps_cb(self, msg: NavSatFix):
        self.cur_lat = msg.latitude
        self.cur_lon = msg.longitude

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self.cur_pos = (p.x, p.y, p.z)
        self.cur_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.cur_vel = (t.linear.x, t.linear.y, t.linear.z)
        self.cur_yaw_rate = t.angular.z

    def wp_list_cb(self, msg: WaypointList):
        # keep the full list (we need CONDITION_YAW too)
        self.raw_all_wps = list(msg.waypoints)
        nav_count = sum(1 for w in self.raw_all_wps if w.command in (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT))
        rospy.loginfo("Mission received: %d total items (%d NAV)", len(self.raw_all_wps), nav_count)

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

        # Reset to start when switching to MID while running
        if self.run_active and cur_state == "MID" and self.prev_rc_state != "MID":
            self._reset_to_start()
            self.last_trigger_time = rospy.Time(0)

        # Start on MID->BOT when idle
        if self.prev_rc_state=="MID" and cur_state=="BOT" and not self.run_active:
            now = rospy.Time.now()
            if (now - self.last_trigger_time).to_sec() >= self.debounce_sec:
                self.last_trigger_time = now
                rospy.loginfo("RC ch%d edge MID→BOT (pwm=%d). Attempting to start mission run…", self.rc_channel, pwm)
                self.start_run()

        self.prev_rc_state = cur_state

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
        self.waiting_settle = False
        rospy.logwarn("Left GUIDED (now %s) — pausing run; waiting to re-enter GUIDED…", cur_mode)
        self.gcs("Paused: left GUIDED")

    def _resume_after_mode_pause(self):
        policy = self.resume_policy if self.resume_policy in ("repeat","skip") else "repeat"
        rospy.loginfo("GUIDED restored — resume policy: %s", policy)
        self.gcs("GUIDED restored - %s" % policy)
        self.mode_paused = False

        # If no goal yet, behave like first start but honor FSM readiness
        if self.idx == 0 and self.cur_wp is None:
            if (self.fsm_state or "") in ("INIT","WAIT_GOAL"):
                self._maybe_send_next()
            else:
                rospy.loginfo("Waiting for FSM to be INIT/WAIT_GOAL before sending first goal…")
            return

        # Continue current leg
        if policy == "repeat":
            self._resend_current_goal()
        else:
            self._maybe_send_next()

        # If already in WAIT_GOAL, kick settle gate
        if (self.fsm_state or "").upper() == "WAIT_GOAL" and not self.waiting_settle:
            self._begin_settle_gate()

    # ---------------- SUPER FSM ----------------
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

        # First goal: if planner idle/ready, send immediately (and only if GUIDED)
        if not self.started_send and state in ("INIT","WAIT_GOAL"):
            self.started_send = True
            if self._is_guided(self.mode):
                self._maybe_send_next()
            else:
                rospy.loginfo("Start armed but not in GUIDED (%s). Will send WP1 after GUIDED.", self.mode or "unknown")
        else:
            # Start settle gate after enroute and back to WAIT_GOAL
            if self.enroute_active:
                if state == "FOLLOW_TRAJ":
                    self.seen_follow_traj = True
                if state == "WAIT_GOAL" and (self.seen_follow_traj or self.mode_paused) and not self.waiting_settle:
                    self._begin_settle_gate()

        self.last_fsm = state

    # ---------------- Mission handling ----------------
    def _pull_mission(self, timeout_s=3.0):
        try:
            rospy.wait_for_service("/mavros/mission/pull", timeout=timeout_s)
            pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
            resp = pull()
            if resp.success:
                rospy.loginfo("Mission pull succeeded (wp_count=%d).", resp.wp_received)
            else:
                rospy.logwarn("Mission pull reported failure.")
        except Exception as e:
            rospy.logwarn("Mission pull failed: %s", e)

    def _ensure_mission_loaded(self):
        if self.raw_all_wps:
            return True
        self._pull_mission()
        t0 = rospy.Time.now()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 2.0:
            if self.raw_all_wps:
                return True
            r.sleep()
        if not self.raw_all_wps:
            rospy.logerr("No mission items available from Pixhawk.")
            return False
        return True

    def _prepare_mission_wps(self):
        """
        Build self.mission_wps (NAV only) and self.yaw_overrides (per NAV), always skipping the FIRST NAV.
        Any CONDITION_YAW encountered after a NAV (and before the next NAV) is attached to that NAV.
        """
        self.mission_wps   = []
        self.yaw_overrides = []
        current_nav_idx = None
        nav_seen = 0

        for it in self.raw_all_wps:
            cmd = it.command

            if cmd in (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT):
                nav_seen += 1
                if nav_seen == 1:
                    # skip first NAV waypoint unconditionally
                    current_nav_idx = None
                    continue
                # add NAV WP
                self.mission_wps.append(it)
                self.yaw_overrides.append(None)
                current_nav_idx = len(self.mission_wps) - 1
                continue

            if cmd == MAV_CMD_CONDITION_YAW and self.use_condition_yaw and current_nav_idx is not None:
                # attach/overwrite yaw override for the last NAV WP
                self.yaw_overrides[current_nav_idx] = dict(
                    heading_deg = float(it.param1),  # target or offset
                    rate_deg_s  = float(it.param2),  # informational
                    direction   = int(it.param3),    # -1 CCW, 0 shortest, 1 CW
                    relative    = int(it.param4) != 0
                )
                continue

        if not self.mission_wps:
            rospy.logerr("No usable NAV waypoints after skipping the first.")
        else:
            rospy.loginfo("Loaded %d NAV waypoints from Pixhawk (first NAV skipped).", len(self.mission_wps))

    # ---------------- Convert a mission WP to local goal ----------------
    def _wp_to_local_goal(self, wp: Waypoint, yaw_override=None):
        # Need current GPS + local pose
        if None in (self.cur_lat, self.cur_lon) or self.cur_pos is None or self.cur_yaw is None:
            return None

        # Reference: current GPS -> waypoint GPS (Earth frame)
        geod = Geodesic.WGS84.Inverse(self.cur_lat, self.cur_lon, wp.x_lat, wp.y_long)
        dist = geod['s12']                    # meters
        azi1_rad = math.radians(geod['azi1']) # 0 = North clockwise, in radians

        # ENU offsets (x=East, y=North)
        dx_e = dist * math.sin(azi1_rad)
        dy_n = dist * math.cos(azi1_rad)

        # Auto alignment ENU->local (handles ExternalNav yaw)
        if self.auto_align_enu and self.align_theta is not None:
            dx_e, dy_n = rot2d(dx_e, dy_n, self.align_theta)

        # Local target
        local_x = self.cur_pos[0] + dx_e
        local_y = self.cur_pos[1] + dy_n

        # Altitude selection (mission/current/fixed)
        if self.alt_mode == "mission":
            if wp.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT:
                z_local = wp.z_alt + self.z_offset_m
            elif wp.frame == MAV_FRAME_GLOBAL:
                if self.global_alt_strategy == "home_rel" and self.home_alt_amsl is not None:
                    z_local = (wp.z_alt - self.home_alt_amsl) + self.z_offset_m
                else:
                    z_local = self.cur_pos[2] + self.z_offset_m
            else:
                z_local = wp.z_alt + self.z_offset_m
        elif self.alt_mode == "fixed":
            z_local = self.fixed_alt_m
        else:  # "current"
            z_local = self.cur_pos[2]

        # ----- Yaw selection -----
        yaw_ros = None

        # Prefer CONDITION_YAW (if present and enabled)
        if yaw_override is not None and self.use_condition_yaw:
            hdg_deg = yaw_override["heading_deg"]
            direction = yaw_override["direction"]   # -1 CCW, 0 shortest, 1 CW
            relative = yaw_override["relative"]

            if relative:
                # relative offset from current yaw; direction sign if provided
                off = math.radians(hdg_deg)
                if direction == 1:       # CW in MAVLink -> negative in ROS (CCW+)
                    yaw_ros = ang_norm(self.cur_yaw - off)
                elif direction == -1:    # CCW in MAVLink -> positive in ROS
                    yaw_ros = ang_norm(self.cur_yaw + off)
                else:                    # "shortest" or unspecified -> add offset as-is
                    yaw_ros = ang_norm(self.cur_yaw + off)
            else:
                # absolute: MAVLink heading 0=N, CW+, convert to ROS yaw: 0=+X(East), CCW+
                yaw_ros = (math.pi/2.0) - math.radians(hdg_deg)
                # adjust by ENU->local rotation if active
                if self.auto_align_enu and self.align_theta is not None:
                    yaw_ros = ang_norm(yaw_ros + self.align_theta)

        # Fallbacks if no CONDITION_YAW or disabled
        if yaw_ros is None:
            if self.yaw_source == "wp":
                # Waypoint param4 is (in Copter) a yaw target in deg (0=N, CW+); if 0, vehicle may keep current.
                yaw_ros = (math.pi/2.0) - math.radians(wp.param4)
                if self.auto_align_enu and self.align_theta is not None:
                    yaw_ros = ang_norm(yaw_ros + self.align_theta)
            elif self.yaw_source == "path":
                yaw_ros = (math.pi/2.0) - azi1_rad
                if self.auto_align_enu and self.align_theta is not None:
                    yaw_ros = ang_norm(yaw_ros + self.align_theta)
            else:  # "current"
                yaw_ros = self.cur_yaw

        yaw_ros = ang_norm(yaw_ros)
        qx,qy,qz,qw = yaw_to_quat(yaw_ros)
        return (local_x, local_y, z_local, qx, qy, qz, qw, yaw_ros)

    # ---------------- Periodic (alignment + settle + live update) ----------------
    def _periodic_cb(self, _evt):
        # 0) Auto-align ENU->local using motion (if enabled)
        if self.auto_align_enu and self.cur_pos and self.cur_lat is not None and self.cur_lon is not None:
            if self.align_ref_gps is None:
                self.align_ref_gps = (self.cur_lat, self.cur_lon)
                self.align_ref_xy  = (self.cur_pos[0], self.cur_pos[1])
            else:
                ge = Geodesic.WGS84.Inverse(self.align_ref_gps[0], self.align_ref_gps[1],
                                            self.cur_lat, self.cur_lon)
                d = ge['s12']
                if d >= self.align_min_move_m:
                    dE = d * math.sin(math.radians(ge['azi1']))
                    dN = d * math.cos(math.radians(ge['azi1']))
                    dX = self.cur_pos[0] - self.align_ref_xy[0]
                    dY = self.cur_pos[1] - self.align_ref_xy[1]
                    if hypot2(dX, dY) >= 0.5 * self.align_min_move_m and hypot2(dE, dN) > 0.1:
                        theta_gps = math.atan2(dN, dE)  # angle vs +E
                        theta_loc = math.atan2(dY, dX)  # angle vs +x
                        self.align_theta = ang_norm(theta_loc - theta_gps)
                        rospy.loginfo("Auto-align ENU->local: θ=%.1f°", math.degrees(self.align_theta))
                    # reset reference to track slow variation
                    self.align_ref_gps = (self.cur_lat, self.cur_lon)
                    self.align_ref_xy  = (self.cur_pos[0], self.cur_pos[1])

        # 1) odom-based settle gate
        self._settle_check_cb_internal()

        # 2) live re-targeting while enroute (not waiting_settle)
        if not (self.run_active and self.enroute_active and not self.waiting_settle):
            return
        if None in (self.cur_pos, self.cur_lat, self.cur_lon) or self.cur_wp is None:
            return
        if self.last_update_xy is None:
            self.last_update_xy = (self.cur_pos[0], self.cur_pos[1])
            return

        moved = hypot2(self.cur_pos[0] - self.last_update_xy[0],
                       self.cur_pos[1] - self.last_update_xy[1])

        if moved >= self.update_dist_m:
            yaw_override = None
            if self.cur_wp_idx is not None and 0 <= self.cur_wp_idx < len(self.yaw_overrides):
                yaw_override = self.yaw_overrides[self.cur_wp_idx]
            g = self._wp_to_local_goal(self.cur_wp, yaw_override=yaw_override)
            if g is None: return
            x,y,z,qx,qy,qz,qw,yaw = g
            # update stored goal & publish without bumping idx
            self.goal_pos = (x,y,z); self.goal_yaw = yaw; self.goal_quat = (qx,qy,qz,qw)

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z
            msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz; msg.pose.orientation.w = qw
            self.goal_pub.publish(msg)

            rospy.loginfo("Updated goal %d/%d → (%.3f, %.3f, %.3f), yaw=%.1f°",
                          self.idx, len(self.mission_wps), x, y, z, math.degrees(yaw))
            self.gcs("Upd goal %d/%d -> (%.1f,%.1f,%.1f)" %
                     (self.idx, len(self.mission_wps), x, y, z))
            self.last_update_xy = (self.cur_pos[0], self.cur_pos[1])

    def _settle_check_cb_internal(self):
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

                    if self.idx >= len(self.mission_wps):
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

        if self.idx >= len(self.mission_wps):
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
        total = len(self.mission_wps)
        rospy.loginfo("Heading to waypoint %d/%d", next_one_based, total)
        self._maybe_send_next()

    # ---------------- Run control ----------------
    def start_run(self):
        if not self._ensure_mission_loaded():
            return
        self._prepare_mission_wps()
        if not self.mission_wps:
            rospy.logerr("No usable NAV waypoints after skipping the first.")
            return

        self.run_active = True
        self.idx = 0
        self.started_send = False
        self.last_fsm = None
        self.enroute_active = False
        self.seen_follow_traj = False
        self.waiting_settle = False
        self.cur_wp = None
        self.cur_wp_idx = None
        self.last_update_xy = None

        # Must be GUIDED to start sending
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
        rospy.logwarn("RC reset: returning to start. Waiting for MID→BOT to send WP1.")
        self.gcs("Run reset; waiting trigger")
        self._cancel_timers()
        self.run_active = False
        self.started_send = False
        self.idx = 0

        self.waiting_settle = False
        self.enroute_active = False
        self.seen_follow_traj = False
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None

        self.goal_pos = None
        self.goal_quat = None
        self.goal_yaw = None
        self.cur_wp = None
        self.cur_wp_idx = None
        self.last_update_xy = None

    def finish_run(self):
        rospy.loginfo("Waypoint run complete (%d waypoints). Waiting for next RC toggle.", len(self.mission_wps))
        self.run_active = False
        self.started_send = False
        self.idx = 0
        self._cancel_timers()
        self.waiting_settle = False
        self.enroute_active = False
        self.seen_follow_traj = False
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None
        self.cur_wp = None
        self.cur_wp_idx = None
        self.last_update_xy = None

    # ---------------- Goal sending ----------------
    def _maybe_send_next(self):
        if self.idx >= len(self.mission_wps):
            self.finish_run(); return

        self.cur_wp_idx = self.idx
        self.cur_wp = self.mission_wps[self.cur_wp_idx]
        self.idx += 1

        yaw_override = self.yaw_overrides[self.cur_wp_idx] if (0 <= self.cur_wp_idx < len(self.yaw_overrides)) else None
        g = self._wp_to_local_goal(self.cur_wp, yaw_override=yaw_override)
        if g is None:
            rospy.logwarn("Missing GPS/local pose to build goal; will retry...")
            self.idx -= 1  # roll back
            self.cur_wp = None
            self.cur_wp_idx = None
            return
        x,y,z,qx,qy,qz,qw,yaw = g

        # Auto-skip extremely near WPs to avoid stalls
        if self.cur_pos is not None:
            dxy = hypot2(x - self.cur_pos[0], y - self.cur_pos[1])
            dz  = abs(z - self.cur_pos[2])
            if dxy <= 0.5 and dz <= max(1.0, self.pos_tol_m):
                rospy.loginfo("Auto-skip near WP %d/%d (dxy=%.2fm, dz=%.2fm)", self.idx, len(self.mission_wps), dxy, dz)
                self.gcs("Skip WP %d/%d (near)" % (self.idx, len(self.mission_wps)))
                if self.idx >= len(self.mission_wps):
                    self.finish_run()
                else:
                    self._schedule_next_after_pause()
                return

        self._send_goal(x,y,z,qx,qy,qz,qw)

    def _send_goal(self, x, y, z, qx, qy, qz, qw):
        # store goal (for settle + re-send/updates)
        self.goal_pos = (x, y, z)
        self.goal_quat = (qx, qy, qz, qw)
        self.goal_yaw = quat_to_yaw(qx, qy, qz, qw)

        # mark enroute phase
        self.enroute_active = True
        self.waiting_settle = False
        self.last_update_xy = (self.cur_pos[0], self.cur_pos[1]) if self.cur_pos else None

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
                      self.idx, len(self.mission_wps), x, y, z, math.degrees(self.goal_yaw))
        self.gcs("Sent waypoint %d/%d" % (self.idx, len(self.mission_wps)))
        self.gcs("Sent goal %d/%d -> (%.3f, %.3f, %.3f), yaw=%.0f." %
                 (self.idx, len(self.mission_wps), x, y, z, math.degrees(self.goal_yaw)))

    def _resend_current_goal(self):
        """Re-publish the last goal without changing idx."""
        if self.cur_wp is None:
            rospy.logwarn("No current mission WP to resend; sending next.")
            self._maybe_send_next(); return

        yaw_override = self.yaw_overrides[self.cur_wp_idx] if (self.cur_wp_idx is not None and 0 <= self.cur_wp_idx < len(self.yaw_overrides)) else None
        g = self._wp_to_local_goal(self.cur_wp, yaw_override=yaw_override)
        if g is None and self.goal_pos and self.goal_quat:
            # fallback to last known local goal
            x,y,z = self.goal_pos
            qx,qy,qz,qw = self.goal_quat
            yaw = self.goal_yaw
        elif g is None:
            rospy.logwarn("Missing pose/GPS to rebuild current goal; will retry later.")
            return
        else:
            x,y,z,qx,qy,qz,qw,yaw = g

        self.goal_pos = (x,y,z); self.goal_quat = (qx,qy,qz,qw); self.goal_yaw = yaw
        self.enroute_active = True

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z
        msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz; msg.pose.orientation.w = qw

        self.goal_pub.publish(msg)
        rospy.loginfo("Resent goal %d/%d → (%.3f, %.3f, %.3f), yaw=%.1f°",
                      self.idx, len(self.mission_wps), x, y, z, math.degrees(yaw))
        self.gcs("Resent goal %d/%d -> (%.3f, %.3f, %.3f), yaw=%.0f." %
                 (self.idx, len(self.mission_wps), x, y, z, math.degrees(yaw)))

        if (self.fsm_state or "").upper() == "WAIT_GOAL" and not self.waiting_settle:
            self._begin_settle_gate()

    # ---------------- Settle gate begin ----------------
    def _begin_settle_gate(self):
        self.waiting_settle = True
        self.settle_start_time = rospy.Time.now()
        self.last_pose_ok_since = None
        self.last_yaw_ok_since  = None
        rospy.loginfo("WAIT_GOAL — checking pose & yaw before next goal…")
        self.gcs("WAIT_GOAL - odom check.")

# ---------------- main ----------------
def main():
    rospy.init_node("send_waypoint_mp", anonymous=False)
    looper = WaypointLooperMP()
    rospy.spin()

if __name__ == "__main__":
    main()
