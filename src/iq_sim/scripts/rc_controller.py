#!/usr/bin/env python
# v1 + preflight checks for takeoff altitude and guided mode  --sets GUIDED mode if not set
# V2 + added PAUSING mode instead of using EMER_STOP
# V3 + change receiving heading commands as well, and refactoring with geo/Transform Utils

import rospy
import math
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from transform_utils import TransformUtils
from pathlib import Path

WP_SOURCE = "file"       # flag to read from "mission" planner or local goal "file"
FILE_PATH = "/home/hammock/Desktop/waypoints.csv" # file path of the local goal file
PAUSE_DURATION = 2          # number of seconds to pause after reaching a waypoint

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('gps_goal_move_base')


        # ─── Initialize variables ────────────────────────────────────
        self.waypoints = []
        self.local_wps = []
        self.wp_headings = []
        self.current_goal_index = None
        self.current_mavros_mode = None
        self.prev_mavros_mode = None
        self.last_fsm_state = None
        self.last_sent_goal = None
        self.paused_goal       = None


        self.current_lat = None
        self.current_lon = None
        self.current_local_x = 0.0
        self.current_local_y = 0.0
        self.current_local_z = 0.0
        self.current_yaw = 0.0

        self.origin_lat     = 0.0
        self.origin_lon     = 0.0
        self.origin_local_x = 0.0
        self.origin_local_y = 0.0
        
        self.last_update_position = None
        self.got_odom = False

        self.mission_state = "IDLE"
        self.ready = False
        self.switch_state = "MID"
        self.prev_switch_state = self.switch_state

        # initialise the TransformUtils
        self.geo = TransformUtils(self.origin_lat, self.origin_lon, self.origin_local_x, self.origin_local_y)

        # ─── Publishers & subscribers ─────────────────────────────
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

        # get GPS + odom + waypoints early, but DO NOT yet subscribe to FSM
        rospy.Subscriber('/mavros/global_position/global',
                         NavSatFix, self.current_gps_callback)
        
        rospy.Subscriber('/mavros/local_position/odom',
                         Odometry, self.current_local_position_callback)
        
        rospy.Subscriber('/mavros/state',
                         State, self.mavros_state_callback)
        
        rospy.Subscriber('/rc/switch_state', 
                         String, self.switch_state_callback)
        
        # give ROS a moment to connect  
        rospy.sleep(0.5)
        # ─────────────────────────────────────────────────────────
        

        # WAITING FOR GUIDED MODE
        timeout   = rospy.Duration(10)  # give 10 seconds
        start     = rospy.Time.now()

        rospy.loginfo("Waiting for GUIDED mode (timeout %.1fs)…", timeout.to_sec())
        rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown() and self.current_mavros_mode == 'LOITER':
            # success!
            if self.current_mavros_mode == 'GUIDED':
                break

            # # timeout?
            # if rospy.Time.now() - start > timeout:
            #     rospy.logerr("Failed to turn on GUIDED mode within %.1fs, aborting mission.", timeout.to_sec())
            #     rospy.signal_shutdown("GUIDED MODE not activated")
            #     return

            rate.sleep()
    

        # pull mission from Pixhawk
        rospy.loginfo("Pilot turned on GUIDED mode, proceeding.")
        # after geo.update_origin(...)
        if WP_SOURCE == "mission":
            self.pull_waypoints()
            rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoints_callback)
            
        #else:  # "file"
        #    self.load_local_csv_waypoints(FILE_PATH)
        #    rospy.loginfo("Loaded %d file waypoints from %s", len(self.local_wps), FILE_PATH)
        #    print(f"waypoints: {self.local_wps}")

        # wait for GPS and odometry before proceeding
        rospy.loginfo("Waiting for first GPS message…")
        while not rospy.is_shutdown() and (
            self.current_lat is None or
            self.current_lon is None or
            not self.got_odom
        ):
            rospy.sleep(0.2)

        rospy.loginfo("GPS fix acquired: lat=%.6f lon=%.6f",
                      self.current_lat, self.current_lon)

        # now that we have a fix, set our map origin
        self.origin_lat     = self.current_lat
        self.origin_lon     = self.current_lon
        self.origin_local_x = self.current_local_x
        self.origin_local_y = self.current_local_y
        rospy.loginfo("Origin set → lat=%.6f lon=%.6f  x=%.2f y=%.2f",
                      self.origin_lat, self.origin_lon,
                      self.origin_local_x, self.origin_local_y)
        
        # now that origin is valid, create your Geo helper
        self.geo.update_origin(self.origin_lat,
                                self.origin_lon,
                                self.origin_local_x,
                                self.origin_local_y)

        self.ready = True
        

        # let callbacks drive waypoint publication
        # only now subscribe to FSM state changes

        rospy.Subscriber('/super_fsm_state',
                        String, self.fsm_state_callback)
        rospy.spin()

    def pull_waypoints(self):
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            pull_service = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
            response = pull_service()
            if response.success:
                rospy.loginfo("Waypoints successfully pulled from Pixhawk.")
            else:
                rospy.logerr("Failed to pull waypoints from Pixhawk.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def waypoints_callback(self, msg):
        # reset
        self.waypoints = []
        self.wp_headings = []

        # scan the raw list of mission items
        raw = msg.waypoints
        i = 0
        while i < len(raw):
            wp = raw[i]

            if wp.command == 16:  # NAV_WAYPOINT
                # pull the position goal
                self.waypoints.append(wp)

                # check if the next item is a matching CONDITION_YAW
                if i + 1 < len(raw) and raw[i+1].command == 115:
                    yaw_deg = raw[i+1].param1
                    i += 1  # skip the yaw command
                else:
                    yaw_deg = wp.param4  # fallback to param4 on the waypoint
                self.wp_headings.append(math.radians(yaw_deg))

            # otherwise ignore any 115 or other commands here
            i += 1

        # now drop the home if present (frame==0) and its heading
        if self.waypoints and self.waypoints[0].frame == 0:
            self.waypoints.pop(0)
            self.wp_headings.pop(0)

        # reset to the first real mission waypoint
        self.current_goal_index = 0

        if not self.waypoints:
            rospy.logwarn("No waypoints received!")
            return

        rospy.loginfo("Received %d waypoints.", len(self.waypoints))

        # Failsafe: if we're already in INIT or WAIT_GOAL, kick off immediately
        if self.last_fsm_state in ('INIT', 'WAIT_GOAL'):
            rospy.loginfo("Waypoints just arrived → publishing first goal")
            self.advance_to_next_waypoint()

    def current_gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def current_local_position_callback(self, msg):
        p = msg.pose.pose.position
        self.current_local_x = p.x
        self.current_local_y = p.y
        self.current_local_z = p.z 
        self.got_odom = True
        q = msg.pose.pose.orientation
        self.current_yaw = self.geo.quat_to_yaw(q)

    def fsm_state_callback(self, msg):
        state = msg.data.strip().upper()
        prev = self.last_fsm_state
        if state == prev:
            return
            
        rospy.loginfo(f"[FSM] State changed {prev} → {state}")
        self.last_fsm_state = state
    

        if state == 'INIT':
            # switch move from mid to bottom?
            if self.mission_state == 'RUNNING':
                self.advance_to_next_waypoint()
            else:
                rospy.loginfo("[FSM] INIT seen but mission_state=%s → waiting for RC start", self.mission_state)
            return
        
        # CHANGED PAUSING. continuing after pilot switches to loiter and then back to guided mode
        elif prev == 'PAUSING' and state == 'WAIT_GOAL':
            pass
        # if moving from either WAIT_GOAL or FOLLOW_TRAJ -> to WAIT_GOAL again
        elif prev != 'INIT' and state == 'WAIT_GOAL':
            hold_secs = 3.0   # ← how many seconds you want to wait
            rospy.loginfo(f"Holding for {hold_secs:.1f}s before next waypoint…")
            rospy.sleep(hold_secs)
            self.advance_to_next_waypoint()
        

    def advance_to_next_waypoint(self):
        if not hasattr(self, 'origin_lat'):
            rospy.logwarn("Origin not set yet; skipping waypoint publish")
            return

        if self.current_goal_index is None:
            self.current_goal_index = 0

        src   = 'mission' if WP_SOURCE == 'mission' else 'file'
        total = len(self.waypoints) if src == 'mission' else len(self.local_wps)
        rospy.loginfo(f"[ADV] src={src} idx={self.current_goal_index} total={total}")

        if self.current_goal_index >= total:
            #instead of rospy.signal_shutdown
            self.mission_state = "COMPLETE"
            rospy.loginfo("Mission complete — flip RC MID→RIGHT to restart (will reload waypoints).")
            return

        if src == 'mission':
            wp      = self.waypoints[self.current_goal_index]
            yaw_rad = self.wp_headings[self.current_goal_index]
            goal    = self.geo.make_pose(wp.x_lat, wp.y_long, wp.z_alt, yaw_rad)
        else:  # file
            rec     = self.local_wps[self.current_goal_index]   # {"pos":(x,y,z), "yaw":...}
            x,y,z   = rec["pos"]
            yaw_rad = rec["yaw"]
            goal    = PoseStamped()
            goal.header.frame_id = rospy.get_param("~frame_id", "world")
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
            qx, qy, qz, qw = self.geo.yaw_to_quat(yaw_rad)  # ensure this returns plain floats, not a Quaternion msg
            goal.pose.orientation.x = float(qx)
            goal.pose.orientation.y = float(qy)
            goal.pose.orientation.z = float(qz)
            goal.pose.orientation.w = float(qw)

        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        self.last_sent_goal = goal

        yaw_deg = (math.degrees(yaw_rad) + 360) % 360
        rospy.loginfo(f"→ Published wp #{self.current_goal_index} ({src}) -- "
                    f"x:{goal.pose.position.x:.2f}, y:{goal.pose.position.y:.2f}, "
                    f"z:{goal.pose.position.z:.2f}, yaw:{yaw_deg:.1f}°")

        self.current_goal_index += 1
        self.last_update_position = (self.current_local_x, self.current_local_y)

    
    def mavros_state_callback(self, msg):
        new_mode = msg.mode
        prev     = self.prev_mavros_mode
        self.prev_mavros_mode = new_mode
        self.current_mavros_mode = msg.mode

        # GUIDED into LOITER?
        if prev == "GUIDED" and new_mode == "LOITER":
            if self.current_goal_index:
                rospy.loginfo("Mode LOITER: pausing on goal #%d", self.current_goal_index-1)
                self.paused_goal = self.last_sent_goal

        #  LOITER back into GUIDED?
        if prev == "LOITER" and new_mode == "GUIDED" and self.paused_goal:
            # stamp it fresh and re‐publish
            self.paused_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.paused_goal)
            rospy.loginfo("Mode GUIDED: re-publishing paused goal")
            self.paused_goal = None


    def has_taken_off(self, hover_threshold=1):
        if self.current_local_z is None:
            return False
        return self.current_local_z >= hover_threshold
    

    def load_local_csv_waypoints(self, path):
        """Parse x,y,z,qx,qy,qz,qw per line → list of dicts."""
        wps = []
        with open(path, 'r') as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith('#'):
                    continue
                parts = [p.strip() for p in ln.split(',')]
                if len(parts) != 7:
                    rospy.logwarn(f"Skipping malformed line: {ln}")
                    continue
                x,y,z,qx,qy,qz,qw = map(float, parts)

                # (optional) normalize quaternion in case of float drift
                n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                if n > 0:
                    qx,qy,qz,qw = qx/n, qy/n, qz/n, qw/n

                # (optional) compute yaw if you want it around later
                yaw = self.geo.quat_xyzw_to_yaw(qx, qy, qz, qw)  # add this tiny util or reuse TransformUtils

                wps.append({
                    "pos": (x, y, z),
                    "quat": (qx, qy, qz, qw),
                    "yaw": yaw,  # handy if you later want Z-only orientation
                })
        if not wps:
            rospy.logwarn(f"No waypoints loaded from {path}")
        self.local_wps = wps
        self.current_goal_index = 0

    def switch_state_callback(self, msg):
        val = (msg.data or "").strip().upper()  
        if val not in {"TOP", "MID", "BOTTOM"}:
            rospy.logwarn("Unexpected switch state: %r", val)
            return
        if val != self.switch_state:
            self.switch_state = val
            rospy.loginfo("Switch state = %s", self.switch_state)

        prev = self.prev_switch_state
        self.prev_switch_state = self.switch_state

        if self.ready and prev == "MID" and self.switch_state == "BOTTOM":
            if self.mission_state in ("IDLE", "COMPLETE"):

                if WP_SOURCE == "file":
                    # 1) File must exist
                    if not Path(FILE_PATH).is_file():
                        rospy.logwarn("[RC] Start requested, but CSV not found: %s — not starting.", FILE_PATH)
                        return

                    # 2) Load and require at least 1 valid row (or use self.min_csv_rows if you have it)
                    self.load_local_csv_waypoints(FILE_PATH)
                    if len(self.local_wps) < 1:   # or: < self.min_csv_rows
                        rospy.logwarn("[RC] CSV has %d valid waypoint(s); need at least 1 — not starting.",
                                    len(self.local_wps))
                        return

                else:  # "mission"
                    self.pull_waypoints()
                    if not self.waypoints:
                        rospy.logwarn("[RC] No mission waypoints on Pixhawk — not starting.")
                        return

                # Only reaches here when checks pass
                self.current_goal_index = 0
                self.mission_state = "RUNNING"
                rospy.loginfo("[RC] Start/Restart requested → publishing first waypoint")
                self.advance_to_next_waypoint()



    # unused -- but keep here 
    def update_goal_if_needed(self):
        # Optional: re-send the same waypoint if you drift too far
        if self.last_update_position is None:
            return

        dx = self.current_local_x - self.last_update_position[0]
        dy = self.current_local_y - self.last_update_position[1]
        if math.hypot(dx, dy) < 0.5:
            return

        rospy.loginfo(f"Drifted {math.hypot(dx, dy):.2f}m—re-publishing current waypoint")
        idx = self.current_goal_index - 1
        if idx < 0 or idx >= len(self.waypoints):
            return

        wp = self.waypoints[idx]
        # if <1m to wp, keep yaw; else reset to heading=0
        g = self.geo._geod.Inverse(
            self.current_lat, self.current_lon,
            wp.x_lat, wp.y_long
        )
        initial = (g['s12'] >= 1.0)
        goal = self.geo.make_pose(wp)
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        self.last_update_position = (self.current_local_x, self.current_local_y)

if __name__ == '__main__':
    try:
        WaypointNavigator()
    except rospy.ROSInterruptException:
        pass
