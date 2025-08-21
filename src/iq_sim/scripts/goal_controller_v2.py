#!/usr/bin/env python

import rospy
import math
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointList, State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from transform_utils import TransformUtils

PAUSE_DURATION = 2  # number of seconds to pause after reaching a waypoint

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('gps_goal_move_base')

        # ─── Initialize variables ────────────────────────────────────
        self.waypoints = []
        self.wp_headings = []
        self.current_goal_index = 0
        self.current_mavros_mode = None
        self.prev_mavros_mode = None
        self.last_fsm_state = None
        self.last_sent_goal = None
        self.paused_goal = None

        self.current_lat = None
        self.current_lon = None
        self.current_local_x = 0.0
        self.current_local_y = 0.0
        self.current_local_z = 0.0
        self.current_yaw = 0.0

        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_local_x = 0.0
        self.origin_local_y = 0.0

        self.last_update_position = None
        self.got_odom = False

        # Initialise the TransformUtils
        self.geo = TransformUtils(self.origin_lat, self.origin_lon, self.origin_local_x, self.origin_local_y)

        # ─── Publishers & subscribers ─────────────────────────────
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

        # Get GPS + odom + waypoints early, but DO NOT yet subscribe to FSM
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.current_gps_callback)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.current_local_position_callback)
        rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)

        # give ROS a moment to connect
        rospy.sleep(0.5)

        # ─────────────────────────────────────────────────────────

        # WAITING FOR GUIDED MODE
        timeout = rospy.Duration(10)  # give 10 seconds
        start = rospy.Time.now()

        rospy.loginfo("Waiting for GUIDED mode (timeout %.1fs)…", timeout.to_sec())
        rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown() and self.current_mavros_mode == 'LOITER':
            if self.current_mavros_mode == 'GUIDED':
                break
            if rospy.Time.now() - start > timeout:
                rospy.logerr("Failed to turn on GUIDED mode within %.1fs, aborting mission.", timeout.to_sec())
                rospy.signal_shutdown("GUIDED MODE not activated")
                return
            rate.sleep()

        # pull mission from Pixhawk
        rospy.loginfo("Pilot turned on GUIDED mode, proceeding.")
        self.pull_waypoints()

        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoints_callback)

        # wait for GPS and odometry before proceeding
        rospy.loginfo("Waiting for first GPS message…")
        while not rospy.is_shutdown() and (
            self.current_lat is None or
            self.current_lon is None or
            not self.got_odom
        ):
            rospy.sleep(0.2)

        rospy.loginfo("GPS fix acquired: lat=%.6f lon=%.6f", self.current_lat, self.current_lon)

        # now that we have a fix, set our map origin
        self.origin_lat = self.current_lat
        self.origin_lon = self.current_lon
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

        # let callbacks drive waypoint publication
        # only now subscribe to FSM state changes
        rospy.Subscriber('/super_fsm_state', String, self.fsm_state_callback)

        # Keep the node running and wait for FSM state changes
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
            self.advance_to_next_waypoint()

        elif prev == 'PAUSING' and state == 'WAIT_GOAL':
            pass

        elif prev != 'INIT' and state == 'WAIT_GOAL':
            hold_secs = 3.0  # how many seconds you want to wait
            rospy.loginfo(f"Holding for {hold_secs:.1f}s before next waypoint…")
            rospy.sleep(hold_secs)
            self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        # guard against missing origin
        if not hasattr(self, 'origin_lat'):
            rospy.logwarn("Origin not set yet; skipping waypoint publish")
            return

        if self.current_goal_index >= len(self.waypoints):
            rospy.loginfo("All waypoints done. Pulling new waypoints.")
            self.current_goal_index = 0  # Reset index to the first waypoint
            self.pull_waypoints()  # Pull new waypoints
            return

        wp = self.waypoints[self.current_goal_index]
        lat, lon, alt = wp.x_lat, wp.y_long, wp.z_alt
        yaw_rad = self.wp_headings[self.current_goal_index]

        goal = self.geo.make_pose(lat, lon, alt, yaw_rad)
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        self.last_sent_goal = goal

        yaw_deg = (math.degrees(yaw_rad) + 360) % 360

        rospy.loginfo(f"→ Published waypoint #{self.current_goal_index}"
                      f" -- x: {goal.pose.position.x:.2f},"
                      f" y: {goal.pose.position.y:.2f},"
                      f" z: {goal.pose.position.z:.2f},"
                      f" yaw: {yaw_deg:.1f}°")

        self.current_goal_index += 1
        self.last_update_position = (self.current_local_x, self.current_local_y)

    def mavros_state_callback(self, msg):
        new_mode = msg.mode
        prev = self.prev_mavros_mode
        self.prev_mavros_mode = new_mode
        self.current_mavros_mode = msg.mode

        if prev == "GUIDED" and new_mode == "LOITER":
            rospy.loginfo("Mode LOITER: pausing on goal #%d", self.current_goal_index - 1)
            self.paused_goal = self.last_sent_goal

        if prev == "LOITER" and new_mode == "GUIDED" and self.paused_goal:
            self.paused_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.paused_goal)
            rospy.loginfo("Mode GUIDED: re-publishing paused goal")
            self.paused_goal = None

    def has_taken_off(self, hover_threshold=1):
        if self.current_local_z is None:
            return False
        return self.current_local_z >= hover_threshold

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
        g = self.geo._geod.Inverse(self.current_lat, self.current_lon, wp.x_lat, wp.y_long)
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
