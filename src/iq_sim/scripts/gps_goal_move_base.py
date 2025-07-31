#!/usr/bin/env python

import rospy
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointList
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped
# from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
import tf.transformations as transformations

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('gps_goal_move_base')

        # Initialize variables
        self.waypoints = []
        self.current_goal_index = 0
        self.goal_reached = False
        self.current_lat = None
        self.current_lon = None
        self.current_local_x = 0.0
        self.current_local_y = 0.0
        self.current_yaw = 0.0  # Initialize current yaw
        self.last_update_position = None  # Keep track of the last update position

        # Set up ROS publishers and subscribers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        # rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        rospy.Subscriber('/super_fsm_state', String, self.fsm_state_callback)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoints_callback)  # Subscribe to waypoints topic
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.current_gps_callback)  # Subscribe to current GPS position
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.current_local_position_callback)  # Current local odometry

        # Pull waypoints from Pixhawk
        self.pull_waypoints()

        # Start navigating through waypoints
        self.navigate_waypoints()
        # rospy.spin()

    def pull_waypoints(self):
        # Service call to pull waypoints from Pixhawk
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
        # Callback to receive waypoints
        self.waypoints = msg.waypoints
        rospy.loginfo(f"Received {len(self.waypoints)} waypoints.")

    def current_gps_callback(self, msg):
        # Callback to receive current GPS position
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def current_local_position_callback(self, msg):
        # Update current local position and yaw from odometry
        self.current_local_x = msg.pose.pose.position.x
        self.current_local_y = msg.pose.pose.position.y

        # Extract yaw from the orientation quaternion in odometry
        orientation_q = msg.pose.pose.orientation
        (_, _, self.current_yaw) = transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def result_callback(self, msg):
        if msg.status.status == 3:  # 3 means goal reached
            rospy.loginfo("Goal reached.")
            self.goal_reached = True
            self.last_update_position = None  # Reset last update position on reaching waypoint

    def navigate_waypoints(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.current_goal_index < len(self.waypoints):
                if self.goal_reached or self.current_goal_index == 0:
                    self.goal_reached = False
                    if self.current_lat is not None and self.current_lon is not None:
                        # Convert current waypoint to a PoseStamped goal for move_base
                        goal = self.convert_to_local_goal(self.waypoints[self.current_goal_index], initial_orientation=True)
                        rospy.loginfo(f"Navigating to waypoint {self.current_goal_index}: {goal}")
                        self.goal_pub.publish(goal)
                        self.current_goal_index += 1
                        self.last_update_position = (self.current_local_x, self.current_local_y)  # Set last update position
                    else:
                        rospy.logwarn("Waiting for current GPS position...")
                else:
                    self.update_goal_if_needed()  # Check if we need to update the goal during flight

            # Ensure recalculations for the last waypoint
            if self.current_goal_index == len(self.waypoints) and not self.goal_reached:
                self.update_goal_if_needed()  # Recalculate goal even for the last waypoint

            elif self.goal_reached and self.current_goal_index == len(self.waypoints):  # Last waypoint reached
                rospy.loginfo("All waypoints reached. Exiting.")
                break  # Exit the loop when all waypoints are reached
            rate.sleep()

    def convert_to_local_goal(self, waypoint, initial_orientation=False):
        # Convert GPS to local coordinates using current drone's GPS as reference
        geod = Geodesic.WGS84.Inverse(self.current_lat, self.current_lon, waypoint.x_lat, waypoint.y_long)
        distance = geod['s12']
        angle = math.radians(geod['azi1'])

        # Convert the angle to the local coordinate system
        relative_x = distance * math.sin(angle)  # East is positive x direction
        relative_y = distance * math.cos(angle)  # North is positive y direction

        # Set the local goal relative to the current odometry
        local_x = self.current_local_x + relative_x
        local_y = self.current_local_y + relative_y

        if initial_orientation:
            # Initially set orientation to east (yaw = 0)
            quaternion = transformations.quaternion_from_euler(0, 0, 0)
        else:
            # Update goal yaw orientation to be the same as the current yaw of the drone
            quaternion = transformations.quaternion_from_euler(0, 0, self.current_yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = local_x
        pose.pose.position.y = local_y
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def update_goal_if_needed(self):
        # Check if we need to update the goal based on distance traveled from last update position
        if self.last_update_position:
            dist_traveled = math.sqrt((self.current_local_x - self.last_update_position[0]) ** 2 +
                                      (self.current_local_y - self.last_update_position[1]) ** 2)
            if dist_traveled >= 0.5:  # Update if distance traveled since last update is more than 0.5m
                rospy.loginfo(f"Recalculating goal as distance traveled exceeded: {dist_traveled} meters")

                # Calculate the distance between the drone's current position and the waypoint
                waypoint = self.waypoints[self.current_goal_index - 1]
                geod = Geodesic.WGS84.Inverse(self.current_lat, self.current_lon, waypoint.x_lat, waypoint.y_long)
                goal_distance = geod['s12']
                
                if goal_distance < 1.0:
                    goal = self.convert_to_local_goal(self.waypoints[self.current_goal_index - 1], initial_orientation=False)
                    rospy.loginfo("Realigning yaw")
                else:
                    goal = self.convert_to_local_goal(self.waypoints[self.current_goal_index - 1], initial_orientation=True)

                self.goal_pub.publish(goal)
                self.last_update_position = (self.current_local_x, self.current_local_y)  # Update last update position

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
    except rospy.ROSInterruptException:
        pass