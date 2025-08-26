#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class PointCloudTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pointcloud_transformer')

        # Subscriber to the /velodyne_points (LiDAR) and /mavros/local_position/odom (odometry)
        # self.pc_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)
        self.pc_sub = rospy.Subscriber('/velodyne_points_filtered', PointCloud2, self.point_cloud_callback)
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)

        # Publisher for the transformed point cloud
        self.pc_pub = rospy.Publisher('/velodyne_points_transformed', PointCloud2, queue_size=10)

        # Initialize tf listener and broadcaster
        self.tf_listener = tf.TransformListener()

        # Store odometry data
        self.odom_data = None

    def odom_callback(self, msg):
        # Save the odometry data
        self.odom_data = msg

    def point_cloud_callback(self, msg):
        # If we don't have odometry data yet, just return
        if self.odom_data is None:
            return
        
        # Get the pose from the odometry message
        position = self.odom_data.pose.pose.position
        orientation = self.odom_data.pose.pose.orientation

        # Convert the LiDAR point cloud to a list of points
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Create a PointCloud2 message for the transformed cloud
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        # Create a list to store the transformed points
        transformed_points = []

        # Convert quaternion to rotation matrix
        rotation = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

        # Transform each point
        for point in pc_data:
            x, y, z = point

            # Apply the rotation and translation
            point_in_world = np.dot(rotation[:3, :3], [x, y, z]) + [position.x, position.y, position.z]

            # Add the transformed point to the list
            transformed_points.append([point_in_world[0], point_in_world[1], point_in_world[2]])

        # Create a new PointCloud2 message with the transformed points
        transformed_cloud = pc2.create_cloud_xyz32(header, transformed_points)

        # Publish the transformed point cloud
        self.pc_pub.publish(transformed_cloud)

if __name__ == '__main__':
    try:
        PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


