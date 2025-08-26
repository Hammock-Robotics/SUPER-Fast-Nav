#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class FastlioToVisionPoseCov:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fastlio_to_vision_pose_cov', anonymous=True)

        # Subscribe to the FAST-LIO Odometry topic
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback, queue_size=1)

        # Publisher for MAVROS vision pose with covariance
        self.pose_cov_pub = rospy.Publisher(
            '/mavros/vision_pose/pose_cov',
            PoseWithCovarianceStamped,
            queue_size=1
        )

    def odom_callback(self, odom_msg: Odometry):
        # Create a PoseWithCovarianceStamped message
        pc = PoseWithCovarianceStamped()
        pc.header.stamp    = odom_msg.header.stamp
        pc.header.frame_id = "world"             # or the frame MAVROS expects
        pc.pose.pose       = odom_msg.pose.pose
        # Copy the full 6×6 covariance matrix
        pc.pose.covariance = odom_msg.pose.covariance

        # Publish only the covariant vision pose
        self.pose_cov_pub.publish(pc)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    FastlioToVisionPoseCov().run()



# Step 3: Configure MAVROS and ArduPilot for Vision Position Estimation
# Set Up MAVROS:

# Make sure mavros is properly installed and configured on your system. You need to connect MAVROS to the Pixhawk via a serial link or telemetry radio.
# Adjust ArduPilot Parameters:

# To enable vision-based position hold, certain parameters in ArduPilot need to be configured correctly. You can change these parameters using a Ground Control Station (GCS) like Mission Planner, QGroundControl, or MAVProxy:
# ArduPilot Parameters to Set:

# EK3_SRC1_POSXY = 6 (Vision) # Sets the primary source for XY position to vision.
# EK3_SRC1_VELXY = 6 (Vision) # Sets the primary source for XY velocity to vision.
# EK3_SRC1_POSZ = 6 (Barometer vision) # Typically, the Z-position is derived from the barometer or rangefinder, but if you have a reliable Z estimate from vision, you could use that too.
# EK3_SRC1_VELZ = 6 (Vision) # Sets the primary source for Z velocity to vision.
# EK3_ENABLE = 1 # Enables the EKF3 filter (required for vision-based estimation).
# AHRS_EKF_TYPE = 3 # Ensures that EKF3 is the active estimator.
# EKF_ENABLE = 1 # Enables the EKF.
# GPS_TYPE = 0 # Optional: Set GPS_TYPE to 0 if you want to disable GPS and rely solely on vision.
# VISO_TYPE = 1 # Enables vision position input.

# SR1_POSITION = 10  # Position update rate 10Hz
# SR1_EXTRA1 = 10    # Additional data update rate 10Hz
# SR1_EXTRA2 = 10    # Additional data update rate 10Hz

# Save Parameters and Reboot:


