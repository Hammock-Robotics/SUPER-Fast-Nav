#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class LivoxBlockFilter:

    def __init__(self):
        rospy.init_node('livox_block_filter', anonymous=True)
        
        # Subscribe to your Livox custom pointcloud topic
        rospy.Subscriber(
            '/livox/lidar',                    # <— your actual topic name
            CustomMsg,                   # <— the custom msg type
            self.callback,
            queue_size=1
        )

        self.wedges = [
            (45.0, 2.0),
            (135.0, 2.0),
            (225.0, 2.0),
            (315.0, 2.0),
        ]

        self.pub = rospy.Publisher(
            '/livox/filtered_points',  # topic for RViz
            PointCloud2,
            queue_size=1
        )

        self.custom_pub = rospy.Publisher(
            '/livox/filtered_custom',
            CustomMsg,
            queue_size=1
        )

    def callback(self, msg):
        filtered_points = []

        for pt in msg.points:
            x, y = pt.x, pt.y
            angle = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

            blocked = False
            for center, half_width in self.wedges:
                delta = abs((angle - center + 180.0) % 360.0 - 180.0)
                if delta <= half_width:
                    blocked = True
                    break

            if not blocked:
                filtered_points.append(pt)

        rospy.loginfo(f"Original: {msg.point_num}, Filtered: {len(filtered_points)}")
        if len(filtered_points) > 0:
            p0 = filtered_points[0]
            rospy.loginfo(f" First kept point → x: {p0.x:.3f}, y: {p0.y:.3f}, z: {p0.z:.3f}, line: {p0.line}")

        # Convert to PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id  # or set manually if needed

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('tag', 16, PointField.UINT8, 1),
            PointField('line', 17, PointField.UINT8, 1),
        ]

        pc_data = [
            (p.x, p.y, p.z, float(p.reflectivity), p.tag, p.line)
            for p in filtered_points
        ]

        cloud_msg = pc2.create_cloud(header, fields, pc_data)
        self.pub.publish(cloud_msg)


        # Publish filtered CustomMsg
        custom_msg = CustomMsg()
        custom_msg.header = msg.header
        custom_msg.timebase = msg.timebase
        custom_msg.lidar_id = msg.lidar_id
        custom_msg.rsvd = msg.rsvd
        custom_msg.point_num = len(filtered_points)
        custom_msg.points = filtered_points  # filtered_points is already a list of CustomPoint

        self.custom_pub.publish(custom_msg)


if __name__ == '__main__':
    LivoxBlockFilter()
    rospy.spin()