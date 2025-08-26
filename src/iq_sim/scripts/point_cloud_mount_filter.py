#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class VelodyneBlockFilter:
    def __init__(self):
        rospy.init_node('velodyne_block_filter', anonymous=True)

        # list of wedge centers (degrees) and half‐width:
        self.wedges = [
            ( 45.0, 2.0),
            (135.0, 2.0),
            (225.0, 2.0),
            (315.0, 2.0),
        ]

        rospy.Subscriber('/velodyne_points',
                         PointCloud2,
                         self.cb,
                         queue_size=1)
                         
        self.pub = rospy.Publisher('/velodyne_points_filtered',
                                   PointCloud2,
                                   queue_size=1)

    def cb(self, msg: PointCloud2):
        header = msg.header
        pts_out = []

        # read all fields, get full tuples
        for p in pc2.read_points(msg, skip_nans=True):  
            x, y, z = p[0], p[1], p[2]
            ang = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

            # drop if in any blocked wedge
            blocked = False
            for center, half_w in self.wedges:
                d = abs((ang - center + 180.0) % 360.0 - 180.0)
                if d <= half_w:
                    blocked = True
                    break

            if not blocked:
                pts_out.append(p)  # append the entire tuple, not just xyz

        # rebuild with the same fields
        filtered = pc2.create_cloud(header, msg.fields, pts_out)
        self.pub.publish(filtered)

if __name__ == '__main__':
    VelodyneBlockFilter()
    rospy.spin()


