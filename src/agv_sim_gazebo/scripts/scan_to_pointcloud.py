#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

rospy.init_node('scan_to_pc_converter')
pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=10)

def cb(scan):
    points = []
    for i in range(360):
        angle = i * math.pi / 180.0
        r = 5.0  # 强制制造一圈 5 米远的点，确保 NDT 有东西可用
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y, 0.0, 0.0))

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'lidar_link'
    
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)
    ]
    pc2_msg = pc2.create_cloud(header, fields, points)
    pub.publish(pc2_msg)

rospy.Subscriber('/scan', LaserScan, cb)
rospy.spin()
