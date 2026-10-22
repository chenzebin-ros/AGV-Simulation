#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

rospy.init_node('dual_lidar_fusion')
pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=10)

def process_scan(scan):
    points = []
    for i, r in enumerate(scan.ranges):
        # 没扫到东西，强行设为 10 米，保证点云不断流
        if math.isinf(r) or math.isnan(r) or r < 0.1:
            r = 10.0
        elif r < 0.1:
            r = 0.1
        angle = scan.angle_min + i * scan.angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        # 纯原始坐标，不旋转不平移
        points.append((x, y, 0.0, 100.0))
    return points

def publish_points(points):
    if not points:
        return
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)
    ]
    msg = pc2.create_cloud(header, fields, points)
    pub.publish(msg)

def cb_left(scan):
    publish_points(process_scan(scan))

def cb_right(scan):
    publish_points(process_scan(scan))

rospy.Subscriber('/scan_left', LaserScan, cb_left)
rospy.Subscriber('/scan_right', LaserScan, cb_right)
rospy.spin()
