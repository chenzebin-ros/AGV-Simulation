#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

rospy.init_node('fake_map_pub')
pub = rospy.Publisher('/points_map', PointCloud2, queue_size=1, latch=True)
header = std_msgs.msg.Header()
header.frame_id = 'map'
# 生成 50x40 米的平地地图
points = [(i, j, 0.0) for i in range(-20, 30) for j in range(-20, 20)]
pcl_msg = pcl2.create_cloud_xyz32(header, points)
rospy.sleep(2)
pub.publish(pcl_msg)
print('✅ 假地图已发布！')
rospy.spin()
