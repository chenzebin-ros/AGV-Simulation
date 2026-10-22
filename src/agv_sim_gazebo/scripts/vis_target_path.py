#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('vis_target_path')
pub = rospy.Publisher('/target_path', Path, queue_size=1)

def cb(msg):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()
    data = msg.data
    num_points = int(data[0])
    for i in range(num_points):
        idx = 1 + i * 9
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = data[idx]
        pose.pose.position.y = data[idx+1]
        path.poses.append(pose)
    pub.publish(path)

rospy.Subscriber('/pathpoint', Float64MultiArray, cb)
rospy.spin()
