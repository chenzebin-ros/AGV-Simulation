#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('vis_actual_path')
pub = rospy.Publisher('/actual_path', Path, queue_size=10)
path = Path()
path.header.frame_id = 'map'

def cb(msg):
    path.header.stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose = msg.pose.pose
    path.poses.append(pose)
    if len(path.poses) > 2000: path.poses.pop(0)
    pub.publish(path)

rospy.Subscriber('/odom', Odometry, cb)
rospy.spin()
