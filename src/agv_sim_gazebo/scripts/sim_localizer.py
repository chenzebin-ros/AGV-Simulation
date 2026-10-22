#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

def odom_cb(msg):
    # 1. 发布 /ndt_pose
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose = msg.pose.pose
    pub_pose.publish(pose)

    # 2. 修复 TF 树：只发布 map -> odom，避免和 Gazebo 打架
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'odom'
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(t)

rospy.init_node('sim_localizer')
pub_pose = rospy.Publisher('/ndt_pose', PoseStamped, queue_size=10)
tf_broadcaster = tf2_ros.TransformBroadcaster()
rospy.Subscriber('/odom', Odometry, odom_cb)
rospy.spin()
