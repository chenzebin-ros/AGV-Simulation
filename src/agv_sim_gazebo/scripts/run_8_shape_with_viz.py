#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray, UInt32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('run_8_shape_with_viz')

# 发布者
pub_path = rospy.Publisher('/pathpoint', Float64MultiArray, queue_size=1)
pub_start = rospy.Publisher('/car_run_star', UInt32, queue_size=1)
pub_target_path = rospy.Publisher('/target_path', Path, queue_size=1)
pub_actual_path = rospy.Publisher('/actual_path', Path, queue_size=1)

# 1. 绘制绿线 (目标 8 字)
def publish_target_path():
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()
    
    num_points = 30
    for i in range(num_points):
        t = i * (2 * math.pi) / num_points
        x = 8.0 * math.cos(t)
        y = 8.0 * math.sin(t) * math.cos(t)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    pub_target_path.publish(path)

# 2. 记录红线 (实际轨迹)
actual_path = Path()
actual_path.header.frame_id = 'map'
def odom_cb(msg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose = msg.pose.pose
    actual_path.poses.append(pose)
    pub_actual_path.publish(actual_path)
rospy.Subscriber('/odom', Odometry, odom_cb)

# 3. 发送 8 字路径任务
def send_task():
    num_points = 30
    data = [num_points]
    for i in range(num_points):
        t = i * (2 * math.pi) / num_points
        x = 8.0 * math.cos(t)
        y = 8.0 * math.sin(t) * math.cos(t)
        dx = -8.0 * math.sin(t)
        dy = 8.0 * math.cos(2 * t)
        yaw = math.atan2(dy, dx)
        
        data.extend([x, y, yaw, 1, 0.3, 0, 0, 0, 0])
        
    msg = Float64MultiArray()
    msg.data = data
    rospy.sleep(1)
    pub_path.publish(msg)
    rospy.sleep(1)
    pub_start.publish(UInt32(1))
    print("✅ 绿线已绘制，8字路径任务已发送！")

if __name__ == '__main__':
    rospy.sleep(1)
    publish_target_path()
    send_task()
    rospy.spin()
