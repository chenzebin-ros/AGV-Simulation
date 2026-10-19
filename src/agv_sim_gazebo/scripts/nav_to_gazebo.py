#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class NavToGazebo:
    def __init__(self):
        rospy.Subscriber("/navigdata", Int16MultiArray, self.callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.wheelbase = 0.9  # 与 URDF 中的轴距保持一致

    def callback(self, msg):
        if len(msg.data) < 2:
            return
        # 你的代码中，data[0]是转角*100，data[1]是速度*100
        angle_raw = msg.data[0] 
        speed_raw = msg.data[1] 
        
        steer_angle_rad = math.radians(angle_raw / 100.0) 
        speed_mps = speed_raw / 100.0
        
        twist = Twist()
        twist.linear.x = speed_mps
        # 运动学转换：角速度 = 速度 * tan(转角) / 轴距
        twist.angular.z = speed_mps * math.tan(steer_angle_rad) / self.wheelbase
        
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('nav_to_gazebo_bridge')
    bridge = NavToGazebo()
    rospy.spin()
