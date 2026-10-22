#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class NavToGazebo:
    def __init__(self):
        rospy.Subscriber("/navigdata", Int16MultiArray, self.callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # 必须与 URDF 轴距一致
        self.wheelbase = 1.0  

    def callback(self, msg):
        if len(msg.data) < 2:
            return
        angle_raw = msg.data[0] 
        speed_raw = msg.data[1] 
        
        steer_angle_rad = math.radians(angle_raw / 100.0) 
        speed_mps = speed_raw / 100.0
        
        twist = Twist()
        twist.linear.x = speed_mps
        # 阿克曼转换为差速轮角速度
        twist.angular.z = speed_mps * math.tan(steer_angle_rad) / self.wheelbase
        
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('nav_to_gazebo_bridge')
    bridge = NavToGazebo()
    rospy.spin()
