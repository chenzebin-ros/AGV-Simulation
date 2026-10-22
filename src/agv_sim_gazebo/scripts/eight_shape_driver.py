#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray, UInt32

SHAPE_SIZE = 8.0      
NUM_POINTS = 30       # 保持适中的点数
DRIVE_SPEED = 0.3     # 速度稍微提到 0.3

def main():
    rospy.init_node('eight_shape_driver_node')
    pub_path = rospy.Publisher('/pathpoint', Float64MultiArray, queue_size=1)
    pub_start = rospy.Publisher('/car_run_star', UInt32, queue_size=1)
    
    data = [NUM_POINTS]
    for i in range(NUM_POINTS):
        # 核心修改：加上了 math.pi/2，让起点从 (0,0) 交叉点开始
        t = (i * (2 * math.pi) / NUM_POINTS) + math.pi / 2
        x = SHAPE_SIZE * math.cos(t)
        y = SHAPE_SIZE * math.sin(t) * math.cos(t)
        
        dx = -SHAPE_SIZE * math.sin(t)
        dy = SHAPE_SIZE * math.cos(2 * t)
        yaw = math.atan2(dy, dx)
        
        data.extend([x, y, yaw, 1, DRIVE_SPEED, 0, 0, 0, 0])
        
    msg = Float64MultiArray()
    msg.data = data
    rospy.sleep(1.0)
    pub_path.publish(msg)
    rospy.sleep(1.0)
    pub_start.publish(UInt32(1))
    rospy.loginfo("✅ 8字路径已发送！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
