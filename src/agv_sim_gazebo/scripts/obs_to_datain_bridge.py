#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8, UInt16

pub = rospy.Publisher('/DataIN', UInt16, queue_size=10)
last_obs_time = rospy.Time(0)
current_obs_level = 0

def obs_callback(msg):
    global last_obs_time, current_obs_level
    # 只要收到 >0 的信号，立刻刷新时间戳，锁定危险状态
    if msg.data > 0:
        last_obs_time = rospy.Time.now()
        current_obs_level = msg.data

def timer_cb(event):
    data_in = 0
    now = rospy.Time.now()
    
    # 只要在过去 1.5 秒内检测到过障碍物，就死死按住刹车不放
    if (now - last_obs_time).to_sec() < 1.5:
        if current_obs_level == 1:
            data_in = data_in | (1 << 10) # DI11 急停
        elif current_obs_level == 2:
            data_in = data_in | (1 << 9)  # DI10 减速
            
    # 无论如何都保持 DI3 为 1，解除控制器的死锁
    data_in = data_in | (1 << 2) 
    
    pub.publish(data_in)

if __name__ == '__main__':
    rospy.init_node('obs_to_datain_bridge')
    rospy.Subscriber('/obs_data', UInt8, obs_callback)
    rospy.Timer(rospy.Duration(0.05), timer_cb)
    rospy.spin()
