#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray, UInt32, Int16MultiArray
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class Loop8Shape:
    def __init__(self):
        rospy.init_node('loop_8_shape_node')
        
        # 发布者
        self.pub_path = rospy.Publisher('/pathpoint', Float64MultiArray, queue_size=1)
        self.pub_start = rospy.Publisher('/car_run_star', UInt32, queue_size=1)
        self.pub_target_path = rospy.Publisher('/target_path', Path, queue_size=1)
        self.pub_actual_path = rospy.Publisher('/actual_path', Path, queue_size=1)
        
        # 实际轨迹容器
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        # 控制变量
        self.current_task_id = 1
        self.is_waiting = False
        
        # 监听控制器状态
        rospy.Subscriber('/navigdata', Int16MultiArray, self.status_cb)
        
        rospy.sleep(1)
        print("🚀 循环 8 字测试启动！请确认 RViz 中已添加 /target_path 和 /actual_path")
        self.send_path_task()

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose = msg.pose.pose
        self.actual_path.poses.append(pose)
        if len(self.actual_path.poses) > 5000: 
            self.actual_path.poses.pop(0)
        self.pub_actual_path.publish(self.actual_path)

    def send_path_task(self):
        self.is_waiting = False
        num_points = 30
        data = [num_points]
        
        target_path = Path()
        target_path.header.frame_id = 'map'
        target_path.header.stamp = rospy.Time.now()

        for i in range(num_points):
            t = i * (2 * math.pi) / num_points
            x = 8.0 * math.cos(t)
            y = 8.0 * math.sin(t) * math.cos(t)
            
            dx = -8.0 * math.sin(t)
            dy = 8.0 * math.cos(2 * t)
            yaw = math.atan2(dy, dx)
            
            # 填充发路径的数据
            data.extend([x, y, yaw, self.current_task_id, 0.3, 0, 0, 0, 0])
            
            # 填充RViz绿线数据
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            target_path.poses.append(pose)
            
        # 发布绿线
        self.pub_target_path.publish(target_path)
        
        msg = Float64MultiArray()
        msg.data = data
        
        print(f"📤 正在下发第 {self.current_task_id} 圈任务...")
        self.pub_path.publish(msg)
        rospy.sleep(0.5)
        self.pub_start.publish(UInt32(1))
        
        self.current_task_id += 1

    def status_cb(self, msg):
        if len(msg.data) >= 3 and msg.data[2] == 1 and not self.is_waiting:
            self.is_waiting = True
            print("🏁 到达终点！2秒后重新下发任务...")
            rospy.Timer(rospy.Duration(2.0), self.restart_task, oneshot=True)

    def restart_task(self, event):
        self.send_path_task()

if __name__ == '__main__':
    try:
        Loop8Shape()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
