#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float64MultiArray, UInt32
import sys

# =========================================================
# 🌟 参数配置区（你可以随时修改这里的数字）
# =========================================================
SHAPE_SIZE = 6.0      # 8 字的大小（幅度，单位：米）。越大弯越缓，越小弯越急。
NUM_POINTS = 100      # 路径点的数量。越多越平滑，但计算量稍大。
DRIVE_SPEED = 0.3     # 行驶速度（单位：米/秒）。调低可以防止差速轮打滑冲出赛道。
CAR_ID = 1            # 任务 ID
# =========================================================

def generate_eight_shape():
    """生成 8 字轨迹点 (利萨如曲线)"""
    points = []
    for i in range(NUM_POINTS):
        # 参数 t 从 0 到 2π
        t = i * (2 * math.pi) / NUM_POINTS
        
        # 8 字数学方程
        x = SHAPE_SIZE * math.cos(t)
        y = SHAPE_SIZE * math.sin(t) * math.cos(t)
        
        # 计算切线方向 (航向角 yaw)
        dx = -SHAPE_SIZE * math.sin(t)
        dy = SHAPE_SIZE * math.cos(2 * t)
        yaw = math.atan2(dy, dx)
        
        points.append((x, y, yaw))
        
    return points

def main():
    rospy.init_node('eight_shape_driver_node')
    
    # 创建发布者
    pub_path = rospy.Publisher('/pathpoint', Float64MultiArray, queue_size=1)
    pub_start = rospy.Publisher('/car_run_star', UInt32, queue_size=1)
    
    rospy.loginfo("正在生成 8 字轨迹...")
    rospy.loginfo(f"规模: {SHAPE_SIZE}m | 速度: {DRIVE_SPEED}m/s | 点数: {NUM_POINTS}")
    
    points = generate_eight_shape()
    
    # 组装 ROS 消息
    # 格式: [总数, x1, y1, yaw1, id1, speed1, f1, t1, do1, w1, x2, y2, ...]
    data = [NUM_POINTS]
    for idx, (x, y, yaw) in enumerate(points):
        data.extend([
            x, y, yaw,          # 坐标和航向
            CAR_ID,             # 任务ID (随便填)
            DRIVE_SPEED,        # 速度
            0, 0, 0, 0          # 特殊标志位、DO输出等（全填0）
        ])
        
    msg = Float64MultiArray()
    msg.data = data
    
    # 延迟 1 秒，确保所有节点准备就绪
    rospy.sleep(1.0)
    
    # 1. 发送路径
    rospy.loginfo("🚀 发送 8 字路径点...")
    pub_path.publish(msg)
    
    # 延迟 1 秒，确保路径被控制器解析
    rospy.sleep(1.0)
    
    # 2. 发送启动信号
    rospy.loginfo("✅ 发送启动指令！请观察 Gazebo 中的车辆。")
    pub_start.publish(UInt32(1))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
