/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Message counter in subscriber's queue.

 Yuki KITSUKAWA
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h> //点云消息，用于订阅。

static int enqueue = 0; //入队计数，记录 points_raw 收到了多少帧。
static int dequeue = 0; //出队计数，记录 ndt_map 处理了多少帧。  //两个计数器都是 static，全局生命周期。

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)    //订阅 points_raw（原始点云）。
{
  enqueue++;    //每收到一帧，enqueue++。 解析点云、不处理数据。
}

static void ndt_map_callback(const sensor_msgs::PointCloud2::ConstPtr& input) //订阅 ndt_map（NDT 建图输出的地图）
{
  dequeue++;    //每收到一帧，dequeue++。

  std::cout << "(Processed/Input): (" << dequeue << " / " << enqueue << ")" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_counter"); //节点名 queue_counter。

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");    //创建两个句柄，但 private_nh 未使用

    ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);   //订阅 points_raw，队列长度 100000（10 万）
    ros::Subscriber ndt_map_sub = nh.subscribe("ndt_map", 100000, ndt_map_callback);    //订阅 ndt_map，队列长度 100000。

    ros::spin();    //阻塞等待回调。

    return 0;
}
