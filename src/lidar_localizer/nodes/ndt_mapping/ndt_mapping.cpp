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
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".//// 是否输出 position_log.txt

#include <fstream>      //（文件）
#include <iostream>
#include <sstream>
#include <string>       //标准库：

#include <nav_msgs/Odometry.h>  //ROS 消息：Odometry、Imu、PointCloud2、Bool、Float32。
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
//#include <velodyne_pointcloud/point_types.h>
//#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>   //坐标变换
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>          //点云库。
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>   //NDT
#include <pcl/registration/ndt.h>

#include <gtsam/geometry/Rot3.h>        //GTSAM：新增！这是一个因子图优化库，用于回环检测和全局优化
#include <gtsam/geometry/Point3.h>      //这里用到了 GTSAM，说明这个建图节点不仅能建图，还能做位姿图优化（Pose Graph Optimization）
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <autoware_config_msgs/ConfigNDTMapping.h>      //Autoware 消息：ConfigNDTMapping（建图参数）、ConfigNDTMappingOutput（输出参数）
#include <autoware_config_msgs/ConfigNDTMappingOutput.h>

#include <time.h>

struct pose     //6 自由度位姿，与 ndt_matching.cpp 一致。
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

enum class MethodType   //4 种 NDT 实现方式。
{
  PCL_GENERIC = 0,
  PCL_ANH = 1,
  PCL_ANH_GPU = 2,
  PCL_OPENMP = 3,
};
static MethodType _method_type = MethodType::PCL_GENERIC;

// global variables 位姿变量
static pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;
/*
previous_pose	上一帧位姿
guess_pose	预测位姿
guess_pose_imu/odom/imu_odom	用 IMU/Odom 预测的位姿
current_pose	当前位姿
ndt_pose	NDT 匹配结果
added_pose	上一次添加点云到地图时的位姿
localizer_pose	雷达坐标系下的位姿
*/
static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;      //传感器偏移量,每个传感器的累积偏移量，用于预测。
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
    offset_imu_odom_yaw;

static double current_velocity_x = 0.0;     //线速度（世界坐标系）
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;
//地图与 NDT
static pcl::PointCloud<pcl::PointXYZI> map;     //map：累积的点云地图，会随着车辆移动不断增长
static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;   //PointXYZI：带强度信息的点云（比 PointXYZ 多了 intensity）。
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;
#ifdef CUDA_FOUND
static gpu::GNormalDistributionsTransform anh_gpu_ndt;
#endif
#ifdef USE_PCL_OPENMP
static pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
#endif

// Default values
static int max_iter = 30;        // Maximum iterations // 最大迭代次数
static float ndt_res = 1.0;      // Resolution          // 分辨率
static double step_size = 0.1;   // Step size           // 步长
static double trans_eps = 0.01;  // Transformation epsilon  // 变换收敛阈值
    
// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;    // 体素滤波叶子大小     //每 2 米一个体素，这是建图的关键参数，决定了地图的稠密程度。

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;     //过滤掉 5 米内的点（可能是车体自身或地面反射）
static double max_scan_range = 200.0;   //// 最大扫描距离
static double min_add_scan_shift = 1.0;     // 最小添加点云的位移  车辆移动 1 米后，才把新点云加入地图（防止地图过密）

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;

static bool _incremental_voxel_update = false; //是否增量更新体素（用于 ANH 版 NDT）

static std::string _imu_topic = "/imu_raw";

static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static std::ofstream ofs;
static std::string filename;

static void param_callback(const autoware_config_msgs::ConfigNDTMapping::ConstPtr& input)   //订阅 config/ndt_mapping，动态调整建图参数
{
  ndt_res = input->resolution;
  step_size = input->step_size;
  trans_eps = input->trans_epsilon;
  max_iter = input->max_iterations;
  voxel_leaf_size = input->leaf_size;
  min_scan_range = input->min_scan_range;
  max_scan_range = input->max_scan_range;
  min_add_scan_shift = input->min_add_scan_shift;

  std::cout << "param_callback" << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "max_scan_range: " << max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
}

static void output_callback(const autoware_config_msgs::ConfigNDTMappingOutput::ConstPtr& input)    //保存地图  订阅 config/ndt_mapping_output，触发地图保存
{
  double filter_res = input->filter_res;
  std::string filename = input->filename;
  std::cout << "output_callback" << std::endl;
  std::cout << "filter_res: " << filter_res << std::endl;
  std::cout << "filename: " << filename << std::endl;
  //// 1. 把地图复制一份
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  map_filtered->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

  // Apply voxelgrid filter// 2. 可选：体素滤波降采样
  if (filter_res == 0.0)  //filter_res：滤波分辨率。如果为 0，保存原始地图；否则降采样。
  {// 不滤波
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_filtered, *map_msg_ptr);
  }
  // 3. 发布地图到 /ndt_map
  ndt_map_pub.publish(*map_msg_ptr);        //同时发布到 / ndt_map 话题。

  // Writing Point Cloud data to PCD file// 4. 保存为 .pcd 文件 保存为 ASCII 格式的 .pcd 文件——方便人工查看。
  if (filter_res == 0.0)
  {
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  }
  else
  {
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
  }
}

static void imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();
  // IMU 角速度积分得到姿态
  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;
  // Odom 线速度积分得到位移
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;
  // 组合成预测位姿
  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

static void odom_calc(ros::Time current_time)       //只用 Odom。
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();
  // IMU 角速度积分得到姿态
  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;
  // Odom 线速度积分得到位移
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;
  // 组合成预测位姿
  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

static void imu_calc(ros::Time current_time)    //只用 IMU（含加速度二次积分）
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}
//角度辅助函数  角度归一化，处理 2π 环绕
static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}
static void odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

static void imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // std::cout << __func__ << std::endl;

  if (_imu_upside_down)
    imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0)
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}
//points_callback —— 核心建图逻辑   这是整个节点的核心，每次收到点云就执行一次。
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;//获取点云头  提取 current_scan_time（当前帧时间戳）

  pcl::fromROSMsg(*input, tmp);
  //逐点过滤：只保留距离在 [5.0, 200.0] 米之间的点
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);    //结果存入 scan
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
/*
这段代码是使用PCL库中的transformPointCloud函数将一个点云scan_ptr通过tf_btol变换后得到变换后的点云transformed_scan_ptr。具体来说，
transformPointCloud函数的作用是将输入的点云通过给定的变换矩阵tf_btol进行坐标变换，将点云从一个坐标系变换到另一个坐标系。这个函数的参
数包括输入点云、输出点云和变换矩阵。在这段代码中，scan_ptr是输入点云，transformed_scan_ptr是输出点云，tf_btol是变换矩阵。通过调用
transformPointCloud函数，将输入点云scan_ptr根据变换矩阵tf_btol进行坐标变换，得到变换后的点云transformed_scan_ptr。
*/
  if (initial_scan_loaded == 0) // 初始点云加载  如果是第一帧点云：
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);    //用 tf_btol（雷达到车体变换）转换点云
    map += *transformed_scan_ptr;   //加入地图
    initial_scan_loaded = 1;        //标记 initial_scan_loaded = 1
  } 

  // Apply voxelgrid filter  体素滤波
  /*
  * 体素滤波降采样：每 voxel_leaf_size × voxel_leaf_size × voxel_leaf_size（默认 2m³）保留一个点。
  * 作用：减少点数，加快 NDT 匹配
  * voxel_leaf_size = 2.0 意味着地图会比较大尺度，适合室外建图
  */
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;// 创建一个VoxelGrid滤波器对象
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);// 设置滤波器的叶子大小，即每个体素的尺寸
  voxel_grid_filter.setInputCloud(scan_ptr);// 设置输入点云数据
  voxel_grid_filter.filter(*filtered_scan_ptr);// 对输入点云进行滤波，将滤波结果保存在filtered_scan_ptr中

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  if (_method_type == MethodType::PCL_GENERIC)  //设置 NDT 参数 根据 _method_type 设置 NDT 对象
  {
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(filtered_scan_ptr);
  }
  else if (_method_type == MethodType::PCL_ANH)
  {
    anh_ndt.setTransformationEpsilon(trans_eps);// 设置变换容差
    anh_ndt.setStepSize(step_size);// 设置步长
    anh_ndt.setResolution(ndt_res);// 设置分辨率
    anh_ndt.setMaximumIterations(max_iter);// 设置最大迭代次数
    anh_ndt.setInputSource(filtered_scan_ptr);// 设置输入源
  }
#ifdef CUDA_FOUND
  else if (_method_type == MethodType::PCL_ANH_GPU)
  {
    anh_gpu_ndt.setTransformationEpsilon(trans_eps);
    anh_gpu_ndt.setStepSize(step_size);
    anh_gpu_ndt.setResolution(ndt_res);
    anh_gpu_ndt.setMaximumIterations(max_iter);
    anh_gpu_ndt.setInputSource(filtered_scan_ptr);
  }
#endif
#ifdef USE_PCL_OPENMP
  else if (_method_type == MethodType::PCL_OPENMP)
  {
    omp_ndt.setTransformationEpsilon(trans_eps);
    omp_ndt.setStepSize(step_size);
    omp_ndt.setResolution(ndt_res);
    omp_ndt.setMaximumIterations(max_iter);
    omp_ndt.setInputSource(filtered_scan_ptr);
  }
#endif
/*
输入源（Input Source）是指作为算法输入的数据源。在这个例子中，filtered_scan_ptr被设置为输入源，它可能是一个经过滤波处理后的激光扫描数据。输入源提供了算法需要处理的原始数据。
输入目标（Input Target）是指算法需要与之进行匹配或比较的目标数据。在这个例子中，map_ptr被设置为输入目标，可能是一个地图或参考数据。输入目标用于与输入源进行比较或匹配，以便
算法能够根据目标数据进行相应的处理。

*/
  static bool is_first_map = true;  //设置 NDT 目标（地图） 首次：把当前累积的 map 设为 NDT 的目标（Target）
  /*
  * InputSource：当前帧点云（待匹配）
  * InputTarget：地图（参考点云）
  * ndt.align(...) 就是把 Source 对齐到 Target
  */
  if (is_first_map == true)
  {
    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setInputTarget(map_ptr);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setInputTarget(map_ptr);// 设置输入目标
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt.setInputTarget(map_ptr);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(map_ptr);
#endif
    is_first_map = false;
  }
  //计算预测位姿
  guess_pose.x = previous_pose.x + diff_x;      //基础预测：previous_pose + diff（用上一帧的位移外推）
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;
  //传感器预测：根据 IMU/Odom 的配置，调用对应的 *_calc 函数
  if (_use_imu == true && _use_odom == true)
    imu_odom_calc(current_scan_time);
  if (_use_imu == true && _use_odom == false)
    imu_calc(current_scan_time);
  if (_use_imu == false && _use_odom == true)
    odom_calc(current_scan_time);
  //选择：根据配置选择最终的预测位姿。
  pose guess_pose_for_ndt;
  if (_use_imu == true && _use_odom == true)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (_use_imu == true && _use_odom == false)
    guess_pose_for_ndt = guess_pose_imu;
  else if (_use_imu == false && _use_odom == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;
	/*
	Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX())：创建一个绕X轴旋转的AngleAxisf对象，其中guess_pose_for_ndt.roll是旋转角度，Eigen::Vector3f::UnitX()表示绕X轴旋转。
	Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY())：创建一个绕Y轴旋转的AngleAxisf对象，其中guess_pose_for_ndt.pitch是旋转角度，Eigen::Vector3f::UnitY()表示绕Y轴旋转。
	Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ())：创建一个绕Z轴旋转的AngleAxisf对象，其中guess_pose_for_ndt.yaw是旋转角度，Eigen::Vector3f::UnitZ()表示绕Z轴旋转。
	Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z)：创建一个平移变换的Translation3f对象，其中guess_pose_for_ndt.x、guess_pose_for_ndt.y、guess_pose_for_ndt.z分别表示在X、Y、Z轴上的平移量。
	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol：将旋转和平移变换按照一定的顺序进行组合，得到一个4x4的变换矩阵Matrix4f对象。tf_btol是一个额外的变换矩阵。
	*/
  //构造初始猜测矩阵   把预测位姿转成 4x4 变换矩阵
  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());// 绕X轴旋转的角度
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());// 绕Y轴旋转的角度
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());// 绕Z轴旋转的角度

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);// 平移向量

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;// 初始化猜测的变换矩阵  tf_btol：base_link → localizer（雷达）的变换

  t3_end = ros::Time::now();// 记录时间戳
  d3 = t3_end - t3_start;// 计算时间差

  t4_start = ros::Time::now();
  //执行 NDT 匹配
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
/*

    ndt.align(*output_cloud, init_guess);：使用ndt匹配算法对输入的点云进行配准，将结果保存在output_cloud中，并使用init_guess作为初始猜测。
    fitness_score = ndt.getFitnessScore();：获取配准后的点云与目标点云之间的适应度得分，用于评估配准的质量。
    t_localizer = ndt.getFinalTransformation();：获取配准后的点云的最终变换矩阵，表示将输入点云变换到目标点云坐标系下的变换。
    has_converged = ndt.hasConverged();：判断配准算法是否收敛，返回一个布尔值，表示配准是否成功。
    final_num_iteration = ndt.getFinalNumIteration();：获取配准算法的最终迭代次数，用于评估算法的收敛速度。
    transformation_probability = ndt.getTransformationProbability();：获取配准后的点云变换的概率，表示变换的可信度。

*/
  if (_method_type == MethodType::PCL_GENERIC)
  {
    ndt.align(*output_cloud, init_guess);// 使用ndt.align函数进行点云匹配  ndt.align(*output_cloud, init_guess)：核心调用，执行 NDT 匹配
    fitness_score = ndt.getFitnessScore();// 获取匹配后的适应度分数
    t_localizer = ndt.getFinalTransformation();// 获取匹配后的最终变换矩阵 匹配结果（4x4 变换矩阵）
    has_converged = ndt.hasConverged();// 判断匹配是否收敛
    final_num_iteration = ndt.getFinalNumIteration();// 获取匹配的最终迭代次数
    transformation_probability = ndt.getTransformationProbability();// 获取匹配的变换概率
  }
  else if (_method_type == MethodType::PCL_ANH)
  {
    anh_ndt.align(init_guess);
    fitness_score = anh_ndt.getFitnessScore();
    t_localizer = anh_ndt.getFinalTransformation();
    has_converged = anh_ndt.hasConverged();
    final_num_iteration = anh_ndt.getFinalNumIteration();
  }
#ifdef CUDA_FOUND
  else if (_method_type == MethodType::PCL_ANH_GPU)
  {
    anh_gpu_ndt.align(init_guess);
    fitness_score = anh_gpu_ndt.getFitnessScore();
    t_localizer = anh_gpu_ndt.getFinalTransformation();
    has_converged = anh_gpu_ndt.hasConverged();
    final_num_iteration = anh_gpu_ndt.getFinalNumIteration();
  }
#endif
#ifdef USE_PCL_OPENMP
  else if (_method_type == MethodType::PCL_OPENMP)
  {
    omp_ndt.align(*output_cloud, init_guess);
    fitness_score = omp_ndt.getFitnessScore();
    t_localizer = omp_ndt.getFinalTransformation();
    has_converged = omp_ndt.hasConverged();
    final_num_iteration = omp_ndt.getFinalNumIteration();
  }
#endif
//将t_localizer与tf_ltob相乘，得到t_base_link。这是将局部坐标系t_localizer转换为基准坐标系t_base_link的变换矩阵。
  t_base_link = t_localizer * tf_ltob;  //把雷达坐标系的结果转成车体坐标系
  
//使用t_localizer对scan_ptr点云进行坐标变换，得到transformed_scan_ptr。这是将原始点云在局部坐标系下的坐标转换为基准坐标系下的坐标。
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);  //transformed_scan_ptr：把当前帧点云变换到世界坐标系。
  
//创建tf::Matrix3x3对象mat_l和mat_b，并将t_localizer和t_base_link的旋转矩阵分别赋值给它们。这是为了获取t_localizer和t_base_link的旋转矩阵，以便后续使用。
  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  //这段代码的作用是更新定位器的姿态信息。具体来说，它将从变换矩阵t_localizer中提取出位置信息，并将其赋值给localizer_pose的x、y和z属性。然后，它使用mat_l.getRPY()函数从变换矩阵中提取出欧拉角信息，并将其赋值给localizer_pose的roll、pitch和yaw属性。
  
  localizer_pose.x = t_localizer(0, 3);// 提取x坐标
  localizer_pose.y = t_localizer(1, 3);// 提取y坐标
  localizer_pose.z = t_localizer(2, 3);// 提取z坐标
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);// 提取欧拉角信息

  // Update ndt_pose.
  //这段代码的作用是更新ndt_pose的值。具体来说，它将ndt_pose的x、y、z坐标分别设置为t_base_link矩阵的第一行、第二行和第三行的第四个元素。然后，它使用mat_b.getRPY函数将ndt_pose的roll、pitch和yaw角设置为mat_b矩阵的欧拉角表示。
  ndt_pose.x = t_base_link(0, 3);   //车体坐标系下的位姿。
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1); // 使用mat_b矩阵的欧拉角表示设置ndt_pose的roll、pitch和yaw角

  current_pose.x = ndt_pose.x;      //最终位姿
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;
//这段代码是用于设置和发送一个tf变换的。具体来说，它执行以下操作：
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));//这将根据当前姿态的x、y和z坐标设置变换的平移部分。
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);//这将根据当前姿态的roll、pitch和yaw角设置变换的旋转部分。
  transform.setRotation(q);//q中存储的旋转角度信息设置给transform对象，从而实现将机器人的旋转信息应用到transform对象中。

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));//这将使用之前设置的变换、当前扫描时间以及"map"和"base_link"作为父子坐标系的名称创建一个StampedTransform对象。br.sendTransform()这将通过ROS的tf库将变换发送到tf变换树中。

  scan_duration = current_scan_time - previous_scan_time;// 计算当前扫描时间与上一次扫描时间的差值
  double secs = scan_duration.toSec();// 将时间差值转换为秒数，并赋值给变量secs

  // Calculate the offset (curren_pos - previous_pos)
  //这段代码的作用是计算当前位置和上一个位置之间的偏移量，并根据偏移量计算当前的速度 供下一帧预测使用
  diff_x = current_pose.x - previous_pose.x;// 计算当前位置和上一个位置在x轴上的偏移量
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);// 计算当前位置和上一个位置在yaw角上的偏移量（以弧度表示）
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);// 计算当前位置和上一个位置之间的距离


  current_velocity_x = diff_x / secs;// 计算当前位置在x轴上的速度
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

//这段注释代码的意思是将当前姿态和速度的值分别赋给不同的变量。具体来说，代码中的current_pose表示当前的姿态，包括位置和旋转角度，而current_velocity_x、current_velocity_y和current_velocity_z表示当前的速度在x、y和z方向上的分量。
  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;

  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;

  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // Update position and posture. current_pos -> previous_pos
  //这段代码是用于更新位置和姿态的。它将当前位置和姿态赋值给先前的位置和姿态，以便在下一次更新时使用。
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // Calculate the shift between added_pos and current_pos
  //这段代码的注释意思是：计算added_pos和current_pos之间的位移，如果位移大于等于min_add_scan_shift，则将transformed_scan_ptr指向的点云数据添加到地图中，并更新added_pose的值为current_pose的值。其中，added_pose包括了位置坐标(x, y, z)和姿态角(roll, pitch, yaw)的信息。
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));//添加新点云到地图
  if (shift >= min_add_scan_shift)  //shift：当前位置与上次添加点云时的位移
  {
    map += *transformed_scan_ptr;       //map += *transformed_scan_ptr：把当前帧点云加入地图
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    if (_method_type == MethodType::PCL_GENERIC)    //更新 NDT 目标：让 NDT 用新的地图做匹配 这就是建图的核心——地图不断增长
      ndt.setInputTarget(map_ptr);//输入目标
    else if (_method_type == MethodType::PCL_ANH)
    {
      if (_incremental_voxel_update == true)
        anh_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        anh_ndt.setInputTarget(map_ptr);
    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt.setInputTarget(map_ptr);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(map_ptr);
#endif
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);  //发布 /ndt_map（地图）和 /current_pose（位姿）
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  // Write log
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << input->header.seq << ","       //把每帧数据写入 CSV 文件
      << input->header.stamp << ","
      << input->header.frame_id << ","
      << scan_ptr->size() << ","
      << filtered_scan_ptr->size() << ","
      << std::fixed << std::setprecision(5) << current_pose.x << ","
      << std::fixed << std::setprecision(5) << current_pose.y << ","
      << std::fixed << std::setprecision(5) << current_pose.z << ","
      << current_pose.roll << ","
      << current_pose.pitch << ","
      << current_pose.yaw << ","
      << final_num_iteration << ","
      << fitness_score << ","
      << ndt_res << ","
      << step_size << ","
      << trans_eps << ","
      << max_iter << ","
      << voxel_leaf_size << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{//初始化
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.  创建以时间戳命名的 CSV 日志。
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "ndt_mapping_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);

  // write header for log file
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << "input->header.seq" << ","
      << "input->header.stamp" << ","
      << "input->header.frame_id" << ","
      << "scan_ptr->size()" << ","
      << "filtered_scan_ptr->size()" << ","
      << "current_pose.x" << ","
      << "current_pose.y" << ","
      << "current_pose.z" << ","
      << "current_pose.roll" << ","
      << "current_pose.pitch" << ","
      << "current_pose.yaw" << ","
      << "final_num_iteration" << ","
      << "fitness_score" << ","
      << "ndt_res" << ","
      << "step_size" << ","
      << "trans_eps" << ","
      << "max_iter" << ","
      << "voxel_leaf_size" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift" << std::endl;

  // setting parameters  读取参数
  int method_type_tmp = 0;
  private_nh.getParam("method_type", method_type_tmp);
  _method_type = static_cast<MethodType>(method_type_tmp);
  private_nh.getParam("use_odom", _use_odom);
  private_nh.getParam("use_imu", _use_imu);
  private_nh.getParam("imu_upside_down", _imu_upside_down);
  private_nh.getParam("imu_topic", _imu_topic);
  private_nh.getParam("incremental_voxel_update", _incremental_voxel_update);

  std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;

  if (private_nh.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

#ifndef CUDA_FOUND      //编译检查
  if (_method_type == MethodType::PCL_ANH_GPU)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_ANH_GPU is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif
#ifndef USE_PCL_OPENMP
  if (_method_type == MethodType::PCL_OPENMP)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_OPENMP is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif
  //把 launch 传入的外参转成矩阵
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();

  map.header.frame_id = "map";
  //创建发布者
  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
  //创建订阅者
  ros::Subscriber param_sub = nh.subscribe("config/ndt_mapping", 10, param_callback);
  ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, output_callback);
  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);
  ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", 100000, odom_callback);
  ros::Subscriber imu_sub = nh.subscribe(_imu_topic, 100000, imu_callback);

  ros::spin();

  return 0;
}
