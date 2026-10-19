/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
  ********************
    Abraham Monrroy
    based on Alexander Carballo's idea for Tsukuba Challenge for NDT monitoring.

    Created on: March 20, 2018
 */
//AGV 定位监控模块，用于监控 NDT 定位的健康状态。
#include "ndt_matching_monitor.h"   //这个头文件里定义了 ROSNDTMatchingMonitor 类、ndt_status 枚举、成员变量等
//GNSS 回调  NDT 出问题时，可以用 GNSS 位姿来重定位。
void ROSNDTMatchingMonitor::gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)    //订阅 gnss_pose 话题，接收 GNSS 定位数据
{
    gnss_pose_.header = input->header;
    gnss_pose_.pose.pose = input->pose; //保存最新的 GNSS 位姿
    gnss_pose_available_ = true;        //标记 GNSS 可用
    gnss_text_ = " - GNSS available";   //用于 RViz 显示的文本
}
//NDT 统计回调
void ROSNDTMatchingMonitor::ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr& input)    //订阅 /ndt_stat 话题，接收 NDT 匹配统计信息。
{
    iteration_count_ = input->iteration;    //NDT 迭代次数（越多说明越难收敛）

    if (last_score_ == 0.f) //上一次匹配得分
        last_score_ = input->score;

    current_score_ = input->score;  //当前匹配得分
    score_delta_ = fabs(current_score_-last_score_);//得分变化量（突然变大说明定位可能跳变）
    //这两个指标是判断 NDT 是否正常工作的核心依据
}
//预测下一个位姿
geometry_msgs::PoseWithCovarianceStamped
                ROSNDTMatchingMonitor::predict_next_pose(geometry_msgs::PoseWithCovarianceStamped prev_pose,
                                                         geometry_msgs::PoseWithCovarianceStamped current_pose)
{
    geometry_msgs::PoseWithCovarianceStamped predicted_pose;

    double time_delta = current_pose.header.stamp.toSec() - prev_pose.header.stamp.toSec(); //两个位姿的时间差

    if (time_delta == 0)    //如果时间差为 0，直接返回当前位姿
    {
        return current_pose;
    }
    //否则，用线性外推预测下一帧位姿
    predicted_pose.header = current_pose.header;
    predicted_pose.header.stamp.sec += time_delta;
    predicted_pose.pose.pose.orientation = current_pose.pose.pose.orientation;

    predicted_pose.pose.pose.position.x = current_pose.pose.pose.position.x +
            (current_pose.pose.pose.position.x - current_pose.pose.pose.position.x) / time_delta;
    predicted_pose.pose.pose.position.y = current_pose.pose.pose.position.y +
            (current_pose.pose.pose.position.y - current_pose.pose.pose.position.y) / time_delta;
    predicted_pose.pose.pose.position.z = current_pose.pose.pose.position.z +
            (current_pose.pose.pose.position.z - current_pose.pose.pose.position.z) / time_delta;

    return predicted_pose;
    //当 NDT 匹配失败时，用这个函数预测车辆应该在的位置，作为重定位的初始位姿。
}
//初始位姿回调  订阅 /initialpose 话题，接收手动重定位请求。
void ROSNDTMatchingMonitor::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
    //if currently blocking reset and receiving a different blocking pose, then try to reset
    if (ndt_status::NDT_FATAL == ndt_status_    //当前状态是 NDT_FATAL（无法自动恢复），且收到一个不同的初始位姿
        && input->pose.pose.position.x != initialpose_.pose.pose.position.x
        && input->pose.pose.position.y != initialpose_.pose.pose.position.y
        && input->pose.pose.position.z != initialpose_.pose.pose.position.z
        && input->pose.pose.orientation.x != initialpose_.pose.pose.orientation.x
        && input->pose.pose.orientation.y != initialpose_.pose.pose.orientation.y
        && input->pose.pose.orientation.z != initialpose_.pose.pose.orientation.z
        && input->pose.pose.orientation.w != initialpose_.pose.pose.orientation.w)
    {   //重置状态为 NOT_INITIALIZED，重新初始化，清空得分
        ndt_status_ = ndt_status::NDT_NOT_INITIALIZED;  
        initialpose_ = *input;
        initialized_ = false;
        last_score_ = 0.;
        ROS_INFO_STREAM(__APP_NAME__ << " RECEIVED NEW INITIAL POSE, RESETTING.");
    }   //当 NDT 彻底崩溃时，操作员可以手动发一个初始位姿，让系统重新开始
}
//核心监控回调 每次收到 /ndt_pose 就执行一次
void ROSNDTMatchingMonitor::ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    geometry_msgs::PoseWithCovarianceStamped initialpose_msg;   //准备用于发布重定位。
    initialpose_msg.header = input->header;
    jsk_rviz_plugins::OverlayText rviz_info_text;   //RViz 上显示的文字
    std_msgs::String ndt_status_msg;                //发布状态字符串。

    if (ndt_status::NDT_FATAL == ndt_status_)   //如果已经是 FATAL 状态，直接：
    {
        rviz_info_text = ndt_fatal_text_;
        ROS_ERROR_STREAM(__APP_NAME__ << " FATAL CANNOT RECOVER - STOPPING.");  //打印错误
        initialpose_msg.pose = initialpose_.pose;   //持续发布 initialpose（保持最后已知位姿）
        initialpose_pub_.publish(initialpose_msg);
        overlay_info_text_pub_.publish(rviz_info_text); //发布 RViz 文字。
        stable_samples_ = 0;
        return;
    }
    //正常状态检测  连续多帧都正常，才认为定位可靠
    if (iteration_count_ < iteration_threshold_warning_
        && score_delta_ < score_delta_threshold_)   //迭代次数 < 警告阈值 且 得分变化 < 阈值。
    {
        //update the last good pose
        initialpose_.pose.pose = input->pose;   
        prev_initialpose_ = initialpose_;       //更新 initialpose_ 为当前位姿（保存最后的正确位姿）
        if (stable_samples_ >= (unsigned int) min_stable_samples_)
        {
            ndt_status_ = ndt_status::NDT_OK;   //如果稳定帧数 >= 最小稳定样本数，状态设为 OK，initialized_ = true
            initialized_ = true;
        }
        stable_samples_++;  //stable_samples_ 计数器递增
    }
    else if ( initialized_ &&
            ((iteration_count_ >= iteration_threshold_stop_)//迭代次数 >= 停止阈值（严重）
             || (iteration_count_ >= iteration_threshold_warning_ && score_delta_ >= score_delta_threshold_)))  //迭代次数 >= 警告阈值 且 得分变化 >= 阈值（中等）
    {
        ndt_status_ = ndt_status::NDT_ERROR;    //状态设为 ERROR
        if (gnss_pose_available_)   //如果有 GNSS：用 GNSS 预测位姿
        {
            initialpose_msg.header.frame_id = gnss_pose_.header.frame_id;
            initialpose_msg.pose = predict_next_pose(prev_gnss_pose_, gnss_pose_).pose;
        }
        else
        {   //如果没有 GNSS：用上一次的正确位姿预测
            initialpose_msg.pose = predict_next_pose(prev_initialpose_, initialpose_).pose;
            //if there are too many predictions in a short time, set fatal status
            if (prediction_samples_ > fatal_time_threshold_     //如果短时间内预测次数太多，说明 NDT 持续失败，升级为 FATAL
                && (input->header.stamp.toSec() - last_prediction_time_.toSec()) <= fatal_time_threshold_ )
            {
                ndt_status_ = ndt_status::NDT_FATAL;
            }
            last_prediction_time_ = input->header.stamp;
            prediction_samples_++;
        }

        if (gnss_pose_available_
            || stable_samples_ >= (unsigned int) min_stable_samples_ )
        {
            ROS_INFO_STREAM(__APP_NAME__ << " Resetting position.");
            initialpose_pub_.publish(initialpose_msg);  //重定位：发布 initialpose，让 NDT 重新初始化
            stable_samples_ = 0;
        }
    }
    else if (initialized_ &&        //WARNING 状态处理
            ((iteration_count_ > iteration_threshold_warning_
              && iteration_count_< iteration_threshold_stop_)//迭代次数在警告和停止阈值之间
              || ndt_status_ == ndt_status::NDT_ERROR))     //迭代次数在警告和停止阈值之间
    {
        ndt_status_ = ndt_status::NDT_WARNING;  //状态设为 WARNING，保存上一帧正确位姿
        prev_initialpose_ = initialpose_;
    }

    prev_gnss_pose_ = gnss_pose_;   //保存 GNSS 上一帧，用于下次预测

    if (!initialized_)  //根据状态选择对应的 RViz 文字：
    {
        rviz_info_text = ndt_not_ready_text_;
    } else
    {
        switch (ndt_status_)
        {
            case ndt_status::NDT_ERROR:
                rviz_info_text = ndt_error_text_;
                break;
            case ndt_status::NDT_WARNING:
                rviz_info_text = ndt_warn_text_;
                break;
            case ndt_status::NDT_OK:
                rviz_info_text = ndt_normal_text_;
                break;
            case ndt_status::NDT_NOT_INITIALIZED:
            default:
                rviz_info_text = ndt_not_ready_text_;
                break;
        }
    }

    rviz_info_text.text+=gnss_text_;    //附加 GNSS 状态文字。

    ndt_status_msg.data = ndt_status_names[ndt_status_];

    overlay_info_text_pub_.publish(rviz_info_text); //发布 /ndt_monitor/ndt_info_text（RViz 显示）。
    ndt_status_pub_.publish(ndt_status_msg);        //发布 /ndt_monitor/ndt_status（状态字符串）
    last_score_ = current_score_;       //更新 last_score_ 为当前得分
}
//主运行函数
void ROSNDTMatchingMonitor::Run()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Geting parameters  读取参数
    private_nh.param("/monitor/iteration_threshold_warn", iteration_threshold_warning_, NDT_THRESHOLD_ITERATION_WARN);  //迭代次数警告阈值，默认值来自宏
    private_nh.param("/monitor/iteration_threshold_stop", iteration_threshold_stop_, NDT_THRESHOLD_ITERATION_STOP);
    private_nh.param("/monitor/score_delta_threshold", score_delta_threshold_, NDT_THRESHOLD_SCORE_MAX_DELTA);
    private_nh.param("/monitor/min_stable_samples", min_stable_samples_, NDT_MIN_STABLE_SAMPLES);
    private_nh.param("/monitor/fatal_time_threshold", fatal_time_threshold_, NDT_TIME_TO_FATAL_PREDICTIONS);


    //swap values in case of error 确保警告阈值 <= 停止阈值
    if (iteration_threshold_warning_ > iteration_threshold_stop_) {
        auto aux = iteration_threshold_stop_;
        iteration_threshold_stop_ = iteration_threshold_warning_;
        iteration_threshold_warning_ = aux;
    }

    // Subscribers
    ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, &ROSNDTMatchingMonitor::ndt_stat_callback, this);
    ros::Subscriber ndt_pose_sub = nh.subscribe("/ndt_pose", 10, &ROSNDTMatchingMonitor::ndt_pose_callback, this);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, &ROSNDTMatchingMonitor::initialpose_callback, this);
    ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, &ROSNDTMatchingMonitor::gnss_callback, this);

    // Publishers
    initialpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    overlay_info_text_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/ndt_monitor/ndt_info_text", 1);
    ndt_status_pub_ = nh.advertise<std_msgs::String>("/ndt_monitor/ndt_status", 1);

    ros::spin();

}
//构造函数
ROSNDTMatchingMonitor::ROSNDTMatchingMonitor()
{   //初始化所有状态变量
    gnss_pose_available_ = false;
    last_score_     = 0.0;
    stable_samples_ = 0;
    prediction_samples_ = 0;
    ndt_status_     = ndt_status::NDT_NOT_INITIALIZED;
    initialized_    = false;
    gnss_text_ = " - NO GNSS available";
    //OverlayText 参数：宽 800、高 100、字号 16、边宽 2、背景透明 0.2。
    ndt_normal_text_.width  = 800;
    ndt_normal_text_.height = 100;
    ndt_normal_text_.left   = 10;
    ndt_normal_text_.top    = 10;
    ndt_normal_text_.text_size  = 16;
    ndt_normal_text_.line_width = 2;

    ndt_normal_text_.bg_color.r = 0.f;
    ndt_normal_text_.bg_color.g = 0.f;
    ndt_normal_text_.bg_color.b = 0.f;
    ndt_normal_text_.bg_color.a = 0.2f;
    //配置 5 种状态的 RViz 显示文字：
    ndt_normal_text_.text = "NDT MONITOR - OK";
    ndt_normal_text_.fg_color.r = 0.0f;
    ndt_normal_text_.fg_color.g = 1.f;
    ndt_normal_text_.fg_color.b = 0.0f;
    ndt_normal_text_.fg_color.a = 1.0f;

    ndt_warn_text_ = ndt_normal_text_;
    ndt_warn_text_.text = "NDT MONITOR - WARNING";
    ndt_warn_text_.fg_color.r = 1.0f;
    ndt_warn_text_.fg_color.g = 0.6f;

    ndt_not_ready_text_ = ndt_warn_text_;
    ndt_not_ready_text_.text = "NDT MONITOR - NOT INITIALIZED";

    ndt_error_text_ = ndt_normal_text_;
    ndt_error_text_.text = "NDT MONITOR - ERROR \n TRYING LAST CORRECT LOCALIZATION";
    ndt_error_text_.fg_color.r = 1.0f;
    ndt_error_text_.fg_color.g = 0.f;

    ndt_fatal_text_ = ndt_error_text_;
    ndt_fatal_text_.text = "NDT MONITOR - FATAL CANNOT RECOVER AUTOMATICALLY";
}