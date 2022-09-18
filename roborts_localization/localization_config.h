/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <ros/ros.h>

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H

namespace roborts_localization {

// 从ROS参数服务器中获取参数
struct LocalizationConfig {

  // 定义获取定位参数的函数
  void GetParam(ros::NodeHandle *nh) 
  {
    /**
     * @param string  参数类型
     * @param odom_frame 在载入yaml文件的参数名
     * @param odom_frame_id  如果yaml或者launch文件里面有odom_frame参数的赋值，则使用该文件定义的参数，后面odom的值忽略
     * @param odom  默认参数，如果yaml那边不生效，则用默认参数*/
      nh->param<std::string>("odom_frame", odom_frame_id, "odom");

      nh->param<std::string>("base_frame", base_frame_id, "base_link");
      nh->param<std::string>("global_frame", global_frame_id, "map");
      nh->param<std::string>("laser_topic_name", laser_topic_name, "scan");
      nh->param<std::string>("map_topic_name", map_topic_name, "map");
      nh->param<std::string>("init_pose_topic_name", init_pose_topic_name, "initialpose");
      nh->param<double>("transform_tolerance", transform_tolerance, 0.1);
      nh->param<double>("initial_pose_x", initial_pose_x, 1);
      nh->param<double>("initial_pose_y", initial_pose_y, 1);
      nh->param<double>("initial_pose_a", initial_pose_a, 0);
      nh->param<double>("initial_cov_xx", initial_cov_xx, 0.1);
      nh->param<double>("initial_cov_yy", initial_cov_yy, 0.1);
      nh->param<double>("initial_cov_aa", initial_cov_aa, 0.1);
      nh->param<bool>("enable_uwb", enable_uwb, false);
      nh->param<std::string>("uwb_frame_id", uwb_frame_id, "uwb");
      nh->param<std::string>("uwb_topic_name", uwb_topic_name, "uwb");
      nh->param<bool>("use_sim_uwb", use_sim_uwb, false);
      nh->param<int>("uwb_correction_frequency", uwb_correction_frequency, 20);
      nh->param<bool>("publish_visualize", publish_visualize, true);
  }
  //omod的坐标系
  std::string odom_frame_id;

  // 机器人的坐标系
  std::string base_frame_id;

  // 地图坐标系
  std::string global_frame_id;

  // 激光雷达发送的话题名
  std::string laser_topic_name;

  // 静态地图发送的话题名
  std::string map_topic_name;

  // 初始化位姿发送的话题名
  std::string init_pose_topic_name;

  // tf发布间隔
  double transform_tolerance;

  // 初始化估计位姿X轴位置
  double initial_pose_x;

  // 初始化估计位姿Y轴位置
  double initial_pose_y;

  // 初始化估计位姿Yaw轴角度
  double initial_pose_a;

  // 初始化估计位姿xx的协方差
  double initial_cov_xx;

  // 初始化估计位姿yy的协方差
  double initial_cov_yy;

  // 初始化估计位姿Yaw角度的协方差
  double initial_cov_aa;

  // 发布可视化界面
  bool publish_visualize;

  // 是否使用uwb定位
  bool enable_uwb;

  // uwb坐标系
  std::string uwb_frame_id;

  // uwb发送的话题名
  std::string uwb_topic_name;

  // 在仿真环境是否使用uwb
  bool use_sim_uwb;

  // uwb当前发送的频率
  int uwb_correction_frequency;

};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
