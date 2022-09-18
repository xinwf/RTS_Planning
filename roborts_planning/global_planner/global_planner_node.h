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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H


#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <tf/transform_listener.h>
#include "actionlib/server/simple_action_server.h"
#include "nav_msgs/Path.h"
#include "roborts_msgs/GlobalPlannerAction.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "io/io.h"
#include "state/node_state.h"

#include "costmap/costmap_interface.h"
#include "global_planner_base.h"
#include "proto/global_planner_config.pb.h"
#include "global_planner_algorithms.h"

namespace roborts_global_planner{

/**
 * @brief 全局规划器模块的节点类。
 */
class GlobalPlannerNode {
 public:
  typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;
  typedef std::shared_ptr<tf::TransformListener> TfPtr;
  typedef std::unique_ptr<GlobalPlannerBase> GlobalPlannerPtr;
  typedef actionlib::SimpleActionServer<roborts_msgs::GlobalPlannerAction> GlobalPlannerServer;
 /**
  * @brief 构造函数，包括所有初始化和配置
  */
  GlobalPlannerNode();

 /**
  * @brief 析构函数，停止所有正在运行的线程。
  */
  ~GlobalPlannerNode();

 private:
  /**
   * @brief 初始化ROS配置，规划算法实例，规划相关参数配置等。
   * @return ErrorInfo，如果成功则OK
   */
  roborts_common::ErrorInfo Init();
  /**
   * @brief actionlib任务的回调函数
   * @param msg 来自actionlib客户端的任务目标消息
   */
  void GoalCallback(const roborts_msgs::GlobalPlannerGoal::ConstPtr &msg);
  /**
   * @brief 设置规划模块状态
   * @param node_state 枚举用于全局规划的状态
   */
  void SetNodeState(roborts_common::NodeState node_state);
  /**
   * @brief 获取规划模块的状态
   * @return 枚举用于全局规划的状态
   */
  roborts_common::NodeState GetNodeState();
  /**
   * @brief 设置规划模块状态
   * @param error_info 全局规划的错误信息
   */
  void SetErrorInfo(roborts_common::ErrorInfo error_info);
  /**
   * @brief 获取规划模块的错误信息
   * @return 全局规划的错误信息
   */
  roborts_common::ErrorInfo GetErrorInfo();
  /**
   * @brief 检查计划线程是否仍在执行中，如果没有，则启动计划线程。
   * @return 如果成功启动计划线程，则为True，否则为false。
   */
  geometry_msgs::PoseStamped GetGoal();
  /**
   * @brief Set the planner goal
   * @param goal planner goal
   */
  void SetGoal(geometry_msgs::PoseStamped goal);
  /**
   * @brief 启动计划线程，并将状态设置为RUNNING
   */
  void StartPlanning();
  /**
   * @brief 停止计划线程，并将状态设置为IDLE
   */
  void StopPlanning();
  /**
   * @brief 规划线程，主要包括一个规划周期过程:
   * 1. 获取当前的起始位姿和目标位姿，并根据costmap的框架进行验证和转换 \n
   * 2. 使用选定的算法制定计划 \n
   * 3. 如果成功，检查当前姿势是否接近目标。如果完成，不需要计划和跳出循环!\n
   * 4. 如果不成功，检查是否达到最大重试次数。如果是这样，就不需要计划和跳出这个循环了!\n
   * 5. 如果仍需要规划，则进入下一个周期，等待全局规划频率决定的持续时间。
   */
  void PlanThread();
  /**
   * @brief 得到两个姿势的欧几里得距离。
   * @param pose1 第一个姿势的形式是geometry_msgs::PoseStamped
   * @param pose2 第二个姿势的形式是geometry_msgs::PoseStamped
   * @return 欧氏距离
   */
  double GetDistance(const geometry_msgs::PoseStamped& pose1,
                     const geometry_msgs::PoseStamped& pose2);
  /**
   * @brief 得到两个姿势的角度差。
   * @param pose1 第一个姿势的形式是geometry_msgs::PoseStamped
   * @param pose2 第二个姿势的形式是geometry_msgs::PoseStamped
   * @return The angle difference
   */
  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2);
  /**
   * @brief 在geometry_msgs::PoseStamped矢量中输入路径，在nav_msgs:: path中发布路径
   * @param path 从全局规划器生成的路径，实际上离散的姿势在几何y_msgs::PoseStamped矢量的形式
   */
  void PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path);

  //! ROS 节点句柄初始化
  ros::NodeHandle nh_;
  // rviz路径可视化的ROS发布者
  ros::Publisher path_pub_;
  // ROS Actionlib Server用于命令全局规划模块
  GlobalPlannerServer as_;
  //! Global planner 指针
  GlobalPlannerPtr global_planner_ptr_;
  //！ Transform 指针
  TfPtr tf_ptr_;
  //! Costmap 指针
  CostmapPtr costmap_ptr_;
  //! 所选算法的类型
  std::string selected_algorithm_;

 
  // 接受来自全局规划的目标点(输入)
  geometry_msgs::PoseStamped goal_;
  //! 目标点互斥
  std::mutex goal_mtx_;
 
  bool pause_;


  //由全局规划器生成的路径(输出)
  nav_msgs::Path path_;
  // Bool标志，指示是否有新的计划路径
  bool new_path_;

  //! 全局规划进展的线程
  std::thread plan_thread_;
  //! 规划条件变量
  std::condition_variable plan_condition_;
  //! 互斥的规划
  std::mutex plan_mutex_;


  //全局规划器节点状态
  roborts_common::NodeState node_state_;

  // 全局规划节点状态互斥
  std::mutex node_state_mtx_;

  // 全局规划器错误信息
  roborts_common::ErrorInfo error_info_;

  // 全局规划器错误信息互斥
  std::mutex error_info_mtx_;

 
  // 周期持续时间，单位为微秒
  std::chrono::microseconds cycle_duration_;

  //! 最大重试次数
  int max_retries_;

  //! 对目标点的距离容忍
  double goal_distance_tolerance_;
  //! 朝向目标的角度公差
  double goal_angle_tolerance_;
};

} //namespace roborts_global_planner
#endif //ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
