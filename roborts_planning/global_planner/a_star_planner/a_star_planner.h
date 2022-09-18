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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "proto/a_star_planner_config.pb.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

namespace roborts_global_planner{

/**
 * @brief 全局规划算法类为A星下的costmap表示
 */
class AStarPlanner : public GlobalPlannerBase {

 public:
  /**
   * @brief A*规划的构造函数, 设置costmap指针和相关的costmap大小。
   * @param costmap_ptr costmap接口的共享指针
   */
  AStarPlanner(CostmapPtr costmap_ptr);
  virtual ~AStarPlanner();  
  /**
   * @brief 主要规划函数(覆盖基类函数)
   * @param start 开始位姿输入
   * @param goal 目标位姿输入
   * @param path 全局规划路径输出
   * @return ErrorInfo，如果成功则OK
   */
  roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &path);

 private:
  /**
   * @brief 单元格的状态枚举。
   */
  enum SearchState {
    NOT_HANDLED, /**< 未处理单元格.*/
    OPEN, /**< 单元格处于打开优先级队列中.*/
    CLOSED /**< 单元格处于关闭队列中*/
  };
  /**
   * @brief 根据1D Costmap表进行规划。在costmap中输入索引，得到计划路径。
   * @param start_index  在1D costmap列表中起始点位姿索引
   * @param goal_index 在1D costmap列表中目标点位姿索引
   * @param path 输出规划出的路径
   * @return ErrorInfo，如果成功则OK
   */
  roborts_common::ErrorInfo SearchPath(const int &start_index,
                                     const int &goal_index,
                                     std::vector<geometry_msgs::PoseStamped> &path);
  /**
   * @brief 计算对角线或平行移动的代价。
   * @param current_index 当前单元格的索引作为输入
   * @param neighbor_index 作为输入的相邻单元的索引
   * @param move_cost 输出移动代价
   * @return ErrorInfo，如果成功则OK
   */
  roborts_common::ErrorInfo GetMoveCost(const int &current_index,
                                      const int &neighbor_index,
                                      int &move_cost) const;
  /**
   * @brief 计算两个单元格指数之间的曼哈顿距离，作为A星算法的启发式函数。
   * @param index1 第一个单元格的索引作为输入
   * @param index2 第二个单元格的索引作为输入
   * @param manhattan_distance 曼哈顿距离作为输出
   */
  void GetManhattanDistance(const int &index1,
                            const int &index2,
                            int &manhattan_distance) const;
  /**
   * @brief 从当前单元格获取9个相邻单元格的索引
   * @param current_index 当前单元格的索引作为输入
   * @param neighbors_index 相邻单元格的索引作为输出
   */
  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;

    /**
   * @brief 路径平滑
   * @param 需要平滑的路径
   */
  std::vector<geometry_msgs::PoseStamped> BezierCurve(std::vector<geometry_msgs::PoseStamped> src_path);
    /**
   * @brief 路径平滑
   * @param 需要平滑的路径
   */
  void PathCurve(std::vector<geometry_msgs::PoseStamped>& path);

  /**
   * @brief 用于优先级队列比较进程。
   */
  struct Compare {
    bool operator()(const int &index1, const int &index2) {
      return AStarPlanner::f_score_.at(index1) > AStarPlanner::f_score_.at(index2);
    }
  };

  //! 启发式因子
  float heuristic_factor_;
  //! 无法访问的成本
  unsigned int inaccessible_cost_;
  //! 搜寻目标的容忍度
  unsigned int goal_search_tolerance_;
  //! gridmap高度尺寸
  unsigned int gridmap_height_;
  //! gridmap 宽度尺寸
  unsigned int gridmap_width_;
  //! gridmap 代价的数组
  unsigned char *cost_;
  //! 搜索算法相关的f评分，f_score = g_score + heuristic_cost_estimate
  static std::vector<int> f_score_;
  //! 搜索算法相关的g分数，它是指从开始单元格到当前单元格的分数
  std::vector<int> g_score_;
  //! vector表示每个单元格的父单元格索引
  std::vector<int> parent_;
  //! vector表示每个单元格的状态
  std::vector<AStarPlanner::SearchState> state_;

  int beziercruve_pointsize_;


};

std::vector<int> AStarPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                 "a_star_planner",
                                 AStarPlanner,
                                 std::shared_ptr<roborts_costmap::CostmapInterface>);

} //namespace roborts_global_planner

#endif // ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
