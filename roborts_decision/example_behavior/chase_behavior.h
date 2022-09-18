#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ChaseBehavior {
 public:
  ChaseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    // chase_buffer_.resize(2);

    can_cancel_goal = true;
  }

  void Run() {
    auto executor_state = Update();
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    auto enemy_pose = blackboard_->GetEnemy();
    blackboard_->SetMyToward(enemy_pose);
    float distance = blackboard_->GetDistance(robot_map_pose, enemy_pose);
    bool is_block = blackboard_->IsBlockByGlass();
    ROS_INFO("is_block %d " ,is_block);
    ROS_INFO("chase enemy (x : %f , y:%f)  distance : %f" ,enemy_pose.pose.position.x , enemy_pose.pose.position.y , distance );
    if (executor_state != BehaviorState::RUNNING 
        || (distance <= 1.0)
        || (distance >= blackboard_->threshold.shoot_distance)
        || is_block
        || blackboard_->HasEnemyBehind(robot_map_pose)
        || !blackboard_->IsSurroundEnemy(enemy_pose , robot_map_pose)) {

      // 到一定距离则取消跟踪
      if ( (distance >= 1.2) 
            && (distance <= blackboard_->threshold.shoot_distance + 0.1) 
            && !is_block  
            && !blackboard_->HasEnemyBehind(robot_map_pose)
            && blackboard_->IsSurroundEnemy(enemy_pose , robot_map_pose)) {

        if (can_cancel_goal) {
        chassis_executor_->Cancel();
        blackboard_->info.is_chase = false;
        can_cancel_goal = false;
        }
        return;
      } 
      // 跟踪
      else {
          if(!ChoosePoint(robot_map_pose , enemy_pose , true)){
            ChoosePoint(robot_map_pose , enemy_pose , false);
          }
      }
    }
  }

  void UpdateChaseBuffer(geometry_msgs::PoseStamped enemy_pose , float dist){
    geometry_msgs::PoseStamped temp_goal;
    temp_goal.header.frame_id = "map";
    temp_goal.header.stamp = ros::Time::now();
    // temp_goal.pose.position.x = (enemy_pose.pose.position.x - dist) > 0 ? (enemy_pose.pose.position.x - dist) : enemy_pose.pose.position.x;
    // temp_goal.pose.position.y = (enemy_pose.pose.position.y - dist) > 0 ? (enemy_pose.pose.position.y - dist) : enemy_pose.pose.position.y;
    // chase_buffer_.push_back(temp_goal);
    // temp_goal.pose.position.x = (enemy_pose.pose.position.x + dist) < 7.8 ? (enemy_pose.pose.position.x + dist) : enemy_pose.pose.position.x;
    // temp_goal.pose.position.y = (enemy_pose.pose.position.y - dist) > 0 ? (enemy_pose.pose.position.y - dist) : enemy_pose.pose.position.y;
    // chase_buffer_.push_back(temp_goal);
    // temp_goal.pose.position.x = (enemy_pose.pose.position.x + dist) < 7.8 ? (enemy_pose.pose.position.x + dist) : enemy_pose.pose.position.x;
    // temp_goal.pose.position.y = (enemy_pose.pose.position.y + dist) < 4.5 ? (enemy_pose.pose.position.y + dist) : enemy_pose.pose.position.y;
    // chase_buffer_.push_back(temp_goal);
    // temp_goal.pose.position.x = (enemy_pose.pose.position.x - dist) > 0 ? (enemy_pose.pose.position.x - dist) : enemy_pose.pose.position.x;
    // temp_goal.pose.position.y = (enemy_pose.pose.position.y + dist) < 4.5 ? (enemy_pose.pose.position.y + dist) : enemy_pose.pose.position.y;
    // chase_buffer_.push_back(temp_goal);
    temp_goal.pose.position.x = (enemy_pose.pose.position.x - dist); 
    temp_goal.pose.position.y = (enemy_pose.pose.position.y - dist);
    chase_buffer_.push_back(temp_goal);
    temp_goal.pose.position.x = (enemy_pose.pose.position.x + dist);
    temp_goal.pose.position.y = (enemy_pose.pose.position.y - dist);
    chase_buffer_.push_back(temp_goal);
    temp_goal.pose.position.x = (enemy_pose.pose.position.x + dist);
    temp_goal.pose.position.y = (enemy_pose.pose.position.y + dist);
    chase_buffer_.push_back(temp_goal);
    temp_goal.pose.position.x = (enemy_pose.pose.position.x - dist);
    temp_goal.pose.position.y = (enemy_pose.pose.position.y + dist);
    chase_buffer_.push_back(temp_goal);
  }

  bool ChoosePoint( geometry_msgs::PoseStamped robot_map_pose ,geometry_msgs::PoseStamped enemy_pose,bool need_surround_enemy){
        auto dx = enemy_pose.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = enemy_pose.pose.position.y - robot_map_pose.pose.position.y;
        auto yaw = std::atan2(dy, dx);
        bool find_goal = false;
        geometry_msgs::PoseStamped reduce_goal;
        // reduce_goal.pose.orientation = robot_map_pose.pose.orientation;
        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        reduce_goal.pose.position.x = enemy_pose.pose.position.x - blackboard_->threshold.shoot_distance * cos(yaw);
        reduce_goal.pose.position.y = enemy_pose.pose.position.y - blackboard_->threshold.shoot_distance * sin(yaw);
        reduce_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);;
        chase_buffer_.push_back(reduce_goal);
        float dist = need_surround_enemy && !blackboard_->IsSurroundEnemy(enemy_pose , robot_map_pose) ? blackboard_->threshold.near_dist*2 : blackboard_->threshold.shoot_distance;
        UpdateChaseBuffer(enemy_pose ,dist);
        for(auto iterator_goal = chase_buffer_.begin(); iterator_goal != chase_buffer_.end(); ++iterator_goal){
            geometry_msgs::PoseStamped enemy_goal;
            enemy_goal.pose.position.x = iterator_goal->pose.position.x;
            enemy_goal.pose.position.y = iterator_goal->pose.position.y;
            enemy_goal.pose.position.z =  0;
            unsigned int goal_cell_x, goal_cell_y;

            // if necessary add mutex lock
            // blackboard_->GetCostMap2D()->GetMutex()->lock();
            auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(enemy_goal.pose.position.x,
                                                                    enemy_goal.pose.position.y,
                                                                    goal_cell_x,
                                                                    goal_cell_y);
            // blackboard_->GetCostMap2D()->GetMutex()->unlock();
            //faile
            if (!get_enemy_cell) {
                ROS_INFO("!!!can't get_enemy_cell");
                continue;
            }

            // chase line iterator
            if ( (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253)
                  || (blackboard_->IsBombAllyGoal(enemy_goal)) 
                  || blackboard_->IsBlockByGlass(enemy_goal , enemy_pose)
                  || blackboard_->HasEnemyBehind(enemy_goal)
                  || (!blackboard_->IsSurroundEnemy(enemy_pose,enemy_goal) && need_surround_enemy)) {
                auto robot_x = robot_map_pose.pose.position.x;
                auto robot_y = robot_map_pose.pose.position.y;
                unsigned int robot_cell_x, robot_cell_y;
                double goal_x, goal_y;
                blackboard_->GetCostMap2D()->World2Map( robot_x,
                                                        robot_y,
                                                        robot_cell_x,
                                                        robot_cell_y);

                // 获取该点的之间的可行点
                for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_y);
                                     line.IsValid(); 
                                     line.Advance()) {
                        auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); //current point's cost

                        blackboard_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                                (unsigned int) (line.GetY()),
                                                                goal_x,
                                                                goal_y);
                
                        if(point_cost >= 253 
                        || blackboard_->IsBombAllyGoal(goal_x, goal_y)
                        || blackboard_->allyInThisArea(blackboard_->Point2PoseStamped(goal_x,goal_y,false))
                        || blackboard_->GetEulerDistance(goal_x, goal_y, blackboard_->info.opp_reload.pose.position.x, blackboard_->info.opp_reload.pose.position.y) <= 1.0
                        || blackboard_->GetEulerDistance(goal_x, goal_y, blackboard_->info.opp_shield.pose.position.x, blackboard_->info.opp_shield.pose.position.y) <= 1.0
                        || blackboard_->IsBlockByGlass(enemy_pose,blackboard_->Point2PoseStamped(goal_x,goal_y,false))
                        || blackboard_->GetEulerDistance(goal_x,goal_y,enemy_pose.pose.position.x , enemy_pose.pose.position.y) > 1.5
                        || blackboard_->HasEnemyBehind(blackboard_->Point2PoseStamped(goal_x,goal_y,false))
                        || (need_surround_enemy && !blackboard_->IsSurroundEnemy(enemy_pose , blackboard_->Point2PoseStamped(goal_x,goal_y,false)))
                        ){
                        continue;
                        } else {
                            find_goal = true;
                            reduce_goal.pose.position.x = goal_x;
                            reduce_goal.pose.position.y = goal_y;
                            break;
                        }
                }

                // 线段历遍完后则判断是否找到合适的点
                    if (find_goal){
                        can_cancel_goal = true;
                        reduce_goal.pose.orientation = blackboard_->GetRelativeQuaternion(reduce_goal , enemy_pose);
                        chassis_executor_->Execute(reduce_goal);
                        blackboard_->SetMyGoal(reduce_goal);
                        blackboard_->SetMyToward(reduce_goal);
                        blackboard_->info.is_chase = true;
                        break;
                    }
            } 
            // 敌方位置可行走
            else{ 
                can_cancel_goal = true;
                find_goal = true;
                enemy_goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_goal , enemy_pose);
                chassis_executor_->Execute(enemy_goal);
                blackboard_->SetMyGoal(enemy_goal);
                blackboard_->SetMyToward(enemy_goal);
                blackboard_->info.is_chase = true;
                if(!blackboard_->info.has_my_enemy){
                    blackboard_->info.is_chase = false;
                }
                ROS_INFO(" go to the enemy goal ");
                break;
            }
       } // 敌人周围四个点
        if(!find_goal) {
            blackboard_->info.is_chase = false;
            if (can_cancel_goal) {
                chassis_executor_->Cancel();
                can_cancel_goal = false;
            }
            ROS_INFO(" CAN'T FIND GOAL !!!!!!!!  " );
        }
       chase_buffer_.clear();
       return find_goal;
  }



  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal) {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;

  //! cancel flag
  bool can_cancel_goal;
};
}

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H