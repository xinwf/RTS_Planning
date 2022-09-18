#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       start_search_(false) {
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
    auto executor_state = Update();
     // for initialize or aborted
    if (executor_state != BehaviorState::RUNNING && blackboard_->info.got_last_enemy ) {  
      blackboard_->info.got_last_enemy = false;
      double min_distance = 1e10;
      double min_i = -1;
      double e_dist;
      double ally_dist;
      for (int i=0; i<search_region_1_.size(); i++){
          e_dist = blackboard_->GetDistance(search_region_1_[i], blackboard_->info.last_enemy);
          if (e_dist < min_distance 
              && e_dist >= 1.0
              && !blackboard_->IsBombAllyGoal(search_region_1_[i])
              && !blackboard_->allyInThisArea(search_region_1_[i])
          ){
              min_distance = e_dist;
              min_i = i;
          }
      }
      if(min_i >= 0){
          search_region_1_[min_i].pose.orientation = blackboard_->GetRelativeQuaternion(blackboard_->info.last_enemy,search_region_1_[min_i]);
          chassis_executor_->Execute(search_region_1_[min_i]);
          blackboard_->SetMyGoal(search_region_1_[min_i]);
          blackboard_->SetMyToward(search_region_1_[min_i]);
      }else{
          auto my_pose = blackboard_->GetRobotMapPose();
          blackboard_->SetMyGoal(my_pose); 
      }
    }else if(executor_state != BehaviorState::RUNNING && !blackboard_->info.got_last_enemy){
        auto my_pose = blackboard_->GetRobotMapPose();
        blackboard_->SetMyGoal(my_pose);
    }

    if(blackboard_->GetDistance(blackboard_->GetRobotMapPose(),
                                blackboard_->info.my_goal) < blackboard_->threshold.near_dist*2
        && !blackboard_->info.has_my_enemy 
        && !blackboard_->GuardEnemyDetected()
        && blackboard_->CanStopSwing()   // 没有受击打
        && (!blackboard_->info.has_ally_enemy || !blackboard_->CanCollaborateWithAlly()))
        {
            SearchTowardYawUpdate();
            blackboard_->info.is_sentrying = false;
            blackboard_->info.is_searching = true;
            if(executor_state != BehaviorState::IDLE){
                chassis_executor_->Cancel();
            }
        }
}
  void Cancel() {
    chassis_executor_->Cancel();
    blackboard_->info.is_searching = false;
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }
    search_region_1_.resize(decision_config.search_region_1().size());
    for (int i =0; i < decision_config.search_region_1().size();i++){
        search_region_1_[i] = (blackboard_->Point2PoseStamped(decision_config.search_region_1(i)));
    }
    return true;
  }



  void SearchTowardYawUpdate(){
    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_update_time_).count()*0.001 >= 3.0){
        float last_toward_yaw = tf::getYaw(blackboard_->info.toward_goal.pose.orientation);
        blackboard_->info.toward_goal.pose.orientation = tf::createQuaternionMsgFromYaw(AngleLimit(last_toward_yaw + M_PI/2));
        blackboard_->SetMyToward(blackboard_->info.toward_goal);
        last_update_time_ = std::chrono::system_clock::now();
    }
  }

    float AngleLimit(float yaw){
    yaw = yaw > M_PI  ? yaw - 2*M_PI : yaw;
    yaw = yaw < -M_PI ? yaw + 2*M_PI : yaw;
    return yaw;
  }

  ~SearchBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;


  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  unsigned int search_count_;
  unsigned int search_index_;
  bool start_search_;
  std::chrono::_V2::system_clock::time_point last_update_time_ = std::chrono::system_clock::now();

};
}

#endif //ROBORTS_DECISION_SEARCH_BEHAVIOR_H
