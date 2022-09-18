#ifndef ROBORTS_DECISION_FIXLOCLIZATION_BEHAVIOR_H
#define ROBORTS_DECISION_FIXLOCLIZATION_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"


namespace roborts_decision {
class FixLocalizationBehavior {
 public:
  FixLocalizationBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard,
               const std::string & proto_file_path) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) {
        if (!LoadParam(proto_file_path)) {
        ROS_ERROR("%s can't open file", __FUNCTION__);
        }
        count_ = 0;
        init_ = false;
    }

  void Run() {
    if(!init_){
        UpdateGobalFrame();
        init_ = true;
    }
      if(auto_fix_localization_ && blackboard_->GuardHasMyPose()){
          geometry_msgs::PoseStamped my_localization_pose = blackboard_->GetRobotMapPose();
          geometry_msgs::PoseStamped my_guard_pose = blackboard_->GetMyGuardPose();
          if(blackboard_->GetDistance(my_localization_pose , my_guard_pose) > 3.0){
              count_ ++ ;
              if(count_ >= 200){
                float map_yaw = AngleLimit(blackboard_->info.gyro_yaw - relative_yaw_);
                my_guard_pose.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);
                blackboard_->InitMyPose(my_guard_pose);
                ROS_INFO(" fix localization !!!!!");
                count_ = 0;
              }
          }
      }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  float AngleLimit(float yaw){
    yaw = yaw > M_PI  ? yaw - 2*M_PI : yaw;
    yaw = yaw < -M_PI ? yaw + 2*M_PI : yaw;
    return yaw;
  }

  void UpdateGobalFrame(){
      // 使陀螺仪的坐标系与 map 坐标系重合
        float gyro_yaw = blackboard_->info.gyro_yaw;
        geometry_msgs::PoseStamped gimbal_pose = blackboard_->GetRobotGimbalMapPose();
        float gimbal_map_yaw = tf::getYaw(gimbal_pose.pose.orientation);
        relative_yaw_ = gyro_yaw - gimbal_map_yaw;
  }

  ~FixLocalizationBehavior() = default;

    bool LoadParam(const std::string &proto_file_path) {
        roborts_decision::DecisionConfig decision_config;
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
        return false;
        }
        auto_fix_localization_ = decision_config.auto_fix_localization();
        return true;
    }
 private:
    

  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  bool auto_fix_localization_;
  int count_;
  float relative_yaw_;
  bool init_;
;


    
};
}

#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
