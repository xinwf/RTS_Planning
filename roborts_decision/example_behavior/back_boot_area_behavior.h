#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class BackBootAreaBehavior {
 public:
  BackBootAreaBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    boot_position_.header.frame_id = "map";
    boot_position_.pose.orientation.x = 0;
    boot_position_.pose.orientation.y = 0;
    boot_position_.pose.orientation.z = 0;
    boot_position_.pose.orientation.w = 1;

    boot_position_.pose.position.x = 0;
    boot_position_.pose.position.y = 0;
    boot_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = boot_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(boot_position_.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2) {
        chassis_executor_->Execute(boot_position_);
        blackboard_->SetMyGoal(boot_position_);
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }
    boot_position_ = blackboard_->info.start_position;
    return true;
  }

  ~BackBootAreaBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped boot_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
