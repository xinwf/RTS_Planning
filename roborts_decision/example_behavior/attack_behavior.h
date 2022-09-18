#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class AttackBehavior {
 public:
  AttackBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       near_enemy_(false),
                                                       init_center_yaw_(false),
                                                       sentry_center_yaw_(0) {
    gimbal_angle_.yaw_angle = 0;
    gimbal_angle_.pitch_angle = 0;
    gimbal_angle_.yaw_mode = true;
    gimbal_angle_.pitch_mode = true;
    gimbal_publisher_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 2, this);
  }

  void Run() {
    auto executor_state = Update();
    geometry_msgs::PoseStamped enemy_pose = blackboard_->GetEnemy();
    geometry_msgs::PoseStamped my_pose = blackboard_->GetRobotMapPose();
    enemy_pose.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_pose ,my_pose);
    blackboard_->SetMyToward(enemy_pose);
    // directly to enemy
    if(blackboard_->GetDistance(enemy_pose,my_pose) < 0.5  && (blackboard_->GuardEnemyDetected()|| blackboard_->info.has_ally_enemy)){
        near_enemy_ = true;
    }else{
        near_enemy_ = false;

    }
    if(near_enemy_ && executor_state != BehaviorState::IDLE){
        chassis_executor_->Cancel();
    }
    
    if (executor_state != BehaviorState::RUNNING || !near_enemy_){
        chassis_executor_->Execute(enemy_pose);
        blackboard_->SetMyGoal(enemy_pose);
        init_center_yaw_ = false;
    }
    else if(near_enemy_ && !blackboard_->info.has_my_enemy){
        if(!init_center_yaw_){
            InitSentryCoordinate();
        }
        blackboard_->info.is_sentrying = true;
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
    near_enemy_ = false;
    init_center_yaw_ = false;
    blackboard_->info.is_sentrying = false;
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~AttackBehavior() = default;

 private:

  geometry_msgs::PoseStamped _GoalToAttack(geometry_msgs::PoseStamped us, geometry_msgs::PoseStamped enemy){
       geometry_msgs::PoseStamped goal;
       goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy, us);
       goal.pose.position.x = us.pose.position.x;
       goal.pose.position.y = us.pose.position.y;
       goal.pose.position.z = 0;
       return goal;
  }

  void InitSentryCoordinate(){
    geometry_msgs::PoseStamped enemy_pose = blackboard_->GetEnemy();
    geometry_msgs::PoseStamped my_pose = blackboard_->GetRobotMapPose();
    sentry_center_yaw_ = tf::getYaw(blackboard_->GetRelativeQuaternion(enemy_pose ,my_pose));
    blackboard_->info.sentry_center_yaw = sentry_center_yaw_;
    init_center_yaw_ = true;
  }
  
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  ros::Publisher gimbal_publisher_;
  ros::NodeHandle nh_;
  
  bool near_enemy_;
  bool init_center_yaw_;
  double sentry_center_yaw_;

  
};
}

#endif //ROBORTS_DECISION_ATTACK_BEHAVIOR_H
