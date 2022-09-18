#ifndef ROBORTS_DECISION_GIMBAL_CONTROL_BEHAVIOR_H
#define ROBORTS_DECISION_GIMBAL_CONTROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/RobotDamage.h>
#include <roborts_msgs/ChassisMode.h>


namespace roborts_decision {
class GimbalControlBehavior {
 public:
  GimbalControlBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       yaw_direction(false),
                                                       pitch_direction(false) {
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.pitch_angle = 0.25;
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_publisher_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 2, this);
        chassis_mode_publisher_ = nh_.advertise<roborts_msgs::ChassisMode>("chassis_mode", 50, this);
        init_ = false;
        lost_enemy_count_ = 0;
        lost_enemy_threshold_ = 150;
  }

  void Run() { 
    if(!init_){
        UpdateGobalFrame();
        init_ = true;
    }
    float map_yaw = blackboard_->info.gyro_yaw - relative_yaw_;
    map_yaw = AngleLimit(map_yaw);
    bool detected_armor = blackboard_->IsEnemyDetected();
    ROS_INFO("guard connect : %d" , blackboard_->GuardEnemyDetected());
    // 控制底盘
    PublishChassisModeMsg();
    // 多次丢失敌人则复位 pitch
    if(lost_enemy_count_ >= lost_enemy_threshold_){
        gimbal_angle_.pitch_angle = 0.105;
        lost_enemy_count_ = 0;
    } 

    // 没有敌方的位置信息且没有受到击打则取消哨兵行为
    if(!blackboard_->GuardEnemyDetected() 
        && (!blackboard_->info.has_ally_enemy || blackboard_->CanCollaborateWithAlly())
        && blackboard_->CanStopSwing()){
            blackboard_->info.is_sentrying = false;
    }

    // 受攻击后且视野内没有敌人
    if(!blackboard_->info.has_my_enemy 
        && blackboard_->info.is_hitted 
        && blackboard_->info.damage_type == roborts_msgs::RobotDamage::ARMOR){
        float base_map_yaw = tf::getYaw(blackboard_->GetRobotMapPose().pose.orientation);
        switch (blackboard_->info.hit_source)
        {
        case roborts_msgs::RobotDamage::FORWARD:
            blackboard_->info.toward_goal.pose.orientation = 
                                tf::createQuaternionMsgFromYaw( AngleLimit(base_map_yaw)); 
            break;
        case roborts_msgs::RobotDamage::LEFT:
            blackboard_->info.toward_goal.pose.orientation = 
                            tf::createQuaternionMsgFromYaw( AngleLimit(base_map_yaw + M_PI / 2)); 
            break;
        case roborts_msgs::RobotDamage::RIGHT:
            blackboard_->info.toward_goal.pose.orientation = 
                            tf::createQuaternionMsgFromYaw(AngleLimit(base_map_yaw - M_PI / 2));
            break;
        case roborts_msgs::RobotDamage::BACKWARD:
            blackboard_->info.toward_goal.pose.orientation = 
                            tf::createQuaternionMsgFromYaw(AngleLimit(base_map_yaw - M_PI ));
            break;
        default:
            break;
        }

        // 如果没有敌方的位置信息且受到击打则进行哨兵行为
        if(!blackboard_->GuardEnemyDetected() 
            && (!blackboard_->info.has_ally_enemy || blackboard_->CanCollaborateWithAlly())){
            blackboard_->info.is_sentrying = true;
            blackboard_->info.is_searching = false;
            UpdateSentryCoordinate(tf::getYaw(blackboard_->info.toward_goal.pose.orientation));
        }
        lost_enemy_count_ ++ ;
    }

    // 视野内没有敌人时按照目标点的 yaw 角
    if(!detected_armor){
        // 普通模式下： 没有检测到敌人且没有装甲板 -> 从哨岗或队友视角获取敌方角度
        if(!blackboard_->info.has_my_enemy 
            && (blackboard_->GuardEnemyDetected() 
                    || (blackboard_->info.has_ally_enemy 
                        && blackboard_->CanCollaborateWithAlly()))){
                geometry_msgs::PoseStamped enemy_pose = blackboard_->GetEnemy();
                geometry_msgs::PoseStamped my_pose = blackboard_->GetRobotMapPose();
                float distance = blackboard_->GetDistance(my_pose,enemy_pose);
                blackboard_->info.toward_goal.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_pose , my_pose);
                chassis_executor_->SetTowardGoal(enemy_pose);
                // 根据车身高度和敌方距离算出 pitch 角度
                gimbal_angle_.pitch_angle = std::atan(0.45 / distance);
        }
        // 检测到敌人但没有装甲板: 保持当前角度
        else if(blackboard_->info.has_my_enemy){
                blackboard_->info.toward_goal.pose.orientation = lost_enemy_quat_;
        }

        // get the yaw need to toward  
        float toward_yaw = tf::getYaw(blackboard_->info.toward_goal.pose.orientation);

        // 哨兵控制 or 普通控制云台
        if(!blackboard_->info.is_sentrying){
            float target_yaw = map_yaw - toward_yaw;
            target_yaw = AngleLimit(target_yaw);
            gimbal_angle_.yaw_angle = -target_yaw ;
            gimbal_angle_.pitch_mode = false;
        }else{
            SentryAction(map_yaw);
        }
        // 发出控制指令
        gimbal_publisher_.publish(gimbal_angle_);

    } 
    // 有敌人的时候更新朝向角
    else{
        blackboard_->info.is_sentrying = false;
        blackboard_->info.is_searching = false;
        blackboard_->info.toward_goal.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);
        chassis_executor_->SetTowardGoal(blackboard_->GetEnemy());
        lost_enemy_quat_ = blackboard_->info.toward_goal.pose.orientation;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(blackboard_->GetRobotGimbalBasePose().pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        gimbal_angle_.pitch_angle = pitch;
        lost_enemy_count_ = 0;
    }
  }

  void Cancel() {
    chassis_mode_.chaiss_mode = roborts_msgs::ChassisMode::CHAISSFOLLOW;
    chassis_mode_publisher_.publish(chassis_mode_);
  }
  
  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ChassisExecutor::ExcutionMode GetExecutionMode(){
      return chassis_executor_->GetExecutionMode();
  }
  
  void UpdateGobalFrame(){
      // 使陀螺仪的坐标系与 map 坐标系重合
        float gyro_yaw = blackboard_->info.gyro_yaw;
        geometry_msgs::PoseStamped gimbal_pose = blackboard_->GetRobotGimbalMapPose();
        float gimbal_map_yaw = tf::getYaw(gimbal_pose.pose.orientation);
        relative_yaw_ = gyro_yaw - gimbal_map_yaw;
  }

  float AngleLimit(float yaw){
    yaw = yaw > M_PI  ? yaw - 2*M_PI : yaw;
    yaw = yaw < -M_PI ? yaw + 2*M_PI : yaw;
    return yaw;
  }
  
  void PublishChassisModeMsg(){
    if(blackboard_->info.is_searching ||  blackboard_->info.is_sentrying){
        chassis_mode_.chaiss_mode = roborts_msgs::ChassisMode::GYRO;
    }else if(blackboard_->CanStopSwing()){
        chassis_mode_.chaiss_mode = roborts_msgs::ChassisMode::CHAISSFOLLOW;
    }
    else{
        chassis_mode_.chaiss_mode = roborts_msgs::ChassisMode::SWING;
    }
    chassis_mode_publisher_.publish(chassis_mode_);
  }

  void SentryAction(float map_yaw){
    float relative_yaw = map_yaw - blackboard_->info.sentry_center_yaw;
    double base_roll ,pitch ,base_yaw ;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(blackboard_->GetRobotGimbalBasePose().pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(base_roll, pitch, base_yaw);

    float degree_20 = 0.34;
    float degree_10 = 0.17;
    float degree_45 = 0.785;
    float degree_6 = 0.1;     
    float degree_3 = 0.052; 

    pitch_direction = pitch >= degree_20 ? false : pitch_direction;    // 0.523 rad = 30 degree
    pitch_direction = pitch <= degree_10 ? true : pitch_direction;

    // pitch_direction : True mean down  , False mean up  
    if(pitch_direction){
         gimbal_angle_.pitch_angle = degree_3;
    }else{
        gimbal_angle_.pitch_angle = -degree_3;
    }
    gimbal_angle_.yaw_angle = -base_yaw;
    gimbal_angle_.pitch_mode = true;
  }

  void UpdateSentryCoordinate(float sentry_center_yaw){
    blackboard_->info.sentry_center_yaw = sentry_center_yaw;
  }

  ~GimbalControlBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  ros::NodeHandle nh_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::ChassisMode  chassis_mode_;
  ros::Publisher gimbal_publisher_;
  ros::Publisher chassis_mode_publisher_;

  double relative_yaw_;
  double last_target_yaw_;
  double was_hit_yaw_;
  geometry_msgs::Quaternion lost_enemy_quat_;

  bool init_;

  int lost_enemy_count_;
  int lost_enemy_threshold_;

  bool yaw_direction;
  bool pitch_direction;

};
}

#endif //ROBORTS_DECISION_DEFEND_BEHAVIOR_H
