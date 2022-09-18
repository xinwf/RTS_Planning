#ifndef ROBORTS_DECISION_SENTRY_BEHAVIOR_H
#define ROBORTS_DECISION_SENTRY_BEHAVIOR_H


#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "roborts_msgs/GimbalAngle.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <geometry_msgs/PoseStamped.h>

namespace roborts_decision {
class SentryBehavior {
 public:
  SentryBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard,
               const std::string & proto_file_path) :
      blackboard_(blackboard) {
        gimbal_angle_.pitch_angle = 0;
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.pitch_mode = true;
        gimbal_angle_.yaw_mode = true;
        gimbal_publisher_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100, this);
        yaw_direction = true;
        pitch_direction = true;
      }
      
  void Run() {
    geometry_msgs::PoseStamped gimbal_angle =  blackboard_->GetRobotGimbalBasePose();
    double roll ,pitch ,yaw ;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(gimbal_angle.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    float degree_10 = 0.174;
    float degree_45 = 0.785;
    float degree_6 = 0.1;     
    float degree_3 = 0.052; 

    yaw_direction = yaw >= degree_45 ? false : yaw_direction; // 0.785 rad = 45 degree
    yaw_direction = yaw <= -degree_45 ? true : yaw_direction; 
    pitch_direction = pitch >= degree_10 ? false : pitch_direction;    // 0.523 rad = 30 degree
    pitch_direction = pitch <= 0 ? true : pitch_direction;
    

    // pitch_direction : True mean left   , False mean right
    if(yaw_direction){
         gimbal_angle_.yaw_angle = degree_6;
    }else{
        gimbal_angle_.yaw_angle = -degree_6;
    }


    // pitch_direction : True mean down  , False mean up  
    if(pitch_direction){
         gimbal_angle_.pitch_angle = degree_3;
    }else{
        gimbal_angle_.pitch_angle = -degree_3;
    }
    gimbal_publisher_.publish(gimbal_angle_);
  }

  void Cancel() {
  }

  BehaviorState Update() {
  }

  ~SentryBehavior() = default;

 private:
  
  //! perception information
  Blackboard* const blackboard_;
  
  ros::NodeHandle nh_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  ros::Publisher gimbal_publisher_;

  bool yaw_direction;
  bool pitch_direction;
  
};
}

#endif //ROBORTS_DECISION_SENTRY_BEHAVIOR_H
