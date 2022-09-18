/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
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

#include "chassis.h"

namespace roborts_base{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle):
    Module(handle){
  SDK_Init();
  ROS_Init();
}
Chassis::~Chassis(){
  //如果主线程与子线程处于连接状态(true)的话,则等待heartbeat_thread_线程执行完才往下执行
  if(heartbeat_thread_.joinable()){
    heartbeat_thread_.join();
  }
  //同上
  if(chassis_thread_.joinable()){
      chassis_thread_.join();
  }
}
void Chassis::SDK_Init(){
  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  verison_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      ROS_INFO("Chassis Firmware Version: %d.%d.%d.%d",
                                               int(future.get()->version_id>>24&0xFF),
                                               int(future.get()->version_id>>16&0xFF),
                                               int(future.get()->version_id>>8&0xFF),
                                               int(future.get()->version_id&0xFF));
                                    });

  handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
                                                           CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                           std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                       std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                        MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  heartbeat_thread_ = std::thread([this]{
                                    roborts_sdk::cmd_heartbeat heartbeat;
                                    heartbeat.heartbeat=0;
                                    while(ros::ok()){
                                      heartbeat_pub_->Publish(heartbeat);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                    }
                                  }
  );
  chassis_thread_ = std::thread(&Chassis::ChassisControlThread,this);
  ros_sub_gimbal =  ros_nh_.subscribe<roborts_msgs::GimbalInfo>("gimbal_info", 100, &Chassis::UpdateGimbalDataCallBack, this);
  ros_sub_chassis_mode_ = ros_nh_.subscribe<roborts_msgs::ChassisMode>("chassis_mode", 100, &Chassis::ChassisModeCallBack ,this);
}

void Chassis::ROS_Init(){
  //ros publisher
  ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 100);
  ros_gimbal_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("gimbal_odom", 100);
//   ros_uwb_pub_ = ros_nh_.advertise<geometry_msgs::PoseStamped>("uwb", 30);
  ros_imu_pub_ = ros_nh_.advertise<sensor_msgs::Imu>("imu", 30);

  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
  ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);

  //ros_message_init
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_link";

  gimbal_odom_.header.frame_id = "odom";
  gimbal_odom_.child_frame_id = "gimbal_odom";
  gimbal_odom_tf_.header.frame_id = "odom";
  gimbal_odom_tf_.child_frame_id = "gimbal_odom";

  uwb_data_.header.frame_id = "uwb";
  imu_.header.frame_id = "base_link";
  chassis_mode_ = roborts_msgs::ChassisMode::CHAISSFOLLOW;
  
  chassis_spd_acc_.vx = 0 ;
  chassis_spd_acc_.vy = 0 ;
  chassis_spd_acc_.vw = 0;
  chassis_spd_acc_.ax = 0;
  chassis_spd_acc_.ay = 0;
  chassis_spd_acc_.wz = 0;
  chassis_spd_acc_.rotate_x_offset = 0;
  chassis_spd_acc_.rotate_y_offset = 0;
  back_motion_count_ = 0;
}
 /**
   * @brief 函数作用:该函数用来发布Odom与odom的tf信息,以及将底盘的odom转换到云台上(在扭腰的时候用gimbal的odom)
   * @brief 基于里程计要发送东西主要包括:1`odom坐标系到base_link坐标系里程变换(线位移和角位移的转换).
   * 2`nav_msgs/Odometry(线位移和角位移+线速度和角速度)
   * @param cmd_chassis_info roborts命名空间下的一个接收下位机信息的结构体类型
   */
void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info){
  //current_time为当前系统时间,用于odom的tf变换
  ros::Time current_time = ros::Time::now();
  odom_.header.stamp = current_time; //给odom的时间戳设置为当前时间
  odom_.pose.pose.position.x = chassis_info->position_x_mm/1000.;  //将机器人的x位置赋值给odom,注意单位转换
  odom_.pose.pose.position.y = chassis_info->position_y_mm/1000.; //将机器人的x位置赋值给odom,注意单位转换
  odom_.pose.pose.position.z = 0.0; //平面小车不需要Z轴方向的位置信息,置0
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI); //使用createQuaternionMsgFromYaw创建yaw轴方向信息(适用平面小车)
  odom_.pose.pose.orientation = q;  //将odom的方向信息置为由createQuaternionMsgFromYaw创建的yaw方向信息
  odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0; //将odom x方向的线速度置为机器人在x上的速度信息,注意单位转换
  odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0; //将odom y方向的线速度置为机器人在y上的速度信息,注意单位转换
  odom_.twist.twist.angular.z = chassis_info->gyro_rate / 1800.0 * M_PI;  //将odom 的z轴角速度置为陀螺仪获取的角速度,注意单位转换
  ros_odom_pub_.publish(odom_);  //将odom发布出去_

  /* odom_tf_这块是表示从odom(fram_id)到base_link(child_frame_id)的转换 */
  odom_tf_.header.stamp = current_time;//给odom的tf转换时间戳置为当前时间
  odom_tf_.transform.translation.x = chassis_info->position_x_mm/1000.0;// 将odom的上x的tf变换置为机器人的x位置
  odom_tf_.transform.translation.y = chassis_info->position_y_mm/1000.0;
  odom_tf_.transform.translation.z = 0.0;//z轴没有变换,置0
  odom_tf_.transform.rotation = q; //旋转变换置为yaw方向角度
  tf_broadcaster_.sendTransform(odom_tf_);//广播odom到base_link的转换

  // gimbal odom  将底盘速度转换成云台坐标系的速度
  gimbal_odom_.header.stamp = current_time;
  gimbal_odom_.pose.pose.position.x = chassis_info->position_x_mm/1000.;
  gimbal_odom_.pose.pose.position.y = chassis_info->position_y_mm/1000.;
  gimbal_odom_.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion gimbal_q = tf::createQuaternionMsgFromYaw(gimbal_yaw_);
  gimbal_odom_.pose.pose.orientation = gimbal_q;
  gimbal_odom_.twist.twist.linear.x =  chassis_info->v_x_mm / 1000.0 * cos(relative_yaw_) + chassis_info->v_y_mm / 1000.0 * sin(relative_yaw_);
  gimbal_odom_.twist.twist.linear.y =  -chassis_info->v_x_mm / 1000.0 * sin(relative_yaw_) + chassis_info->v_y_mm / 1000.0 * cos(relative_yaw_);
  gimbal_odom_.twist.twist.angular.z = gimbal_yaw_rate_;
  ros_gimbal_odom_pub_.publish(gimbal_odom_);
  gimbal_odom_tf_.header.stamp = current_time;
  gimbal_odom_tf_.transform.translation.x = 0;
  gimbal_odom_tf_.transform.translation.y = 0;
  gimbal_odom_tf_.transform.translation.z = 0.0;
  gimbal_odom_tf_.transform.rotation = gimbal_q;
  tf_broadcaster_.sendTransform(gimbal_odom_tf_);

  imu_.header.stamp = current_time;

  imu_.orientation = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI);
  ros_imu_pub_.publish(imu_);

}
/**
 * @brief UWB信息回调处理(官方停止使用UWB,该部分无效)
 * 
 * @param uwb_info  接受下位机发送的uwb信息
 */
void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info){
  uwb_data_.header.stamp = ros::Time::now();
  uwb_data_.pose.position.x = ((double)uwb_info->x)/100.0;
  uwb_data_.pose.position.y = ((double)uwb_info->y)/100.0;
  uwb_data_.pose.position.z = 0;
  uwb_data_.pose.orientation = tf::createQuaternionMsgFromYaw(uwb_info->yaw/ 180.0 * M_PI);
//   ros_uwb_pub_.publish(uwb_data_);

}
/**
 * @brief 底盘速度控制回调函数(由上位机控制机器人)
 * 
 * @param vel 
 */
void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel){
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x*1000*cos(relative_yaw_) - vel->linear.y*1000*sin(relative_yaw_) ;
  chassis_speed.vy = vel->linear.x*1000*sin(relative_yaw_) + vel->linear.y*1000*cos(relative_yaw_);
  chassis_speed.vw = 0;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}
/**
 * @brief 底盘加速度控制回调函数(由上位机控制机器人)
 * 
 * @param vel_acc 
 */
void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc){
  chassis_spd_acc_.vx = vel_acc->twist.linear.x*1000*cos(relative_yaw_) - vel_acc->twist.linear.y*1000*sin(relative_yaw_) ;
  chassis_spd_acc_.vy = vel_acc->twist.linear.x*1000*sin(relative_yaw_) + vel_acc->twist.linear.y*1000*cos(relative_yaw_);
  chassis_spd_acc_.vw = vel_acc->twist.angular.z * 1800.0 / M_PI;
  chassis_spd_acc_.ax = vel_acc->accel.linear.x*1000*cos(relative_yaw_) - vel_acc->accel.linear.y*1000*sin(relative_yaw_) ;
  chassis_spd_acc_.ay = vel_acc->accel.linear.x*1000*sin(relative_yaw_) + vel_acc->accel.linear.y*1000*cos(relative_yaw_);
  chassis_spd_acc_.wz = vel_acc->accel.angular.z * 1800.0 / M_PI;
}
/**
 * @brief 用于更新云台数据的回调函数
 * 
 * @param gimbal_info 
 */
void Chassis::UpdateGimbalDataCallBack(const roborts_msgs::GimbalInfo::ConstPtr &gimbal_info){
    gimbal_yaw_rate_ = gimbal_info->yaw_rate/ 1800.0 * M_PI;
    gimbal_yaw_ = gimbal_info->gyro_yaw/1800.0 * M_PI;
    relative_yaw_ = gimbal_info->ecd_yaw/1800.0 * M_PI;
}


void Chassis::ChassisModeCallBack(const roborts_msgs::ChassisMode::ConstPtr &chassis_mode){
    chassis_mode_ = chassis_mode->chaiss_mode;
}
/**
 * @brief 判断是否为反向运动模式
 * 
 * @return true 
 * @return false 
 */
bool Chassis::IsBackMotion(){
    if(chassis_spd_acc_.vx < 0){
        return true;
        // back_motion_count_ ++;
    }
    if(back_motion_count_ >= 3){
        back_motion_count_ = 0;
        return true;
    }else{
        return false;
    }
}
/**
 * @brief 底盘控制线程
 * 
 */
void Chassis::ChassisControlThread(){
    ros::Rate rate(100);
    while(ros::ok()){
        float swing_twist = 0;
        float center_angle = 0.52;
        //如果反向运动模式为true,则设置底盘模式为扭腰模式
        if(IsBackMotion()){
            chassis_mode_ = roborts_msgs::ChassisMode::SWING;
        }
        switch(chassis_mode_){
            //扭腰模式
            case (roborts_msgs::ChassisMode::SWING) :
                //  yaw_direction_ : True mean chaiss left rotating  , False mean chaiss right rotating
                //如(relative_yaw_ >=  center_angle)为true,则为true,反之为false
                yaw_direction_ = relative_yaw_ >=  center_angle   ? true : yaw_direction_;
                yaw_direction_ = relative_yaw_ <=  -center_angle  ? false : yaw_direction_; 
                if(std::fabs(relative_yaw_) > center_angle + 15/180*M_PI){
                    swing_twist = 3* 1800.0 / M_PI * sin(relative_yaw_);
                }else{
                    if(yaw_direction_){
                        swing_twist = 2.0* 1800.0 / M_PI * cos(relative_yaw_/2);
                    }else{
                        swing_twist = -2.0 * 1800.0 / M_PI * cos(relative_yaw_/2);
                    }
                }
                chassis_spd_acc_.vw = swing_twist;
                break;
            //底盘跟随模式
            case (roborts_msgs::ChassisMode::CHAISSFOLLOW) :
                swing_twist = 3 * 1800.0 / M_PI* sin(relative_yaw_);
                chassis_spd_acc_.vw = swing_twist ;
                break;
            //陀螺仪模式
            case (roborts_msgs::ChassisMode::GYRO) :
                swing_twist = 3 * 1800.0 / M_PI;
                chassis_spd_acc_.vw = swing_twist ;
                break;
            default : break;
        }
        chassis_spd_acc_pub_->Publish(chassis_spd_acc_);
        rate.sleep();
    }
}

}
