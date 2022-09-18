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

#include "localization_node.h"

namespace roborts_localization{


/**
* @brief 定位构造函数,完成对定位的初始化
* @param name 节点名字
**/
LocalizationNode::LocalizationNode(std::string name) {
  CHECK(Init()) << "Module "  << name <<" initialized failed!";
  initialized_ = true;
}

/**
* @brief 定位初始化函数
* @return True if no errors
**/
bool LocalizationNode::Init() {

  // 参数类初始化
  LocalizationConfig localization_config;

  localization_config.GetParam(&nh_);
  /* 以下参数赋值使用的是localization_config.h参数文件赋值到localization_node.h里面的参数 */
  odom_frame_   = std::move(localization_config.odom_frame_id);
  global_frame_ = std::move(localization_config.global_frame_id);
  base_frame_   = std::move(localization_config.base_frame_id);
  laser_topic_ = std::move(localization_config.laser_topic_name);
  init_pose_ = {localization_config.initial_pose_x,
                localization_config.initial_pose_y,
                localization_config.initial_pose_a};
  init_cov_ = {localization_config.initial_cov_xx,
               localization_config.initial_cov_yy,
               localization_config.initial_cov_aa};
  transform_tolerance_  = ros::Duration(localization_config.transform_tolerance);
  publish_visualize_ = localization_config.publish_visualize;



  tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

  // 发布机器人在地图上的位置信息
  distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
  // 发布位姿粒子
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  
  // 使用消息过滤器的时间同步器(laser scan和tf之间的odom和base frame)
  laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);

  // 同步时间戳
  laser_scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                   *tf_listener_ptr_,
                                                                                   odom_frame_,
                                                                                   100);
  //  注册回调函数
  laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback, this, _1));

  uwb_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>>(nh_, "uwb", 2);
  uwb_sub_filter_ = std::make_unique<tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>>(*uwb_sub_,
                                                                                   *tf_listener_ptr_,
                                                                                   odom_frame_,
                                                                                   2);
  uwb_sub_filter_->registerCallback(boost::bind(&LocalizationNode::UwbCallback, this, _1));

  // 订阅初始化位姿话题
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);
  // 发布经过AMCL定位后的位姿信息
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 2, true);
  amcl_ptr_= std::make_unique<Amcl>();
  amcl_ptr_->GetParamFromRos(&nh_);
  amcl_ptr_->Init(init_pose_, init_cov_);

  // 全局静态地图初始化
  map_init_ = GetStaticMap();

  // 初始化雷达扫描位置信息
  laser_init_ = GetLaserPose();

  // 订阅uwb信息
  uwb_pose_sub_ = nh_.subscribe("uwb", 2, &LocalizationNode::UwbCallback,this);

  return map_init_&&laser_init_;
}

bool LocalizationNode::GetStaticMap(){
  // 客户端请求全局静态地图
  static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  // 等待地图服务
  ros::service::waitForService("static_map", -1);
  // 定义地图请求
  nav_msgs::GetMap::Request req;
  // 定义地图回复
  nav_msgs::GetMap::Response res;
  // 如果地图请求服务call成功了
  if(static_map_srv_.call(req,res)) {
    LOG_INFO << "Received Static Map";
    amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
    first_map_received_ = true;
    return true;
  } else{
    LOG_ERROR << "Get static map failed";
    return false;
  }
}


bool LocalizationNode::GetLaserPose() {
  // 等待激光雷达信息进来
  auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);
  // 定义用来储存空间X,Y,Z点位置的laser_pose
  Vec3d laser_pose;
  // 进行存之间先清零
  laser_pose.setZero();
  //获取目标位置的TF信息,并同步时间戳
  GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
  //不需要进行旋转,将Z置0
  laser_pose[2] = 0; // No need for rotation, or will be error
  DLOG_INFO << "Received laser's pose wrt robot: "<<
            laser_pose[0] << ", " <<
            laser_pose[1] << ", " <<
            laser_pose[2];
  // 将laser_pose传到SetLaserSensorPose里面
  amcl_ptr_->SetLaserSensorPose(laser_pose);

  return true;
}

void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg) {

  if (init_pose_msg->header.frame_id == "") {
    LOG_WARNING << "Received initial pose with empty frame_id.";
  } // 只接受全局框架中的初始姿态估计
  else if (tf_listener_ptr_->resolve(init_pose_msg->header.frame_id) !=
           tf_listener_ptr_->resolve(global_frame_)) {
    LOG_ERROR << "Ignoring initial pose in frame \" "
              << init_pose_msg->header.frame_id
              << "\"; initial poses must be in the global frame, \""
              << global_frame_;
    return;
  }

  // In case the client sent a pose estimate in the past, integrate the
  // intervening odometric change.
  //如果客户在过去发送了一个姿态估计，集成其间的里程数变化。
  tf::StampedTransform tx_odom;
  try {
    // 更新时间
    ros::Time now = ros::Time::now();
    // 等待TF变换
    tf_listener_ptr_->waitForTransform(base_frame_, 
                                       init_pose_msg->header.stamp,
                                       base_frame_,
                                       now,
                                       odom_frame_,
                                       ros::Duration(0.5));
    // 寻找可用的TF变换    
    // 在stamp时间下的base_frame_变换到now时间下的base_frame_,odom_frame_指的是不随时间变化的坐标系,tx_odom是变换结果保存的量.                    
    tf_listener_ptr_->lookupTransform(base_frame_,
                                      init_pose_msg->header.stamp,
                                      base_frame_,
                                      now,
                                      odom_frame_, tx_odom);
  }
  catch (tf::TransformException &e) {
    tx_odom.setIdentity();
  }
  tf::Pose pose_new;
  tf::Pose pose_old;
  tf::poseMsgToTF(init_pose_msg->pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // 转换到全局坐标系
  DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
            << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();
  //定义储存新位姿的vec
  Vec3d init_pose_mean;
  //定义储存位姿的协方差
  Mat3d init_pose_cov;
  init_pose_mean.setZero();
  init_pose_cov.setZero();
  double yaw, pitch, roll;
  init_pose_mean(0) = pose_new.getOrigin().x();
  init_pose_mean(1) = pose_new.getOrigin().y();
  pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
  init_pose_mean(2) = yaw;
  init_pose_cov = math::MsgCovarianceToMat3d(init_pose_msg->pose.covariance);
  
  amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);
}

void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr){

  last_laser_msg_timestamp_ = laser_scan_msg_ptr->header.stamp;

  Vec3d pose_in_odom;
  if(!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
  {
    LOG_ERROR << "Couldn't determine robot's pose";
    return;
  }

  double angle_min = 0 , angle_increment = 0;
  sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;
  TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);

  amcl_ptr_->Update(pose_in_odom,
                    laser_scan_msg,
                    angle_min,
                    angle_increment,
                    particlecloud_msg_,
                    hyp_pose_);

  LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";

  if(publish_visualize_){
    PublishVisualize();
  }

}

void LocalizationNode::UwbCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &uwb_info){
    Vec3d pose(uwb_info->pose.pose.position.x , uwb_info->pose.pose.position.y , 0.2);
    Vec3d cov(uwb_info->pose.covariance[0] ,uwb_info->pose.covariance[6+1] , 0);
    amcl_ptr_->UpdateUwb(pose,cov, hyp_pose_ ,uwb_info->header.stamp);
    LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";
    if(publish_visualize_){
        PublishVisualize();
    }
}

void LocalizationNode::PublishVisualize(){

  if(pose_pub_.getNumSubscribers() > 0){
    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.header.frame_id = global_frame_;
    pose_msg_.pose.position.x = hyp_pose_.pose_mean[0];
    pose_msg_.pose.position.y = hyp_pose_.pose_mean[1];
    pose_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(hyp_pose_.pose_mean[2]);
    pose_pub_.publish(pose_msg_);
  }

  if(particlecloud_pub_.getNumSubscribers() > 0){
    particlecloud_msg_.header.stamp = ros::Time::now();
    particlecloud_msg_.header.frame_id = global_frame_;
    particlecloud_pub_.publish(particlecloud_msg_);
  }

  if(!publish_first_distance_map_) {
    distance_map_pub_.publish(amcl_ptr_->GetDistanceMapMsg());
    publish_first_distance_map_ = true;
  }
}

//发布base/map/odom之间的TF变换
bool LocalizationNode::PublishTf() {
  ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
  if (amcl_ptr_->CheckTfUpdate()) {
    // Subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try {
      tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_.pose_mean[2]),
                           tf::Vector3(hyp_pose_.pose_mean[0],
                                       hyp_pose_.pose_mean[1],
                                       0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                           last_laser_msg_timestamp_,
                                           base_frame_);
      this->tf_listener_ptr_->transformPose(odom_frame_,
                                            tmp_tf_stamped,
                                            odom_to_map);
    } catch (tf::TransformException &e) {
      LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
      return false;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
    return true;
  } else if (latest_tf_valid_) {
    // 没有任何更改，所以我们将重新发布最后一个转换
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    return true;
  }
  else{
    return false;
  }
}
  /**
   * @brief 获取目标位置的TF信息,并同步时间戳
   */
bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,
                                     const std::string &source_frame,
                                     const ros::Time &timestamp,
                                     Vec3d &pose)
{
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              timestamp,
                              source_frame);
  tf::Stamped<tf::Pose> pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(target_frame,
                                          ident,
                                          pose_stamp);
  } catch (tf::TransformException &e) {
    LOG_ERROR << "Couldn't transform from "
              << source_frame
              << "to "
              << target_frame;
    return false;
  }

  pose.setZero();
  pose[0] = pose_stamp.getOrigin().x();
  pose[1] = pose_stamp.getOrigin().y();
  double yaw,pitch, roll;
  pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
  pose[2] = yaw;
  return true;
}

void LocalizationNode::TransformLaserscanToBaseFrame(double &angle_min,
                                                     double &angle_increment,
                                                     const sensor_msgs::LaserScan &laser_scan_msg) {

  // 为了考虑倒置安装的激光器，我们确定了激光器在基础框架中的最小、最大和增量角度。在base_link帧中构造激光的最小和最大角度。
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min
      + laser_scan_msg.angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);

  try {
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          min_q,
                                          min_q);
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          inc_q,
                                          inc_q);
  }
  catch (tf::TransformException &e) {
    LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
    return;
  }

  angle_min = tf::getYaw(min_q);
  angle_increment = (tf::getYaw(inc_q) - angle_min);

  // Wrapping angle to [-pi .. pi]
  angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);

}

}// roborts_localization

int main(int argc, char **argv) {
  roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "localization_node");
  roborts_localization::LocalizationNode localization_node("localization_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}

