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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// roborts_msgs 
#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/ArmorsPos.h"

#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/GameRobotHP.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/LurkStatus.h"
#include "roborts_msgs/ShootMode.h"
#include "roborts_msgs/AllyPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "roborts_msgs/Target.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/CmdAim.h"
#include "roborts_msgs/GimbalInfo.h"
#include "roborts_msgs/MyArmor.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

// communication.h
#include "communication.h"


// wifi part
#include <zmq.hpp>
#include <unistd.h>
#include <thread>



namespace roborts_decision{


struct Threshold{
        float near_dist;
        float near_angle;
        float heat_upper_bound;
        float detect_dist;
        float shoot_distance;

};

struct DecisionInfoPool{
        int remaining_time;
        int game_status;
        int shoot_delay_ms;
        bool is_begin;
        bool is_master;
        bool team_blue;
        bool has_ally;
        bool has_my_enemy;
        bool has_ally_enemy;
        bool has_first_enemy;
        bool has_second_enemy;
        bool can_shoot;
        bool is_supplying;
        bool is_shielding;
        bool is_sentrying;
        bool is_searching;
        bool got_last_enemy;
        bool use_refree;

        // ally enemy part
        bool has_ally_first_enemy;
        bool has_ally_second_enemy;
        //enemy hp 
        float emeny_first_hp;
        float emeny_second_hp;
        //enemy bullet
        float  emeny_first_bullet;
        float  emeny_second_bullet;
        //buff
        bool bullet_buff_active;
        bool shield_buff_active;

        bool is_hitted;
        bool is_chase;
        // for lurk 
        int lurk_mode;              // 是否进入潜伏模式
        bool is_aim_target_armor;  // 潜伏模式下是否瞄准了目标装甲板
        bool ready_for_lurk;       // 自己是否准备好进入潜伏模式
        bool ally_ready_for_lurk; // 队友是否准备好进入潜伏模式
        bool ally_is_aim_target_armor;
        bool has_identify_our_undercover;
        int lurk_switch_mode;    // 潜伏模式的两种模式
        int undercover_id;
        roborts_msgs::CmdAim  cmd_aim;  
        roborts_msgs::CmdAim  ally_cmd_aim;

        // bool in_dodge;
        bool valid_camera_armor;
        int remain_bullet;
        int remain_hp;
        int frequency;
        float speed;
        float gyro_yaw;
        float sentry_center_yaw;
        int ally_remain_bullet;
        int ally_remain_hp;
        int heat;
        int hit_source;
        int damage_type;
        double ally_dist;
        double first_enemy_dist;
        double second_enemy_dist;
        float ally_yaw;
        std::string strategy;
        geometry_msgs::PoseStamped ally;
        geometry_msgs::PoseStamped first_enemy;
        geometry_msgs::PoseStamped second_enemy;
        // ally enemy part
        geometry_msgs::PoseStamped ally_first_enemy;
        geometry_msgs::PoseStamped ally_second_enemy;
        // all goal
        geometry_msgs::PoseStamped my_goal;
        geometry_msgs::PoseStamped ally_goal;
        geometry_msgs::PoseStamped toward_goal;

        geometry_msgs::PoseStamped last_enemy;

        //reload 子弹区   shiled 补血
        geometry_msgs::PoseStamped my_reload;
        geometry_msgs::PoseStamped my_shield;
        geometry_msgs::PoseStamped opp_reload;
        geometry_msgs::PoseStamped opp_shield;
        geometry_msgs::PoseStamped start_position;

};


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      is_in_shoot_state(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      context_(1),guardcontext_(1),heatbeat_context_(1),
      connect_guard(false),
      connect_ally(false),
      use_camera(false),
      use_camera_first_enemy_pose_(false),
      use_camera_second_enemy_pose_(false),
      guard_has_first_enemy_(false),
      guard_has_second_enemy_(false),
      guard_has_my_pose_(false),
      aim_can_shooting_(false),
      map_offset_x(0),
      map_offset_y(0),
      gyro_relative_map_(0),
      map_yaw_(0),
      clear_vel_(true),
      faile_count_(0),
      masterSocket_(context_, ZMQ_REP), 
      clientSocket_(context_, ZMQ_REQ),
      guardSocket_(guardcontext_,ZMQ_SUB),
      heatbeatSocket_(heatbeat_context_,ZMQ_PUB){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
    
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    // node handle
    ros::NodeHandle nh;
    // armor_info
    camera_armor_sub_ = nh.subscribe<roborts_msgs::ArmorsPos>("front_camera_robot_pos", 10, &Blackboard::FrontCameraArmorCallback, this);  // Front Camera        
    //referee_system
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 10, &Blackboard::RobotStatusCallback, this);
    robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat", 10, &Blackboard::RobotHeatCallback, this);
    robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 10, &Blackboard::RobotShootCallback, this);
    game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status", 10, &Blackboard::GameStatusCallback, this);
    robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage", 30, &Blackboard::RobotDamageCallback, this);
    game_zone_sub_ = nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 10, &Blackboard::GameZoneCallback, this);
    emeny_hp_sub_ = nh.subscribe<roborts_msgs::GameRobotHP>("game_robot_hp", 30, &Blackboard::EnemyHpCallback, this);
    emeny_bullet_sub_ = nh.subscribe<roborts_msgs::GameRobotBullet>("game_robot_bullet", 30, &Blackboard::EnemyBulletCallback, this);
    lurk_mode_sub_ = nh.subscribe<roborts_msgs::LurkStatus>("game_lurk_status",30,&Blackboard::LurkModeCallback, this);
    toward_goal_sub_ = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point",30,&Blackboard::TowardGoalCallback, this);
    //car_info
    cmd_gimbal_sub_ = nh.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 10, &Blackboard::CmdGimbalCallback, this);
    gimbal_info_sub_ = nh.subscribe<roborts_msgs::GimbalInfo>("gimbal_info", 50 ,&Blackboard::GimbalInfoCallback,this);
    // pub
    cmd_vel_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 10, this);
    cmd_aim_pub_ = nh.advertise<roborts_msgs::CmdAim>("emeny_target_id",30,this);
    pose_init_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,this);
    ally_position_pub_ = nh.advertise<roborts_msgs::AllyPose>("ally_pose",50,this);
    my_armor_pub_ = nh.advertise<roborts_msgs::MyArmor>("my_armor",2,this);
    clear_buff_area_pub_ = nh.advertise<roborts_msgs::BuffClearIndex>("buff_index",10,this);

    // all frame ID update
    std::string tf_prefix = tf::getPrefixParam(nh);
    pose_frameID = "base_link";
    gimbal_frameID = "gimbal";
    if (!tf_prefix.empty()) {
        pose_frameID = tf::resolve(tf_prefix, pose_frameID);
        gimbal_frameID = tf::resolve(tf_prefix, gimbal_frameID);
    }
    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    use_camera = decision_config.use_camera();

    // initialize setting----------------------------------------------------------
    info.remaining_time = 299;
    info.shoot_delay_ms = decision_config.shoot_delay_ms();
    info.use_refree = decision_config.use_refree();
    info.is_begin = false;
    info.is_master = decision_config.master();
    info.team_blue = decision_config.isblue();
    info.can_shoot = decision_config.can_shoot();
    info.has_ally = false;
    info.has_my_enemy = false;
    info.has_ally_enemy = false;
    info.has_first_enemy = false;
    info.has_second_enemy = false;
    info.has_ally_first_enemy = false;
    info.has_ally_second_enemy = false;
    info.is_hitted = false;
    info.is_chase = false;
    info.is_supplying = false;
    info.is_shielding = false;
    info.is_sentrying = false;
    info.is_searching = false;
    info.damage_type = 6;
    info.got_last_enemy = false;

    // for lurk aim init 
    info.lurk_mode = roborts_msgs::LurkStatus::lurk_status_normal;
    info.ready_for_lurk = false;
    info.ally_ready_for_lurk = false;
    info.lurk_switch_mode = LurkSwitchMode::EXCHANGED_SAME_ID;
    info.has_identify_our_undercover = false;
    
    // cmd aim
    info.cmd_aim.my_color = info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
    info.cmd_aim.target_armor.id = roborts_msgs::Armor::ONE;
    info.cmd_aim.target_armor.color = info.team_blue ? roborts_msgs::Armor::RED : roborts_msgs::Armor::BLUE;
    info.cmd_aim.mask_armor.id = roborts_msgs::Armor::NONE;
    info.cmd_aim.mask_armor.color = info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
    info.cmd_aim.lurk_mode = false;
    info.cmd_aim.must_aim_target = false;
  
    info.valid_camera_armor = false;
    info.remain_bullet = decision_config.remain_bullet();
    info.remain_hp = 2000;
    remain_hp = info.remain_hp;
    info.ally_remain_bullet = 50;
    info.ally_remain_hp = 2000;
    info.first_enemy_dist = 0;
    info.second_enemy_dist = 0;
    info.emeny_first_hp = 2000;
    info.emeny_second_hp = 2000;
    info.ally_dist = 0;
    info.heat = 0;
    info.frequency = 0;
    info.speed = 0.0;
    info.sentry_center_yaw = 0.0;
    info.strategy = decision_config.strategy();

    // goal passport
    info.my_goal = InitMapPose();
    info.ally_goal = InitMapPose();

    //buff init
    info.my_shield = InitMapPose();
    info.my_reload = InitMapPose();
    info.opp_reload = InitMapPose();
    info.opp_shield = InitMapPose();

    // PostStamped information update
    info.ally = InitMapPose();
    info.ally_first_enemy = InitMapPose();
    info.ally_second_enemy = InitMapPose();
    info.last_enemy = InitMapPose();
    first_enemy_guard_pose_ = InitMapPose();
    second_enemy_guard_pose_ = InitMapPose();
    map_offset_x = decision_config.map_offset_x();
    map_offset_y = decision_config.map_offset_y();
    // save enemy armor 
    int enemy_color = info.team_blue ? roborts_msgs::Armor::RED : roborts_msgs::Armor::BLUE;
    SaveEnemyArmor(roborts_msgs::Armor::ONE , enemy_color);
    SaveEnemyArmor(roborts_msgs::Armor::TWO , enemy_color);
    int my_color = info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
    int my_id = info.is_master ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
    int ally_id =  info.is_master ? roborts_msgs::Armor::TWO : roborts_msgs::Armor::ONE;
    SaveOurArmor("my_armor" , my_id , my_color);
    SaveOurArmor("ally_armor" , ally_id ,my_color);


    // glass obstacle  position init 
    // b1
    glass_obstacle[0] = {
        Pos_xy {7.08f + (float)map_offset_x,1.2f + (float)map_offset_y},
        Pos_xy {8.08f + (float)map_offset_x,1.2f + (float)map_offset_y},
        Pos_xy {8.08f + (float)map_offset_x,1.0f + (float)map_offset_y},
        Pos_xy {7.08f + (float)map_offset_x,1.0f + (float)map_offset_y}
    };
    // b3
    glass_obstacle[1] = {
        Pos_xy {6.38f + (float)map_offset_x,4.48f + (float)map_offset_y},
        Pos_xy {6.58f + (float)map_offset_x,4.48f + (float)map_offset_y},
        Pos_xy {6.58f + (float)map_offset_x,3.48f + (float)map_offset_y},
        Pos_xy {6.38f + (float)map_offset_x,3.48f + (float)map_offset_y}
    };
    // b7
    glass_obstacle[2] = {
        Pos_xy {1.50f + (float)map_offset_x,1.0f + (float)map_offset_y},
        Pos_xy {1.70f + (float)map_offset_x,1.0f + (float)map_offset_y},
        Pos_xy {1.70f + (float)map_offset_x,1.0f + (float)map_offset_y},
        Pos_xy {1.50f + (float)map_offset_x,0.0f + (float)map_offset_y}
    };
    // b7
    glass_obstacle[3] = {
        Pos_xy {0.0f + (float)map_offset_x,3.48f + (float)map_offset_y},
        Pos_xy {1.0f + (float)map_offset_x,3.48f + (float)map_offset_y},
        Pos_xy {1.0f + (float)map_offset_x,3.28f + (float)map_offset_y},
        Pos_xy {0.0f+ (float)map_offset_x,3.28f + (float)map_offset_y}

    };




    // initalize threshold----------------------------------------------------------------------
    threshold.near_dist = 0.5;  // m
    threshold.near_angle = 0.2;  // rad ~ 10 degree
    threshold.detect_dist = 5.0;  // m
    threshold.heat_upper_bound = 160; // limit
    threshold.shoot_distance = 1.8;

    // initialize enemy
    _front_first_enemy = false;
    _front_second_enemy = false;
    my_shoot_1_cnt = 0;
    my_shoot_2_cnt = 0;
    ally_shoot_1_cnt = 0;
    ally_shoot_2_cnt = 0;

    if (use_camera){
      ROS_INFO("Armor detection module wait for  connect!");
      armor_detection_actionlib_client_.waitForServer();
      ROS_INFO("Armor detection module  connected success!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
      UpdateAimCmd();
    }


    // load all shield reload points
    if (info.team_blue){     
        info.start_position = info.is_master? Point2PoseStamped(decision_config.blue().master_bot().start_position()):Point2PoseStamped(decision_config.blue().wing_bot().start_position());
        info.start_position.pose.orientation = info.is_master ? tf::createQuaternionMsgFromYaw(decision_config.blue().master_bot().start_position().yaw())
                                                              : tf::createQuaternionMsgFromYaw(decision_config.blue().wing_bot().start_position().yaw());
        info.first_enemy  = Point2PoseStamped(decision_config.red().master_bot().start_position());
        info.second_enemy =  Point2PoseStamped(decision_config.red().wing_bot().start_position());
    }else{
        info.start_position = info.is_master? Point2PoseStamped(decision_config.red().master_bot().start_position()): Point2PoseStamped(decision_config.red().wing_bot().start_position());
        info.start_position.pose.orientation = info.is_master ? tf::createQuaternionMsgFromYaw(decision_config.red().master_bot().start_position().yaw())
                                                              : tf::createQuaternionMsgFromYaw(decision_config.red().wing_bot().start_position().yaw());
        info.first_enemy  = Point2PoseStamped(decision_config.red().master_bot().start_position());
        info.second_enemy =  Point2PoseStamped(decision_config.red().wing_bot().start_position());
    }
    if(decision_config.auto_init_pose()){
        InitMyPose(info.start_position);
    }

    info.toward_goal = info.second_enemy;

    // use wifi
    if (decision_config.usewifi()){
        std::string master_ip;
        ros::param::get("master_ip" ,master_ip);
        if (info.is_master){
            // std::string master_ip = nh.param::get("guard_ip");
            std::string port = "";
            int length = master_ip.length();
            for (int i=length-1; i>=0; i--){
                if (master_ip[i] != ':')
                    port = master_ip[i] + port;
                else
                    break; 
            }
            port = "tcp://*:"  + port;
            masterSocket_.bind(port);
            masterThread = std::thread(&Blackboard::CommunicateMaster, this);
        }
        else{
            clientSocket_.connect(master_ip);
            clientThread = std::thread(&Blackboard::CommunicateClient, this);
        }
        std::string guard_ip;
        ros::param::get("guard_ip" ,guard_ip);
        // guardSocket_.connect(decision_config.guard_ip());
        guardSocket_.connect(guard_ip);
        guardSocket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        guardThread = std::thread(&Blackboard::CommunicateGuard,this);

        heatbeatSocket_.bind("tcp://*:5555");
        heatbeatThread = std::thread(&Blackboard::HeatBeatPublish,this);
    }
  }

    ~Blackboard(){
        if(masterThread.joinable()){
            masterThread.join();
        }
        if(clientThread.joinable()){
            clientThread.join();
        }
        if(guardThread.joinable()){
            guardThread.join();
        }
        if(heatbeatThread.joinable()){
            heatbeatThread.join();
        }
    }


    // supply request
    void SupplyRequest(int number){
        info.is_supplying = true;
    }
    void PrintfVaildArmor(){
        if(vaild_armors.size()> 0 ){
            for(auto armor = vaild_armors.begin(); armor != vaild_armors.end() ; armor++){
                std::cout << "id :  " << (int)armor->id << "color : " << (int)armor->color <<std::endl;
            }
        }

    }

    void Shoot(int shoot_delay_ms){
        if (!is_in_shoot_state && info.can_shoot){
            is_in_shoot_state = true;
            shoot_thread = std::thread(&Blackboard::_ShootThread, this, shoot_delay_ms);
        }
    }

    void StartFirWhl(){
        armor_detection_goal_.command = roborts_msgs::ShootMode::STARTFRICWHl;
        armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
    }

    void StopFirWhl(){
        armor_detection_goal_.command = roborts_msgs::ShootMode::STOPFRICWHl;
        armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
    }

    void StopShoot(){
        if (is_in_shoot_state){
            is_in_shoot_state = false;
            shoot_thread.join();
            armor_detection_goal_.command = roborts_msgs::ShootMode::STARTFRICWHl ;
            armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
        }  
    }

// 逻辑判断选择击打目标敌人
geometry_msgs::PoseStamped GetEnemy() {  // const: Can not introduce New Arguments
    geometry_msgs::PoseStamped cur_pose = GetRobotMapPose();
    geometry_msgs::PoseStamped enemyPose;
    unsigned int shoot_1_cnt = my_shoot_1_cnt + ally_shoot_1_cnt;
    unsigned int shoot_2_cnt = my_shoot_2_cnt + ally_shoot_2_cnt;
    bool get_new_enemy = true;
    // 2. fixed  1 enemy and attack
    // The different mark
    if (info.has_first_enemy && info.has_second_enemy){
        int enemy_id = ChooseEnemyFromTow();
        if(enemy_id == roborts_msgs::Armor::ONE){
            enemyPose = ChooseEnemyCount(roborts_msgs::Armor::ONE);
        }else{
            enemyPose = ChooseEnemyCount(roborts_msgs::Armor::TWO);
        } 
    }
    else if (info.has_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = ChooseEnemyCount(roborts_msgs::Armor::ONE);
        }
        else{
            enemyPose = ChooseEnemyCount(roborts_msgs::Armor::TWO);
        }
    }
    else if (info.has_second_enemy  && info.has_ally_first_enemy ){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            ChooseEnemyCount(roborts_msgs::Armor::ONE);
        }
        else{
            enemyPose = info.second_enemy;
            info.last_enemy = enemyPose;
            ChooseEnemyCount(roborts_msgs::Armor::TWO);
        }
    }
    else if (info.has_ally_first_enemy && info.has_ally_second_enemy){
        if (shoot_1_cnt >= shoot_2_cnt){
            enemyPose = info.ally_first_enemy;
            info.last_enemy = enemyPose;
            ChooseEnemyCount(roborts_msgs::Armor::ONE);
        }
        else{
            enemyPose = info.ally_second_enemy;
            info.last_enemy = enemyPose;
            ChooseEnemyCount(roborts_msgs::Armor::TWO);
        }
    }
    // Then, my first enemy
    else if (info.has_first_enemy){
        enemyPose = ChooseEnemyCount(roborts_msgs::Armor::ONE);
    }
    else if (info.has_second_enemy){
        enemyPose = ChooseEnemyCount(roborts_msgs::Armor::TWO);
    }
    else if (info.has_ally_first_enemy){
        enemyPose = info.ally_first_enemy;
        info.last_enemy = enemyPose;
    }
    else if (info.has_ally_second_enemy){
        enemyPose = info.ally_second_enemy;
        info.last_enemy = enemyPose;
    }
    else {
        if(GuardEnemyDetected()){
            if(guard_has_first_enemy_ && guard_has_second_enemy_){
                int enemy_id = ChooseEnemyFromTow();
                if(enemy_id == roborts_msgs::Armor::ONE){
                    enemyPose = info.first_enemy;
                }else{
                    enemyPose = info.second_enemy;
                } 
            }else{
                enemyPose = guard_has_first_enemy_ ? info.first_enemy : info.second_enemy;
            }
        }
        else{
            get_new_enemy = false;
            enemyPose = info.last_enemy;
        }
    } 
    enemyPose.pose.orientation = GetRelativeQuaternion(enemyPose, cur_pose);
    if(get_new_enemy){
        info.last_enemy = enemyPose;
        info.got_last_enemy = true;
    }
    UpdateAimCmd();
    return enemyPose;
  }

  geometry_msgs::PoseStamped ChooseEnemyCount(int enemy_id){
    geometry_msgs::PoseStamped enemy_pose;
    if(enemy_id == roborts_msgs::Armor::ONE){
        if(CanShoot()){
            my_shoot_1_cnt++; 
        }
        info.cmd_aim.target_armor = first_enemy_armor_;
        enemy_pose = info.first_enemy;
    }
    else{
        if(CanShoot()){
            my_shoot_2_cnt++;
        }
        info.cmd_aim.target_armor = second_enemy_armor_;
        enemy_pose = info.second_enemy;
    }
    info.last_enemy = enemy_pose;
    return enemy_pose;
  }

  int ChooseEnemyFromTow(){
    geometry_msgs::PoseStamped cur_pose = GetRobotMapPose();
    info.first_enemy_dist = GetDistance(cur_pose,info.first_enemy);
    info.second_enemy_dist = GetDistance(cur_pose,info.second_enemy);
    int enemy_id;
    // if(info.first_enemy_dist > threshold.detect_dist || info.second_enemy_dist > threshold.detect_dist)
    // {   
    //         //有一个超范围则选择最近距离
    //         enemy_id = info.first_enemy_dist <= info.second_enemy_dist ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
    //     }
        // else{
            if(info.emeny_first_hp < 400 || info.emeny_second_hp < 400  ){
                // 都不超范围则选择若有一个低血量则选择最低血量
                enemy_id = info.emeny_first_hp <= info.emeny_second_hp ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
            }
            else{
                if(info.emeny_first_bullet < 0 || info.emeny_second_bullet < 0){
                // 都不超范围且无低血量若有无子弹的车则选择该车
                    enemy_id = info.emeny_first_bullet <= info.emeny_first_bullet ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
                }
                else{
                    // 都不超范围且无血量最低的车且都有子弹则选择较低血量的车
                    enemy_id = info.emeny_first_hp <= info.emeny_second_hp ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
                }
            }      
    // }
    return enemy_id;
  }

  bool CanShoot(){
       if (_gimbal_can 
           && aim_can_shooting_                   
           && info.heat < threshold.heat_upper_bound   // heater bound
           && !IsBlockByGlass(valid_armor_id_)
           ){     //valid_camera_armor
            return true;
           }
       else{
           StopShoot();
           return false;
       }
  }

  bool HasIdentifyUndercover(){
      return (info.cmd_aim.mask_armor.id == info.ally_cmd_aim.mask_armor.id)
            &&(info.cmd_aim.mask_armor.color == info.ally_cmd_aim.mask_armor.color);
  }

  bool HasEnemyInThisArea(geometry_msgs::PoseStamped area) {
      if(GetDistance(info.first_enemy , area) < 1.0 
        || GetDistance(info.second_enemy , area) < 1.0){
          return true;
      }else{
          return false;
      }
  }

  bool BulletBuffHasGrayCar(){
      if((bullet_index_ == 1  && F1_has_gary_car_)
         || (bullet_index_ == 6  && F6_has_gary_car_)){
          return true;
      }else{
          return false;
      }
  }
  
  bool ShieldBuffHasGrayCar(){
      if((shield_index_ == 1  && F1_has_gary_car_)
         || (shield_index_ == 6  && F6_has_gary_car_)){
          return true;
      }else{
          return false;
      }
  }

  bool IsEnemyDetected() const{
    // ROS_INFO("%s: %d", __FUNCTION__, (int)info.valid_camera_armor);
    return info.valid_camera_armor;
  }

  bool CanCollaborateWithAlly() const{
    return info.lurk_mode == LurkSwitchMode::EXCHANGED_SAME_ID;
  }

  bool GuardEnemyDetected() const{
      return connect_guard && (guard_has_first_enemy_||guard_has_second_enemy_);
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

   geometry_msgs::PoseStamped GetMyGoal() const {
    return info.my_goal;
  }
  
  void SetMyGoal(geometry_msgs::PoseStamped goal){
      info.my_goal = goal;
  }
  
  void SetMyToward(geometry_msgs::PoseStamped toward_goal){
      info.toward_goal = toward_goal;
  }
  
  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }


  bool IsBombAllyGoal(const geometry_msgs::PoseStamped goal){
      auto distance = GetDistance(goal, info.ally_goal);
      return distance < threshold.near_dist && !info.is_master;
  }

  bool allyInThisArea(const geometry_msgs::PoseStamped target){
          //   是否有单位存在 该目标点中
        geometry_msgs::PoseStamped  others;
        others = info.ally;
        double dist;
        dist = GetDistance(target, others);
        if (dist <= threshold.near_dist) return true;
        else return false;

  }


  bool IsBombAllyGoal(const float x, const float y){
      auto distance = GetEulerDistance(x, y, info.ally_goal.pose.position.x, info.ally_goal.pose.position.y);
      return distance < threshold.near_dist;
  }

  bool IsInStuckArea(){
      geometry_msgs::PoseStamped curPose = GetRobotMapPose();
      double yaw = tf::getYaw(GetRobotGimbalMapPose().pose.orientation);
      float cur_x = curPose.pose.position.x, cur_y = curPose.pose.position.y;
      float x[4], y[4];
      float deg_30 = 30 / 180 * 3.1415926;
      float d = 0.025;
      x[0] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw + deg_30)));
      y[0] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw + deg_30)));
      x[1] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw - deg_30)));
      y[1] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw - deg_30)));
      x[2] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw + deg_30)));
      y[2] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw + deg_30)));
      x[3] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw - deg_30)));
      y[3] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw - deg_30)));
      
      unsigned int mx, my;
      for (int i=0; i<4; i++){
          GetCostMap2D()->World2Map(x[i], y[i], mx, my);
          if (GetCostMap2D()->GetCost(mx, my) < 253){
            if(!clear_vel_){
                    roborts_msgs::TwistAccel tw;
                    tw.twist.linear.x = 0;
                    tw.twist.linear.y = 0;
                    // has sent cmd_vel
                    cmd_vel_pub_.publish(tw);
                    clear_vel_ = true;
            }
            return false;
          }
      }
        
      // In stuck area
      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d/sqrt2, 0, d/sqrt2, d, d/sqrt2, 0, -d/sqrt2};
      double int_y[] = {0, -d/sqrt2, -d, d/sqrt2, 0, d/sqrt2, d, -d/sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i=0; i<=7; i++){
            c_x = cur_x + int_x[i];
            c_y = cur_y + int_y[i];
            GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
            cost = GetCostMap2D()->GetCost(u_x, u_y);
            if (cost >= 253){
                cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
                cur_angle = cur_angle >0? cur_angle : cur_angle + 2 * 3.1415926;
                acc_angle += cur_angle;
                count++;
            }   
      }
      acc_angle = acc_angle / count;
      acc_angle = acc_angle - 3.1415926;
      acc_angle = acc_angle < -3.1415926 ? acc_angle + 2 * 3.1415926 : acc_angle;

      // in case of rotate in a roll.
      c_x = cur_x + d * std::cos(acc_angle);
      c_y = cur_y + d * std::sin(acc_angle);
      GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
      cost = GetCostMap2D()->GetCost(u_x, u_y);
      if (cost >=253){
          acc_angle = acc_angle >0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
      }
    //   printf("acc_angle: %f\n", acc_angle * 180 / 3.14);

      // get yaw
      acc_angle = acc_angle - yaw;
      double vx, vy;
      vx = cos(acc_angle);
      vy = sin(acc_angle);

      roborts_msgs::TwistAccel tw;
      tw.twist.linear.x = vx;
      tw.twist.linear.y = vy;
      // has sent cmd_vel
      cmd_vel_pub_.publish(tw);
      clear_vel_ = false;
      return true;
  }

    /******************** Callback *******************/
    // fake enemy Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  
    // Front Camera Armor Detection
  void FrontCameraArmorCallback(const roborts_msgs::ArmorsPos::ConstPtr &armors){
    temp_armors.clear();
    aim_can_shooting_ = armors->aim_can_shoot;
    info.is_aim_target_armor = armors->is_aim_target_armor;
    // 是否使用 车载摄像头内的地方坐标
    bool camera_first_enemy_pose = false;
    bool camera_second_enemy_pose = false;
    if (armors->num_armor == 0){
        _front_first_enemy = false;
        _front_second_enemy = false;
        camera_first_enemy_pose = false;
        camera_second_enemy_pose = false;

    }
    else{
        valid_armor_id_ = armors ->valid_armor_id;
        geometry_msgs::PoseStamped enemy_pose , global_pose_msg;
        tf::Stamped<tf::Pose> temp_tf_pose ,global_tf_pose;
        enemy_pose.header.frame_id = gimbal_frameID;
        // Set up valid set
        std::set<int> valid_id_set; 
        for (int i=0; i<armors->num_armor; i++){
            roborts_msgs::Armor armor;
            armor = armors->vaild_armors[i];
            temp_armors.push_back(armor);
            valid_id_set.insert(armor.id);
            double dx,dy;
            // if (dx<=0 || dx>=8 || dy<=0 || dy>=5)
            //     continue;
            if (armor.id == roborts_msgs::Armor::ONE)  _front_first_enemy = true;
            else  _front_second_enemy = true;
            // 近距离或者哨岗探测不到的情况下 保存自瞄下的敌方坐标系  mm -> m 
            enemy_pose.pose.position.x = armor.position.x/1000;;
            enemy_pose.pose.position.y = armor.position.y/1000;;
            temp_tf_pose.stamp_ = ros::Time(0);
            try
            {
                poseStampedMsgToTF(enemy_pose, temp_tf_pose);
                tf_ptr_->transformPose("map", temp_tf_pose, global_tf_pose);
                if(_front_first_enemy && dx < 1.0){
                    tf::poseStampedTFToMsg(global_tf_pose,info.first_enemy);
                    camera_first_enemy_pose = true;
                    info.first_enemy = PoseLimit(info.first_enemy);
                }else if(_front_second_enemy && dx < 1.0){
                    tf::poseStampedTFToMsg(global_tf_pose,info.second_enemy);
                    camera_second_enemy_pose = true;
                    info.second_enemy = PoseLimit(info.second_enemy);
                }
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("tf error when transform enemy pose from camera to map");
            }
        }
        // Clear invalid set
        for (int id=1; id<3; id++){
            if (valid_id_set.find(id) == valid_id_set.end()){
                if (id == 1) _front_first_enemy = false;
                else if (id == 2) _front_second_enemy = false;
            }
        }
        vaild_armors.assign(temp_armors.begin(),temp_armors.end());
        valid_id_set.clear();
    }
    info.valid_camera_armor = armors->valid_front_camera_armor;
    info.has_first_enemy = _front_first_enemy;
    info.has_second_enemy = _front_second_enemy;
    info.has_my_enemy = info.has_first_enemy || info.has_second_enemy;
    use_camera_first_enemy_pose_ = camera_first_enemy_pose;
    use_camera_second_enemy_pose_ = camera_second_enemy_pose;
  }



    void GimbalInfoCallback(const roborts_msgs::GimbalInfo::ConstPtr &msg){
        info.gyro_yaw = msg->gyro_yaw/ 1800.0 * M_PI;
        map_yaw_ = msg->gyro_yaw - gyro_relative_map_;
    }
  
    // Game Status call back
    void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &msg){
        info.remaining_time = msg->remaining_time;
        info.is_begin = bool(msg->game_status == roborts_msgs::GameStatus::GAME);
        info.game_status = msg->game_status;
    }

    void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr& damage) {
            info.damage_type = damage->damage_type;
            info.hit_source = damage->damage_source;
        }

    // Cmd Gimbal call back
    void CmdGimbalCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){
        bool yaw_mode = msg->yaw_mode;
        bool pitch_mode = msg->pitch_mode;
        float yaw_angle = msg->yaw_angle;
        float pitch_angle = msg->pitch_angle;
        if (yaw_mode && std::abs(yaw_angle) <= 0.2){
            _gimbal_can = true;
        }
        else{
            _gimbal_can = false;
        }
        // gimbal pitch
        // printf("yaw_mode:%d yaw_angle:%f, pitch_mode:%d, pitch_angle:%f\n", yaw_mode, yaw_angle, pitch_mode, pitch_angle);
    }

    // Robot status call back
    void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &rb_status){
        if(info.remain_hp > rb_status->remain_hp){
            info.is_hitted = true;
            last_hurt_time_ = std::chrono::system_clock::now();
        }else{
            info.is_hitted = false;
        }
        info.remain_hp = rb_status->remain_hp;
        remain_hp = info.remain_hp;
    }
  
    // Robot Heat call back
    void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr &rh){
        info.heat = rh->shooter_heat;
    }

    // Robot Shoot call back
    void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr &msg){
        info.frequency = msg->frequency;
        info.speed = msg->speed;
    }
    
    // buff zone x y call back 
    void GameZoneCallback(const roborts_msgs::GameZoneArrayConstPtr& zone) {
        geometry_msgs::PoseStamped blue_bullet_point , blue_shield_point, red_bullet_point, red_shield_point;
        bool blue_bullet_buff_active,blue_shield_buff_active,red_bullet_buff_active,red_shield_buff_active;
        auto blue_bullet = roborts_msgs::GameZone::BLUE_BULLET_SUPPLY;
        auto red_bullet = roborts_msgs::GameZone::RED_BULLET_SUPPLY;
        auto blue_shield = roborts_msgs::GameZone::BLUE_HP_RECOVERY;
        auto red_shield = roborts_msgs::GameZone::RED_HP_RECOVERY;
        int blue_bullet_index = -1 ,blue_shield_index = -1;
        int red_bullet_index = -1,red_shield_index = -1;

        // F1 ~ F6
        float buff_point[12] = {7.78, 1.852, 6.567, 3.049, 4.30, 0.61, 4.3008, 4.161, 2.284, 1.85, 0.746, 3.159};
        
        for (int m = 0; m < 6; m++) {
            if (zone->zone[m].type == blue_bullet) {
                blue_bullet_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2],false);
                blue_bullet_buff_active  = zone->zone[m].active;
                blue_bullet_index = m + 1 ; 
            }
            if (zone->zone[m].type == blue_shield) {
                blue_shield_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2],false);
                blue_shield_buff_active  = zone->zone[m].active;
                blue_shield_index = m + 1 ; 
            }
            if (zone->zone[m].type == red_bullet) {
                red_bullet_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2],false);
                red_bullet_buff_active  = zone->zone[m].active;
                red_bullet_index = m + 1 ; 
            }
            if (zone->zone[m].type == red_shield) {
                red_shield_point = Point2PoseStamped(buff_point[0 + m*2],buff_point[1 + m*2],false);
                red_shield_buff_active  = zone->zone[m].active;
                red_shield_index = m + 1 ; 
            }
        }
        if(info.team_blue){
            info.my_reload = blue_bullet_point;
            info.my_shield = blue_shield_point;
            info.opp_reload = red_bullet_point;
            info.opp_shield = red_shield_point;
            info.bullet_buff_active = blue_bullet_buff_active;
            info.shield_buff_active = blue_shield_buff_active;
            bullet_index_ = blue_bullet_index;
            shield_index_ = blue_shield_index;
        }
        else{
            info.my_reload = red_bullet_point;
            info.my_shield = red_shield_point;
            info.opp_reload = blue_bullet_point;
            info.opp_shield = blue_shield_point;
            info.bullet_buff_active = red_bullet_buff_active;
            info.shield_buff_active = red_shield_buff_active;
            bullet_index_ = red_bullet_index;
            shield_index_ = red_shield_index;
        }
  }


  void EnemyHpCallback(const roborts_msgs::GameRobotHPConstPtr& hp){
    int ally_hp;
    if(info.team_blue){
      info.emeny_first_hp = hp-> red1;
      info.emeny_second_hp = hp-> red2;
      ally_hp = info.is_master ? hp-> blue2  : hp-> blue1;
    }
    else{
      info.emeny_first_hp = hp-> blue1;
      info.emeny_second_hp = hp-> blue2;
      ally_hp = info.is_master ? hp-> red2  : hp-> red1;
    }
    if(ally_hp <= 0){
        info.has_ally = false;
        info.has_ally_enemy = false;
    }
  }

  void EnemyBulletCallback(const roborts_msgs::GameRobotBulletConstPtr& bullet){
    if(info.team_blue){
        info.emeny_first_bullet = bullet-> red1;
        info.emeny_second_bullet = bullet-> red2;
        info.remain_bullet = info.is_master ? bullet-> blue1 : bullet-> blue2;
    }
    else{
        info.emeny_first_bullet = bullet-> blue1;
        info.emeny_second_bullet = bullet-> blue2;
        info.remain_bullet = info.is_master ? bullet-> red1 : bullet-> red2;
    }
  }

  // Enemy  messages are always zero for enemy.
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
      return;
  }

  void TowardGoalCallback(const  geometry_msgs::PointStampedConstPtr& Point){
      info.toward_goal.pose.position.x = Point->point.x;
      info.toward_goal.pose.position.y = Point->point.y;
      return;
  }

  void LurkModeCallback(const roborts_msgs::LurkStatusConstPtr& lurkmode){
    info.lurk_mode = lurkmode->lurk_mode;
  }


    /******************** tool *******************/
    geometry_msgs::Quaternion GetRelativeQuaternion(const geometry_msgs::PoseStamped to,const geometry_msgs::PoseStamped from){
        double dy, dx;
        dy = to.pose.position.y - from.pose.position.y;
        dx = to.pose.position.x - from.pose.position.x;
        double yaw = std::atan2(dy, dx);    
        return tf::createQuaternionMsgFromYaw(yaw);
    }

    geometry_msgs::Quaternion GetRelativeQuaternion(const double to_x, const double to_y, const geometry_msgs::PoseStamped from){
        double dx, dy;
        dy = to_y - from.pose.position.y;
        dx = to_x - from.pose.position.x;
        double yaw = std::atan2(dy, dx);
        return tf::createQuaternionMsgFromYaw(yaw);
    }

    double GetDistance(const geometry_msgs::PoseStamped &pose1,
        const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Point point1 = pose1.pose.position;
        const geometry_msgs::Point point2 = pose2.pose.position;
        const double dx = point1.x - point2.x;
        const double dy = point1.y - point2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double GetEulerDistance(const float x1, const float y1, const float x2, const float y2){
        return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    }

    double GetAngle(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
        const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(quaternion1, rot1);
        tf::quaternionMsgToTF(quaternion2, rot2);
        return rot1.angleShortestPath(rot2);
    }

    geometry_msgs::PoseStamped Point2PoseStamped(Point point){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.pose.position.x = point.x() + map_offset_x;
        ps.pose.position.y = point.y() + map_offset_y;
        ps.pose.position.z = point.z();
        tf::Quaternion q = tf::createQuaternionFromRPY(point.roll(),
                                                    point.pitch(),
                                                    point.yaw());
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        return ps;
    }

    geometry_msgs::PoseStamped Point2PoseStamped(float x , float y){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = x + map_offset_x;
        ps.pose.position.y = y + map_offset_y;
        return ps;
    }

    geometry_msgs::PoseStamped Point2PoseStamped(float x , float y , bool need_offset){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = ros::Time::now();
        if(need_offset){
            ps.pose.position.x = x + map_offset_x;
            ps.pose.position.y = y + map_offset_y;
        }else{
            ps.pose.position.x = x;
            ps.pose.position.y = y;
        }

        return ps;
    }

    geometry_msgs::PoseStamped PoseLimit(geometry_msgs::PoseStamped pose){
        pose.pose.position.x = pose.pose.position.x > 8.0 + map_offset_x ? 8.0 + map_offset_x : pose.pose.position.x;
        pose.pose.position.x = pose.pose.position.x < map_offset_x ? map_offset_x : pose.pose.position.x;
        pose.pose.position.y = pose.pose.position.y > 4.2 + map_offset_y ? 4.2 + map_offset_y : pose.pose.position.y;
        pose.pose.position.y = pose.pose.position.y < map_offset_y ? map_offset_y : pose.pose.position.y;     
        return pose;
    }

    void SaveEnemyArmor(int id , int color){
        switch(id){
            case roborts_msgs::Armor::ONE : first_enemy_armor_.color = color; break;
            case roborts_msgs::Armor::TWO : second_enemy_armor_.color = color; break;
            default : break;
        }
    }

    void SaveOurArmor(std::string name , int id , int color){
        if(!name.compare("my_armor")){
            my_armor_.id = id;
            my_armor_.color = color;
        }else{
            ally_armor_.id = id;
            ally_armor_.color = color;
        }
    }

    void SaveRelativeYaw(float relative_yaw){
        gyro_relative_map_ = relative_yaw;
    }

    void InitMyPose(geometry_msgs::PoseStamped position){
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        init_pose.header.stamp = ros::Time::now();
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.position =  position.pose.position;
        init_pose.pose.pose.orientation = position.pose.orientation;
        pose_init_pub_.publish(init_pose);
    }
    
    bool VaildHasTargetArmor(roborts_msgs::Armor target_armor){
        if(vaild_armors.size()> 0){
            for(auto armor = vaild_armors.begin(); armor != vaild_armors.end(); armor++){
                    if(armor->id == target_armor.id  && armor->color == target_armor.color){
                        return true;
                    }
                }
        }
 
        return false;
    }

    // 以 pose 为中心，与两个敌人之间的夹角角度大于 90 则背后有敌人
    bool HasEnemyBehind(geometry_msgs::PoseStamped point){
        // return false;
        if((guard_has_first_enemy_ && guard_has_second_enemy_ )){
            float ef_distance = GetDistance(GetRobotMapPose() , first_enemy_guard_pose_);
            float es_distance = GetDistance(GetRobotMapPose() , first_enemy_guard_pose_);
            if(ef_distance > 2.0 || es_distance > 2.0){
                return false;
            }
            float angle = GetInAngle(point , first_enemy_guard_pose_ , second_enemy_guard_pose_);
            if(angle > M_PI / 0.75){
                return true ;
            }else{
                return false;
            }
        }else{
            return false;
        }
    }

    bool IsSurroundEnemy(geometry_msgs::PoseStamped enemy_point , geometry_msgs::PoseStamped target_point){
        float angle = 0 ;
        if(info.has_ally){
            angle = GetInAngle(enemy_point , info.ally , target_point);
        }
        return (info.has_ally && angle > M_PI/2) || !info.is_master || !info.has_ally;
    }



    void UpdateAimCmd(){
      cmd_aim_pub_.publish(info.cmd_aim);
    }

    void ClearBulletBuffArea(){
        roborts_msgs::BuffClearIndex msg;
        msg.buff_index = bullet_index_;
        clear_buff_area_pub_.publish(msg);
    }

    void ClearShieldBuffArea(){
        roborts_msgs::BuffClearIndex msg;
        msg.buff_index = shield_index_;
        clear_buff_area_pub_.publish(msg);
    }

    void RebuildBuffArea(){
        roborts_msgs::BuffClearIndex msg;
        msg.buff_index = -1;
        clear_buff_area_pub_.publish(msg);
    }

    roborts_msgs::Armor GetAllyArmor(){
        return ally_armor_;
    }

    roborts_msgs::Armor GetMyArmor(){
        return my_armor_;
    }

    const bool IsBlockByGlass(){
        return is_block_by_glass_;
    }

    const geometry_msgs::PoseStamped GetRobotMapPose() {
        UpdateRobotPose();
        return self_pose_;
    }

    const bool GuardHasMyPose(){
        return guard_has_my_pose_;
    }

    const float GetGimbalMapYaw(){
        return map_yaw_;
    }

    const geometry_msgs::PoseStamped GetGuardFirstEnemyPose(){
        return first_enemy_guard_pose_;
    }

    const geometry_msgs::PoseStamped GetGuardSecondEnemyPose(){
        return second_enemy_guard_pose_;
    }
    
    const geometry_msgs::PoseStamped GetMyGuardPose() {
        return my_guard_pose_;
    }

    const geometry_msgs::PoseStamped GetRobotGimbalMapPose(){
        return UpdateRobotGimbalPose("map");
    }

    const geometry_msgs::PoseStamped GetRobotGimbalBasePose(){
        return UpdateRobotGimbalPose(pose_frameID);
    }

    const std::shared_ptr<CostMap> GetCostMap(){
        return costmap_ptr_;
    }

    const CostMap2D* GetCostMap2D() {
        return costmap_2d_;
    }

    const unsigned char* GetCharMap() {
        return charmap_;
    }



    bool IsBlockByGlass(int enemy_id){
        // 判断是否被玻璃板遮挡
        bool is_block = false;
        switch(enemy_id){
            case roborts_msgs::Armor::ONE:
                is_block = IsBlockByGlass(GetRobotMapPose(),info.first_enemy);
                break;
            case roborts_msgs::Armor::TWO:
                is_block = IsBlockByGlass(GetRobotMapPose(),info.second_enemy);
                break;
            default:
                break;
        }
        is_block_by_glass_ = is_block;
        return is_block;
    }

    bool CanStopSwing(){
        if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_hurt_time_).count()*0.001 >= 5){
            return true;
        }else{
            return false;
        }
    }

    bool IsBlockByGlass(geometry_msgs::PoseStamped pose_from , geometry_msgs::PoseStamped pose_to){
        Pos_xy point_from , point_to;
        point_from.x = pose_from.pose.position.x;
        point_from.y = pose_from.pose.position.y;
        point_to.x = pose_to.pose.position.x;
        point_to.y = pose_to.pose.position.y;
        for(int i = 0 ;i < 4; ++i){
                //判断线段是否在矩形内
            if(checkIfOutsideobstacles(glass_obstacle[i].left_up ,glass_obstacle[i].right_down ,point_to)){            
                //判断线段是否与矩形的两条对角线相交
                if(Intersect(glass_obstacle[i].left_up ,
                                glass_obstacle[i].right_down,
                                point_from,
                                point_to)){
                    return true;
                }
                if(Intersect(glass_obstacle[i].left_down ,
                                glass_obstacle[i].right_up,
                                point_from,
                                point_to)){
                    return true;
                }
            }
        }
        return false;
    }

    //A,C为对角线的两点
    bool checkIfOutsideobstacles(Pos_xy A, Pos_xy C, Pos_xy M) {
        if(std::min(A.x, C.x) <= M.x 
          && M.x <= std::max(A.x, C.x) 
          && std::min(A.y, C.y) <= M.y 
          && M.y <= std::max(A.y, C.y)){
            return false;
        }
        return true;
    }
     //  计算BA叉乘CA；
    double Cross_Prouct(Pos_xy A,Pos_xy B,Pos_xy C){  
        return (B.x-A.x)*(C.y-A.y)-(B.y-A.y)*(C.x-A.x);   //向量叉积运算   
    }  

    bool Intersect(Pos_xy A,Pos_xy B,Pos_xy C,Pos_xy D){  //  通过叉乘判断线段是否相交；
    if(std::min(A.x,B.x)<=std::max(C.x,D.x)&&            //  快速排斥实验；  
       std::min(C.x,D.x)<=std::max(A.x,B.x)&&  
       std::min(A.y,B.y)<=std::max(C.y,D.y)&&  
       std::min(C.y,D.y)<=std::max(A.y,B.y)&&  
       Cross_Prouct(A,B,C)*Cross_Prouct(A,B,D)<0&&      //  跨立实验；  
       Cross_Prouct(C,D,A)*Cross_Prouct(C,D,B)<0)       //  叉乘异号表示在两侧；  
       return true;  
    else return false;  
    }

    // return rad 
    float GetInAngle(geometry_msgs::PoseStamped center_point , 
                            geometry_msgs::PoseStamped point1 , 
                            geometry_msgs::PoseStamped point2){
        float t = (point1.pose.position.x - center_point.pose.position.x)
                * (point2.pose.position.x - center_point.pose.position.x)
                + (point1.pose.position.y - center_point.pose.position.y)
                * (point2.pose.position.y - center_point.pose.position.y);
        float len1 = std::fabs((point1.pose.position.x - center_point.pose.position.x) * (point1.pose.position.x - center_point.pose.position.x));
        float len2 = std::fabs((point1.pose.position.y - center_point.pose.position.y) * (point1.pose.position.y - center_point.pose.position.y));
        float len3 = std::fabs((point2.pose.position.x - center_point.pose.position.x) * (point2.pose.position.x - center_point.pose.position.x));
        float len4 = std::fabs((point2.pose.position.y - center_point.pose.position.y) * (point2.pose.position.y - center_point.pose.position.y));
        float angle = acos(t / std::sqrt( (len1+len2) * (len3+len4) ));
        return angle;
    }


    DecisionInfoPool info;
    Threshold threshold;

 private:
    ComInfo setComInfo(){
        ComInfo ci;
        ci.hp = info.remain_hp;
        ci.shoot_1 = my_shoot_1_cnt;
        ci.shoot_2 = my_shoot_2_cnt;
        ci.has_enemy = info.has_my_enemy;
        ci.bullet = info.remain_bullet;
        ci.pose_x = self_pose_.pose.position.x;
        ci.pose_y = self_pose_.pose.position.y;
        ci.yaw = tf::getYaw(self_pose_.pose.orientation);

        //是否看见第一个敌人
        ci.first_valid = info.has_first_enemy;
        ci.first_enemy_x = info.first_enemy.pose.position.x;
        ci.first_enemy_y = info.first_enemy.pose.position.y;
        //是否看见第二个敌人
        ci.second_valid = info.has_second_enemy;
        ci.second_enemy_x = info.second_enemy.pose.position.x;
        ci.second_enemy_y = info.second_enemy.pose.position.y;
        
        ci.goal_x = info.my_goal.pose.position.x;
        ci.goal_y = info.my_goal.pose.position.y;
        ci.mask_armor = info.cmd_aim.mask_armor;
        ci.ready_for_lurk = info.ready_for_lurk;
        ci.is_aim_target_armor = info.is_aim_target_armor;
        return ci;
    }

    void getComInfo(ComInfo ci){  
        info.ally_remain_hp = ci.hp;
        info.ally_remain_bullet = ci.bullet;
        info.ally.header.stamp = ros::Time::now();
        info.ally.pose.position.x = ci.pose_x;
        info.ally.pose.position.y = ci.pose_y;
        info.ally.pose.position.z = 0;
        info.ally.pose.orientation = tf::createQuaternionMsgFromYaw(ci.yaw);
        info.has_ally_first_enemy = ci.first_valid;
        if (ci.first_valid){ 
            info.ally_first_enemy.pose.position.x = ci.first_enemy_x;
            info.ally_first_enemy.pose.position.y = ci.first_enemy_y;
        }
        info.has_ally_second_enemy = ci.second_valid;
        if (ci.second_valid) {
            info.ally_second_enemy.pose.position.x = ci.second_enemy_x;
            info.ally_second_enemy.pose.position.y = ci.second_enemy_y;
        }
        info.ally_goal.pose.position.x = ci.goal_x;
        info.ally_goal.pose.position.y = ci.goal_y;
        info.has_ally_enemy = ci.has_enemy;
        // 同色同号情况 不接受队友的敌方信息
        if(info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID){
            info.has_ally_enemy = false;
            info.has_ally_first_enemy = false;
            info.has_ally_second_enemy = false;
        }
        info.ally_cmd_aim.mask_armor = ci.mask_armor;
        info.ally_ready_for_lurk = ci.ready_for_lurk;
        info.ally_is_aim_target_armor = ci.is_aim_target_armor;
        // ally shoot cnt
        ally_shoot_1_cnt = ci.shoot_1;
        ally_shoot_2_cnt = ci.shoot_2;
    }

    void getGuardInfo( CarPositionSend gi){
        // cm -> m
        if(info.remaining_time < 43 && info.is_begin){
            info.lurk_switch_mode = gi.lurkswitchmode;
            if(gi.undercover_id < 0){
                faile_count_ ++;
            }else{
                info.undercover_id = gi.undercover_id;   
                info.has_identify_our_undercover = true;
            }
            if(faile_count_ > 500){
                info.undercover_id = 1;   //随机 1号
                info.has_identify_our_undercover = true;
            }
        }


        // F1 和 F6 buff区域是否有灰车车辆
        F1_has_gary_car_ = gi.F1;
        F6_has_gary_car_ = gi.F6;
        bool has_first_enemy = false , has_second_enemy = false , has_my_pose = false;
        if(info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID){
            if(!info.has_ally){
                // 同色同号情况下，获取与友方装甲板一样的敌方
                switch(ally_armor_.color){
                    case roborts_msgs::Armor::RED :
                        if(ally_armor_.id == roborts_msgs::Armor::ONE && gi.red1.x > 0 && gi.red1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            }
                        }else if(ally_armor_.id == roborts_msgs::Armor::TWO && gi.red2.x > 0 && gi.red2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            }
                        }
                        break;
                    case roborts_msgs::Armor::BLUE :
                        if(ally_armor_.id == roborts_msgs::Armor::ONE && gi.blue1.x > 0 && gi.blue1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            }
                        }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.blue2.x > 0 && gi.blue2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            }
                        }
                        break;
                }

                 // 同色同号情况下，获取与自身装甲板一样的敌方
                switch(my_armor_.color){
                    case roborts_msgs::Armor::RED :
                        if(my_armor_.id == roborts_msgs::Armor::ONE && gi.red1.x > 0 && gi.red1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            }
                        }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.red2.x > 0 && gi.red2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            }
                        }
                        break;
                    case roborts_msgs::Armor::BLUE :
                        if(my_armor_.id == roborts_msgs::Armor::ONE && gi.blue1.x > 0 && gi.blue1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            }
                        }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.blue2.x > 0 && gi.blue2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            }
                        }
                        break;
                }
            }else{
                // 同色同号情况下，获取与自身装甲板一样的敌方
                switch(my_armor_.color){
                    case roborts_msgs::Armor::RED :
                        if(my_armor_.id == roborts_msgs::Armor::ONE && gi.red1.x > 0 && gi.red1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                            }
                        }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.red2.x > 0 && gi.red2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                            }
                        }
                        break;
                    case roborts_msgs::Armor::BLUE :
                        if(my_armor_.id == roborts_msgs::Armor::ONE && gi.blue1.x > 0 && gi.blue1.y > 0){
                            has_first_enemy = true;
                            first_enemy_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            if(!use_camera_first_enemy_pose_){
                                info.first_enemy = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                            }
                        }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.blue2.x > 0 && gi.blue2.y > 0){
                            has_second_enemy = true;
                            second_enemy_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            if(!use_camera_second_enemy_pose_){
                                info.second_enemy = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                            }
                        }
                        break;
                }
            }
            
        }
        // 非同色同号模式下，正常获取信息
        else{
            switch(first_enemy_armor_.color){
                case roborts_msgs::Armor::RED : 
                    if(gi.red1.x > 0 && gi.red1.y > 0){
                        has_first_enemy = true;
                        first_enemy_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                        if(!use_camera_first_enemy_pose_){
                            info.first_enemy = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                        }
                    }
                    break;
                case roborts_msgs::Armor::BLUE :
                    if(gi.blue1.x > 0 && gi.blue1.y > 0){
                        has_first_enemy = true;
                        first_enemy_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                        if(!use_camera_first_enemy_pose_){
                            info.first_enemy = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                        }
                    }
                    break;
                default :
                    ROS_WARN("enemy armor color must Red Or Blue");
                    break;
            }
            switch(second_enemy_armor_.color){
                case roborts_msgs::Armor::RED : 
                    if(gi.red2.x > 0 && gi.red2.y > 0){
                        has_second_enemy = true;
                        second_enemy_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                        if(!use_camera_second_enemy_pose_){
                            info.second_enemy = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                        }
                    }
                    break;
                case roborts_msgs::Armor::BLUE :
                    if(gi.blue2.x > 0 && gi.blue2.y > 0){
                        has_second_enemy = true;
                        second_enemy_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                        if(!use_camera_second_enemy_pose_){
                            info.second_enemy = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                        }
                    }
                    break;
                default : 
                    ROS_WARN("enemy armor color must Red Or Blue");
                    break;
            }
        }
        guard_has_first_enemy_ = has_first_enemy;
        guard_has_second_enemy_ = has_second_enemy;


        // 获取自己和队友的坐标
        switch(my_armor_.color){
            case roborts_msgs::Armor::RED :
                if(my_armor_.id == roborts_msgs::Armor::ONE && gi.red1.y > 0){
                    has_my_pose = true;
                    my_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.red2.y > 0) {  
                    has_my_pose = true;
                    my_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                }
                break;
            case roborts_msgs::Armor::BLUE :
                if(my_armor_.id == roborts_msgs::Armor::ONE && gi.blue1.y > 0){
                    has_my_pose = true;
                    my_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.blue2.y > 0) {  
                    has_my_pose = true;
                    my_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                }
                break;
        }
        guard_has_my_pose_ = has_my_pose;
        switch(ally_armor_.color){
            case roborts_msgs::Armor::RED :
                if(my_armor_.id == roborts_msgs::Armor::ONE && gi.red1.y > 0){
                    ally_guard_pose_ = Point2PoseStamped(gi.red1.y/100 , gi.red1.x/100,true);
                }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.red2.y > 0) {  
                    ally_guard_pose_ = Point2PoseStamped(gi.red2.y/100 , gi.red2.x/100,true);
                }
                break;
            case roborts_msgs::Armor::BLUE :
                if(my_armor_.id == roborts_msgs::Armor::ONE && gi.blue1.y > 0){
                    ally_guard_pose_ = Point2PoseStamped(gi.blue1.y/100 , gi.blue1.x/100,true);
                }else if(my_armor_.id == roborts_msgs::Armor::TWO && gi.blue2.y > 0) {  
                    ally_guard_pose_ = Point2PoseStamped(gi.blue2.y/100 , gi.blue2.x/100,true);
                }
                break;
        }
        
  }

  // timeit shoot bullet control
    void _ShootThread(int shoot_delay_ms){
        ros::Rate loop(60);
        auto start = std::chrono::system_clock::now();
        while (ros::ok()){
            // shoot once command
            if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() > shoot_delay_ms){
                armor_detection_goal_.command = roborts_msgs::ShootMode::SHOOTONCES;
                armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
                start = std::chrono::system_clock::now();
            }
            if (! is_in_shoot_state)
                break;
            loop.sleep();   
        }
  }

  // master and client communication function
    void CommunicateMaster(){
        ComInfo ci;
        ros::Rate loop(50);
        while (ros::ok()){
            zmq::message_t rec_message;
            masterSocket_.recv(&rec_message);
            memcpy(&ci, rec_message.data(), sizeof(ci));
            getComInfo(ci);
            roborts_msgs::AllyPose ally_msg;
            ally_msg.x = ci.pose_x;
            ally_msg.y = ci.pose_y;
            ally_position_pub_.publish(ally_msg);   
            //---------------------------------------------------------------------------------------------------------------------------------------
            ci = setComInfo();
            zmq::message_t send_message(sizeof(ci));
            memcpy(send_message.data(), &ci, sizeof(ci));
            
            masterSocket_.send(send_message);
            info.has_ally = true; 
            loop.sleep();
        }
    }

  void CommunicateClient(){
    ComInfo ci;
    ros::Rate loop(50);
    while (ros::ok()){
        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));    
        clientSocket_.send(send_message);
        //----------------------------------------------------------------------------------------------------------------------------------------
        zmq::message_t rec_message;
        clientSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);    
        roborts_msgs::AllyPose ally_msg;
        ally_msg.x = ci.pose_x;
        ally_msg.y = ci.pose_y;
        ally_msg.yaw = tf::getYaw(info.ally.pose.orientation);
        ally_position_pub_.publish(ally_msg);  
        info.has_ally = true;
        loop.sleep();       
    }
  }

    // communicate guard function
    void CommunicateGuard(){
        CarPositionSend gi;
        ros::Rate loop(60);
        int lost_count = 0;
        while (ros::ok()){
            zmq::message_t rec_message(sizeof(CarPositionSend));
            if(!guardSocket_.recv(&rec_message , ZMQ_DONTWAIT)){
                lost_count ++ ;
            }else{
                memcpy(&gi, rec_message.data(), sizeof(gi));
                while (guardSocket_.recv(&rec_message , ZMQ_DONTWAIT)) {
                    memcpy(&gi, rec_message.data(), sizeof(gi));
                }
                getGuardInfo(gi);  
                connect_guard = true; 
                lost_count = 0;     
            }
            if(lost_count > 100){
                guard_has_first_enemy_ = false;
                guard_has_second_enemy_ = false;
                connect_guard = false;
                lost_count = 0;
            }
            loop.sleep();       
        }
    }


    void HeatBeatPublish(){
        HeatBeat heatbeat;
        ros::Rate loop(100);
        while (ros::ok()){
            auto my_pose = GetRobotMapPose();
            geometry_msgs::PoseStamped guard_pose_msg;
            tf::Stamped<tf::Pose> temp_tf_pose ,global_tf_pose;
            try
            {
                poseStampedMsgToTF(my_pose, temp_tf_pose);
                tf_ptr_->transformPose("guard_link", temp_tf_pose, global_tf_pose);
                 tf::poseStampedTFToMsg(global_tf_pose,guard_pose_msg);
                zmq::message_t send_message(sizeof(HeatBeat));
                heatbeat.my_pose = Pos_xy{
                    static_cast<float>(guard_pose_msg.pose.position.x), 
                    static_cast<float>(guard_pose_msg.pose.position.y)};
                heatbeat.has_ally = info.has_ally;
                memcpy(send_message.data(),&heatbeat,sizeof(HeatBeat));
                if((info.remaining_time < 50 && info.is_begin) || !info.use_refree){
                    heatbeatSocket_.send(send_message);
                }
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("tf error when transform my pose from map to guard_link");
            }
            roborts_msgs::MyArmor myarmor_msg;
            myarmor_msg.my_armor = GetMyArmor();
            myarmor_msg.lurk_mode = info.lurk_switch_mode;
            if(info.remaining_time < 50 && !info.has_identify_our_undercover){
                myarmor_msg.lurk_mode = LurkSwitchMode::EXCHANGED_DIFF_ID;
            }

            int sameId_enemy_hp = GetMyArmor().id == roborts_msgs::Armor::ONE ? info.emeny_first_hp : info.emeny_second_hp;
            if(myarmor_msg.lurk_mode == LurkSwitchMode::EXCHANGED_DIFF_ID && sameId_enemy_hp <= 0){
                // 该状态下可以获取哨岗的定位信息
                myarmor_msg.lurk_mode = LurkSwitchMode::EXCHANGED_SAME_ID;
            }
            my_armor_pub_.publish(myarmor_msg);
            loop.sleep();       
        }
    }

  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> temp_tf_pose;
    temp_tf_pose.setIdentity();
    temp_tf_pose.frame_id_ = pose_frameID;
    temp_tf_pose.stamp_ = ros::Time(0);
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(temp_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, self_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }


  geometry_msgs::PoseStamped UpdateRobotGimbalPose(std::string frame){
        tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();
        gimbal_tf_pose.frame_id_ = gimbal_frameID;
        gimbal_tf_pose.stamp_ = ros::Time(0);
        geometry_msgs::PoseStamped gimbal_pose;
        geometry_msgs::PoseStamped target_pose;
        try{
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            tf_ptr_->transformPose(frame, gimbal_pose, target_pose);
        }
        catch (tf::LookupException &ex){
            ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
        }
        return target_pose;
  }

  geometry_msgs::PoseStamped InitMapPose(){
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = ros::Time::now();
      p.pose.position.x = -99;
      p.pose.position.y = -99;
      return p;
  }



  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! fake Enenmy  sub
  ros::Subscriber enemy_sub_;
  // Info  sub
  ros::Subscriber armor_sub_, camera_armor_sub_, back_camera_sub_, info_sub_, fusion_target_sub_, cmd_gimbal_sub_ , gimbal_info_sub_;
  // referee sub
  ros::Subscriber robot_status_sub_, robot_shoot_sub_, robot_heat_sub_, game_status_sub_, supply_sub_, buff_sub_, toward_goal_sub_,
                  vel_acc_sub_, robot_damage_sub_ , game_zone_sub_ , emeny_hp_sub_, emeny_bullet_sub_ , lurk_mode_sub_;
  //info  publisher
  ros::Publisher   cmd_vel_pub_ , cmd_aim_pub_ , ally_position_pub_ , pose_init_pub_ , my_armor_pub_ , clear_buff_area_pub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;



  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped self_pose_, enemy_;
  geometry_msgs::PoseStamped my_guard_pose_, ally_guard_pose_;
  //! 哨岗是否探测到敌人
  bool guard_has_first_enemy_,guard_has_second_enemy_;
  //! 哨岗是否探测到自己
  bool guard_has_my_pose_;
  //! 哨岗获取的敌方位置信息
  geometry_msgs::PoseStamped first_enemy_guard_pose_,second_enemy_guard_pose_;
  //! robot gimbal pose
  geometry_msgs::PoseStamped gimbal_pose_;

  //! glass obstacle
  Obstacle glass_obstacle[4];

  //! remain hp
  int remain_hp;

  // zmq
  zmq::context_t context_;
  zmq::context_t guardcontext_;
  zmq::context_t heatbeat_context_;

  zmq::socket_t masterSocket_, clientSocket_;
  zmq::socket_t guardSocket_;
  zmq::socket_t heatbeatSocket_;

  std::thread masterThread, clientThread;
  std::thread guardThread;
  std::thread heatbeatThread;
  std::thread shoot_thread;
  //! 是否连接上哨岗 
  bool connect_guard;
  //! 是否连接上队友
  bool connect_ally;
  //! 是否在打击状态
  bool is_in_shoot_state;
  //! frame id
  std::string pose_frameID, gimbal_frameID;
  //! 是否使用自瞄
  bool use_camera;
  //! 是否使用车载摄像头上的敌方坐标
  bool use_camera_first_enemy_pose_;
  bool use_camera_second_enemy_pose_;
  //! 自瞄判断是否可以击打
  bool aim_can_shooting_;
  //! 自瞄装甲板的 id 号
  int  valid_armor_id_;
  //! 敌方选择计数
  unsigned int my_shoot_1_cnt, my_shoot_2_cnt, ally_shoot_1_cnt, ally_shoot_2_cnt;
  //! 地图偏移量
  double map_offset_x , map_offset_y;
  //! 云台是否可以移动
  bool _gimbal_can ;
  //! 是否被玻璃板遮挡
  bool is_block_by_glass_;
  //! 视野内的装甲板
  std::vector<roborts_msgs::Armor> vaild_armors;
  std::vector<roborts_msgs::Armor> temp_armors;
  //! 敌方装甲板
  roborts_msgs::Armor first_enemy_armor_;
  roborts_msgs::Armor second_enemy_armor_;
  //! 自身装甲板
  roborts_msgs::Armor my_armor_;
  //! 友方装甲板
  roborts_msgs::Armor ally_armor_;
  bool _front_first_enemy, _front_second_enemy;
  
  //!陀螺仪坐标系与地图坐标系偏差角度
  float gyro_relative_map_;
  //! 地图坐标系下的云台角度
  float map_yaw_;
  
  // 是否清空速度指令
  bool clear_vel_;
  std::chrono::_V2::system_clock::time_point last_hurt_time_ = std::chrono::system_clock::now();
  
  // buff区域索引号
  int bullet_index_;
  int shield_index_;

  // buff 区域是否有灰车
  bool F1_has_gary_car_;
  bool F6_has_gary_car_;

  int faile_count_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H