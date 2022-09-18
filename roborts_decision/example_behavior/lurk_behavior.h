#ifndef ROBORTS_DECISION_LURK_BEHAVIOR_H
#define ROBORTS_DECISION_LURK_BEHAVIOR_H

#include "io/io.h"
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {
class LurkBehavior {
 public:
  LurkBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard,
               const std::string & proto_file_path) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) { 
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
        blackboard_->info.has_identify_our_undercover = false;
        has_identify_mask_armor_ = false;
        undercover_ = false;
        detect_count = 0;
        has_select_position_ = false;
        start_ = false;
        has_identify_enemy_undercover_ = false;
        before_lurk_ally_armor_ = blackboard_->GetAllyArmor();
        enemy_color_ = blackboard_->info.team_blue 
                                        ? roborts_msgs::Armor::RED 
                                        : roborts_msgs::Armor::BLUE;
        assume_ally_undercover_armor.id = before_lurk_ally_armor_.id;
        assume_ally_undercover_armor.color = enemy_color_;
    }

  void Run() {
    if(!blackboard_->info.has_ally){
        blackboard_->info.cmd_aim.my_color = roborts_msgs::Armor::ALL;
        blackboard_->info.cmd_aim.mask_armor.id = roborts_msgs::Armor::NONE;
        blackboard_->info.cmd_aim.must_aim_target = false;
        blackboard_->info.ready_for_lurk = false;
    }
    else {
        if(ReadyForLurk() || blackboard_->info.has_identify_our_undercover){
            std::cout << "undercover_  : " << undercover_; 
            if(blackboard_->info.has_identify_our_undercover){
                blackboard_->info.ready_for_lurk = false;
                blackboard_->info.cmd_aim.my_color = roborts_msgs::Armor::ALL;
                undercover_ = blackboard_->info.undercover_id == blackboard_->GetMyArmor().id ?  true : false;
                // 判断敌方卧底的颜色与 id
                IdentifyEnemyArmor();
                // 选择击打敌方装甲板 颜色与 id 
                ChooseEnemyArmor();
                // 屏蔽队友卧底装甲板 颜色与 id 
                IdentifyMaskArmor();
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
  
  // 为辨别己方卧底的 Armor 做准备
  bool ReadyForLurk(){
        if(!blackboard_->info.has_identify_our_undercover){
            float ef_distance = blackboard_->GetDistance(blackboard_->GetRobotMapPose(),blackboard_->info.first_enemy);
            float es_distance = blackboard_->GetDistance(blackboard_->GetRobotMapPose(),blackboard_->info.second_enemy);

            if(!has_select_position_ || ef_distance < 1.0 || es_distance < 1.0 || blackboard_){
                SelectPositionForLurk();
            }
            if(ef_distance < 1.0 || es_distance < 1.0 ){
                blackboard_->info.ready_for_lurk = false;
            }
            BehaviorState chaiss_state = Update();
            float goal_dis = blackboard_->GetDistance(blackboard_->GetRobotMapPose(),ready_for_lurk_position_);
            if(!blackboard_->info.ready_for_lurk && Update() != BehaviorState::RUNNING ){
                    chassis_executor_->Execute(ready_for_lurk_position_);
                    blackboard_->SetMyGoal(ready_for_lurk_position_);
                    blackboard_->SetMyToward(ready_for_lurk_position_);
            }else if(goal_dis <= blackboard_->threshold.near_dist*4){
                blackboard_->info.ready_for_lurk = true;
            }
        }
        return blackboard_->info.ready_for_lurk; 
    }
  
    // 辨别敌方卧底的 Armor 
    void IdentifyEnemyArmor(){
        if(undercover_ && blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_SAME_ID){
            enemy_undercover_armor_.id = blackboard_->info.is_master ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
        }else if(!undercover_ && blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_SAME_ID){
            enemy_undercover_armor_.id = blackboard_->info.is_master ? roborts_msgs::Armor::TWO : roborts_msgs::Armor::ONE;
        }else if(undercover_&& blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID){
            enemy_undercover_armor_.id = blackboard_->info.is_master ? roborts_msgs::Armor::TWO : roborts_msgs::Armor::ONE;
        }else{
            enemy_undercover_armor_.id = blackboard_->info.is_master ? roborts_msgs::Armor::ONE : roborts_msgs::Armor::TWO;
        }
        enemy_undercover_armor_.color = blackboard_->info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
        blackboard_->SaveEnemyArmor(enemy_undercover_armor_.id , enemy_undercover_armor_.color);
        has_identify_enemy_undercover_ = true ; 
    }

    // 潜伏阶段下选择对手的 Armor
    void ChooseEnemyArmor(){
        if(blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID){
            // 在两类车的模式下， 目标击杀与自己相同颜色和 id的车牌
            blackboard_->info.cmd_aim.must_aim_target = true;
            blackboard_->info.cmd_aim.target_armor = enemy_undercover_armor_;
        }
    }

    bool CanBackHome(){
        if(has_identify_enemy_undercover_ 
        && enemy_undercover_armor_.id == roborts_msgs::Armor::ONE
        && blackboard_->info.emeny_first_hp <= 0
        && blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID
        && blackboard_->info.has_ally){
            return true;
        }
        else if(has_identify_enemy_undercover_ 
        && enemy_undercover_armor_.id == roborts_msgs::Armor::TWO
        && blackboard_->info.emeny_second_hp <= 0
        && blackboard_->info.lurk_switch_mode == LurkSwitchMode::EXCHANGED_DIFF_ID
        && blackboard_->info.has_ally){
            return true;
        }
        else{
            return false;
        }
    }

    // 屏蔽队友的 Armor 
    void IdentifyMaskArmor(){
        roborts_msgs::Armor my_armor;
        if(!undercover_){
            after_lurk_ally_armor_.color = blackboard_->info.team_blue ? roborts_msgs::Armor::RED : roborts_msgs::Armor::BLUE;
            after_lurk_ally_armor_.id = before_lurk_ally_armor_.id;
            my_armor.color =  blackboard_->info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
            my_armor.id = blackboard_->GetMyArmor().id;
        }else{
            after_lurk_ally_armor_.color = blackboard_->info.team_blue ? roborts_msgs::Armor::BLUE : roborts_msgs::Armor::RED;
            after_lurk_ally_armor_.id = before_lurk_ally_armor_.id;
            my_armor.color =  blackboard_->info.team_blue ? roborts_msgs::Armor::RED : roborts_msgs::Armor::BLUE;
            my_armor.id = blackboard_->GetMyArmor().id;
        }
        blackboard_->info.cmd_aim.mask_armor = after_lurk_ally_armor_;
        blackboard_->SaveOurArmor("ally_armor" , after_lurk_ally_armor_.id , after_lurk_ally_armor_.color);
        blackboard_->SaveOurArmor("my_armor",my_armor.id , my_armor.color);
        has_identify_mask_armor_ = true;
    }

    void SelectPositionForLurk(){
        geometry_msgs::PoseStamped ally_map_pose = blackboard_->info.ally;
        // 寻找距离敌方最近的埋伏点
        double max_distance = -1;
        double max_i = 0;
        double fenemy_dist,senemy_dist , e_dist;
        for (int i = 0; i < lurk_position_vector_.size(); i++){
            fenemy_dist = blackboard_->GetDistance(lurk_position_vector_[i], blackboard_->info.first_enemy);
            senemy_dist = blackboard_->GetDistance(lurk_position_vector_[i], blackboard_->info.second_enemy);
            e_dist = fenemy_dist + senemy_dist;
            if (e_dist > max_distance 
                && !blackboard_->IsBombAllyGoal(lurk_position_vector_[i])
            ){
                max_distance = e_dist;
                max_i = i;
            }
        }
        ready_for_lurk_position_ = lurk_position_vector_[max_i];
    }

    bool CanStartReadyLurk(){
        return true;
    }


  ~LurkBehavior() = default;

 private:
    bool LoadParam(const std::string &proto_file_path) {
            roborts_decision::DecisionConfig decision_config;
            if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            return false;
            }
            int point_size = (unsigned int)decision_config.lurk_position().size();
            lurk_position_vector_.resize(point_size);
            for (int i = 0; i != point_size; i++) {
                lurk_position_vector_[i] = blackboard_->Point2PoseStamped(decision_config.lurk_position(i));
            }
            return true;
    }
    
    geometry_msgs::PoseStamped GetMasterPose(){
        if(blackboard_->GetMyArmor().id == roborts_msgs::Armor::ONE){
            return blackboard_->GetRobotMapPose();
        }else{
            return blackboard_->info.ally ;
        }
    }
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  bool has_identify_enemy_undercover_;
  bool has_identify_mask_armor_;
  bool undercover_;
  bool has_select_position_;
  bool start_;
  geometry_msgs::PoseStamped ready_for_lurk_position_;
  std::vector<geometry_msgs::PoseStamped> lurk_position_vector_;
  roborts_msgs::Armor enemy_undercover_armor_;
  roborts_msgs::Armor before_lurk_ally_armor_;
  roborts_msgs::Armor after_lurk_ally_armor_;
  roborts_msgs::Armor assume_ally_undercover_armor;
  int enemy_color_;
  int detect_count;
};
}

#endif //ROBORTS_DECISION_LURK_BEHAVIOR_H