#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/reload_behavior.h"
#include "example_behavior/shield_behavior.h"
#include "example_behavior/test_behavior.h"
#include "example_behavior/ambush_behavior.h"
#include "example_behavior/attack_behavior.h"
#include "example_behavior/sentry_behavior.h"
#include "example_behavior/lurk_behavior.h"
#include "example_behavior/fixlocalization_behavior.h"
#include "example_behavior/gimbal_control_behavior.h"


enum BehaviorStateEnum{
        INIT = -1,
        BACKBOOT = 0,
        CHASE=1,
        SEARCH=2,
        ESCAPE=3,
        PATROL=4,
        RELOAD=5,
        SHIELD=6,
        AMBUSH=7,
        ATTACK=8,
        LURK=9

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "decision_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto blackboard = new roborts_decision::Blackboard(full_path);
    ros::Duration(0.5).sleep();  //延时 0.5 秒
    // Behavior State Enum
    BehaviorStateEnum last_state, cur_state;
    last_state = BehaviorStateEnum::INIT;
    cur_state = BehaviorStateEnum::INIT;
    
    roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
    roborts_decision::ReloadBehavior     reload_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::ShieldBehavior     shield_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::TestBehavior      test_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::AmbushBehavior    ambush_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::AttackBehavior    attack_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::SentryBehavior    sentrybehavior(chassis_executor, blackboard, full_path);
    roborts_decision::LurkBehavior   lurk_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::FixLocalizationBehavior fixlocalization_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::GimbalControlBehavior gimbalcontrol_behavior(chassis_executor, blackboard, full_path);

    ros::Rate rate(60);
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    ros::Duration(1.5).sleep();
    gimbalcontrol_behavior.UpdateGobalFrame();
    // for filter noise command
    unsigned int count=0;
    const unsigned int count_bound = 3;
    while(ros::ok()){
            ros::spinOnce();
            printf("-----------------------------------------\n");
            printf("Remaining Time:%d and is begin:%d\n", blackboard->info.remaining_time, blackboard->info.is_begin);
            printf("\n");
            printf("Ally hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.ally_remain_hp, blackboard->info.ally_remain_bullet, blackboard->info.ally.pose.position.x, blackboard->info.ally.pose.position.y);
            printf("has_ally_enemy:%d, has_ally_first_enemy:%d, has_ally_second_endmy:%d\n", blackboard->info.has_ally_enemy, blackboard->info.has_ally_first_enemy, blackboard->info.has_ally_second_enemy);
            printf("ally first enemy pose:(%f, %f), ally second enemy pose:(%f, %f)\n", blackboard->info.ally_first_enemy.pose.position.x, blackboard->info.ally_first_enemy.pose.position.y, blackboard->info.ally_second_enemy.pose.position.x, blackboard->info.ally_second_enemy.pose.position.y);
            printf("\n");
            printf("Buff activate:  bullet : %d , shield : %d\n", blackboard->info.bullet_buff_active , blackboard->info.shield_buff_active);
            printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y);    
            printf("has_my_enemy:%d, has_first_enemy:%d, has_second_enemy:%d\n", blackboard->info.has_my_enemy, blackboard->info.has_first_enemy, blackboard->info.has_second_enemy);  
            printf("my first enemy pose:(%f, %f), my second enemy pose:(%f, %f)\n", blackboard->info.first_enemy.pose.position.x, blackboard->info.first_enemy.pose.position.y, blackboard->info.second_enemy.pose.position.x, blackboard->info.second_enemy.pose.position.y);
            printf("my goal:(%f, %f), ally goal:(%f, %f)\n", blackboard->info.my_goal.pose.position.x, blackboard->info.my_goal.pose.position.y, blackboard->info.ally_goal.pose.position.x, blackboard->info.ally_goal.pose.position.y);
            printf("has_identify_our_undercover :%d\n", blackboard->info.has_identify_our_undercover);
            printf("lurk mode  :%d\n", blackboard->info.lurk_switch_mode);
            printf("undercover id  :%d\n", blackboard->info.undercover_id);
            printf("my armor color : %d  id  :%d\n", blackboard->GetMyArmor().color , blackboard->GetMyArmor().id);
            printf("ally armor color : %d  id  :%d\n", blackboard->GetAllyArmor().color , blackboard->GetAllyArmor().id);
            // printf("Is In Struck Area: %d , \n", blackboard->IsInStuckArea());
            if(blackboard->info.is_begin || !blackboard->info.use_refree){   //!!!!!!!!!!!!test
                // 若陷入障碍层中，发布速度走出障碍
                // 默认开启扭腰
                gimbalcontrol_behavior.Run();
                if (blackboard->CanShoot()) blackboard->Shoot(blackboard->info.shoot_delay_ms);
                if(blackboard->info.has_identify_our_undercover) lurk_behavior.Run();
                blackboard->UpdateAimCmd();
                // my pose
                geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();
                // not first buff strategy--------------------------------------------------------------------------------------------------------
                if (blackboard->info.strategy == "no_go_buff"){
                        if (blackboard->info.remain_hp >= 400){
                                    if(blackboard->info.remaining_time <= 55 && !blackboard->info.has_identify_our_undercover){
                                            cur_state = BehaviorStateEnum::LURK;
                                    }
                                    else if (!blackboard->info.has_my_enemy 
                                        && !blackboard->GuardEnemyDetected() 
                                        && !blackboard->info.has_ally_enemy){
                                            cur_state = BehaviorStateEnum::SEARCH;
                                    }
                                    else{
                                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor){
                                            cur_state = BehaviorStateEnum::CHASE;
                                        }
                                        else if(blackboard->info.has_ally_enemy || blackboard->GuardEnemyDetected()){
                                            cur_state = BehaviorStateEnum::ATTACK;
                                        }
                                        else{
                                            cur_state = BehaviorStateEnum::PATROL;
                                        }
                                    }
                            }
                            else{
                                if (blackboard->info.remain_bullet > 10){
                                    if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor){
                                            cur_state = BehaviorStateEnum::CHASE;
                                    }
                                    else if(blackboard->info.has_ally_enemy || blackboard->GuardEnemyDetected()){
                                        cur_state = BehaviorStateEnum::ATTACK;
                                    }
                                    else{
                                        cur_state = BehaviorStateEnum::PATROL;
                                    }
                                }
                                else{
                                    cur_state = BehaviorStateEnum::BACKBOOT;
                                }
                            }
                }
                // attack originally for chasing  
                else if (blackboard->info.strategy == "attack"){
                        // state decision behavior
                        if(blackboard->info.remaining_time <= 55 && !blackboard->info.has_identify_our_undercover){
                                cur_state = BehaviorStateEnum::LURK;
                        }
                        else if (!blackboard->ShieldBuffHasGrayCar()
                                && ((blackboard->info.shield_buff_active 
                                    && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield) 
                                        >= blackboard->GetDistance(mypose, blackboard->info.my_shield)
                                    && blackboard->info.remain_hp < 1800  && blackboard->info.ally_remain_hp < 1800)
                                    || (!blackboard->info.has_ally && blackboard->info.shield_buff_active && blackboard->info.remain_hp < 1800)
                                    || blackboard->info.is_shielding)){
                                cur_state = BehaviorStateEnum::SHIELD;
                        }
                        else if (!blackboard->BulletBuffHasGrayCar()
                                && ((blackboard->info.bullet_buff_active  
                                    && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)
                                        >= blackboard->GetDistance(mypose, blackboard->info.my_reload)
                                    && (blackboard->info.remain_bullet < 10 || blackboard->info.ally_remain_bullet < 10))
                                    || (!blackboard->info.has_ally && blackboard->info.bullet_buff_active && blackboard->info.remain_bullet < 20)
                                    || blackboard->info.is_supplying)){
                                cur_state = BehaviorStateEnum::RELOAD;  
                        }
                        else{
                            if (blackboard->info.remain_hp >= 400){
                                if (blackboard->info.remain_bullet > 0){
                                    if (!blackboard->info.has_my_enemy 
                                        && !blackboard->GuardEnemyDetected() 
                                        && (!blackboard->info.has_ally_enemy || !blackboard->CanCollaborateWithAlly())
                                        && !lurk_behavior.CanBackHome()){
                                            cur_state = BehaviorStateEnum::SEARCH;
                                    }
                                    else{
                                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor){
                                            cur_state = BehaviorStateEnum::CHASE;
                                        }
                                        else if((blackboard->info.has_ally_enemy && blackboard->CanCollaborateWithAlly()) || blackboard->GuardEnemyDetected()){
                                            cur_state = BehaviorStateEnum::ATTACK;
                                        }
                                        else{
                                            if(lurk_behavior.CanBackHome()){
                                                cur_state = BehaviorStateEnum::BACKBOOT;
                                            }else{
                                                cur_state = BehaviorStateEnum::PATROL;
                                            }
                                        }
                                    }
                                }
                                else{
                                    cur_state = BehaviorStateEnum::ESCAPE;  
                                }
                            }
                            else{
                                if (blackboard->info.remain_bullet > 10){
                                    if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor){
                                            cur_state = BehaviorStateEnum::CHASE;
                                    }
                                    else if((blackboard->info.has_ally_enemy && blackboard->CanCollaborateWithAlly()) || blackboard->GuardEnemyDetected()){
                                        cur_state = BehaviorStateEnum::ATTACK;
                                    }
                                    else{
                                        if(lurk_behavior.CanBackHome()){
                                            cur_state = BehaviorStateEnum::BACKBOOT;
                                        }else{
                                            cur_state = BehaviorStateEnum::PATROL;
                                        }
                                    }
                                }
                                else{
                                    cur_state = BehaviorStateEnum::ESCAPE;
                                }
                            }
                        }
                }

                // Defense
                else if (blackboard->info.strategy == "defense"){
                        // state decision behavior
                       if (!blackboard->ShieldBuffHasGrayCar()
                                && ((blackboard->info.shield_buff_active 
                                    && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield) 
                                        >= blackboard->GetDistance(mypose, blackboard->info.my_shield)
                                    && blackboard->info.remain_hp < 1800  && blackboard->info.ally_remain_hp < 1800)
                                    || (!blackboard->info.has_ally && blackboard->info.shield_buff_active && blackboard->info.remain_hp < 1800)
                                    || blackboard->info.is_shielding)){
                                cur_state = BehaviorStateEnum::SHIELD;
                        }
                        else if (!blackboard->BulletBuffHasGrayCar()
                                && ((blackboard->info.bullet_buff_active  
                                    && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)
                                        >= blackboard->GetDistance(mypose, blackboard->info.my_reload)
                                    && (blackboard->info.remain_bullet < 10 || blackboard->info.ally_remain_bullet < 10))
                                    || (!blackboard->info.has_ally && blackboard->info.bullet_buff_active && blackboard->info.remain_bullet < 20)
                                    || blackboard->info.is_supplying)){
                                cur_state = BehaviorStateEnum::RELOAD;  
                        }
                        else{
                            if (blackboard->info.remain_hp >= 400){
                                if (blackboard->info.remain_bullet > 0){
                                    if (!blackboard->info.has_my_enemy 
                                        && !blackboard->GuardEnemyDetected() 
                                        && (!blackboard->info.has_ally_enemy || !blackboard->CanCollaborateWithAlly())){
                                            cur_state = BehaviorStateEnum::SEARCH;
                                    }
                                    else{
                                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor){
                                            cur_state = BehaviorStateEnum::CHASE;
                                        }
                                        else if((blackboard->info.has_ally_enemy && blackboard->CanCollaborateWithAlly()) || blackboard->GuardEnemyDetected()){
                                            cur_state = BehaviorStateEnum::AMBUSH;
                                        }
                                        else{
                                            if(lurk_behavior.CanBackHome()){
                                                cur_state = BehaviorStateEnum::BACKBOOT;
                                            }else{
                                                cur_state = BehaviorStateEnum::PATROL;
                                            }
                                        }
                                    }
                                }
                                else{
                                    if(blackboard->GuardEnemyDetected()){
                                        cur_state = BehaviorStateEnum::ESCAPE;
                                    }
                                    else{
                                        cur_state = BehaviorStateEnum::BACKBOOT;
                                    }
                                }
                            }
                            else{
                                if (blackboard->info.remain_bullet > 0){
                                    // 自己和队友都看不到敌人，哨岗探测到敌人，执行埋伏行为
                                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy && blackboard->GuardEnemyDetected()){
                                        cur_state = BehaviorStateEnum::AMBUSH;
                                    }
                                    // 自己和队友都看不到敌人 ， 哨岗探测不到敌人，执行内环巡逻行为
                                    else if(!blackboard->info.has_my_enemy && (!blackboard->info.has_ally_enemy || !blackboard->CanCollaborateWithAlly())){
                                        cur_state = BehaviorStateEnum::PATROL;
                                    }
                                    // 探测到敌人，执行埋伏行为
                                    else{
                                         cur_state = BehaviorStateEnum::AMBUSH;
                                    }
                                }
                                else{
                                    if(blackboard->GuardEnemyDetected()){
                                        cur_state = BehaviorStateEnum::ESCAPE;
                                    }
                                    else{
                                        cur_state = BehaviorStateEnum::BACKBOOT;
                                    }
                                }
                            }
                        }
                }

                // // filter
                // if (!(last_state == BehaviorStateEnum::RELOAD  ||  last_state == BehaviorStateEnum::SHIELD)){
                //         count = last_state != cur_state ? count + 1: 0;
                //         cur_state = count>=count_bound? cur_state: last_state;
                // }

            }

               if (blackboard->info.remain_hp <=0 || blackboard->info.game_status == roborts_msgs::GameStatus::END){
                    // ros::shutdown();
                    cur_state = BehaviorStateEnum::BACKBOOT;
                    blackboard->StopFirWhl();
                    if(blackboard->info.remain_hp <=0){
                        blackboard->StopFirWhl();
                        ros::shutdown();
                    }
                    // break;
                }

                   // remain hp is 0, them dead!

                // cancel last state
            if ( (last_state != BehaviorStateEnum::INIT && last_state != cur_state)  || blackboard->info.remain_hp <= 0 ){
                    switch (last_state){
                            case BehaviorStateEnum::BACKBOOT:
                                    back_boot_area_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::CHASE:
                                    chase_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::ESCAPE:
                                    escape_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::PATROL:
                                    patrol_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::RELOAD:
                                    reload_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::SHIELD:
                                    shield_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::SEARCH:
                                    search_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::AMBUSH:
                                    ambush_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::ATTACK:
                                    attack_behavior.Cancel();
                                    break;
                            case BehaviorStateEnum::LURK:
                                    lurk_behavior.Cancel();
                                    break;
                    }
            }

     


            switch (cur_state){
                    case BehaviorStateEnum::BACKBOOT:
                            back_boot_area_behavior.Run();
                            std::cout<<"BackBoot" << std::endl;
                            break;
                    case BehaviorStateEnum::CHASE:
                            chase_behavior.Run();
                            std::cout<<"CHASE" << std::endl;
                            break;
                    case BehaviorStateEnum::ESCAPE:
                            escape_behavior.Run();
                            std::cout<<"ESCAPE" << std::endl;
                            break;
                    case BehaviorStateEnum::PATROL:
                            patrol_behavior.Run();
                            std::cout<<"PATROL" << std::endl;
                            break;
                    case BehaviorStateEnum::RELOAD:
                            reload_behavior.Run();
                            std::cout<<"RELOAD" << std::endl;
                            break;
                    case BehaviorStateEnum::SHIELD:
                            shield_behavior.Run();
                            std::cout<<"SHIELD" << std::endl;
                            break;
                    case BehaviorStateEnum::SEARCH:
                            search_behavior.Run();
                            std::cout<<"SEARCH" << std::endl;
                            break;
                    case BehaviorStateEnum::AMBUSH:
                            ambush_behavior.Run();
                            std::cout<<"AMBUSH" << std::endl;
                            break;
                    case BehaviorStateEnum::ATTACK:
                            attack_behavior.Run();
                            std::cout<<"ATTACK" << std::endl;
                            break;
                    case BehaviorStateEnum::LURK:
                            lurk_behavior.Run();
                            std::cout<<"LURK" << std::endl;
                            break;
            }

            last_state = cur_state;
            rate.sleep();

    }

    return 0;
}
