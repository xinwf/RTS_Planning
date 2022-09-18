#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "roborts_msgs/Armor.h"
enum LurkSwitchMode{
    EXCHANGED_SAME_ID,    // four different cars 
    EXCHANGED_DIFF_ID     // tow different cars
};

struct ComInfo{
    int hp;
    int shoot_1, shoot_2;
    int bullet;
    float pose_x, pose_y, yaw;
    bool has_enemy;
    bool first_valid;
    float first_enemy_x, first_enemy_y;
    bool second_valid;
    float second_enemy_x, second_enemy_y;
    float goal_x, goal_y; 
    roborts_msgs::Armor mask_armor;
    bool ready_for_lurk;
    bool is_aim_target_armor;
};

// 哨岗
typedef  struct Position_xy
{
    float x;
    float y;
}Pos_xy;

struct HeatBeat{
    Pos_xy my_pose;
    bool has_ally;
};

struct Obstacle{
    Pos_xy left_up;
    Pos_xy right_up;
    Pos_xy right_down;
    Pos_xy left_down;
};

struct CarPositionSend{
   int lurkswitchmode;
   int undercover_id;
   bool F6;   
   bool F1;
   Pos_xy blue1;
   Pos_xy blue2;
   Pos_xy red1;
   Pos_xy red2;
};

#endif //COMMUNICATION_H