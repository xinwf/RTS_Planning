 #include <ros/ros.h>
 #include <roborts_msgs/GameRobotHP.h>
 #include <roborts_msgs/GameRobotBullet.h>
 #include <roborts_msgs/GameStatus.h>
 #include <roborts_msgs/GameZoneArray.h>
 #include <roborts_msgs/RobotHeat.h>
 #include <roborts_msgs/RobotStatus.h>

class referee_system
{
    public:
        referee_system();
        ~referee_system();
        void loss_hp(int _id ,int _loss=20 );
        void loss_bullet( int _id ,int _loss=20 );
        void refresh_hp();
        void game_start();
        void refresh_buff();
        void close_buff();
        void cout_info();

    private:
        roborts_msgs::GameRobotHP gamehp_;
        roborts_msgs::GameRobotBullet gamebullet_;
        roborts_msgs::GameStatus game_status_;
        roborts_msgs::GameZoneArray buff_zones;
        ros::NodeHandle ref_nh_;
        // publisher
        ros::Publisher game_status_pub_;
        ros::Publisher hp_pub_;
        ros::Publisher bullet_pub_;
        ros::Publisher buff_pub_;
};

referee_system::referee_system()
{  
    game_status_pub_ = ref_nh_.advertise<roborts_msgs::GameStatus>("game_status", 10,this);
    hp_pub_ = ref_nh_.advertise<roborts_msgs::GameRobotHP>("game_robot_hp", 30, this);
    bullet_pub_ = ref_nh_.advertise<roborts_msgs::GameRobotBullet>("game_robot_bullet",30, this);
    buff_pub_ = ref_nh_.advertise<roborts_msgs::GameZoneArray>("game_zone_array_status",30,this);
}

referee_system::~referee_system()= default;

void referee_system::loss_hp(int _id ,int _loss){
    if(_id == 0){
        gamehp_.blue1 -= _loss;
        gamehp_.blue1 =  gamehp_.blue1 < 0 ? 0 : gamehp_.blue1;
    }
    if(_id == 1){
        gamehp_.blue2 -= _loss;
        gamehp_.blue2 = gamehp_.blue2 < 0 ? 0 : gamehp_.blue2;
    }
    if(_id == 2){
        gamehp_.red1 -= _loss;
        gamehp_.red1 =  gamehp_.red1 < 0 ? 0 : gamehp_.red1;
    }
    if(_id == 3){
        gamehp_.red2 -= _loss;
        gamehp_.red2 = gamehp_.red2 < 0 ? 0 : gamehp_.red2;
    }
    hp_pub_.publish(gamehp_);
}


void referee_system::loss_bullet(int _id ,int _loss){
    if(_id == 0){
        gamebullet_.blue1 -= _loss;
        gamebullet_.blue1 = gamebullet_.blue1 < 0 ? 0 : gamebullet_.blue1;
    }
    if(_id == 1){
        gamebullet_.blue2 -= _loss;
        gamebullet_.blue2 = gamebullet_.blue2 < 0 ? 0 : gamebullet_.blue2;
    }
    if(_id == 2){
        gamebullet_.red1 -= _loss;
        gamebullet_.red1 = gamebullet_.red1 < 0 ? 0 : gamebullet_.red1;
    }
    if(_id == 3){
        gamebullet_.red2 -= _loss;
        gamebullet_.red2 = gamebullet_.red2 < 0 ? 0 : gamebullet_.red2;
    }
    bullet_pub_.publish(gamebullet_);
}

void referee_system::refresh_hp(){
    gamehp_.blue1 = 2000;
    gamehp_.blue2 = 2000;
    gamehp_.red1 = 2000;
    gamehp_.red2 = 2000;
    hp_pub_.publish(gamehp_);
}


void referee_system::game_start(){
    game_status_.game_status = game_status_.GAME;
    game_status_pub_.publish(game_status_);
}

void referee_system::refresh_buff(){
    buff_zones.zone[0].type = roborts_msgs::GameZone::BLUE_BULLET_SUPPLY;
    buff_zones.zone[5].type = roborts_msgs::GameZone::RED_BULLET_SUPPLY;
    buff_zones.zone[1].type = roborts_msgs::GameZone::BLUE_HP_RECOVERY;
    buff_zones.zone[4].type = roborts_msgs::GameZone::RED_HP_RECOVERY;
    buff_zones.zone[2].type = roborts_msgs::GameZone::DISABLE_MOVEMENT;
    buff_zones.zone[3].type = roborts_msgs::GameZone::DISABLE_SHOOTING;
    for (size_t i = 0; i < sizeof(buff_zones); i++)
    {
        buff_zones.zone[i].active = true;
    }
    buff_pub_.publish(buff_zones);
}

void referee_system::close_buff(){
    for (size_t i = 0; i < sizeof(buff_zones); i++)
    {
        buff_zones.zone[i].active = false;
    }
    buff_pub_.publish(buff_zones);
}

void referee_system::cout_info(){
    std::cout << "**************************************************************************************" << std::endl;
        std::cout << "*********************************please send a command********************************" << std::endl;
        std::cout << "blue 1 hp  " << gamehp_.blue1 << std::endl
                << "blue 2 hp  " << gamehp_.blue2 << std::endl
                << "red 1 hp  " << gamehp_.red1 << std::endl
                << "red 2 hp  " << gamehp_.red2 << std::endl;
        std::cout << "**************************************************************************************" << std::endl;
        std::cout << "> ";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "referee_system_node");
    referee_system referee_system;
    char command = '0';
    while (command != '10') {
        std::cout << "**************************************************************************************" << std::endl;
        std::cout << "*********************************please send a command********************************" << std::endl;
        std::cout << "1: game start" << std::endl
                << "2: bullet loss 20  " << std::endl
                << "3: blue 1   hp loss 20 " << std::endl
                << "4: blue 2   hp loss 20 " << std::endl
                << "5: red  1   hp loss 20 " << std::endl
                << "6: red  2   hp loss 20 " << std::endl
                << "7: refresh all buff  "   << std::endl
                << "8: close all buff  "     << std::endl
                << "9: cout info  "          << std::endl;
        std::cout << "**************************************************************************************" << std::endl;
        std::cout << "> ";
        std::cin >> command;
        if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' && command != '8' && command != '9') {
        std::cout << "please inpugain!" << std::endl;
        std::cout << "> ";
        std::cin >> command;
        }

        switch (command) {
            case '1':
                referee_system.game_start();
                break;
            case '2':
                referee_system.loss_bullet(0);
                break;
            case '3':
                referee_system.loss_hp(0);
                break;
            case '4':
                referee_system.loss_hp(1);
                break;
            case '5':
                referee_system.loss_hp(2);
                break;
            case '6':
                referee_system.loss_hp(3);
                break;
            case '7':
                referee_system.refresh_buff();
                break;
            case '8':
                referee_system.close_buff();
                break;
            case '9':
                referee_system.cout_info();
                break;
            default:
                break;
        }
    }
  return 0;
}