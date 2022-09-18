# WildWolf_ICRA
## 一、安装 ROS

### 1.1 ROS 版本：
|ROS version  |  ubuntu version|
|--|--|
| Noetic | ubuntu 20.04 |
| Melodic | ubuntu 18.04 |

### 1.2 安装方法：
 官网下载： [ROS Installation Guide](http://wiki.ros.org/ROS/Installation)
脚本安装：[ROS Insatll script](https://github.com/RocShi/rostaller)

## 二、安装依赖
运行以下命令安装一些 ROS 包 和 google-glog 库：
```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   						  ros-${ROS_DISTRO}-tf 
   							ros-${ROS_DISTRO}-nav-msgs 
   							ros-${ROS_DISTRO}-geometry-msgs 
   																					 
sudo apt-get install -y ros-${ROS_DISTRO}-cv-bridge           \
                        ros-${ROS_DISTRO}-image-transport     \
                        ros-${ROS_DISTRO}-stage-ros           \
                        ros-${ROS_DISTRO}-map-server          \
                        ros-${ROS_DISTRO}-laser-geometry      \
                        ros-${ROS_DISTRO}-interactive-markers \
                        ros-${ROS_DISTRO}-tf                  \
                        ros-${ROS_DISTRO}-pcl-*               \
                        ros-${ROS_DISTRO}-libg2o              \
                        ros-${ROS_DISTRO}-rplidar-ros         \
                        ros-${ROS_DISTRO}-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \、
                        libgoogle-glog-dev							
```

## 三、软件框架介绍

[RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/)

## 四、 Quick start

### 4.1 编译

```bash
cd catkin_ws/src
git clone git@github.com:zezeuser/WildWolf_ICRA.git
cd ..
catkin_make
```
### 4.2 运行仿真 

#### 4.2.1 修改参数

```bash
vim catkin_ws/src/roborts_costmap/config/costmap_parameter_config_for_local_plan.prototxt
# 第 7 行
robot_base_frame: "base_link"

vim /catkin_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
# 第 2 行
odom_frame: "odom"
```

#### 4.2.2 运行仿真环境、导航、定位、rviz节点

```bash
roslaunch icra_robomaster_emulator decision_test.launch 
```

#### 4.2.3 运行决策节点

```bash
rosrun roborts_decision sel_behavior_node
```

#### 4.2.4 仿真常见错误
若仿真不能正常显示，则运行以下命令将环境复制到本地 gazebo model  文件夹中
  
  ```shell
  cd  ~/catkin_ws/src/icra_robomaster_emulator/wall-2020
  cp -r icra_ground_plane/ ~/.gazebo/models/
  ```

### 4.3 运行 robomaster A 型机器人
#### 4.3.1 修改参数 
```bash
vim catkin_ws/src/roborts_costmap/config/costmap_parameter_config_for_local_plan.prototxt
# 第 7 行
robot_base_frame: "gimbal"

vim /catkin_ws/src/roborts_planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt
# 第 2 行
odom_frame: "gimbal_odom"
```

#### 4.3.2 运行雷达、雷达数据融合与屏蔽 、导航、定位、rviz 节点
注意：运行前请确保雷达与底盘接入
```bash
roslaunch roborts_bringup base.launch 
roslaunch roborts_bringup roborts.launch 
```

#### 4.3.3 运行自瞄
注意运行前请确保接入相机，否则初始化失败
```bash
rosrun armor_detection armor_detection_node
```

#### 4.3.3 运行决策节点
注意运行前请确保导航定位正常，否则初始化失败
```bash
rosrun roborts_decision sel_behavior_node
```
更多决策参数修改细节请参考

```bash
cat /catkin_ws/src/roborts_decision/config/decision.prototxt
```

## 五、雷达驱动SDK问题
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```
 [ydlidar G4   使用说明](https://ydlidar.cn/Public/upload/files/2021-08-31/YDLIDAR%20G4%20%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C%20V1.3.pdf)
 [ydlidar X4   使用说明](https://ydlidar.cn/Public/upload/files/2021-08-20/YDLIDAR%20X4%20%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C%20V1.3.pdf)
 

