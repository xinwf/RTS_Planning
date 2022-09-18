project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
roscpp
tf
nav_msgs
geometry_msgs
actionlib
roborts_common
roborts_msgs
roborts_costmap
)


find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
include
${catkin_INCLUDE_DIRS}
)



#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
DecisionProtoSrc
DecisionProtoHds
${ProtoFiles}
)

catkin_package()

# decision_node
add_executable(decision_node
decision_node.cpp
${DecisionProtoHds}
${DecisionProtoSrc}
example_behavior/escape_behavior.h
example_behavior/back_boot_area_behavior.h
example_behavior/chase_behavior.h
example_behavior/patrol_behavior.h
example_behavior/search_behavior.h
example_behavior/lurk_behavior.h
example_behavior/sentry_behavior.h
example_behavior/fixlocalization_behavior.h
example_behavior/gimbal_control_behavior.h
executor/chassis_executor.cpp

)

target_link_libraries(decision_node 
PRIVATE
roborts_costmap
${catkin_LIBRARIES}
${PROTOBUF_LIBRARIES}
-lzmq
)

# sel_behavior_node
add_executable(sel_behavior_node
sel_behavior_node.cpp
${DecisionProtoHds}
${DecisionProtoSrc}
example_behavior/escape_behavior.h
example_behavior/back_boot_area_behavior.h
example_behavior/chase_behavior.h
example_behavior/patrol_behavior.h
example_behavior/search_behavior.h
example_behavior/sentry_behavior.h
example_behavior/lurk_behavior.h
example_behavior/fixlocalization_behavior.h
example_behavior/gimbal_control_behavior.h
executor/chassis_executor.cpp
)

target_link_libraries(sel_behavior_node 
    PRIVATE
    roborts_costmap
    ${catkin_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    -lzmq
)


add_executable(referee_system_node
    referee_system_node.cpp
)

target_link_libraries(referee_system_node 
    PRIVATE
    ${catkin_LIBRARIES}
)

add_dependencies(decision_node
  roborts_msgs_generate_messages)
add_dependencies(sel_behavior_node
  roborts_msgs_generate_messages)
add_dependencies(referee_system_node
  roborts_msgs_generate_messages)
