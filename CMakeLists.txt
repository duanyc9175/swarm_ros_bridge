cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  global_descriptor.msg
  loop_info.msg
  neighbor_estimate.msg
  ScoutStatus.msg
  ScoutMotorState.msg
  ScoutLightState.msg
  ScoutLightCmd.msg
  ScoutBmsStatus.msg
  ScoutDriverState.msg
  start.msg
)
# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)





catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)

add_executable(bridge_node src/bridge_node.cpp)
add_dependencies(bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bridge_node
   ${catkin_LIBRARIES}
   zmq
   zmqpp
)
add_executable(start_node src/start.cpp)
add_dependencies(start_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(start_node
   ${catkin_LIBRARIES}
)
