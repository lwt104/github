cmake_minimum_required(VERSION 2.8.3)
project(multi_agent_formation)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
