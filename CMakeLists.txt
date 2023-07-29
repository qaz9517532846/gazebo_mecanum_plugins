cmake_minimum_required(VERSION 3.1.4)
project(gazebo_mecanum_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  tf
  message_generation
  dynamic_reconfigure
)

find_package(gazebo)

add_message_files(
  FILES
  gazebo_mecanum_plugins_vel.msg
  gazebo_mecanum_plugins_pid.msg
)

generate_dynamic_reconfigure_options(
  cfg/gazebo_ros_mecanum_pid.cfg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_mecanum_plugins
  CATKIN_DEPENDS
    nav_msgs
    geometry_msgs
    joint_state_controller
    robot_state_publisher
    roscpp
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_mecanum_drive SHARED src/gazebo_ros_mecanum_drive.cpp)
target_link_libraries(gazebo_ros_mecanum_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_mecanum_drive gazebo_mecanum_plugins_gencfg gazebo_mecanum_plugins_generate_messages_cpp)

add_library(gazebo_ros_mecanum_pid_drive SHARED src/gazebo_ros_mecanum_pid_drive.cpp src/gazebo_ros_mecanum_pid.cpp)
target_link_libraries(gazebo_ros_mecanum_pid_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_mecanum_pid_drive gazebo_mecanum_plugins_gencfg gazebo_mecanum_plugins_generate_messages_cpp)

add_executable(gazebo_mecanum_plugin_pid src/gazebo_ros_mecanum_pid_param.cpp)
target_link_libraries(gazebo_mecanum_plugin_pid ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_mecanum_plugin_pid gazebo_mecanum_plugins_gencfg gazebo_mecanum_plugins_generate_messages_cpp)

install(TARGETS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
