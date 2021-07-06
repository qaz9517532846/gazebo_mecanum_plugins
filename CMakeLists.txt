cmake_minimum_required(VERSION 3.0.2)
project(gazebo_mecanum_plugins)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  tf
)

find_package(gazebo)

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

add_library(gazebo_ros_mecanum_diff_drive SHARED src/gazebo_ros_mecanum_diff_drive.cpp)
target_link_libraries(gazebo_ros_mecanum_diff_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)