
cmake_minimum_required(VERSION 3.5)
project(bot_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  tf
#  std_msgs
#  geometry_msgs
#  sensor_msgs
#  nav_msgs
#  visualization_msgs

#  message_generation
  
  ## ublox_msgs
#)

#find_package(Boost REQUIRED COMPONENTS thread)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(tf_conversions REQUIRED)

#add_message_files(
#  FILES
#  odom_rad.msg
#  nucleo_serial.msg
#)

#add_message_files(
#  FILES
#  nav_manager_command.msg
#  nav_manager_status.msg
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavManagerCommand.msg"
  "msg/NavManagerStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

#add_service_files(
#  FILES
#  nav_command.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

#catkin_package(
#   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${BOOST_INCLUDE_DIRS}
#)

install(PROGRAMS
  scripts/cancel_navigation
  scripts/latlng2pos
  scripts/pos2latlng
  scripts/lawnmower_odometry
  scripts/navigation_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
