cmake_minimum_required(VERSION 3.0.2)
project(navigation_modes)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)

add_service_files(
   FILES
   LocationID.srv
)

add_action_files(
   FILES
   GoLocation.action
   SetBehaviour.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)