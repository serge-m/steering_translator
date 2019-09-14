cmake_minimum_required(VERSION 2.8.3)
project(steering_translator)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/SteeringTranslator.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############
