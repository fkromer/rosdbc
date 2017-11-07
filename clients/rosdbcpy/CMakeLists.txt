cmake_minimum_required(VERSION 2.8.3)
project(rosdbcpy)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs rosunit)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()
catkin_python_setup()

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_parameter_checks.py)
endif()
