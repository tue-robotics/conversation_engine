cmake_minimum_required(VERSION 3.0.2)
project(conversation_engine)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    message_generation
    rospy
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_action_files(
  DIRECTORY 
    action
  FILES
    Converse.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    message_runtime
    rospy
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
