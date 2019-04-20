cmake_minimum_required(VERSION 2.8.3)
project(rot)

find_package(catkin REQUIRED COMPONENTS
  rosserial_python
  message_generation
  std_msgs
  rospy
  ackermann_msgs
  sensor_msgs
  audio_common_msgs
  lex_common_msgs)
find_package(Boost REQUIRED)
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    rospy
    std_msgs
    ackermann_msgs
    sensor_msgs
    audio_common_msgs
    message_runtime
    lex_common_msgs
    voice_interaction_robot_msgs
  DEPENDS Boost
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/alexaop.py  
  scripts/joyop.py
  scripts/keyop.py
  scripts/textop.py
  scripts/voiceop.py
  nodes/audio_output
  nodes/voice_input
  nodes/voice_output
  nodes/voice_interaction
  nodes/voice_command_translator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  assets
  config
  launch
  nodes
  scripts
  rviz
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)

endif()


