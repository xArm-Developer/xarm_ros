cmake_minimum_required(VERSION 3.0.2)
project(xarm_moveit_servo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_servo
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xarm_moveit_servo
#  CATKIN_DEPENDS roscpp sersor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xarm_joy_to_servo_node
  src/xarm_joystick_input.cpp
)
target_link_libraries(xarm_joy_to_servo_node ${catkin_LIBRARIES})

add_executable(xarm_keyboard_to_servo_node
  src/xarm_keyboard_input.cpp
)
target_link_libraries(xarm_keyboard_to_servo_node ${catkin_LIBRARIES})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS
  xarm_joy_to_servo_node
  xarm_keyboard_to_servo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
