cmake_minimum_required(VERSION 2.8.3)
project(xarm_api)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  genmsg
  xarm_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs
  xarm_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xarm_ros_client
  CATKIN_DEPENDS roscpp std_msgs actionlib actionlib_msgs control_msgs xarm_sdk
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(xarm_ros_client
  src/xarm_ros_client.cpp
)

# Make sure xarm_msgs header files are generated before building xarm_ros_client
# add_dependencies(xarm_ros_client xarm_msgs_generate_messages_cpp xarm_cxx_sdk)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(xarm_driver_node
  src/xarm_driver_node.cpp
  src/xarm_driver.cpp
  # src/xarm_driver_callback.cpp
)
# add_dependencies(xarm_driver_node xarm_msgs_generate_messages_cpp xarm_cxx_sdk)

add_executable(example1_report_norm_node 
  test/example1_report_norm.cc 
)

add_executable(move_test
  test/move_test.cpp
)

add_executable(test_tool_modbus
  test/test_tool_modbus.cpp
)

add_executable(servo_cart_test
  test/servo_cartesian_test.cpp
)

add_executable(test_xarm_ros_client
  test/test_xarm_ros_client.cpp
)

add_executable(test_xarm_velo_move
  test/test_xarm_velo_move.cpp
)

add_executable(test_xarm_states
  test/test_xarm_states.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against

target_link_libraries(xarm_ros_client 
  ${catkin_LIBRARIES}
)

target_link_libraries(example1_report_norm_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(xarm_driver_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(move_test 
  ${catkin_LIBRARIES}
)

target_link_libraries(test_tool_modbus 
  xarm_ros_client
  ${catkin_LIBRARIES}
)

target_link_libraries(servo_cart_test 
  ${catkin_LIBRARIES}
)

target_link_libraries(test_xarm_ros_client 
  xarm_ros_client
  ${catkin_LIBRARIES}
)

target_link_libraries(test_xarm_velo_move 
  xarm_ros_client
  ${catkin_LIBRARIES}
)

target_link_libraries(test_xarm_states 
  ${catkin_LIBRARIES}
)

# #############
# ## Install ##
# #############
# catkin_install_python(PROGRAMS scripts/servo_cartesian_test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark executables and/or libraries for installation
install(TARGETS xarm_ros_client xarm_driver_node example1_report_norm_node move_test test_tool_modbus servo_cart_test test_xarm_ros_client test_xarm_velo_move test_xarm_states
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

