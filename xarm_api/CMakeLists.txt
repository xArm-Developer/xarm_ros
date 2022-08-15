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
  geometry_msgs
  actionlib
  actionlib_msgs
  control_msgs
  xarm_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xarm_ros_driver xarm_ros_client
  CATKIN_DEPENDS roscpp std_msgs actionlib actionlib_msgs control_msgs xarm_sdk xarm_msgs
#  DEPENDS system_lib
)

# set the include directories
include_directories(
  include
  
  ${xarm_sdk_INCLUDE_DIRS}

  # catkin_make_isolated --install
  ${xarm_sdk_INCLUDE_DIRS}/xarm_sdk

  ${catkin_INCLUDE_DIRS}
)

# xarm_ros_driver: add library and add dependencies and set link libraries
add_library(xarm_ros_driver
  src/xarm_driver.cpp
)
add_dependencies(xarm_ros_driver xarm_msgs_generate_messages_cpp)
target_link_libraries(xarm_ros_driver 
  ${xarm_sdk_LIBRARIES}
  ${catkin_LIBRARIES}
)

# xarm_ros_client: add library and add dependencies and set link libraries
add_library(xarm_ros_client
  src/xarm_ros_client.cpp
)
add_dependencies(xarm_ros_client xarm_msgs_generate_messages_cpp)
# target_link_libraries(xarm_ros_client 
#   ${xarm_sdk_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# xarm_driver_node: add executable and add dependencies and set link libraries
add_executable(xarm_driver_node
  src/xarm_driver_node.cpp
)
add_dependencies(xarm_driver_node xarm_msgs_generate_messages_cpp)
target_link_libraries(xarm_driver_node 
  xarm_ros_driver
  ${catkin_LIBRARIES}
)

# set the exec list who link the xarm_sdk library
set(link_xarm_sdk_exec_list 
  example1_report_norm
  move_test
  servo_cartesian_test
)
FOREACH(exec ${link_xarm_sdk_exec_list})
  add_executable(${exec} 
    test/${exec}.cpp
  )
  add_dependencies(${exec} xarm_msgs_generate_messages_cpp)
  target_link_libraries(${exec} 
    ${xarm_sdk_LIBRARIES}
    ${catkin_LIBRARIES}
  )
ENDFOREACH(exec)

# set the exec list who link the xarm_ros_client library
set(link_xarm_ros_client_exec_list 
  test_tool_modbus
  test_xarm_ros_client
  test_xarm_velo_move
  test_xarm_states
)
FOREACH(exec ${link_xarm_ros_client_exec_list})
  add_executable(${exec} 
    test/${exec}.cpp
  )
  add_dependencies(${exec} xarm_msgs_generate_messages_cpp)
  target_link_libraries(${exec} 
    xarm_ros_client
    ${catkin_LIBRARIES}
  )
ENDFOREACH(exec)

# install the include path
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install the library and executable
install(TARGETS
  xarm_ros_driver
  xarm_driver_node
  xarm_ros_client
  ${link_xarm_sdk_exec_list}
  ${link_xarm_ros_client_exec_list}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
