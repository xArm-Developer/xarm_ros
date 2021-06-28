cmake_minimum_required(VERSION 2.8.3)

project(xarm_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
  actionlib
  actionlib_msgs
  xarm_api
  xarm_msgs
)

include_directories(
  include
  
  ${xarm_sdk_INCLUDE_DIRS}
  ${xarm_api_INCLUDE_DIRS}

  # catkin_make_isolated --install
  ${xarm_sdk_INCLUDE_DIRS}/xarm_sdk
  ${xarm_sdk_INCLUDE_DIRS}/xarm_api

  # colcon build
  ${xarm_sdk_INCLUDE_DIRS}/../../xarm_api/include/xarm_api

  ${catkin_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES Move.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs xarm_api xarm_msgs
)

find_package(roslaunch)

add_executable(gripper_action_server 
   src/gripper_action_server.cpp 
)

# Make sure xarm_gripper header files are generated before building gripper_action_server
add_dependencies(gripper_action_server xarm_msgs_generate_messages_cpp xarm_gripper_generate_messages_cpp)

add_dependencies(gripper_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_action_server 
   ${catkin_LIBRARIES}
)

add_executable(gripper_client 
   src/gripper_client.cpp 
)

# Make sure xarm_gripper header files are generated before building gripper_client
add_dependencies(gripper_client xarm_msgs_generate_messages_cpp xarm_gripper_generate_messages_cpp)

add_dependencies(gripper_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_client 
   ${catkin_LIBRARIES}
)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
