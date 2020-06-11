cmake_minimum_required(VERSION 2.8.3)

project(xarm_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
  std_msgs
  actionlib
  actionlib_msgs
  xarm_api
)

include_directories(
  include
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
  CATKIN_DEPENDS actionlib_msgs xarm_api
)

find_package(roslaunch)

add_executable(gripper_action_server 
   src/gripper_action_server.cpp 
)

add_dependencies(gripper_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_action_server 
   ${catkin_LIBRARIES}
)

add_executable(gripper_client 
   src/gripper_client.cpp 
)
add_dependencies(gripper_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_client 
   ${catkin_LIBRARIES}
)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
