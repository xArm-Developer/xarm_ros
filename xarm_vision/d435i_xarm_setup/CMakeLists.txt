cmake_minimum_required(VERSION 2.8.3)
project(d435i_xarm_setup)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    tf
    object_recognition_msgs
    xarm_api
    xarm_planner
    xarm_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES d435i_xarm_setup
 CATKIN_DEPENDS sensor_msgs std_msgs xarm_planner xarm_msgs xarm_api
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
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

add_executable(findobj2d_grasp_api src/findobj_grasp_xarm_api.cpp)
target_link_libraries(findobj2d_grasp_api ${catkin_LIBRARIES})

add_executable(findobj2d_grasp_moveit src/findobj_grasp_moveit_planner.cpp)
add_dependencies(findobj2d_grasp_moveit xarm_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(findobj2d_grasp_moveit ${catkin_LIBRARIES})

add_executable(obj2base src/tf_object_to_base.cpp)
target_link_libraries(obj2base ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS
  findobj2d_grasp_api
  findobj2d_grasp_moveit
  obj2base
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
