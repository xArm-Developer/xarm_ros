cmake_minimum_required(VERSION 3.5)
project(xarm_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED)

set(CXX_SDK_DIR cxx)
set(CXX_SDK_INC ${CXX_SDK_DIR}/include)
set(CXX_SDK_SRC ${CXX_SDK_DIR}/src)

set(CXX_SDK_SRC_XARM ${CXX_SDK_SRC}/xarm)
set(CXX_SDK_SRC_XARM_WRAPPER ${CXX_SDK_SRC_XARM}/wrapper)
set(CXX_SDK_SRC_XARM_CORE ${CXX_SDK_SRC_XARM}/core)
set(CXX_SDK_SRC_SERIAL ${CXX_SDK_SRC}/serial)

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_INC}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_INC}/xarm/wrapper/xarm_api.h
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_WRAPPER}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_CORE}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_SERIAL}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_WRAPPER}/xarm_api.cc
)
  message(FATAL_ERROR "xarm_sdk contains sub-modules, Please use the following command to get or update the sub-module code first
  $ git pull
  $ git submodule sync
  $ git submodule update --init --remote")
endif()

catkin_package(
  INCLUDE_DIRS ${CXX_SDK_INC}
  LIBRARIES xarm_cxx_sdk
)

include_directories(
  ${CXX_SDK_INC}
  ${catkin_INCLUDE_DIRS}
)

add_library(xarm_cxx_sdk
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_api.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_bio.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_events.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_gpio.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_gripper.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_motion.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_params.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_record.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_report.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_robotiq.cc
  ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_servo.cc

  ${CXX_SDK_SRC_XARM_CORE}/connect.cc
  ${CXX_SDK_SRC_XARM_CORE}/report_data.cc

  ${CXX_SDK_SRC_XARM_CORE}/common/crc16.cc
  ${CXX_SDK_SRC_XARM_CORE}/common/queue_memcpy.cc

  ${CXX_SDK_SRC_XARM_CORE}/debug/debug_print.cc

  ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd.cc
  ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd_ser.cc
  ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd_tcp.cc

   ${CXX_SDK_SRC_XARM_CORE}/os/network.cc

  ${CXX_SDK_SRC_XARM_CORE}/port/ser.cc
  ${CXX_SDK_SRC_XARM_CORE}/port/socket.cc

  ${CXX_SDK_SRC_SERIAL}/serial.cc
  ${CXX_SDK_SRC_SERIAL}/impl/unix.cc
  ${CXX_SDK_SRC_SERIAL}/impl/win.cc
  ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_linux.cc
  ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_osx.cc
  ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_win.cc
)

#dd_dependencies(xarm_cxx_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(MSVC)
  target_link_libraries(xarm_cxx_sdk 
    Ws2_32.lib
    ${catkin_LIBRARIES}
  )
endif()

install(TARGETS xarm_cxx_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# this is a bit non-standard, but doing otherwise would require changes to the external cxx sdk - TODO: later
install(DIRECTORY ${CXX_SDK_INC}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

