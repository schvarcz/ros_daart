cmake_minimum_required(VERSION 2.8.3)
project(daart)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Find other dependencies are found with CMake's conventions

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/)
#
# set(IBEX_ROOT "/home/schvarcz/ibex-2.3.1-cplex/")
# find_package(IbexLib)
# if(NOT IBEX_FOUND)
#   message(ERROR "Unable to find IbexLib. Maybe you need to set IBEX_ROOT ${IBEX_ROOT}")
# endif()
#
# set(VIBES_ROOT "/home/schvarcz/vibes-lib/")
# find_package(VibesLib)
# if(NOT VIBES_FOUND)
#   message(ERROR "Unable to find VibesLib. Maybe you need to set VIBES_ROOT ${VIBES_ROOT}")
# endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES daart
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${IBEX_INCLUDE_DIRS}
  # ${VIBES_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(crc src/crc.c)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(daart ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(daart_speed_control_node src/daart_speed_control_node.cpp)
add_executable(daart_speed_control_node2 src/daart_speed_control_node2.cpp)
add_executable(daart_speed_control_node2_1 src/daart_speed_control_node2_1.cpp)
add_executable(daart_speed_control_node2_2 src/daart_speed_control_node2_2.cpp)
add_executable(daart_speed_control_node3 src/daart_speed_control_node3.cpp)
add_executable(daart_odom_node src/daart_odom_node.cpp)
add_executable(daart_odom_imu_node src/daart_odom_imu_node.cpp)
add_executable(daart_odom_scan_node src/daart_odom_scan_node.cpp)
add_executable(daart_odom_node2 src/daart_odom_node2.cpp)
add_executable(daart_rotate src/daart_rotate.cpp)
add_executable(daart_waypoints src/daart_waypoints.cpp)
add_executable(daart_waypoints2 src/daart_waypoints2.cpp)
add_executable(daart_waypoints3 src/daart_waypoints3.cpp)
add_executable(daart_waypoints3_1 src/daart_waypoints3_1.cpp)
add_executable(daart_waypoints3_2 src/daart_waypoints3_2.cpp)
add_executable(daart_waypoints_line src/daart_waypoints_line.cpp)
add_executable(daart_random_walk src/daart_random_walk.cpp)
add_executable(scan_copy src/scan_copy.cpp)
add_executable(pose2odom src/pose2odom.cpp)
add_executable(daart_odom_clean_node src/daart_odom_clean_node.cpp)
add_executable(daart_odom_trans_node src/daart_odom_trans_node.cpp)
# add_executable(robot_pose_iv src/robot_pose_iv.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(daart_speed_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(daart_speed_control_node
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_speed_control_node2
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_speed_control_node2_1
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_speed_control_node2_2
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_speed_control_node3
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_node
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_imu_node
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_scan_node
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_node2
   crc
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_rotate
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints2
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints3
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints3_1
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints3_2
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_waypoints_line
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_random_walk
   ${catkin_LIBRARIES}
)

target_link_libraries(scan_copy
   ${catkin_LIBRARIES}
)

target_link_libraries(pose2odom
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_clean_node
   ${catkin_LIBRARIES}
)

target_link_libraries(daart_odom_trans_node
   ${catkin_LIBRARIES}
)

#
# target_link_libraries(robot_pose_iv
#    ${catkin_LIBRARIES}
#    ${IBEX_LIBRARIES}
#    ${VIBES_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS daart daart_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(DIRECTORY launchers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_daart.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
