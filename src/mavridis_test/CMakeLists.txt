cmake_minimum_required(VERSION 2.8.3)
project(mavridis_test)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  rospy
  urdf
  tf
  control_msgs
  trajectory_msgs
  kdl_parser
  hardware_interface
  controller_interface
  controller_manager
  angles
  genmsg
  actionlib_msgs
  actionlib
  rosbag
)

# find_package(Eigen3 REQUIRED)
# include_directories(${Eigen3_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

add_action_files(DIRECTORY action FILES RightCatchObject.action)
add_action_files(DIRECTORY action FILES LeftCatchObject.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)


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
  INCLUDE_DIRS include
  LIBRARIES cepheus_robot
  CATKIN_DEPENDS ros_control roscpp std_msgs rospy
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(  ${catkin_INCLUDE_DIRS} )

## Declare a C++ library
add_library(librtd_dm7820 include/${PROJECT_NAME}/librtd-dm7820.c)
add_library(cepheus_init_ctrls src/cepheus_init_ctrls.cpp)
add_library(cepheus_arms_operations src/cepheus_arms_operations)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(cepheus_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(camera_odom_node src/camera_odom.cpp)
#add_executable(mouse_odom_node src/mouse_odom_2.cpp)
# add_executable(phase_space_inverted_node src/phase_space_axes_swap.cpp)
#add_executable(thrusters_node src/thruster_controller.cpp)
add_executable(cepheus_interface_node src/cepheus_interface.cpp src/cepheus_hardware.cpp src/cepheus_hardware.h)
#add_executable(FT_interface_node src/bota_FTS.cpp)
#add_executable(thrusters_interface_node src/thrusters_interface.cpp)
add_executable(base_controller_node src/base_controller.cpp)
add_executable(base_planner_node src/base_planner.cpp)
add_executable(cepheus_ctrl src/cepheus_ctrl.cpp)
#add_executable(cepheus_force_sensor src/bota_FTS.cpp)
#add_executable(gripper_target_broadcaster src/gripper_target_broadcaster.cpp)
add_executable(new_base_planner_node src/new_base_planner.cpp)
# add_executable(alex_gripper_control_node src/alex_gripper_control.cpp)
add_executable(initialise_arm_node src/initialise_arm.cpp)
################################################################################################################
# add_executable(eross_control_node src/eross/eross_control.cpp)
################################################################################################################
# add_executable(eross_control_relative_node src/eross/eross_control_relative.cpp)
# add_executable(EROSSGripper src/eross/EROSSGripper_v3.cpp)
# add_executable(EROSSGripper_v3 src/eross/EROSSGripper_v3.cpp src/eross/Definitions.h)
# add_executable(test_node src/test.cpp)
# add_executable(calibrate_imu_node src/calibrate_imu.cpp)
# add_executable(obsidian_bp1a_node src/obsidian/obsidian_bp1a.cpp)
# add_executable(obsidian_bp2_node src/obsidian/obsidian_bp2.cpp)
# add_executable(obsidian_bp1b_node src/obsidian/obsidian_bp1b.cpp)
# add_executable(thruster_exp_node src/obsidian/thruster_exp.cpp)
# add_executable(obsidian_sc1_node src/obsidian/obsidian_sc1.cpp)
# add_executable(obsidian_sc2_node src/obsidian/obsidian_sc2.cpp)
# add_executable(obsidian_sc2_validation_node src/obsidian/obsidian_sc2_validation.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(cepheus_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(camera_odom_node  ${catkin_LIBRARIES})
#target_link_libraries(mouse_odom_node  ${catkin_LIBRARIES})
# target_link_libraries(phase_space_inverted_node  ${catkin_LIBRARIES})
#target_link_libraries(thrusters_node  ${catkin_LIBRARIES} librtd_dm7820)
target_link_libraries(cepheus_interface_node  ${catkin_LIBRARIES} librtd_dm7820 cepheus_init_ctrls cepheus_arms_operations ${gsl_LIBRARIES})
#target_link_libraries(FT_interface_node  ${catkin_LIBRARIES})
target_link_libraries(cepheus_ctrl  ${catkin_LIBRARIES})
#target_link_libraries(thrusters_interface_node  ${catkin_LIBRARIES} librtd_dm7820)
target_link_libraries(base_controller_node  ${catkin_LIBRARIES} )
target_link_libraries(base_planner_node  ${catkin_LIBRARIES} )
#target_link_libraries(cepheus_force_sensor ${catkin_LIBRARIES})
#target_link_libraries(gripper_target_broadcaster ${catkin_LIBRARIES})
target_link_libraries(new_base_planner_node ${catkin_LIBRARIES})
# target_link_libraries(alex_gripper_control_node ${catkin_LIBRARIES})
target_link_libraries(initialise_arm_node ${catkin_LIBRARIES})
# target_link_libraries(eross_control_node ${catkin_LIBRARIES})
# target_link_libraries(eross_control_relative_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(EROSSGripper ${catkin_LIBRARIES} libEposCmd.so)
# target_link_libraries(EROSSGripper_v3 ${catkin_LIBRARIES})
# target_link_libraries(test_node ${catkin_LIBRARIES})
# target_link_libraries(calibrate_imu_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(obsidian_bp1a_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(obsidian_bp2_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(obsidian_bp1b_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(thruster_exp_node ${catkin_LIBRARIES})
# target_link_libraries(obsidian_sc1_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(obsidian_sc2_node ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(obsidian_sc2_validation_node ${catkin_LIBRARIES} yaml-cpp)


# set_target_properties(EROSSGripper PROPERTIES COMPILE_FLAGS "-lEposCmd")
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
# install(TARGETS cepheus_robot cepheus_robot_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cepheus_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
