cmake_minimum_required(VERSION 2.8.3)
# cmake_minimum_required(VERSION 3.1.3)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# add_definitions(-std=c++14 -Wall -Werror)
project(cepheus_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
	rosbag
	roscpp
	std_msgs
	sensor_msgs
	cmake_modules
)

find_package(Eigen3 REQUIRED)

#find_package()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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

catkin_python_setup() #gia to python plotter

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
#  LIBRARIES cepheus_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)
#include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cepheus_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(cepheus_controller_node src/cepheus_controller.cpp)

# add_executable(foros_simcontroller_node src/foros_simcontroller.cpp) #nikiforos new
# add_executable(eross_control_node src/eross_control.cpp) #nikiforos, took from eros
# add_executable(foros_user_node src/foros_user.cpp)
# tou gazebo se sxolia
# add_executable(testsetter_node src/gazebo_nodes/testsetter.cpp)
# add_executable(new_foros_simcontroller_node src/gazebo_nodes/new_foros_simcontroller.cpp)
# add_executable(simple_publisher_node src/simple_publisher.cpp)
# add_executable(simple_recorder_node src/simple_recorder.cpp)
add_executable(robot_foros_controller_node src/robot_foros_controller.cpp) 
add_executable(arduino_test_node src/arduino_test.cpp)
add_executable(vicon_test_node src/vicon_test.cpp)
# add_executable(robot_foros_ignition_node src/robot_foros_ignition.cpp)
add_executable(motors_test_node src/motors_test.cpp)
# add_executable(gazebo_motors_test_node src/gazebo_nodes/gazebo_motors_test.cpp)
add_executable(recorder_node src/recorder.cpp)





## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(foros_user_node ${catkin_LIBRARIES}  )
# target_link_libraries(cepheus_controller_node ${catkin_LIBRARIES}  )
# target_link_libraries(foros_simcontroller_node ${catkin_LIBRARIES}  ) #nikiforos
# target_link_libraries(testsetter_node ${catkin_LIBRARIES}  ) #nikiforos
# target_link_libraries(new_foros_simcontroller_node ${catkin_LIBRARIES} ) #nikiforos
# target_link_libraries(simple_publisher_node ${catkin_LIBRARIES} )
# target_link_libraries(simple_recorder_node ${catkin_LIBRARIES} )
target_link_libraries(robot_foros_controller_node ${catkin_LIBRARIES} )
target_link_libraries(arduino_test_node ${catkin_LIBRARIES} )
target_link_libraries(vicon_test_node ${catkin_LIBRARIES} )
# target_link_libraries(robot_foros_ignition_node ${catkin_LIBRARIES})
target_link_libraries(motors_test_node ${catkin_LIBRARIES})
# target_link_libraries(gazebo_motors_test_node ${catkin_LIBRARIES})
target_link_libraries(recorder_node ${catkin_LIBRARIES})




# target_link_libraries(eross_control_node ${catkin_LIBRARIES}  _ros) #nikiforos, took from eros
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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cepheus_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(PROGRAMS
  src/real_time_plotter.py   # Assuming you placed the script in a "scripts" folder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) #gia to python plotter




