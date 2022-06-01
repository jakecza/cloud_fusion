cmake_minimum_required(VERSION 2.8.3)
project(cloud_fusion)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++14)

set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

################################
## Add Paths to External Libs ##
################################


######################################
## Find Catkin Macros and Libraries ##
######################################
## Use to find ROS packages installed using:
## sudo apt-get install ros-${DISTRO}-${PKG}

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
  tf2
  pcl_ros
)

########################################################
## Find System-wide Packages (like Eigen, PCL, etc..) ##
########################################################
## System dependencies are found with CMake's conventions

find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
    MESSAGE("<cloud_fusion> CMake DID NOT FIND the package {PCL}")
endif(NOT PCL_FOUND)
if(PCL_FOUND)
    MESSAGE("<cloud_fusion> CMake FOUND the package {PCL}")
endif(PCL_FOUND)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    MESSAGE("<cloud_fusion> CMake DID NOT FIND the package {Eigen3}")
endif(NOT EIGEN3_FOUND)
if(EIGEN3_FOUND)
    MESSAGE("<cloud_fusion> CMake FOUND the package {Eigen3}")
endif(EIGEN3_FOUND)

############################
## Run Setup.py (if used) ##
############################

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
#   msg_to_generate.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   service_file.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
## Example vvv
# generate_messages(
#   DEPENDENCIES
#   _msgs
#   _msgs
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
## Catkin Specific Configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

#########################
## Include Directories ##
#########################
## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    #${pthread_INCLUDE_DIRS}
    #${rt_INCLUDE_DIRS}
)

## Add any definitions to be passed into CMake args from included package
# add_definitions(${PKG_DEFINITIONS})

###################
## Add Libraries ##
###################


## Add Clouds Library ##
########################
## Contains necessary src files to handle pointclouds 

# add_library(clouds
#   src/lib/.cpp
# )

## Add cmake target dependencies of the library as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#####################
## Add Executables ##
#####################
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


## Add CloudNode Executable ##
################################

add_executable(cloud_node
  src/lib/sensor_clouds.cpp
  src/lib/CloudNodeClass.cpp
  src/CloudNode.cpp
)

target_link_libraries(cloud_node 
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(cloud_node ${catkin_EXPORTED_TARGETS})

## Add Test Executables ##
##########################

#add_executable(test_auto_cal
#        src/test/test_auto_cal.cpp
#        src/lib/ros_measurement_models.cpp
#        src/lib/sixDOF_dynamic_system.cpp
#        src/lib/sixDOF_measurement_models.cpp
#        src/lib/smoother_subscriber.cpp
#        )
#target_link_libraries(test_auto_cal
#        ${catkin_LIBRARIES}
#        ${EIGEN3_LIBRARIES}
#        ${PCL_LIBRARIES}
#        fusion
#        )

#add_executable(test_EKS
#        src/test/test_EKS.cpp
#        src/lib/ros_measurement_models.cpp
#        src/lib/sixDOF_dynamic_system.cpp
#        src/lib/sixDOF_measurement_models.cpp
#        src/lib/smoother_subscriber.cpp
#        )
#target_link_libraries(test_EKS
#        ${catkin_LIBRARIES}
#        ${EIGEN3_LIBRARIES}
#        ${PCL_LIBRARIES}
#        fusion
#        )

#####################
## Install Targets ##
#####################
## All install targets should use catkin DESTINATION variables
## See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mmwave_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
