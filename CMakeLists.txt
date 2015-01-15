cmake_minimum_required(VERSION 2.8.3)
project(image_cloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  cv_bridge
  nodelet
  dynamic_reconfigure
  roscpp
  rospy
  rostime
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/edge_detector.cfg
  cfg/fusion.cfg
  cfg/harris3d.cfg
  cfg/pcl_image.cfg
  cfg/sync_image.cfg
  cfg/sync_remap.cfg
)

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
  LIBRARIES image_cloud
  CATKIN_DEPENDS image_geometry image_transport pcl_conversions pcl_ros OpenCV PCL boost cv_bridge nodelet roscpp rospy rostime sensor_msgs std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(PCL 1.7 REQUIRED)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/image_cloud
)

## Fusion Nodelets
add_library(fusion
  src/fusion/fusion.cpp
)

target_link_libraries(fusion
  ${catkin_LIBRARIES}
)

add_dependencies(fusion
  ${PROJECT_NAME}_gencfg
)

## image feature Nodelets
add_library(edge
  src/image_features/edge_detector_nodelet.cpp
)

add_dependencies(edge 
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(edge
  ${catkin_LIBRARIES}
)

## pcl feature Nodelets
add_library(harris3d
  src/pcl_features/harris3d_nodelet.cpp
)

add_dependencies(harris3d
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(harris3d
  ${catkin_LIBRARIES}
)

## Calibration Nodelets
add_library(pcl_to_image
  src/calibration/pcl_to_image_nodelet.cpp
)

target_link_libraries(pcl_to_image
  ${catkin_LIBRARIES}
)

add_dependencies(pcl_to_image 
  ${PROJECT_NAME}_gencfg
)


## Sync Nodelets
add_library(sync_remap
  src/sync/remap.cpp
)

target_link_libraries(sync_remap
  ${catkin_LIBRARIES}
)

add_dependencies(sync_remap 
  ${PROJECT_NAME}_gencfg
)

# Sync image nodelet
add_library(sync_image
  src/sync/sync_image.cpp
)

target_link_libraries(sync_image
  ${catkin_LIBRARIES}
)

add_dependencies(sync_image 
  ${PROJECT_NAME}_gencfg
)
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
# install(TARGETS image_cloud image_cloud_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_cloud.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
