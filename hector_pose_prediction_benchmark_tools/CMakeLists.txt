cmake_minimum_required(VERSION 3.0.2)
project(hector_pose_prediction_benchmark_tools)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  hector_pose_prediction_interface
  nav_msgs
  hector_stability_metrics
  geometry_msgs
  tf2_msgs
  sensor_msgs
  rosbag
  eigen_conversions
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hector_pose_prediction_benchmark_tools
 CATKIN_DEPENDS hector_pose_prediction_interface nav_msgs hector_stability_metrics geometry_msgs tf2_msgs sensor_msgs rosbag eigen_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS
  include/${PROJECT_NAME}/pose_prediction_benchmark.h
  include/${PROJECT_NAME}/data_point.h
  include/${PROJECT_NAME}/util.h
)

set(SOURCES
  src/pose_prediction_benchmark.cpp
  src/data_point.cpp
  src/util.cpp
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
