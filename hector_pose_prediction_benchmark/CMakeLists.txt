cmake_minimum_required(VERSION 3.0.2)
project(hector_pose_prediction_benchmark)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  hector_pose_prediction_benchmark_tools
  roscpp
  sdf_contact_estimation
  hector_heightmap_pose_prediction
  hector_world_heightmap
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS hector_pose_prediction_benchmark_tools roscpp sdf_contact_estimation hector_heightmap_pose_prediction gpu_heightmap_pose_prediction hector_world_heightmap
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
  include/${PROJECT_NAME}/benchmark_suite.h
  include/${PROJECT_NAME}/pose_predictor_factory.h
)

set(SOURCES
  src/benchmark_suite.cpp
  src/pose_predictor_factory.cpp
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/benchmark_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME benchmark_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

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

