cmake_minimum_required(VERSION 3.8)
project(cost_map_proto)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Werror")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  sensor_msgs
  tf
  pcl_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp 0.5 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cost_map_proto
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
  src/scan_frame.cpp
  src/occupancy_grid.cpp
  src/depth_to_scan.cpp
)

add_library(${PROJECT_NAME}
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(depth_buffer_node
  src/depth_buffer_node.cpp
)
add_executable(scan_buffer_node
  src/scan_buffer_node.cpp
)

target_link_libraries(depth_buffer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(scan_buffer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_scan_buffer_node
  src/test_scan_buffer_node.cpp
)
target_link_libraries(test_scan_buffer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_occupancy_grid
  src/test_occupancy_grid.cpp
)
target_link_libraries(test_occupancy_grid
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_crop
  src/test_crop.cpp
)
target_link_libraries(test_crop
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(scan_image_gen_node
  src/scan_image_gen_node.cpp
)
target_link_libraries(scan_image_gen_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cost_map_proto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
