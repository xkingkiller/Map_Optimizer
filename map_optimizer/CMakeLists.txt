cmake_minimum_required(VERSION 2.8)
project(map_optimizer)

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf rosbag_storage visualization_msgs laser_geometry map_optimizer_msg)

find_package(Boost REQUIRED signals)

#############################
# Eigen ####################
#############################
find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb3 -march=native")

catkin_package()



add_executable(map_optimizer src/map_optimizer.cpp src/main.cpp)
target_link_libraries(map_optimizer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(map_optimizer ${catkin_EXPORTED_TARGETS})
endif()



install(TARGETS map_optimizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
