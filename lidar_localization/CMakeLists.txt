cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization )

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  tf2
  eigen_conversions
  message_generation 
  std_srvs
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss_pos
  tf

)

add_service_files(
   FILES
   saveMap.srv
   optimizeMap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
#include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

#add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
#target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(grid_map_node src/apps/grid_map_node.cpp ${ALL_SRCS})
add_dependencies(grid_map_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_map_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(AEB_node src/apps/AEB_node.cpp ${ALL_SRCS})
add_dependencies(AEB_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(AEB_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(save_map_csv_node src/apps/save_map_csv_node.cpp ${ALL_SRCS})
add_dependencies(save_map_csv_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_map_csv_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(gnss_map_pub_node src/apps/gnss_map_pub_node.cpp ${ALL_SRCS})
add_dependencies(gnss_map_pub_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_map_pub_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(path_tracking_node src/apps/path_tracking_node.cpp ${ALL_SRCS})
add_dependencies(path_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracking_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})



add_executable(RRT_test_node src/apps/RRT_test_node.cpp ${ALL_SRCS})
add_dependencies(RRT_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(RRT_test_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(Just_RRT_test_node src/apps/Just_RRT_test_node.cpp ${ALL_SRCS})
add_dependencies(Just_RRT_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(Just_RRT_test_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


