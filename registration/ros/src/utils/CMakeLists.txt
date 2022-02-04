cmake_minimum_required(VERSION 3.0.2)
project(utils)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_COMPILER_VERSION 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  sensor_msgs
  tf
  tf2
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package()

add_executable(pcl2pcd_node src/pcl2pcd.cpp)
target_include_directories(pcl2pcd_node PRIVATE ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} src)
target_link_libraries(pcl2pcd_node PRIVATE ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
