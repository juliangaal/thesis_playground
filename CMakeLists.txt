cmake_minimum_required(VERSION 3.0.2)
project(utils)

set(CMAKE_CXX_COMPILER_VERSION 14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  pcl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES utils
#  CATKIN_DEPENDS sensor_msgs
  DEPENDS PCL
)

add_executable(pcl_hline_reducer src/pcl_hline_reducer.cpp)
target_include_directories(pcl_hline_reducer PRIVATE include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_hline_reducer PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(pcl_hline_reducer PRIVATE -Wall -Werror)

add_executable(pizza_filter src/pizza_filter.cpp)
target_include_directories(pizza_filter PRIVATE include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pizza_filter PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(pizza_filter PRIVATE -Wall -Werror)
