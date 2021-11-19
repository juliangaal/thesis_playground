cmake_minimum_required(VERSION 3.0.2)
project(subvoxelmap)

set(CMAKE_CXX_COMPILER_VERSION 14)

find_package(Catch2 3 REQUIRED)
find_package(fmt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    visualization_msgs
    roscpp
)

catkin_package(
    DEPENDS roscpp
)

add_executable(rviz_test src/rviz_test.cpp)
target_compile_options(rviz_test PRIVATE -Wall -Weffc++)
target_include_directories(rviz_test PRIVATE ${fmt_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
target_link_libraries(rviz_test PRIVATE fmt::fmt ${catkin_LIBRARIES})

add_subdirectory(ext/HighFive)
add_executable(tsdf_rviz src/tsdf_rviz.cpp)
target_compile_options(tsdf_rviz PRIVATE -Wall -Weffc++)
target_include_directories(tsdf_rviz PRIVATE ${fmt_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
target_link_libraries(tsdf_rviz PRIVATE fmt::fmt ${catkin_LIBRARIES} HighFive)

add_executable(subvoxelmap_test src/test.cpp)
target_include_directories(subvoxelmap_test PRIVATE ${Catch2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
target_link_libraries(subvoxelmap_test PRIVATE Catch2::Catch2WithMain fmt::fmt)

add_test(NAME test COMMAND subvoxelmap_test)