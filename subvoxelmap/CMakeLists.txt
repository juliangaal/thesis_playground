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

add_executable(subvoxelmap src/main.cpp)
target_compile_options(subvoxelmap PRIVATE -Wall -Weffc++)
target_include_directories(subvoxelmap PRIVATE ${fmt_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
target_link_libraries(subvoxelmap PRIVATE fmt::fmt ${catkin_LIBRARIES})

add_executable(subvoxelmap_test src/test.cpp)
target_include_directories(subvoxelmap_test PRIVATE ${Catch2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
target_link_libraries(subvoxelmap_test PRIVATE Catch2::Catch2WithMain fmt::fmt)

add_test(NAME test COMMAND subvoxelmap_test)