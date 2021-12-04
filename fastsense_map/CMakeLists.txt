cmake_minimum_required(VERSION 3.0.2)
project(fastsense_map)

set(CMAKE_CXX_COMPILER_VERSION 17)

Include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.0.0-preview3
)
FetchContent_MakeAvailable(Catch2)

FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG        8.0.1
)
FetchContent_MakeAvailable(fmt)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

FetchContent_Declare(
        HighFive
        GIT_REPOSITORY https://github.com/BlueBrain/HighFive.git
        GIT_TAG        v2.3.1
)
FetchContent_MakeAvailable(HighFive)

add_executable(fastsense_map_test src/test.cpp src/global_map.cpp src/local_map.cpp)
target_include_directories(fastsense_map_test PRIVATE ${Eigen3_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(fastsense_map_test PRIVATE Catch2::Catch2WithMain fmt::fmt HighFive)
target_compile_options(fastsense_map_test PRIVATE -Wall -Weffc++ -Winitialized)

add_executable(fastsense_map_1d src/1d/test.cpp)
target_include_directories(fastsense_map_1d PRIVATE ${Eigen3_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(fastsense_map_1d PRIVATE Catch2::Catch2WithMain fmt::fmt HighFive)
target_compile_options(fastsense_map_1d PRIVATE -Wall -Weffc++)

add_test(NAME test COMMAND fastsense_map_test)