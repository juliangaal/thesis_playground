cmake_minimum_required(VERSION 3.0.2)
project(umeyama)

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

find_package(PkgConfig REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 COMPONENTS filesystem system program_options REQUIRED)

add_executable(umeyama src/umeyama.cpp)
target_include_directories(umeyama PRIVATE ${Eigen3_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(umeyama PRIVATE Catch2::Catch2WithMain fmt::fmt)
target_compile_options(umeyama PRIVATE -Wall -Weffc++ -Wuninitialized)

add_executable(pcl_normals src/pcl_normals.cpp)
target_include_directories(pcl_normals PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(pcl_normals PRIVATE fmt::fmt ${PCL_LIBRARIES})
target_compile_options(pcl_normals PRIVATE -Wall -Weffc++ -Wuninitialized -Wno-dev)

add_executable(dca src/dca_pcd.cpp src/dca.cpp src/dca.h src/util.cpp src/ply.cpp)
target_include_directories(dca PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(dca PRIVATE fmt::fmt ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(dca PRIVATE -Wall -Weffc++ -Wuninitialized -Wno-dev)

add_executable(registration_test src/test.cpp src/dca.cpp)
target_include_directories(registration_test PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(registration_test PRIVATE Catch2::Catch2WithMain fmt::fmt ${PCL_LIBRARIES})
