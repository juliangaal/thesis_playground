cmake_minimum_required(VERSION 3.0.2)
project(registration)

set(CMAKE_CXX_STANDARD 17)

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

add_executable(dca_pcd src/dca_pcd.cpp src/dca.cpp src/viewer.cpp)
target_include_directories(dca_pcd PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(dca_pcd PRIVATE fmt::fmt ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(dca_pcd PRIVATE -Wall -Weffc++ -Wuninitialized -Wno-dev)

add_executable(dca_registration src/dca_registration.cpp src/dca.cpp src/viewer.cpp src/util.cpp)
target_include_directories(dca_registration PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src)
target_link_libraries(dca_registration PRIVATE fmt::fmt ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(dca_registration PRIVATE -Wall -Weffc++ -Wuninitialized -Wno-dev)

add_executable(test src/test.cpp src/dca.cpp src/util.cpp)
target_include_directories(test PRIVATE ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(test PRIVATE Catch2::Catch2WithMain fmt::fmt ${PCL_LIBRARIES})
