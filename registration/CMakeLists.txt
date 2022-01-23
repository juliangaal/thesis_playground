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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(umeyama src/umeyama.cpp)
target_include_directories(umeyama PRIVATE ${Eigen3_INCLUDE_DIRS} ${Catch2_INCLUDE_DIRS} src)
target_link_libraries(umeyama PRIVATE Catch2::Catch2WithMain fmt::fmt)
target_compile_options(umeyama PRIVATE -Wall -Weffc++ -Wuninitialized)