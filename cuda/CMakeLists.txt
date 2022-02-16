cmake_minimum_required(VERSION 3.21)
project(cuda CUDA CXX)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)

add_executable(main src/main.cu)
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(main PRIVATE -std=c++11)

add_executable(cuda_knn_cu src/knn.cu)
set_target_properties(cuda_knn_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_library(hello_world_cu src/hello_world.cu src/helpers.cu)
set_target_properties(hello_world_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world PRIVATE hello_world_cu)
target_compile_options(hello_world PRIVATE -Wall -Weffc++)
