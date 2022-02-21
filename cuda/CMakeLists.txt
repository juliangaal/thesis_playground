cmake_minimum_required(VERSION 3.16)
project(cuda CUDA CXX C)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

add_executable(main src/main.cu)
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(main PRIVATE -std=c++11)

add_executable(cuda_knn_cu src/knn.cu)
set_target_properties(cuda_knn_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_library(helpers_cu src/helpers.cu)
set_target_properties(helpers_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_library(hello_world_cu src/hello_world.cu)
set_target_properties(hello_world_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world PRIVATE hello_world_cu helpers_cu)
target_compile_options(hello_world PRIVATE -Wall -Weffc++)

add_executable(sum_arrays src/sum_arrays.c)

add_executable(sum_arrays_cu src/sum_arrays.cu)
set_target_properties(sum_arrays_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(checkDimension src/checkDimension.cu)
set_target_properties(checkDimension PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(defineGridBlock src/defineGridBlock.cu)
set_target_properties(defineGridBlock PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(checkThreadIndex_cu src/checkThreadIndex.cu)
set_target_properties(checkThreadIndex_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrix_2dgrid_2dblock src/sumMatrix_2dgrid_2dblock.cu)
set_target_properties(sumMatrix_2dgrid_2dblock PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrix_2dgrid_1dblock src/sumMatrix_2dgrid_1dblock.cu)
set_target_properties(sumMatrix_2dgrid_1dblock PROPERTIES CUDA_SEPARABLE_COMPILATION ON)