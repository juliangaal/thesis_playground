cmake_minimum_required(VERSION 3.16)
project(cuda CUDA CXX C)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

add_executable(sumMatrixOnGPU-timer src/chapter02/sumMatrixOnGPU-timer.cu)
set_target_properties(sumMatrixOnGPU-timer PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_subdirectory(src/chapter02)