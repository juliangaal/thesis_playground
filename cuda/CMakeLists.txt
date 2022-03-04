cmake_minimum_required(VERSION 3.16)
project(cuda CUDA CXX C)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

set(MY_CACHE_VARIABLE "VALUE" CACHE STRING "Description")

add_subdirectory(src/chapter02)
add_subdirectory(src/chapter03)
add_subdirectory(src/chapter04)