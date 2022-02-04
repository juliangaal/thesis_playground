cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(cuda_test LANGUAGES CUDA CXX)

enable_language(CUDA)
include(CheckLanguage)
check_language(CUDA)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_executable(cuda_knn src/knn.cu)
set_target_properties(cuda_knn PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(hello_world src/hello_world.cu)
set_target_properties(hello_world PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

