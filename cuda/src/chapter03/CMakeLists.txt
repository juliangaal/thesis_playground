add_executable(simpleDivergence simpleDivergence.cu)
set_target_properties(simpleDivergence PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(simpleDeviceQuery simpleDeviceQuery.cu)
set_target_properties(simpleDeviceQuery PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrix sumMatrix.cu)
set_target_properties(sumMatrix PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(sumMatrix PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-m64 -gencode arch=compute_75,code=sm_75>)