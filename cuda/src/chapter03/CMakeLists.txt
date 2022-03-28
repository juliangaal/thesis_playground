add_executable(simpleDivergence simpleDivergence.cu)
set_target_properties(simpleDivergence PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(simpleDeviceQuery simpleDeviceQuery.cu)
set_target_properties(simpleDeviceQuery PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrix sumMatrix.cu)
set_target_properties(sumMatrix PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(sumMatrix PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-m64 -gencode arch=compute_75,code=sm_75>)

add_executable(reduceInteger reduceInteger.cu)
set_target_properties(reduceInteger PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(reduceMin reduceMin.cu)
set_target_properties(reduceMin PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(nestedHelloWorld nestedHelloWorld.cu)
set_target_properties(nestedHelloWorld PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#add_executable(nestedReduce nestedReduce.cu)
#set_target_properties(nestedReduce PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
#
#add_executable(nestedReduce2 nestedReduce2.cu)
#set_target_properties(nestedReduce2 PROPERTIES CUDA_SEPARABLE_COMPILATION ON)