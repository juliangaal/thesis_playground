add_executable(sumMatrixOnGPU-timer-2index sumMatrixOnGPU-timer-2index.cu)
set_target_properties(sumMatrixOnGPU-timer-2index PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(checkDimension checkDimension.cu)
set_target_properties(checkDimension PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(defineGridBlock defineGridBlock.cu)
set_target_properties(defineGridBlock PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(checkThreadIndex_cu checkThreadIndex.cu)
set_target_properties(checkThreadIndex_cu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrixOnGPU-2D-grid-2D-block sumMatrixOnGPU-2D-grid-2D-block.cu)
set_target_properties(sumMatrixOnGPU-2D-grid-2D-block PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrixOnGPU-2D-grid-1D-block sumMatrixOnGPU-2D-grid-1D-block.cu)
set_target_properties(sumMatrixOnGPU-2D-grid-1D-block PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(sumMatrixOnGPU-2D-grid-1D-block-int sumMatrixOnGPU-2D-grid-2D-block-int.cu)
set_target_properties(sumMatrixOnGPU-2D-grid-1D-block-int PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(checkDeviceInformation checkDeviceInformation.cu)
set_target_properties(checkDeviceInformation PROPERTIES CUDA_SEPARABLE_COMPILATION ON)