#include "helpers.cuh"

void cuda::reset_device()
{
    cudaDeviceReset();
}
