#include "hello_world.cuh"
#include <stdio.h>

__global__ void hello_from_gpu_impl()
{
    printf("Hello from GPU\n");
}

void cuda::hello_from_gpu()
{
    hello_from_gpu_impl<<<1, 10>>>();
}


