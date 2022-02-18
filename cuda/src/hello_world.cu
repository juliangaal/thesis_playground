#include "hello_world.h"
#include <stdio.h>

__global__ void hello_from_gpu_krnl()
{
    printf("Hello from GPU\n");
}

void cuda::hello_from_gpu()
{
    hello_from_gpu_krnl<<<1, 10>>>();
}
