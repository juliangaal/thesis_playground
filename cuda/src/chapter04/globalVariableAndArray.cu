#include "../common/common.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <assert.h>
#include <algorithm>

/*
 * An example of using a statically declared global variable (devData) to store
 * a floating-point value on the device.
 */

#define SIZE 100

__device__ float devData;
__device__ unsigned int size;
__device__ float data[SIZE];

__global__ void checkGlobalVariable()
{
    // display the original value
    printf("Device: the value of the global variable is %f\n", devData);

    // alter the value
    devData += 2.0f;
}

__global__ void chapter4exercise1()
{
    auto t = threadIdx.x;
    printf("%d %d\n", blockIdx.x, t);
    if (t < size)
    {
        data[t] = 100;
    }
}

void checkResult(float *hostRef, float *gpuRef, const int size, float value)
{
    double epsilon = 1.0E-8;

    for (int i = 0; i < size; i++)
    {
        assert(abs(hostRef[i] - value) < epsilon);
    }

    printf("Arrays match.\n\n");
}


int main(void)
{
    // initialize the global variable
    float value = 3.14f;
    CHECK(cudaMemcpyToSymbol(devData, &value, sizeof(float)));
    printf("Host:   copied %f to the global variable devData\n", value);

    auto n = SIZE;
    CHECK(cudaMemcpyToSymbol(size, &n, sizeof(unsigned int)));
    printf("Host:   copied %d to the global variable size\n", n);

    // invoke the kernel
    checkGlobalVariable<<<1, 1>>>();

    // copy the global variable back to the host
    CHECK(cudaMemcpyFromSymbol(&value, devData, sizeof(float)));
    printf("Host:   the value changed by the kernel  to %f\n", value);

    float h_A[SIZE] = { 0 };
    std::fill(h_A, h_A+SIZE, value);
    for (int i = 0; i < n; i++)
    {
        assert(h_A[i] == value);
    }

    // copy data from host to device
    cudaMemcpyToSymbol(data, h_A, n * sizeof(float));
    dim3 block(SIZE);
    dim3 grid(1);
    chapter4exercise1<<<grid, block>>>();
    CHECK(cudaDeviceSynchronize());
    CHECK(cudaMemcpyFromSymbol(h_A, data, n * sizeof(float)));

    checkResult(h_A, data, n, 100);

    CHECK(cudaDeviceReset());
    return EXIT_SUCCESS;
}
