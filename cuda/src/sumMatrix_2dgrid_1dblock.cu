#include <cuda_runtime.h>
#include <stdio.h>
#include <time.h>
#include <iostream>

#include "common.h"

void checkResult(float *hostRef, float *gpuRef, const int N)
{
    double epsilon = 1.0E-8;

    for (int i = 0; i < N; i++)
    {
        if (abs(hostRef[i] - gpuRef[i]) > epsilon)
        {
            printf("Arrays do not match!\n");
            printf("host %5.2f gpu %5.2f at current %d\n", hostRef[i],
                   gpuRef[i], i);
            return;
        }
    }

    printf("Arrays match.\n\n");
}

void sumMatrixOnHost(float *A, float *B, float *C, const int nx,
                     const int ny)
{
    float *ia = A;
    float *ib = B;
    float *ic = C;

    for (int iy = 0; iy < ny; iy++)
    {
        for (int ix = 0; ix < nx; ix++)
        {
            ic[ix] = ia[ix] + ib[ix];

        }

        ia += nx;
        ib += nx;
        ic += nx;
    }
}

void initialData(float *ip, int size)
{
    // generate different seed for random number
    time_t t;
    srand((unsigned) time(&t));

    for (int i = 0; i < size; i++)
    {
        ip[i] = (float)(rand() & 0xFF ) / 10.0f;
    }
}


__global__ void sumMatrixOnGPU1D(const float *A, const float *B, float *C, int nx, int ny)
{
    // data point in kernel refers to this block coordinate in 2D matrix
    auto ix = blockIdx.x * blockDim.x + threadIdx.x;
    auto iy = blockIdx.y;
    auto idx = iy * nx + ix;

    if (ix < nx && iy < ny)
    {
        C[idx] = A[idx] + B[idx];
    }
}

int main()
{
    int dev = 0;
    cudaDeviceProp deviceProp;
    CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using Device %d: %s\n", dev, deviceProp.name);
    CHECK(cudaSetDevice(dev));

    // setup size
    int nx = 1 << 14;
    int ny = 1 << 14;

    int nxy = nx * ny;
    int nBytes = nxy * sizeof(float);
    printf("Matrix size: nx %d ny %d\n", nx, ny);

    // initialize arrays (host)
    float *h_A, *h_B, *hostRef, *gpuRef;
    auto start = seconds();
    h_A     = (float*)malloc(nBytes);
    h_B     = (float*)malloc(nBytes);
    hostRef = (float*)malloc(nBytes);
    gpuRef  = (float*)malloc(nBytes);

    memset(hostRef, 0, nBytes);
    memset(gpuRef, 0, nBytes);
    auto end = seconds() - start;
    printf("host memory alloc took %f sec\n", end);

    initialData(h_A, nxy);
    initialData(h_B, nxy);

    // run host code
    start = seconds();
    sumMatrixOnHost(h_A, h_B, hostRef, nx, ny);
    end = seconds() - start;
    printf("sumMatrixOnHost elapsed %f sec\n", end);

    // initialize arrays (device) and malloc device global memory
    float *d_A, *d_B, *d_C;
    start = seconds();
    CHECK(cudaMalloc((float**)&d_A, nBytes));
    CHECK(cudaMalloc((float**)&d_B, nBytes));
    CHECK(cudaMalloc((float**)&d_C, nBytes));

    // copy host data to device data
    CHECK(cudaMemcpy(d_A, h_A, nBytes, cudaMemcpyHostToDevice));
    CHECK(cudaMemcpy(d_B, h_B, nBytes, cudaMemcpyHostToDevice));
    CHECK(cudaMemcpy(d_C, gpuRef, nBytes, cudaMemcpyHostToDevice));
    end = seconds() - start;
    printf("Cuda memory alloc: %f sec\n", end);

    // launch kernel and wait for results
    dim3 block(128);
    dim3 grid((nx + block.x - 1) / block.x, ny);

    start = seconds();
    sumMatrixOnGPU1D<<<grid, block>>>(d_A, d_B, d_C, nx, ny);
    CHECK(cudaDeviceSynchronize());
    end = seconds() - start;
    printf("sumMatrixOnGPU2D <<<(%d,%d), (%d,%d)>>> elapsed %f sec\n", grid.x, grid.y, block.x, block.y, end);

    // copy device data to host
    CHECK(cudaMemcpy(gpuRef, d_C, nBytes,cudaMemcpyDeviceToHost));
    end = seconds() - start;
    printf("Result in host memory after %f sec", end);

    // compare
    checkResult(hostRef, gpuRef, nxy);

    // free memory device
    CHECK(cudaFree(d_A));
    CHECK(cudaFree(d_B));
    CHECK(cudaFree(d_C));

    // free memory host
    free(h_A);
    free(h_B);
    free(hostRef);
    free(gpuRef);

    return 0;
}

