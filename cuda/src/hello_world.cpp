#include <iostream>
#include "hello_world.cuh"
#include "helpers.cuh"

int main(void)
{
    std::cout << "Hello World from CPU!\n";
    cuda::hello_from_gpu();
    cuda::reset_device();
    return 0;
}

