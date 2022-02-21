#include <iostream>
#include "hello_world.h"
#include "helpers.h"

int main(void)
{
    std::cout << "Hello World from CPU!\n";
    cuda::hello_from_gpu();
    cuda::reset_device();
    return 0;
}
