#include <stdio.h>

__global__ void hello_from_gpu(void)
{
	printf("Hello from GPU\n");
}

int main(void)
{
	printf("Hello World from CPU!\n");
	
	hello_from_gpu<<<1, 10>>>();
	cudaDeviceReset();
	return 0;
}
