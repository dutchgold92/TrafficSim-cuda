#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>

using namespace std;

__device__ unsigned int cuda_get_clearance(signed int* cells, unsigned int index, unsigned int road_length)
{
    for(unsigned int i = (index + 1); i < road_length; i++)
    {
        if(cells[i] >= 0)
            return(i - index);
    }

    return UINT_MAX;
}

__global__ void cuda_apply_vehicle_rules(signed int* cells, signed int* temp_cells, unsigned int road_length, unsigned int max_speed)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if(i >= road_length)
        return;

    if(cells[i] >= 0 && cells[i] < max_speed)   // accelerate
        if(cuda_get_clearance(cells, i, road_length) > (cells[i] + 1))
            cells[i]++;

    if(cells[i] > 0)    // decelerate
    {
        unsigned int clearance = cuda_get_clearance(cells, i, road_length);

        if(clearance <= cells[i])
            cells[i] = (clearance - 1);
    }

    if((cells[i] > 0) && false)    // random. FIXME: shoddy!
        cells[i]--;

    if(cells[i] >= 0)   // progress
    {
        unsigned int new_position = (i + cells[i]);
        temp_cells[i] = -1;

        if(new_position < road_length)
            temp_cells[new_position] = cells[i];
    }
}

extern "C"
void cuda_vehicle_rules(signed int* cells, unsigned int road_length, unsigned int max_speed);

extern "C"
void cuda_vehicle_rules(signed int* cells, unsigned int road_length, unsigned int max_speed)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int size = (sizeof(cells_d) * road_length);

    cudaMalloc(&cells_d, size);
    cudaMalloc(&temp_cells_d, size);
    cudaMemcpy(cells_d, cells, size, cudaMemcpyHostToDevice);
    cudaMemcpy(temp_cells_d, cells, size, cudaMemcpyHostToDevice);

    cuda_apply_vehicle_rules<<<(road_length / 10),32>>>(cells_d, temp_cells_d, road_length, max_speed);   // FIXME: should use floor or ceil perhaps for blocks?

    cudaMemcpy(cells, temp_cells_d, size, cudaMemcpyDeviceToHost);
    cudaFree(cells_d);
    cudaFree(temp_cells_d);

    //cout << cudaGetErrorString(cudaGetLastError()) << endl;
}
