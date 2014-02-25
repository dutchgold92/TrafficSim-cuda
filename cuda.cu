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

__global__ void cuda_apply_accelerate_rule(signed int* cells, signed int* temp_cells, unsigned int road_length, unsigned int max_speed)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if(i >= road_length || cells[i] < 0 || cells[i] >= max_speed)
        return;

    if(cuda_get_clearance(cells, i, road_length) > (cells[i] + 1))
        temp_cells[i] = (cells[i] + 1);

}

extern "C"
void cuda_accelerate_rule(signed int *cells, signed int *temp_cells, unsigned int road_length, unsigned int max_speed);

void cuda_accelerate_rule(signed int* cells, signed int* temp_cells, unsigned int road_length, unsigned int max_speed)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int size = (sizeof(cells_d) * road_length);

    cudaMalloc(&cells_d, size);
    cudaMalloc(&temp_cells_d, size);

    cudaMemcpy(cells_d, cells, size, cudaMemcpyHostToDevice);
    cudaMemcpy(temp_cells_d, temp_cells, size, cudaMemcpyHostToDevice);

    cuda_apply_accelerate_rule<<<(road_length / 10),32>>>(cells_d, temp_cells_d, road_length, max_speed);   // FIXME: should use floor or ceil perhaps for blocks?

    cudaMemcpy(temp_cells, temp_cells_d, size, cudaMemcpyDeviceToHost);
    cudaFree(cells_d);
    cudaFree(temp_cells_d);

    //cout << cudaGetErrorString(cudaGetLastError()) << endl;
}

__global__ void cuda_apply_decelerate_rule(signed int* cells, signed int* temp_cells, unsigned int road_length)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if(i >= road_length || cells[i] <= 0)
        return;

    unsigned int clearance = cuda_get_clearance(cells, i, road_length);

    if(clearance <= cells[i])
        temp_cells[i] = (clearance - 1);

}

extern "C"
void cuda_decelerate_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

void cuda_decelerate_rule(signed int *cells, signed int *temp_cells, unsigned int road_length)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int size = (sizeof(cells_d) * road_length);

    cudaMalloc(&cells_d, size);
    cudaMalloc(&temp_cells_d, size);

    cudaMemcpy(cells_d, cells, size, cudaMemcpyHostToDevice);
    cudaMemcpy(temp_cells_d, temp_cells, size, cudaMemcpyHostToDevice);

    cuda_apply_decelerate_rule<<<(road_length / 10),32>>>(cells_d, temp_cells_d, road_length);   // FIXME: should use floor or ceil perhaps for blocks?

    cudaMemcpy(temp_cells, temp_cells_d, size, cudaMemcpyDeviceToHost);
    cudaFree(cells_d);
    cudaFree(temp_cells_d);

    //cout << cudaGetErrorString(cudaGetLastError()) << endl;
}

__global__ void cuda_apply_random_rule(signed int* cells, signed int* temp_cells, unsigned int road_length)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if(i >= road_length)
        return;

    if((cells[i] > 0) && false)    // FIXME: shoddy!
        temp_cells[i] = (cells[i] - 1);
}

extern "C"
void cuda_random_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

void cuda_random_rule(signed int *cells, signed int *temp_cells, unsigned int road_length)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int size = (sizeof(cells_d) * road_length);

    cudaMalloc(&cells_d, size);
    cudaMalloc(&temp_cells_d, size);

    cudaMemcpy(cells_d, cells, size, cudaMemcpyHostToDevice);
    cudaMemcpy(temp_cells_d, temp_cells, size, cudaMemcpyHostToDevice);

    cuda_apply_random_rule<<<(road_length / 10),32>>>(cells_d, temp_cells_d, road_length);   // FIXME: should use floor or ceil perhaps for blocks?

    cudaMemcpy(temp_cells, temp_cells_d, size, cudaMemcpyDeviceToHost);
    cudaFree(cells_d);
    cudaFree(temp_cells_d);

    //cout << cudaGetErrorString(cudaGetLastError()) << endl;
}

__global__ void cuda_apply_progress_rule(signed int* cells, signed int* temp_cells, unsigned int road_length)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    if(i >= road_length || cells[i] < 0)
        return;

    unsigned int new_position = (i + cells[i]);

    if(new_position < road_length)
        temp_cells[new_position] = cells[i];
}

extern "C"
void cuda_progress_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

void cuda_progress_rule(signed int *cells, signed int *temp_cells, unsigned int road_length)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int size = (sizeof(cells_d) * road_length);

    cudaMalloc(&cells_d, size);
    cudaMalloc(&temp_cells_d, size);

    cudaMemcpy(cells_d, cells, size, cudaMemcpyHostToDevice);
    cudaMemcpy(temp_cells_d, temp_cells, size, cudaMemcpyHostToDevice);

    cuda_apply_progress_rule<<<(road_length / 10),32>>>(cells_d, temp_cells_d, road_length);   // FIXME: should use floor or ceil perhaps for blocks?

    cudaMemcpy(temp_cells, temp_cells_d, size, cudaMemcpyDeviceToHost);
    cudaFree(cells_d);
    cudaFree(temp_cells_d);

    //cout << cudaGetErrorString(cudaGetLastError()) << endl;
}
