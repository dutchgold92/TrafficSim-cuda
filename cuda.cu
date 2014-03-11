#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>
#include <thrust/count.h>
#include <thrust/device_vector.h>

#define THREADS_PER_BLOCK 32

using namespace std;

__device__ unsigned int cuda_get_clearance(signed int *cells, unsigned int index, unsigned int road_index, unsigned int road_length)
{
    for(unsigned int i = (index + 1), r_i = (road_index + 1); r_i < road_length; i++ && r_i++)
    {
        if(cells[i] >= 0)
            return(r_i - road_index);
    }

    return UINT_MAX;
}

__global__ void cuda_apply_rules(signed int *cells, signed int *temp_cells, unsigned int *road_lengths, unsigned int road_count, unsigned int max_speed, unsigned int *vehicle_counts)
{
    unsigned int index = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int road = UINT_MAX;
    unsigned int road_index = index;
    unsigned int road_length;

    for(unsigned int i = 0, road_start = 0; i < road_count; i++)
    {
        if(index >= road_start && index < (road_start + road_lengths[i]))
        {
            road = i;
            break;
        }

        road_start += road_lengths[i];
    }

    if(road == UINT_MAX)
        return;

    road_length = road_lengths[road];

    for(unsigned int i = 0; i < road; i++)
        road_index -= road_lengths[i];

    if(cells[index] >= 0 && cells[index] < max_speed)   // accelerate
        if(cuda_get_clearance(cells, index, road_index, road_length) > (cells[index] + 1))
        {
            __syncthreads();
            cells[index]++;
        }

    if(cells[index] > 0)    // decelerate
    {
        unsigned int clearance = cuda_get_clearance(cells, index, road_index, road_length);
        __syncthreads();

        if(clearance <= cells[index])
            cells[index] = (clearance - 1);
    }

    if((cells[index] > 0) && false)    // random. FIXME: shoddy!
        cells[index]--;

    if(cells[index] >= 0)   // progress
    {
        unsigned int new_position = (index + cells[index]);
        unsigned int new_road_position = (road_index + cells[index]);
        temp_cells[index] = -1;

        if(new_road_position < road_length)
            temp_cells[new_position] = cells[index];
    }

    __syncthreads();

    if(temp_cells[index] >= 0)  // density
    {
        atomicAdd(&vehicle_counts[road], 1);
    }
}

extern "C"
float cuda_process_model(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed);

extern "C"
float cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int *road_lengths_d;
    unsigned int blocks_per_road = ceil((float)max_road_length / (float)THREADS_PER_BLOCK);
    unsigned int size = 0;
    unsigned int cell_count = 0;
    unsigned int vehicle_count = 0;
    unsigned int *vehicle_counts = new unsigned int[road_count];
    unsigned int *vehicle_counts_d;

    for(unsigned int i = 0; i < road_count; i++)
    {
        cell_count += road_lengths[i];
        vehicle_counts[i] = 0;
    }

    size = cell_count * sizeof(int);

    cudaMalloc((void**)&cells_d, size);
    cudaMalloc((void**)&temp_cells_d, size);
    cudaMalloc((void**)&road_lengths_d, (sizeof(int) * road_count));
    cudaMalloc((void**)&vehicle_counts_d, (sizeof(int) * road_count));

    int *cells_d_ptr = &cells_d[0];
    int *temp_cells_d_ptr = &temp_cells_d[0];

    for(int i = 0; i < road_count; i++)
    {
        cudaMemcpy(cells_d_ptr, cells[i], (sizeof(int) * road_lengths[i]), cudaMemcpyHostToDevice);
        cudaMemcpy(temp_cells_d_ptr, cells[i], (sizeof(int) * road_lengths[i]), cudaMemcpyHostToDevice);
        cells_d_ptr += road_lengths[i];
        temp_cells_d_ptr += road_lengths[i];
    }

    cudaMemcpy(road_lengths_d, road_lengths, (sizeof(int) * road_count), cudaMemcpyHostToDevice);
    cudaMemcpy(vehicle_counts_d, vehicle_counts, (sizeof(int) * road_count), cudaMemcpyHostToDevice);

    cuda_apply_rules<<<(blocks_per_road * road_count), THREADS_PER_BLOCK>>>(cells_d, temp_cells_d, road_lengths_d, road_count, max_speed, vehicle_counts_d);

    temp_cells_d_ptr = &temp_cells_d[0];

    for(int i = 0; i < road_count; i++)
    {
        cudaMemcpy(cells[i], temp_cells_d_ptr, (sizeof(int) * road_lengths[i]), cudaMemcpyDeviceToHost);
        temp_cells_d_ptr += road_lengths[i];
    }

    cudaMemcpy(vehicle_counts, vehicle_counts_d, (sizeof(int) * road_count), cudaMemcpyDeviceToHost);

    for(unsigned int i = 0; i < road_count; i++)
        vehicle_count += vehicle_counts[i];

    cudaFree(cells_d);
    cudaFree(temp_cells_d);
    cudaFree(road_lengths_d);
    cudaFree(vehicle_counts_d);
    delete[] vehicle_counts;

//    cout << cudaGetErrorString(cudaGetLastError()) << endl;
    return((float)vehicle_count / (float)cell_count);
}
