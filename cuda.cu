#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>
#include <thrust/random.h>

#define THREADS_PER_BLOCK 32

using namespace std;

__device__
unsigned int cuda_hash(unsigned int a)
{
    a = (a+0x7ed55d16) + (a<<12);
    a = (a^0xc761c23c) ^ (a>>19);
    a = (a+0x165667b1) + (a<<5);
    a = (a+0xd3a2646c) ^ (a<<9);
    a = (a+0xfd7046c5) + (a<<3);
    a = (a^0xb55a4f09) ^ (a>>16);
    return a;
}

__device__
float cuda_get_random(unsigned int seed_input)
{
    unsigned int seed = cuda_hash(seed_input);
    thrust::default_random_engine rng(seed);
    thrust::uniform_real_distribution<float> random(0,1);
    return random(rng);
}

__device__ unsigned int cuda_get_clearance(signed int *cells, unsigned int index, unsigned int road_index, unsigned int road_length)
{
    for(unsigned int i = (index + 1), r_i = (road_index + 1); r_i < road_length; i++ && r_i++)
    {
        if(cells[i] >= 0)
            return(r_i - road_index);
    }

    return UINT_MAX;
}

__global__ void cuda_apply_rules(signed int *cells, signed int *temp_cells, unsigned int *road_lengths, unsigned int road_count, unsigned int max_speed, unsigned int *vehicle_counts, unsigned int *seeds)
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

    if((cells[index] > 0) && (cuda_get_random(seeds[index] * threadIdx.x) <= 0.2))    // random
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
    unsigned int *seeds;
    unsigned int *seeds_d;

    for(unsigned int i = 0; i < road_count; i++)
    {
        cell_count += road_lengths[i];
        vehicle_counts[i] = 0;
    }
    size = cell_count * sizeof(int);

    seeds = new unsigned int[cell_count];
    for(int i = 0; i < cell_count; i++)
        seeds[i] = rand() % 10000;

    cudaMalloc((void**)&cells_d, size);
    cudaMalloc((void**)&temp_cells_d, size);
    cudaMalloc((void**)&road_lengths_d, (sizeof(int) * road_count));
    cudaMalloc((void**)&vehicle_counts_d, (sizeof(int) * road_count));
    cudaMalloc((void**)&seeds_d, size);

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
    cudaMemcpy(seeds_d, seeds, size, cudaMemcpyHostToDevice);

    cuda_apply_rules<<<(blocks_per_road * road_count), THREADS_PER_BLOCK>>>(cells_d, temp_cells_d, road_lengths_d, road_count, max_speed, vehicle_counts_d, seeds_d);

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
    cudaFree(seeds_d);
    delete[] vehicle_counts;
    delete[] seeds;

//    cout << cudaGetErrorString(cudaGetLastError()) << endl;
    return((float)vehicle_count / (float)cell_count);
}
