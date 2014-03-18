#include "roadlink.h"
#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>
#include <thrust/random.h>

#define THREADS_PER_BLOCK 32

using namespace std;

road_link *road_links_d;
signed int *cells_d;

extern "C"
void cuda_init(road_link *road_links, unsigned int road_link_count);

void cuda_init(road_link *road_links, unsigned int road_link_count)
{
    cudaMalloc((void**)&road_links_d, (sizeof(road_link) * road_link_count));
    cudaMemcpy(road_links_d, road_links, (sizeof(road_link) * road_link_count), cudaMemcpyHostToDevice);
}

__device__ void cuda_toggle_road_links_by_origin(road_link *road_links, unsigned int road_link_count)
{
    bool toggled = false;

    for(unsigned int loops = 0; loops < 2; loops++)
    {
        for(unsigned int i = 0; i < road_link_count; i++)
        {
            if(!toggled && road_links[i].origin_road == blockIdx.x && road_links[i].active)
            {
                road_links[i].active = false;
                toggled = true;
            }
            else if(toggled && road_links[i].origin_road == blockIdx.x)
            {
                road_links[i].active = true;
                return;
            }
        }
    }
}

__device__ void cuda_toggle_road_links_by_destination(road_link *road_links, unsigned int road_link_count)
{
    bool toggled = false;

    for(unsigned int loops = 0; loops < 2; loops++)
    {
        for(unsigned int i = 0; i < road_link_count; i++)
        {
            if(!toggled && road_links[i].destination_road == blockIdx.x && road_links[i].active)
            {
                road_links[i].active = false;
                toggled = true;
            }
            else if(toggled && road_links[i].destination_road == blockIdx.x)
            {
                road_links[i].active = true;
                return;
            }
        }
    }
}

__global__ void cuda_toggle_road_links(road_link *road_links, unsigned int road_link_count)
{
    cuda_toggle_road_links_by_origin(road_links, road_link_count);
    cuda_toggle_road_links_by_destination(road_links, road_link_count);
}

__device__ unsigned int cuda_hash(unsigned int a)
{
    a = (a+0x7ed55d16) + (a<<12);
    a = (a^0xc761c23c) ^ (a>>19);
    a = (a+0x165667b1) + (a<<5);
    a = (a+0xd3a2646c) ^ (a<<9);
    a = (a+0xfd7046c5) + (a<<3);
    a = (a^0xb55a4f09) ^ (a>>16);
    return a;
}

__device__ float cuda_get_random(unsigned int seed_input)
{
    unsigned int seed = cuda_hash(seed_input);
    thrust::default_random_engine rng(seed);
    thrust::uniform_real_distribution<float> random(0,1);
    return random(rng);
}

/**
 * Returns index of next open road.
 * Returns -1 if no next road is open, or -2 if there is no next road.
 */
__device__ signed int cuda_get_next_road(unsigned int road, road_link *road_links, unsigned int road_link_count)
{
    for(unsigned int i = 0; i < road_link_count; i++)
        if(road_links[i].active && road_links[i].origin_road == road)
            return road_links[i].destination_road;

    for(unsigned int i = 0; i < road_link_count; i++)
        if(road_links[i].origin_road == road)
            return -1;

    return -2;
}

__device__ signed int cuda_get_road_link(unsigned int origin_road, unsigned int destination_road, road_link *road_links, unsigned int road_link_count)
{
    for(unsigned int i = 0; i < road_link_count; i++)
        if(road_links[i].origin_road == origin_road && road_links[i].destination_road == destination_road)
            return i;

    return -1;
}

__device__ unsigned int cuda_get_first_index_of_road(unsigned int road, unsigned int *road_lengths)
{
    unsigned int index = 0;

    for(unsigned int i = 0; i < road; i++)
        index += road_lengths[i];

    return index;
}

__device__ unsigned int cuda_get_clearance(signed int *cells, unsigned int index, unsigned int road, unsigned int road_index, unsigned int road_length, unsigned int *road_lengths, road_link *road_links, unsigned int road_link_count, unsigned int max_speed)
{
    unsigned int clearance = 0;
    signed int next_road;

    for(unsigned int i = (index + 1), r_i = (road_index + 1); r_i < road_length && clearance <= max_speed; i++ && r_i++)
    {
        clearance++;

        if(cells[i] >= 0)
            return(clearance);
        else if(r_i == (road_length - 1))
        {
            next_road = cuda_get_next_road(road, road_links, road_link_count);

            if(next_road >= 0)
            {
                r_i = 0;
                road_length = road_lengths[next_road];
                i = cuda_get_first_index_of_road(next_road, road_lengths);
            }
            else if(next_road == -1)
                return clearance;
            else if(next_road == -2)
                return UINT_MAX;
        }
    }

    return UINT_MAX;
}

__global__ void cuda_apply_rules(signed int *cells, signed int *temp_cells, unsigned int *road_lengths, unsigned int road_count, unsigned int max_speed, unsigned int *vehicle_counts, unsigned int random_seed, road_link *road_links, unsigned int road_link_count)
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

    if(cells[index] >= 0 && cells[index] < max_speed) // accelerate
        if(cuda_get_clearance(cells, index, road, road_index, road_length, road_lengths, road_links, road_link_count, max_speed) > (cells[index] + 1))
        {
            __syncthreads();
            cells[index]++;
        }

    if(cells[index] > 0) // decelerate
    {
        unsigned int clearance = cuda_get_clearance(cells, index, road, road_index, road_length, road_lengths, road_links, road_link_count, max_speed);
        __syncthreads();

        if(clearance <= cells[index])
            cells[index] = (clearance - 1);
    }

    if((cells[index] > 0) && (cuda_get_random(random_seed * threadIdx.x) <= 0.2))    // random
        cells[index]--;

    if(cells[index] >= 0) // progress
    {
        unsigned int new_position = (index + cells[index]);
        unsigned int new_road_position = (road_index + cells[index]);
        temp_cells[index] = -1;

        if(new_road_position < road_length)
            temp_cells[new_position] = cells[index];
        else
        {
            signed int next_road = cuda_get_next_road(road, road_links, road_link_count);

            if(next_road >= 0)
            {
                new_position = (cuda_get_first_index_of_road(next_road, road_lengths)) + (cells[index] - (road_lengths[road] - road_index));
                temp_cells[new_position] = cells[index];
            }
        }
    }

    __syncthreads();

    if(temp_cells[index] >= 0) // density
    {
        atomicAdd(&vehicle_counts[road], 1);
    }
}

extern "C"
float cuda_process_model(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed, unsigned int road_link_count);

extern "C"
float cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed, unsigned int road_link_count)
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
    unsigned int random_seed;

    for(unsigned int i = 0; i < road_count; i++)
    {
        cell_count += road_lengths[i];
        vehicle_counts[i] = 0;
    }
    size = cell_count * sizeof(int);

    random_seed = rand() % UINT_MAX;

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

    cuda_apply_rules<<<(blocks_per_road * road_count), THREADS_PER_BLOCK>>>(cells_d, temp_cells_d, road_lengths_d, road_count, max_speed, vehicle_counts_d, random_seed, road_links_d, road_link_count);

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

    cuda_toggle_road_links<<<road_count,1>>>(road_links_d, road_link_count);

//    cout << cudaGetErrorString(cudaGetLastError()) << endl;
    return((float)vehicle_count / (float)cell_count);
}
