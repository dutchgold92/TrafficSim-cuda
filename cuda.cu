#include "roadlink.h"
#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>
#include <thrust/random.h>

#define THREADS_PER_BLOCK 32
#define TOGGLE_ROAD_LINKS_ON_GENERATION_MULTIPLE 4

using namespace std;

road_link *road_links_d;
signed int *cells_d;
signed int *temp_cells_d;
unsigned int *road_lengths_d;
unsigned int *input_roads_d;
unsigned int *input_road_device_indices_d;
unsigned int input_road_count;
unsigned int max_road_length;
unsigned int road_count;
unsigned int max_speed;
unsigned int road_link_count;
unsigned int blocks_per_road;
unsigned int cell_count;
unsigned int size;
signed int follow_vehicle_road;
signed int follow_vehicle_cell;

__device__ void cuda_find_input_road_devices_indices(signed int *cells, unsigned int road_count, unsigned int *road_lengths, unsigned int *input_roads, unsigned int input_road_count, unsigned int *input_road_device_indices)
{
    for(unsigned int i = 0, road_device_index = 0; i < input_road_count; i++)
    {
        for(unsigned int x = 0; x < road_count; x++)
        {
            if(input_roads[i] == x)
                input_road_device_indices[i] = road_device_index;
            else
                road_device_index += road_lengths[x];
        }

        road_device_index = 0;
    }
}

__global__ void cuda_device_init(signed int *cells, unsigned int road_count, unsigned int *road_lengths, unsigned int *input_roads, unsigned int input_road_count, unsigned int *input_road_device_indices)
{
    cuda_find_input_road_devices_indices(cells, road_count, road_lengths, input_roads, input_road_count, input_road_device_indices);
}

extern "C"
void cuda_init(signed int **_cells, unsigned int *road_lengths, unsigned int _max_road_length, unsigned int _road_count, unsigned int _max_speed, road_link *road_links, unsigned int _road_link_count, unsigned int *_input_roads, unsigned int _input_road_count);

void cuda_init(signed int **cells, unsigned int *road_lengths, unsigned int _max_road_length, unsigned int _road_count, unsigned int _max_speed, road_link *road_links, unsigned int _road_link_count, unsigned int *_input_roads, unsigned int _input_road_count)
{
    max_road_length = _max_road_length;
    road_count = _road_count;
    max_speed = _max_speed;
    road_link_count = _road_link_count;
    input_road_count = _input_road_count;
    blocks_per_road = ceil((float)max_road_length / (float)THREADS_PER_BLOCK);
    follow_vehicle_cell = -1;
    follow_vehicle_road = -1;

    cell_count = 0;
    size = 0;

    for(unsigned int i = 0; i < road_count; i++)
        cell_count += road_lengths[i];
    size = cell_count * sizeof(int);

    cudaMalloc((void**)&cells_d, size);
    cudaMalloc((void**)&temp_cells_d, size);
    cudaMalloc((void**)&road_lengths_d, (sizeof(int) * road_count));
    cudaMalloc((void**)&road_links_d, (sizeof(road_link) * road_link_count));
    cudaMalloc((void**)&input_roads_d, (sizeof(int) * input_road_count));
    cudaMalloc((void**)&input_road_device_indices_d, (sizeof(int) * input_road_count));

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
    cudaMemcpy(road_links_d, road_links, (sizeof(road_link) * road_link_count), cudaMemcpyHostToDevice);
    cudaMemcpy(input_roads_d, _input_roads, (sizeof(int) * input_road_count), cudaMemcpyHostToDevice);

    cuda_device_init<<<1,1>>>(cells_d, road_count, road_lengths_d, input_roads_d, input_road_count, input_road_device_indices_d);
}

extern "C"
void cuda_deinit();

void cuda_deinit()
{
    cudaFree(cells_d);
    cudaFree(temp_cells_d);
    cudaFree(road_lengths_d);
    cudaFree(road_links_d);
    cudaFree(input_roads_d);
    cudaFree(input_road_device_indices_d);
}

extern "C"
void cuda_set_follow_vehicle(unsigned int road, unsigned int cell);

extern "C"
void cuda_set_follow_vehicle(unsigned int road, unsigned int cell)
{
    follow_vehicle_road = road;
    follow_vehicle_cell = cell;
}

extern "C"
unsigned int cuda_get_follow_vehicle_road();

extern "C"
unsigned int cuda_get_follow_vehicle_road()
{
    return follow_vehicle_road;
}

extern "C"
unsigned int cuda_get_follow_vehicle_cell();

extern "C"
unsigned int cuda_get_follow_vehicle_cell()
{
    return follow_vehicle_cell;
}

__device__ unsigned int cuda_get_first_index_of_road(unsigned int road, unsigned int *road_lengths)
{
    unsigned int index = 0;

    for(unsigned int i = 0; i < road; i++)
        index += road_lengths[i];

    return index;
}

__global__ void cuda_synthesize_traffic(signed int *cells, unsigned int *input_roads, unsigned int *input_road_device_indices, unsigned int *road_lengths, unsigned int input_road_count, unsigned int vehicles_needed, unsigned int max_speed, unsigned int road_count, bool realistic_traffic_synthesis)
{
    unsigned int vehicles;
    unsigned int cell_index;

    if(realistic_traffic_synthesis)
    {
        vehicles = ceil((float)vehicles_needed / (float)input_road_count);
        cell_index = input_road_device_indices[blockIdx.x];

        for(unsigned int i = 0; i < vehicles && i < max_speed && i < road_lengths[input_roads[blockIdx.x]]; i++ && cell_index++)
        {
            if(cells[cell_index] < 0)
                cells[cell_index] = max_speed;
            else
                return;
        }
    }
    else
    {
        vehicles = ceil((float)vehicles_needed / (float)road_count);
        cell_index = cuda_get_first_index_of_road(blockIdx.x, road_lengths);

        for(unsigned int i = 0; i < vehicles && i < road_lengths[blockIdx.x]; i++ && cell_index++)
        {
            if(cells[cell_index] < 0)
                cells[cell_index] = max_speed;
        }
    }
}

__device__ void cuda_toggle_road_link_origins(road_link *road_links, unsigned int road_link_index)
{
    bool toggled = false;

    if(road_links[road_link_index].origin_road_count > 1)
    {
        for(unsigned int loop = 0; loop < 2; loop++)
        {
            for(unsigned int i = 0; i < road_links[road_link_index].origin_road_count; i++)
            {
                if(!toggled && road_links[road_link_index].origin_roads_active[i])
                {
                    road_links[road_link_index].origin_roads_active[i] = false;
                    toggled = true;
                }
                else if(toggled && !road_links[road_link_index].origin_roads_active[i])
                {
                    road_links[road_link_index].origin_roads_active[i] = true;
                    return;
                }
            }
        }
    }
}

__device__ void cuda_toggle_road_link_destinations(road_link *road_links, unsigned int road_link_index)
{
    bool toggled = false;

    if(road_links[blockIdx.x].destination_road_count > 1)
    {
        for(unsigned int loop = 0; loop < 2; loop++)
        {
            for(unsigned int i = 0; i < road_links[blockIdx.x].destination_road_count; i++)
            {
                if(!toggled && road_links[blockIdx.x].destination_roads_active[i])
                {
                    road_links[blockIdx.x].destination_roads_active[i] = false;
                    toggled = true;
                }
                else if(toggled && !road_links[blockIdx.x].destination_roads_active[i])
                {
                    road_links[blockIdx.x].destination_roads_active[i] = true;
                    return;
                }
            }
        }
    }
}

__global__ void cuda_toggle_road_links(road_link *road_links)
{
    cuda_toggle_road_link_origins(road_links, blockIdx.x);
    cuda_toggle_road_link_destinations(road_links, blockIdx.x);
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
__device__ signed int cuda_get_next_road(unsigned int origin_road, road_link *road_links, unsigned int road_link_count)
{
    for(unsigned int i = 0; i < road_link_count; i++)
    {
        for(unsigned int x = 0; x < road_links[i].origin_road_count; x++)
        {
            if(road_links[i].origin_roads[x] == origin_road)
            {
                if(!road_links[i].origin_roads_active[x])
                    return -1;

                for(unsigned int y = 0; y < road_links[i].destination_road_count; y++)
                {
                    if(road_links[i].destination_roads_active[y])
                        return road_links[i].destination_roads[y];
                }

                return -1;
            }
        }
    }

    return -2;
}

__device__ signed int cuda_get_road_link(unsigned int origin_road, unsigned int destination_road, road_link *road_links, unsigned int road_link_count)
{
    for(unsigned int i = 0; i < road_link_count; i++)
    {
        bool origin_found = false;
        bool destination_found = false;

        for(unsigned int x = 0; x < road_links[i].origin_road_count; x++)
        {
            if(road_links[i].origin_roads[x] == origin_road)
            {
                origin_found = true;
                break;
            }
        }

        for(unsigned int x = 0; x < road_links[i].destination_road_count; x++)
        {
            if(road_links[i].destination_roads[x] == destination_road)
            {
                destination_found = true;
                break;
            }
        }

        if(origin_found && destination_found)
            return i;
        else
        {
            origin_found = false;
            destination_found = false;
        }
    }

    return -1;
}

__device__ unsigned int cuda_get_clearance(signed int *cells, unsigned int index, unsigned int road, unsigned int road_index, unsigned int road_length, unsigned int *road_lengths, road_link *road_links, unsigned int road_link_count, unsigned int max_speed)
{
    unsigned int clearance = 0;
    signed int next_road;

    for(unsigned int i = (index + 1), r_i = (road_index + 1); clearance <= max_speed; i++ && r_i++)
    {
        clearance++;

        if(r_i == road_length)
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
        else if(cells[i] >= 0)
            return(clearance);
    }

    return UINT_MAX;
}

__global__ void cuda_apply_rules(signed int *cells, signed int *temp_cells, unsigned int *road_lengths, unsigned int road_count, unsigned int max_speed, unsigned int *vehicle_counts, unsigned int random_seed, road_link *road_links, unsigned int road_link_count, signed int *follow_vehicle_road, signed int *follow_vehicle_cell)
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
        {
            temp_cells[new_position] = cells[index];

            if(*follow_vehicle_road == road && *follow_vehicle_cell == road_index)
                *follow_vehicle_cell = new_road_position;
        }
        else
        {
            signed int next_road = cuda_get_next_road(road, road_links, road_link_count);

            if(next_road >= 0)
            {
                new_position = (cuda_get_first_index_of_road(next_road, road_lengths)) + (cells[index] - (road_lengths[road] - road_index));
                unsigned int new_road_position = (0 + (cells[index] - (road_lengths[road] - road_index)));
                temp_cells[new_position] = cells[index];

                if(*follow_vehicle_road == road && *follow_vehicle_cell == road_index)
                {
                    *follow_vehicle_road = next_road;
                    *follow_vehicle_cell = new_road_position;
                }
            }
            else
            {
                if(*follow_vehicle_road == road && *follow_vehicle_cell == road_index)
                {
                    *follow_vehicle_road = -1;
                    *follow_vehicle_cell = -1;
                }
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
float cuda_process_model(signed int **cells, unsigned int *road_lengths, unsigned int generation, float desired_density, bool realistic_traffic_synthesis);

extern "C"
float cuda_process_model(signed int **cells, unsigned int *road_lengths, unsigned int generation, float desired_density, bool realistic_traffic_synthesis)
{
    unsigned int vehicle_count = 0;
    unsigned int *vehicle_counts = new unsigned int[road_count];
    unsigned int *vehicle_counts_d;
    unsigned int random_seed;
    signed int *follow_vehicle_road_d;
    signed int *follow_vehicle_cell_d;

    for(unsigned int i = 0; i < road_count; i++)
        vehicle_counts[i] = 0;

    random_seed = rand() % UINT_MAX;
    cudaMalloc((void**)&vehicle_counts_d, (sizeof(int) * road_count));
    cudaMalloc((void**)&follow_vehicle_road_d, sizeof(int));
    cudaMalloc((void**)&follow_vehicle_cell_d, sizeof(int));
    cudaMemcpy(vehicle_counts_d, vehicle_counts, (sizeof(int) * road_count), cudaMemcpyHostToDevice);
    cudaMemcpy(follow_vehicle_road_d, &follow_vehicle_road, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(follow_vehicle_cell_d, &follow_vehicle_cell, sizeof(int), cudaMemcpyHostToDevice);

    cuda_apply_rules<<<(blocks_per_road * road_count), THREADS_PER_BLOCK>>>(cells_d, temp_cells_d, road_lengths_d, road_count, max_speed, vehicle_counts_d, random_seed, road_links_d, road_link_count, follow_vehicle_road_d, follow_vehicle_cell_d);

    int *temp_cells_d_ptr = &temp_cells_d[0];

    for(int i = 0; i < road_count; i++)
    {
        cudaMemcpy(cells[i], temp_cells_d_ptr, (sizeof(int) * road_lengths[i]), cudaMemcpyDeviceToHost);
        temp_cells_d_ptr += road_lengths[i];
    }

    cudaMemcpy(cells_d, temp_cells_d, size, cudaMemcpyDeviceToDevice);
    cudaMemcpy(&follow_vehicle_road, follow_vehicle_road_d, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&follow_vehicle_cell, follow_vehicle_cell_d, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(vehicle_counts, vehicle_counts_d, (sizeof(int) * road_count), cudaMemcpyDeviceToHost);
    for(unsigned int i = 0; i < road_count; i++)
        vehicle_count += vehicle_counts[i];

    cudaFree(vehicle_counts_d);
    cudaFree(follow_vehicle_road_d);
    cudaFree(follow_vehicle_cell_d);
    delete[] vehicle_counts;

    if(generation % TOGGLE_ROAD_LINKS_ON_GENERATION_MULTIPLE == 0)
        cuda_toggle_road_links<<<road_link_count,1>>>(road_links_d);

    float density = ((float)vehicle_count / (float)cell_count);

    if(density < desired_density)
    {
        unsigned int vehicles_needed = 0;

        for(unsigned int i = 0; i < cell_count; i++)
        {
            if(((float)vehicles_needed / (float)cell_count) > (desired_density - density))
            {
                if(realistic_traffic_synthesis)
                    cuda_synthesize_traffic<<<input_road_count,1>>>(cells_d, input_roads_d, input_road_device_indices_d, road_lengths_d, input_road_count, vehicles_needed, max_speed, road_count, realistic_traffic_synthesis);
                else
                    cuda_synthesize_traffic<<<road_count,1>>>(cells_d, input_roads_d, input_road_device_indices_d, road_lengths_d, input_road_count, vehicles_needed, max_speed, road_count, realistic_traffic_synthesis);

                break;
            }
            else
                vehicles_needed++;
        }
    }

    return(density);
}
