#include <iostream>
#include <cuda.h>
#include <stdio.h>
#include <limits.h>

using namespace std;

__device__ unsigned int cuda_get_clearance(signed int *cells, unsigned int index, unsigned int road_index, unsigned int road_length)
{
    for(unsigned int i = (index + 1), r_i = (road_index + 1); r_i < road_length; i++ && r_i++)
    {
        if(cells[i] >= 0)
            return(i - road_index);
    }

    return UINT_MAX;
}

__global__ void cuda_apply_rules(unsigned int block_count, signed int *cells, signed int *temp_cells, unsigned int *road_lengths, unsigned int road_count, unsigned int max_speed)
{
    unsigned int index = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int road = (blockIdx.x / block_count);
    unsigned int road_index = index;
    unsigned int road_length = road_lengths[road];

    for(unsigned int i = 0; i < road; i++)
        road_index -= road_lengths[i];

    if(road_index >= road_length)
        return;

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
}

extern "C"
void cuda_process_model(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed);

extern "C"
void cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed)
{
    signed int *cells_d;
    signed int *temp_cells_d;
    unsigned int *road_lengths_d;
    unsigned int blocks;
    unsigned int size = 0;

    for(unsigned int i = 0; i < road_count; i++)
        size += road_lengths[i];

    size = size * sizeof(int);
    blocks = ceil((float)max_road_length / 32.0);

    cudaMalloc((void**)&cells_d, size);
    cudaMalloc((void**)&temp_cells_d, size);
    cudaMalloc((void**)&road_lengths_d, (sizeof(int) * road_count));

    for(int i = 0; i < road_count; i++)
    {
        int add = 0;
        cudaMemcpy((cells_d + add), cells[i], (sizeof(int) * road_lengths[i]), cudaMemcpyHostToDevice);
        cudaMemcpy((temp_cells_d + add), cells[i], (sizeof(int) * road_lengths[i]), cudaMemcpyHostToDevice);
        add += road_lengths[i];
    }

    cudaMemcpy(road_lengths_d, road_lengths, (sizeof(int) * road_count), cudaMemcpyHostToDevice);

    cuda_apply_rules<<<blocks, 32>>>(blocks, cells_d, temp_cells_d, road_lengths_d, road_count, max_speed);

    for(int i = 0; i < road_count; i++)
    {
        int add = 0;
        cudaMemcpy(cells[i], (temp_cells_d + add), (sizeof(int) * road_lengths[i]), cudaMemcpyDeviceToHost);
        add += road_lengths[i];
    }

    cudaFree(cells_d);
    cudaFree(temp_cells_d);
    cudaFree(road_lengths_d);

//    cout << cudaGetErrorString(cudaGetLastError()) << endl;
}

__global__ void cuda_calculate_model_density(unsigned int block_count, signed int *cells, unsigned int *road_lengths, unsigned int road_count, unsigned int *vehicle_count_per_block)
{
    unsigned int index = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int road = (blockIdx.x / block_count);
    unsigned int road_index = index;
    unsigned int road_length = road_lengths[road];

    for(unsigned int i = 0; i < road; i++)
        road_index -= road_lengths[i];

    if(road_index >= road_length)
        return;

    if(cells[road_index] >= 0)
        atomicAdd(&vehicle_count_per_block[blockIdx.x], 1);
}

extern "C"
float cuda_get_model_density(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count);

extern "C"
float cuda_get_model_density(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count)
{
    unsigned int cell_count = 0;
    unsigned int vehicle_count = 0;
    unsigned int *vehicle_count_per_block;
    unsigned int *vehicle_count_per_block_d;
    signed int *cells_d;
    unsigned int *road_lengths_d;
    unsigned int blocks;
    unsigned int size = 0;

    for(unsigned int i = 0; i < road_count; i++)
        cell_count += road_lengths[i];

    size = cell_count * sizeof(int);
    blocks = ceil((float)max_road_length / 32.0);

    vehicle_count_per_block = new unsigned int[blocks];

    cudaMalloc((void**)&vehicle_count_per_block_d, (sizeof(int) * blocks));
    cudaMalloc((void**)&cells_d, size);
    cudaMalloc((void**)&road_lengths_d, (sizeof(int) * road_count));

    for(int i = 0; i < road_count; i++)
    {
        int add = 0;
        cudaMemcpy((cells_d + add), cells[i], (sizeof(int) * road_lengths[i]), cudaMemcpyHostToDevice);
        add += road_lengths[i];
    }

    cudaMemcpy(road_lengths_d, road_lengths, (sizeof(int) * road_count), cudaMemcpyHostToDevice);

    cuda_calculate_model_density<<<blocks, 32>>>(blocks, cells_d, road_lengths_d, road_count, vehicle_count_per_block_d);

    cudaMemcpy(&vehicle_count_per_block, vehicle_count_per_block_d, (sizeof(int) * blocks), cudaMemcpyDeviceToHost);

    cudaFree(road_lengths_d);
    cudaFree(cells_d);
    cudaFree(vehicle_count_per_block_d);
    delete[] vehicle_count_per_block;

    for(unsigned int i = 0; i < blocks; i++)
        vehicle_count += vehicle_count_per_block[i];

    cout << cell_count << " / " << vehicle_count << endl;
    cout << cudaGetErrorString(cudaGetLastError()) << endl;
    return(vehicle_count / cell_count);
}
