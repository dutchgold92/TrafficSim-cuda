#ifndef MODEL_H
#define MODEL_H
#define DEFAULT_ROAD_COUNT 100;
#define DEFAULT_ROAD_LENGTH 200
#define DEFAULT_VEHICLE_SPEED_LIMIT 5

#include <iostream>
#include <limits>
#include <stdlib.h>
#include <time.h>

extern "C"
void cuda_accelerate_rule(signed int *cells, signed int *temp_cells, unsigned int road_length, unsigned int max_speed);

extern "C"
void cuda_decelerate_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

extern "C"
void cuda_random_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

extern "C"
void cuda_progress_rule(signed int* cells, signed int* temp_cells, unsigned int road_length);

using namespace std;

class Model
{
public:
    Model();
    signed int** get_cells();
    unsigned int get_road_count();
    unsigned int* get_road_lengths();
    void update();
    void display();
private:
    signed int **cells;
    unsigned int road_count;
    unsigned int *road_lengths;
    unsigned int vehicle_speed_limit;
    void init();
    signed int** init_empty_cells();
    void init_vehicles();
    void deallocate_cells(signed int** cells);
    void accelerate_rule(signed int** cells);
    void decelerate_rule(signed int** cells);
    void random_rule(signed int** cells);
    void progress_rule(signed int** cells);
    unsigned int get_clearance_ahead(unsigned int road, unsigned int cell);
};

#endif // MODEL_H
