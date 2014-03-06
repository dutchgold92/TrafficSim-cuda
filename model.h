#ifndef MODEL_H
#define MODEL_H
//#define DEFAULT_ROAD_COUNT 100
//#define DEFAULT_ROAD_LENGTH 50000
#define DEFAULT_ROAD_COUNT 3
#define DEFAULT_ROAD_LENGTH 60
#define DEFAULT_VEHICLE_SPEED_LIMIT 5

#include <iostream>
#include <limits>
#include <stdlib.h>
#include <time.h>

extern "C"
void cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed);

extern "C"
float cuda_get_model_density(signed int **cells, unsigned int *road_lengths, unsigned int max_road_length, unsigned int road_count);

using namespace std;

class Model
{
public:
    Model();
    signed int** get_cells();
    unsigned int get_road_count();
    unsigned int* get_road_lengths();
    float get_model_density();
    float get_road_density(unsigned int road_index);
    void update();
    void display();
private:
    signed int **cells;
    unsigned int road_count;
    unsigned int *road_lengths;
    unsigned int vehicle_speed_limit;
    unsigned int max_road_length;
    void init();
    signed int** init_empty_cells();
    void init_vehicles();
    void synthesize_traffic();
    void vehicle_rules();
    // legacy serial functions
    void accelerate_rule();
    void decelerate_rule();
    void random_rule();
    void progress_rule();
    unsigned int get_clearance_ahead(unsigned int road, unsigned int cell);
};

#endif // MODEL_H
