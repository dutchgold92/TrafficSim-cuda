#ifndef MODEL_H
#define MODEL_H
//#define DEFAULT_ROAD_COUNT 100
//#define DEFAULT_ROAD_LENGTH 50000
#define DEFAULT_ROAD_COUNT 3
#define DEFAULT_ROAD_LENGTH 30
#define DEFAULT_VEHICLE_SPEED_LIMIT 5
#define DEFAULT_DESIRED_DENSITY 0.2

#include <roadlink.h>
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <time.h>
#include <vector>

extern "C"
void cuda_init(road_link *road_links, unsigned int road_link_count);

extern "C"
float cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int max_road_length, unsigned int road_count, unsigned int max_speed, unsigned int road_link_count);

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
    road_link *road_links;
    unsigned int road_count;
    unsigned int road_link_count;
    unsigned int *road_lengths;
    unsigned int vehicle_speed_limit;
    unsigned int max_road_length;
    float desired_density;
    float model_density;
    void init();
    void init_road_links();
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
    void toggle_road_links();
};

#endif // MODEL_H
