#ifndef MODEL_H
#define MODEL_H
#define DEFAULT_VEHICLE_SPEED_LIMIT 5
#define DEFAULT_DESIRED_DENSITY 0.2

#include <roadlink.h>
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <time.h>
#include <vector>

extern "C"
void cuda_init(signed int **cells, unsigned int *road_lengths, unsigned int _max_road_length, unsigned int _road_count, unsigned int _max_speed, road_link *road_links, unsigned int _road_link_count, unsigned int *_input_roads, unsigned int _input_road_count);

extern "C"
void cuda_deinit();

extern "C"
float cuda_process_model(signed int** cells, unsigned int* road_lengths, unsigned int generation, float desired_density, bool realistic_traffic_synthesis);

using namespace std;

class Model
{
public:
    Model();
    ~Model();
    enum Direction {Up, Down, Left, Right};
    signed int** get_cells();
    road_link *get_road_links();
    unsigned int get_road_link_count();
    Direction *get_road_directions();
    unsigned int get_road_count();
    unsigned int* get_road_lengths();
    float get_model_density();
    float get_road_density(unsigned int road_index);
    void update();
    void display();
    unsigned long get_generation();
    void set_desired_density(float desired_density);
    void set_realistic_traffic_synthesis(bool realistic_traffic_synthesis);
private:
    signed int **cells;
    road_link *road_links;
    unsigned int *input_roads;
    unsigned int input_road_count;
    Direction *road_directions;
    unsigned int road_count;
    unsigned int road_link_count;
    unsigned int *road_lengths;
    unsigned int vehicle_speed_limit;
    unsigned int max_road_length;
    float desired_density;
    float model_density;
    unsigned long generation;
    bool realistic_traffic_synthesis;
    void init();
    void init_roads();
    void init_road_links();
    void identify_input_roads();
    signed int** init_empty_cells();
    void init_vehicles();
};

#endif // MODEL_H
