#include "model.h"

Model::Model()
{
    srand(time(NULL));
    this->init();
}

Model::~Model()
{
    cuda_deinit();
    delete[] this->cells;
    delete[] this->road_directions;
    delete[] this->road_lengths;
    delete[] this->road_links;
    delete[] this->input_roads;
}

signed int** Model::get_cells()
{
    return this->cells;
}

unsigned int Model::get_road_count()
{
    return this->road_count;
}

unsigned int* Model::get_road_lengths()
{
    return this->road_lengths;
}

void Model::init()
{
    this->generation = 0;
    this->road_count = 12;
    this->road_directions = new Direction[this->road_count];
    this->road_lengths = new unsigned int[this->road_count];
    this->vehicle_speed_limit = DEFAULT_VEHICLE_SPEED_LIMIT;
    this->desired_density = DEFAULT_DESIRED_DENSITY;
    this->init_roads();
    this->max_road_length = 0;
    this->realistic_traffic_synthesis = true;

    for(unsigned int i = 0; i < this->road_count; i++)
        if(this->road_lengths[i] > this->max_road_length)
            this->max_road_length = this->road_lengths[i];

    this->init_road_links();
    this->identify_input_roads();
    this->init_vehicles();
    cuda_init(this->cells, this->road_lengths, this->max_road_length, this->road_count, this->vehicle_speed_limit, this->road_links, this->road_link_count, this->input_roads, this->input_road_count);
}

void Model::init_roads()
{
    this->road_lengths[0] = 25;
    this->road_directions[0] = this->Right;
    this->road_lengths[1] = 25;
    this->road_directions[1] = this->Up;
    this->road_lengths[2] = 25;
    this->road_directions[2] = this->Right;
    this->road_lengths[3] = 25;
    this->road_directions[3] = this->Down;
    this->road_lengths[4] = 5;
    this->road_directions[4] = this->Up;
    this->road_lengths[5] = 25;
    this->road_directions[5] = this->Right;
    this->road_lengths[6] = 25;
    this->road_directions[6] = this->Right;
    this->road_lengths[7] = 5;
    this->road_directions[7] = this->Down;
    this->road_lengths[8] = 25;
    this->road_directions[8] = this->Right;
    this->road_lengths[9] = 25;
    this->road_directions[9] = this->Down;
    this->road_lengths[10] = 25;
    this->road_directions[10] = this->Left;
    this->road_lengths[11] = 10;
    this->road_directions[11] = this->Down;

    this->cells = this->init_empty_cells();
}

void Model::init_road_links()
{
    road_link r0;
    r0.origin_road_count = 2;
    r0.destination_road_count = 1;
    r0.origin_roads[0] = 0;
    r0.origin_roads[1] = 1;
    r0.destination_roads[0] = 2;

    road_link r1;
    r1.origin_road_count = 1;
    r1.destination_road_count = 3;
    r1.origin_roads[0] = 2;
    r1.destination_roads[0] = 3;
    r1.destination_roads[1] = 4;
    r1.destination_roads[2] = 5;


    road_link r2;
    r2.origin_road_count = 1;
    r2.destination_road_count = 1;
    r2.origin_roads[0] = 4;
    r2.destination_roads[0] = 6;

    road_link r3;
    r3.origin_road_count = 1;
    r3.destination_road_count = 1;
    r3.origin_roads[0] = 6;
    r3.destination_roads[0] = 7;

    road_link r4;
    r4.origin_road_count = 2;
    r4.destination_road_count = 2;
    r4.origin_roads[0] = 5;
    r4.origin_roads[1] = 7;
    r4.destination_roads[0] = 8;
    r4.destination_roads[1] = 9;

    road_link r5;
    r5.origin_road_count = 1;
    r5.destination_road_count = 1;
    r5.origin_roads[0] = 9;
    r5.destination_roads[0] = 10;

    road_link r6;
    r6.origin_road_count = 2;
    r6.destination_road_count = 1;
    r6.origin_roads[0] = 3;
    r6.origin_roads[1] = 10;
    r6.destination_roads[0] = 11;

    this->road_link_count = 7;
    this->road_links = new road_link[this->road_link_count];
    this->road_links[0] = r0;
    this->road_links[1] = r1;
    this->road_links[2] = r2;
    this->road_links[3] = r3;
    this->road_links[4] = r4;
    this->road_links[5] = r5;
    this->road_links[6] = r6;

    for(unsigned int x = 0; x < this->road_link_count; x++) // activate/deactivate road links
    {
        for(unsigned int y = 0; y < this->road_links[x].origin_road_count; y++) // origins
        {
            if(y == 0)
                this->road_links[x].origin_roads_active[y] = true;
            else
                this->road_links[x].origin_roads_active[y] = false;
        }

        for(unsigned int y = 0; y < this->road_links[x].destination_road_count; y++)    // destinations
        {
            if(y == 0)
                this->road_links[x].destination_roads_active[y] = true;
            else
                this->road_links[x].destination_roads_active[y] = false;
        }
    }
}

void Model::identify_input_roads()
{
    this->input_road_count = 0;

    for(unsigned int i = 0; i < this->road_count; i++)
    {
        bool found = false;

        for(unsigned int x = 0; x < this->road_link_count; x++)
        {
            for(unsigned int y = 0; y < this->road_links[x].destination_road_count; y++)
            {
                if(this->road_links[x].destination_roads[y] == i)
                {
                    found = true;
                    break;
                }
            }

            if(found)
               break;
        }

        if(!found)
            this->input_road_count++;
    }

    this->input_roads = new unsigned int[this->input_road_count];
    unsigned int found_count = 0;

    for(unsigned int i = 0; i < this->road_count; i++)
    {
        bool found = false;

        for(unsigned int x = 0; x < this->road_link_count; x++)
        {
            for(unsigned int y = 0; y < this->road_links[x].destination_road_count; y++)
            {
                if(this->road_links[x].destination_roads[y] == i)
                {
                    found = true;
                    break;
                }
            }

            if(found)
               break;
        }

        if(!found)
        {
            this->input_roads[found_count] = i;
            found_count++;
        }
    }
}

signed int** Model::init_empty_cells()
{
    signed int** cells = new signed int*[this->road_count];

    for(unsigned int x = 0; x < this->road_count; x++)
    {
        cells[x] = new signed int[this->road_lengths[x]];

        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
            cells[x][y] = -1;
    }

    return cells;
}

void Model::init_vehicles()
{  
    float required_density = DEFAULT_DESIRED_DENSITY;
    unsigned int total_cells = 0;
    for(unsigned int i = 0; i < this->road_count; i++)
        total_cells += this->road_lengths[i];
    unsigned int required_vehicles = (total_cells * required_density);
    unsigned int actual_vehicles = 0;

    while(actual_vehicles < required_vehicles)
    {
        unsigned int road = (rand() % this->road_count);
        unsigned int cell = (rand() % this->road_lengths[road]);

        if(this->cells[road][cell] < 0)
        {
            this->cells[road][cell] = this->vehicle_speed_limit;
            actual_vehicles++;
        }
    }
}

void Model::update()
{
    this->process();
    this->generation++;
}

void Model::display()
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(this->cells[x][y] == -1)
                cout << ".";
            else
                cout << this->cells[x][y];
        }

        cout << endl;
    }

    cout << endl;
}

void Model::process()
{
    this->model_density = cuda_process_model(this->cells, this->road_lengths, this->generation, this->desired_density, this->realistic_traffic_synthesis);
}

float Model::get_model_density()
{
    return this->model_density;
}

unsigned long Model::get_generation()
{
    return this->generation;
}

road_link *Model::get_road_links()
{
    return this->road_links;
}

unsigned int Model::get_road_link_count()
{
    return this->road_link_count;
}

Model::Direction *Model::get_road_directions()
{
    return this->road_directions;
}

void Model::set_desired_density(float desired_density)
{
    this->desired_density = desired_density;
}

void Model::set_realistic_traffic_synthesis(bool realistic_traffic_synthesis)
{
    this->realistic_traffic_synthesis = realistic_traffic_synthesis;
}
