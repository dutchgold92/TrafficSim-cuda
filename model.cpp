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
//    this->road_count = DEFAULT_ROAD_COUNT;
    this->road_directions = new Direction[this->road_count];
    this->road_lengths = new unsigned int[this->road_count];
    this->vehicle_speed_limit = DEFAULT_VEHICLE_SPEED_LIMIT;
    this->desired_density = DEFAULT_DESIRED_DENSITY;
    this->init_roads();
    this->max_road_length = 0;

    for(unsigned int i = 0; i < this->road_count; i++)
        if(this->road_lengths[i] > this->max_road_length)
            this->max_road_length = this->road_lengths[i];

    this->init_road_links();
    this->identify_input_roads();
    this->init_vehicles();
    cuda_init(this->cells, this->road_lengths, this->max_road_length, this->road_count, this->vehicle_speed_limit, this->road_links, this->road_link_count);
}

void Model::init_roads()
{
//    for(unsigned int i = 0; i < this->road_count; i++)
//        this->road_lengths[i] = DEFAULT_ROAD_LENGTH;

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
    for(unsigned int x = 0; x < this->input_road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[this->input_roads[x]]; y++)
        {
            if(y < 5)
                this->cells[x][y] = 0;  // FIXME: shoddy workmanship! COWBOYS!
        }
    }
}

void Model::update()
{
    this->vehicle_rules();
    this->generation++;
//    this->accelerate_rule();
//    this->decelerate_rule();
//    this->random_rule();
//    this->progress_rule();

//    this->synthesize_traffic();
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

//    cout << "Model density: " << this->model_density << endl;

    cout << endl;
}

void Model::vehicle_rules()
{
    this->model_density = cuda_process_model(this->cells, this->road_lengths, this->generation);
}

void Model::synthesize_traffic()
{
    unsigned int iterations = 0;

    while(this->get_model_density() < this->desired_density && iterations < (this->road_count * this->road_count))
    {
        unsigned int road = (rand() % this->road_count);

        for(unsigned int i = this->vehicle_speed_limit; i < this->road_lengths[road] && i-->0;)
        {
            if(this->cells[road][i] >= 0)
                continue;
            else
                this->cells[road][i] = 1;
        }

        iterations++;
    }
}

float Model::get_model_density()
{
    return this->model_density;

//    float vehicles = 0;
//    float cells = 0;

//    for(unsigned int x = 0; x < this->road_count; x++)
//    {
//        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
//        {
//            if(this->cells[x][y] >= 0)
//                vehicles++;
//        }

//        cells += this->road_lengths[x];
//    }

//    return(vehicles / cells);
}

float Model::get_road_density(unsigned int road_index)
{
    if(road_index >= this->road_count)
        return 0;

    float vehicles = 0;

    for(unsigned int i = 0; i < this->road_lengths[road_index]; i++)
        if(this->cells[road_index][i] >= 0)
            vehicles++;

    return(vehicles / (float)this->road_lengths[road_index]);
}

void Model::accelerate_rule()
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(this->cells[x][y] < 0)
                continue;  // skip empty
            else if(this->cells[x][y] < this->vehicle_speed_limit)
            {
                if(this->get_clearance_ahead(x, y) > (this->cells[x][y] + 1))
                    this->cells[x][y]++;
            }
        }
    }
}

void Model::decelerate_rule()
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(this->cells[x][y] < 0)
                continue;   // skip empty
            else if(this->cells[x][y] > 0)
            {
                unsigned int clearance = this->get_clearance_ahead(x, y);

                if(clearance <= this->cells[x][y])
                    this->cells[x][y] = (clearance - 1);
            }
        }
    }
}

void Model::random_rule()
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(this->cells[x][y] > 0)
            {
                if((rand() % 10) == 0)  // FIXME: shoddy workmanship!
                    this->cells[x][y]--;
            }
        }
    }
}

void Model::progress_rule()
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = this->road_lengths[x]; y--> 0;)
        {
            if(this->cells[x][y] <= 0)
                continue;   // skip empty, skip stopped
            else if(this->cells[x][y] > 0)
            {
                unsigned int new_position = (y + this->cells[x][y]);

                if(new_position < this->road_lengths[x])
                {
                    this->cells[x][new_position] = this->cells[x][y];
                    this->cells[x][y] = -1;
                }
            }
        }
    }
}

unsigned int Model::get_clearance_ahead(unsigned int road, unsigned int cell)
{
    for(unsigned int i = (cell + 1); i < this->road_lengths[road]; i++)
    {
        if(this->cells[road][i] >= 0)
            return(i - cell);
    }

    return numeric_limits<unsigned int>::max();
}

void Model::toggle_road_links()
{
//    vector<road_link*> links;

//    for(unsigned int x = 0; x < this->road_count; x++)
//    {
//        for(unsigned int y = 0; y < this->road_link_count; y++)
//        {
//            if(this->road_links[y].origin_road == x)
//                links.push_back(&this->road_links[y]);
//        }

//        if(!links.empty())
//        {
//            unsigned int new_active_index = (rand() % links.size());
//            links.at(new_active_index)->active = true;

//            for(unsigned int i = 0; i < links.size(); i++)
//            {
//                if(i == new_active_index)
//                    continue;
//                else
//                    links.at(i)->active = false;
//            }

//            links.clear();
//        }
//    }
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
