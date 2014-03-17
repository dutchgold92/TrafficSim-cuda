#include "model.h"

Model::Model()
{
    srand(time(NULL));
    this->init();
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
    this->road_count = DEFAULT_ROAD_COUNT;
    this->road_lengths = new unsigned int[this->road_count];
    this->vehicle_speed_limit = DEFAULT_VEHICLE_SPEED_LIMIT;
    this->desired_density = DEFAULT_DESIRED_DENSITY;

    for(unsigned int i = 0; i < this->road_count; i++)
        this->road_lengths[i] = DEFAULT_ROAD_LENGTH;

    this->cells = this->init_empty_cells();
    this->init_vehicles();

    this->max_road_length = 0;

    for(unsigned int i = 0; i < this->road_count; i++)
        if(this->road_lengths[i] > this->max_road_length)
            this->max_road_length = this->road_lengths[i];

    this->init_road_links();
    cuda_init(this->road_links, this->road_link_count);
}

void Model::init_road_links()
{
    road_link r1;
    r1.origin_road = 0;
    r1.destination_road = 1;
    r1.active = false;
    road_link r2;
    r2.origin_road = 0;
    r2.destination_road = 2;
    r2.active = true;

    this->road_link_count = 2;
    this->road_links = new road_link[this->road_link_count];
    this->road_links[0] = r1;
    this->road_links[1] = r2;
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
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(x == 0 && y < 5)    // FIXME: shoddy workmanship! COWBOYS!
                this->cells[x][y] = 0;
        }
    }
}

void Model::update()
{
    this->vehicle_rules();
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
    this->model_density = cuda_process_model(this->cells, this->road_lengths, this->max_road_length, this->road_count, this->vehicle_speed_limit, this->road_link_count);
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
    vector<road_link*> links;

    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_link_count; y++)
        {
            if(this->road_links[y].origin_road == x)
                links.push_back(&this->road_links[y]);
        }

        if(!links.empty())
        {
            unsigned int new_active_index = (rand() % links.size());
            links.at(new_active_index)->active = true;

            for(unsigned int i = 0; i < links.size(); i++)
            {
                if(i == new_active_index)
                    continue;
                else
                    links.at(i)->active = false;
            }

            links.clear();
        }
    }
}
