#include "model.h"

Model::Model()
{
    srand(time(NULL));
    this->init();
    this->init_vehicles();
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

    for(unsigned int i = 0; i < this->road_count; i++)
        this->road_lengths[i] = DEFAULT_ROAD_LENGTH;

    this->cells = this->init_empty_cells();
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
            if(y < 5)    // FIXME: shoddy workmanship! COWBOYS!
                this->cells[x][y] = 0;
        }
    }
}

void Model::deallocate_cells(signed int **cells)
{
    for(unsigned int i = 0; i < this->road_count; i++)
        delete[] cells[i];

    delete[] cells;
}

void Model::update()
{
    signed int** temp_cells = this->init_empty_cells();

    for(unsigned int x = 0; x < this->road_count; x++)
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
            temp_cells[x][y] = this->cells[x][y];

    this->accelerate_rule(temp_cells);
    this->decelerate_rule(temp_cells);
    this->random_rule(temp_cells);

    this->deallocate_cells(this->cells);
    this->cells = temp_cells;
    temp_cells = this->init_empty_cells();

    this->progress_rule(temp_cells);
    this->deallocate_cells(this->cells);
    this->cells = temp_cells;
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

void Model::accelerate_rule(signed int **cells)
{
//    for(unsigned int x = 0; x < this->road_count; x++)
//    {
//        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
//        {
//            if(this->cells[x][y] < 0)
//                continue;  // skip empty
//            else if(this->cells[x][y] < this->vehicle_speed_limit)
//            {
//                if(this->get_clearance_ahead(x, y) > (this->cells[x][y] + 1))
//                    cells[x][y] = (this->cells[x][y] + 1);
//            }
//        }
//    }

    for(unsigned int i = 0; i < this->road_count; i++)
        cuda_accelerate_rule(this->cells[i], cells[i], this->road_lengths[i], this->vehicle_speed_limit);
}

void Model::decelerate_rule(signed int **cells)
{
//    for(unsigned int x = 0; x < this->road_count; x++)
//    {
//        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
//        {
//            if(this->cells[x][y] < 0)
//                continue;   // skip empty
//            else if(this->cells[x][y] > 0)
//            {
//                unsigned int clearance = this->get_clearance_ahead(x, y);

//                if(clearance <= this->cells[x][y])
//                    cells[x][y] = (clearance - 1);
//            }
//        }
//    }

    for(unsigned int i = 0; i < this->road_count; i++)
        cuda_decelerate_rule(this->cells[i], cells[i], this->road_lengths[i]);
}

void Model::random_rule(signed int **cells)
{
    for(unsigned int x = 0; x < this->road_count; x++)
    {
        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
        {
            if(this->cells[x][y] > 0)
            {
                if((rand() % 10) == 0)  // FIXME: shoddy workmanship!
                    cells[x][y]--;
            }
        }
    }

//    for(unsigned int i = 0; i < this->road_count; i++)
//        cuda_random_rule(this->cells[i], cells[i], this->road_lengths[i]);
}

void Model::progress_rule(signed int **cells)
{
//    for(unsigned int x = 0; x < this->road_count; x++)
//    {
//        for(unsigned int y = 0; y < this->road_lengths[x]; y++)
//        {
//            if(this->cells[x][y] < 0)
//                continue;   // skip empty
//            else if(this->cells[x][y] >= 0)
//            {
//                unsigned int new_position = (y + this->cells[x][y]);

//                if(new_position < this->road_lengths[x])
//                    cells[x][new_position] = this->cells[x][y];
//            }
//        }
//    }

    for(unsigned int i = 0; i < this->road_count; i++)
        cuda_progress_rule(this->cells[i], cells[i], this->road_lengths[i]);
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
