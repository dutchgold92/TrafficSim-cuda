#ifndef ROADLINK_H
#define ROADLINK_H

struct road_link
{
    unsigned int origin_road_count;
    unsigned int destination_road_count;
    unsigned int origin_roads[3];
    bool origin_roads_active[3];
    unsigned int destination_roads[3];
    bool destination_roads_active[3];
};

#endif // ROADLINK_H
