#ifndef PATH_PLANNING_CONSTANTS
#define PATH_PLANNING_CONSTANTS

/**
 * @file constants.h
 * @brief all common constants/defines/helper-functions go in here
 * 
 * @author Georgian Besleaga, Tyler Irving Buchman, Petre Munteanu, Jacobs University Bremen, g (dot) besleaga (at) jacobs-university (dot) de, t (dot) buchman (at) jacobs-university (dot) de, p (dot) munteanu (at) jacobs-university (dot) de
 */
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/thread/mutex.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#define DEFAULT_MAX_SPEED 1.69
#define DEFAULT_MAX_ROTATIONAL_SPEED 1.69
#define DEFAULT_LOOP_RATE 10

const double MAX_ROTATIONAL_SPEED = DEFAULT_MAX_ROTATIONAL_SPEED;
const double MAX_SPEED = DEFAULT_MAX_SPEED;
const int LOOP_RATE = DEFAULT_LOOP_RATE;

typedef std::vector<std::vector <signed char> > OccupancyMap;

void updateMap(const std::vector<signed char> &data, unsigned int width, unsigned int height, OccupancyMap &occMap, boost::mutex &mutex)
{
    boost::mutex::scoped_lock scoped_lock(mutex);
    occMap.clear();
    int index = 0;
    std::vector<signed char> row;
    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            row.push_back(data[index++]);
        }
        occMap.push_back(row);
        row.clear();
    }
}

/// the maths sgn/signum function
template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void checkMaxSpeed (double &v)
{
    double sign = sgn(v);
    if(abs(v) > MAX_SPEED)
    {
        v = MAX_SPEED * sign;
    }
}

void checkMaxRotationalSpeed (double &v)
{
    double sign = sgn(v);
    if(abs(v) > MAX_ROTATIONAL_SPEED)
    {
        v = MAX_ROTATIONAL_SPEED * sign;
    }
}

/// generally useful code from Ravi
void mapCellToWorld(const nav_msgs::OccupancyGridConstPtr &map, const int &x_map, const int &y_map, double &x_world, double &y_world)
{
                x_world  = (x_map + 0.5) * map->info.resolution + map->info.origin.position.x;
                y_world  = (y_map + 0.5) * map->info.resolution + map->info.origin.position.y;
}

void worldToMapCell(const nav_msgs::OccupancyGridConstPtr &map, const double &x_world, const double &y_world, int &x_map, int &y_map)
{
                x_map = static_cast<int>( (x_world - map->info.origin.position.x) / map->info.resolution );
                y_map = static_cast<int>( (y_world - map->info.origin.position.y) / map->info.resolution );
}

size_t mapCellToIndex(const nav_msgs::OccupancyGridConstPtr &map, const int x_map, const int &y_map)
{
                return y_map * map->info.width + x_map;
}

void mapIndexToCell(const nav_msgs::OccupancyGridConstPtr &map, const size_t &index, int &x_map, int &y_map )
{
                x_map  = index % map->info.width;
                y_map = index / map->info.width;
}

void mapIndexToWorld(const nav_msgs::OccupancyGridConstPtr &map, size_t index, double &x_world, double &y_world)
{
                int x_map, y_map;
                mapIndexToCell(map, index, x_map, y_map);
                mapCellToWorld(map, x_map, y_map, x_world, y_world);
}

#endif /* PATH_PLANNING_CONSTANTS */
