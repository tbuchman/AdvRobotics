/**
 * 
 * 
 * @file path_planner_straight_heuristic.cpp
 * 
 * @brief node to implement the "go straight && avoid with min cost" heuristic (idea no.3 in the exploration task solutions paper)
 * @author Petre Munteanu, clasianvmk (at) gmail (dot) com
 * 
 * 
 * 
 */

#include "ros/ros.h"
#include "constants.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

boost::mutex data_mutex;
std::vector<signed char> mapData;
unsigned int width;
unsigned int height;

void digestOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    ROS_INFO("I heard of a new OccupancyGrid");
    geometry_msgs::Pose origin = map->info.origin;
    boost::mutex::scoped_lock scoped_lock(data_mutex);
    width  = map->info.width;
    height = map->info.height;
    mapData.clear();
    mapData = map->data;
}

void digestPoseStampedOffOdometry(const nav_msgs::Odometry::ConstPtr &odometry)
{
    const geometry_msgs::Pose pose = odometry->pose.pose;
    ROS_INFO("I heard of a new Pose");
    double posx = pose.position.x;
    double posy = pose.position.y;
    double posz = pose.position.z;
    
    double orix = pose.orientation.x;
    double oriy = pose.orientation.y;
    double oriz = pose.orientation.z;
    double oriw = pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    ros::Publisher pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber subOccupancyGrid = n.subscribe("/grid_mapping/costmap/costmap",    1032, digestOccupancyGrid);
    ros::Subscriber subPoseStamped   = n.subscribe("/uwsim/girona500_odom_RAUVI", 1032, digestPoseStampedOffOdometry);

    char c;
    while (ros::ok())
    {
        std::cout << "Will not print current occupancy grid. Hit ENTER to continue...\n\n";
        {
            std::cin.get();
            boost::mutex::scoped_lock scoped_lock(data_mutex);
            std::cout << "map height = " << height << " | map width = " << width << '\n';
            if(mapData.size())
            {
                std::cout << "Sample data: " << (int)mapData[0] << ' ' << (int)mapData[1] << ' ' << (int)mapData[2] << '\n';
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
