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
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

void digestOccupancyGrid(const nav_msgs/OccupancyGrid)
{
    ROS_INFO("I heard of an OccupancyGrid");
}

void digestPoseStamped()
{
    ROS_INFO("I heard of a PoseStamped");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("nav_msgs/OccupancyGrid", 1032, digestOccupancyGrid);
    ros::Subscriber sub = n.subscribe("geometry_msgs/PoseStamped", 1032, digestPoseStamped);

    ros::spin();
    return 0;
}
            