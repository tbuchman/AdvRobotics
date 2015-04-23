#include "ros/ros.h"
#include "std_msgs/String.h"

void digestOccupancyGrid()
{
    ROS_INFO("I heard of an OccupancyGrid");
}

void digestPoseStamped()
{
    ROS_INFO("I heard PoseStamped");
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
