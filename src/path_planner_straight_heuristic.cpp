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

#include <vector>
#include "ros/ros.h"
#include "constants.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

void digestOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    ROS_INFO("I heard of an OccupancyGrid");
    unsigned int width  = map->info.width;
    unsigned int height = map->info.height;
    geometry_msgs::Pose origin = map->info.origin;

    const std::vector<signed char> data = map->data;
}

void digestPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ROS_INFO("I heard of a PoseStamped");
    double posx = pose->pose.position.x;
    double posy = pose->pose.position.y;
    double posz = pose->pose.position.z;
    
    double orix = pose->pose.orientation.x;
    double oriy = pose->pose.orientation.y;
    double oriz = pose->pose.orientation.z;
    double oriw = pose->pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    ros::Publisher pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber subOccupancyGrid = n.subscribe("nav_msgs/OccupancyGrid",    1032, digestOccupancyGrid);
    ros::Subscriber subPoseStamped   = n.subscribe("geometry_msgs/PoseStamped", 1032, digestPoseStamped);

    std::cout << "Introduce the 6 values:\n";
    double x, y, z, rx, ry, rz;
    std::cin >> x >> y >> z >> rx >> ry >> rz;
    checkMaxSpeed(x);
    checkMaxSpeed(y);
    checkMaxSpeed(z);
    while (ros::ok())
    {
        nav_msgs::Odometry odometryCommand;

        odometryCommand.pose.pose.orientation.w = 1.;
        odometryCommand.twist.twist.linear.x  =  x;
        odometryCommand.twist.twist.linear.y  =  y;
        odometryCommand.twist.twist.linear.z  =  z;
        odometryCommand.twist.twist.angular.x = rx;
        odometryCommand.twist.twist.angular.y = ry;
        odometryCommand.twist.twist.angular.z = rz;

        pubOdometry.publish(odometryCommand);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
