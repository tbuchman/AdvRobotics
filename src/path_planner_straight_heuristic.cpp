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
#include "geometry_msgs/PoseStamped.h"

void digestOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    ROS_INFO("I heard of an OccupancyGrid");
}

void digestPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ROS_INFO("I heard of a PoseStamped");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    ros::Publisher pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
//    ros::Rate loop_rate(100);
    ros::Subscriber subOccupancyGrid = n.subscribe("nav_msgs/OccupancyGrid",    1032, digestOccupancyGrid);
    ros::Subscriber subPoseStamped   = n.subscribe("geometry_msgs/PoseStamped", 1032, digestPoseStamped);

    while (ros::ok())
    {
        std::cout << "Introduce the 6 values:\n";
        double x, y, z, rx, ry, rz;
        std::cin >> x >> y >> z >> rx >> ry >> rz;
        nav_msgs::Odometry odometryCommand;
        odometryCommand.twist.twist.linear.x  =  x;
        odometryCommand.twist.twist.linear.y  =  y;
        odometryCommand.twist.twist.linear.z  =  z;
        odometryCommand.twist.twist.angular.x = rx;
        odometryCommand.twist.twist.angular.y = ry;
        odometryCommand.twist.twist.angular.z = rz;

        pubOdometry.publish(odometryCommand);
//        ros::spinOnce();
//        loop_rate.sleep();
    }
    return 0;
}
