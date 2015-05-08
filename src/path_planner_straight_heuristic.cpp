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
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

//boost::mutex data_mutex;
//std::vector<signed char> mapData;
//unsigned int width;
//unsigned int height;

float angle_min, angle_max, angle_increment, range_min = 0., range_max = 0.;
std::vector <float> ranges;
int beams;

//void digestOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &map)
//{
//    ROS_INFO("I heard of a new OccupancyGrid");
//    geometry_msgs::Pose origin = map->info.origin;
////    boost::mutex::scoped_lock scoped_lock(data_mutex);
//    width  = map->info.width;
//    height = map->info.height;
//    mapData.clear();
//    mapData = map->data;
//}

//void digestPoseStampedOffOdometry(const nav_msgs::Odometry::ConstPtr &odometry)
//{
//    ROS_INFO("I heard of a new Pose");
//    const geometry_msgs::Pose pose = odometry->pose.pose;
//    double posx = pose.position.x;
//    double posy = pose.position.y;
//    double posz = pose.position.z;
//    
//    double orix = pose.orientation.x;
//    double oriy = pose.orientation.y;
//    double oriz = pose.orientation.z;
//    double oriw = pose.orientation.w;
//}

void digestLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_increment = scan->angle_increment;
    range_min = scan->range_min;
    range_max = scan->range_max;
    ranges = scan->ranges;
    beams = ((int)((angle_max - angle_min) / angle_increment)) + 1;
//    ROS_INFO("I heard of a new LaserScan of %d beams and a matching data size %lu", beams, ranges.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    ros::Publisher pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
    ros::Rate loop_rate(LOOP_RATE);
//    ros::Subscriber subOccupancyGrid = n.subscribe("/grid_mapping/costmap/costmap", 1032, digestOccupancyGrid);
//    ros::Subscriber subPoseStamped   = n.subscribe("/uwsim/girona500_odom_RAUVI", 1032, digestPoseStampedOffOdometry);
    ros::Subscriber subLaserScan     = n.subscribe("/girona500_RAUVI/multibeam", 1032, digestLaserScan);

    while (ros::ok())
    {
        if(ranges.size())
        {
            if(ranges.size() != beams)
            {
                ROS_WARN("Skipped one laser scan due to inconsistent ranges[] size %lu with beams no. %d", ranges.size(), beams);
            }
            nav_msgs::Odometry odometryCommand;

            if(ranges[beams >> 1] >= SAFE_DIST_WEIGHT * range_max)
            {
                odometryCommand.pose.pose.orientation.w = 1.;
                odometryCommand.twist.twist.linear.x    = 1.;
                odometryCommand.twist.twist.linear.y    = 0.;
                odometryCommand.twist.twist.linear.z    = 0.;
                odometryCommand.twist.twist.angular.x   = 0.;
                odometryCommand.twist.twist.angular.y   = 0.;
                odometryCommand.twist.twist.angular.z   = 0.;

                pubOdometry.publish(odometryCommand);
            }
            else
            {
                
                
                
                
            }
            ROS_INFO("threshold range = %f | range found = %f", SAFE_DIST_WEIGHT * range_max, ranges[beams >> 1]);
        }
        else
        {
            ROS_WARN("Empty ranges. Skipping...");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
