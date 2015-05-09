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

float average_range(int index)
{
    if(index - (AVERAGING_STREAK >> 1) < 0 || index + (AVERAGING_STREAK >> 1) >= ranges.size())
    {
        ROS_ERROR("Error: for index = %d the %d-long-averanging cannot be applied (note: max range size = %lu)", index, AVERAGING_STREAK, ranges.size());
        return 0.;
    }
    float sum = ranges[index];
    if(SIMPLE_STREAK)
    {
        return sum;
    }
    int limit = (AVERAGING_STREAK >> 1);
    for(int i = 1; i <= limit; ++i)
    {
        sum += ranges[index - i] + ranges[index + i];
    }
    return sum / AVERAGING_STREAK;
}

/// The robot avoids obstacles with a minimum effort and keeps on going with no stop performing 2D exploration
/// NOTE: it is assumed that the robot is initially aligned with the to-be-explored 2D plane
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    ros::Publisher pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
    ros::Rate loop_rate(LOOP_RATE);
//    ros::Subscriber subOccupancyGrid = n.subscribe("/grid_mapping/costmap/costmap", 1032, digestOccupancyGrid);
//    ros::Subscriber subPoseStamped   = n.subscribe("/uwsim/girona500_odom_RAUVI", 1032, digestPoseStampedOffOdometry);
    ros::Subscriber subLaserScan     = n.subscribe("/girona500_RAUVI/multibeam", 1032, digestLaserScan);

    
    int times_forced = 0;
    float to_turn;
    while (ros::ok())
    {
        if(ranges.size())
        {
            if(ranges.size() != beams)
            {
                ROS_WARN("Skipped one laser scan due to inconsistent ranges[] size %lu with beams no. %d", ranges.size(), beams);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            nav_msgs::Odometry odometryCommand;
            int half_range = beams >> 1;
            int target_index;
            float range_averaged = average_range(half_range);
            if(range_averaged >= SAFE_DIST_WEIGHT * range_max && range_averaged <= range_max)
            {
                odometryCommand.pose.pose.orientation.w = 1.;
                odometryCommand.twist.twist.linear.x    = MAX_SPEED;
                odometryCommand.twist.twist.linear.y    = 0.;
                odometryCommand.twist.twist.linear.z    = 0.;
                odometryCommand.twist.twist.angular.x   = 0.;
                odometryCommand.twist.twist.angular.y   = 0.;
                odometryCommand.twist.twist.angular.z   = 0.;
                
                AVERAGING_STREAK = DEFAULT_AVERAGING_STREAK;
                pubOdometry.publish(odometryCommand);
            }
            else
            {
                // looking for the closest beam with convenient ranges[] value
                float target_min = SAFE_DIST_WEIGHT * range_max;
                to_turn = 0.;
                target_index = -1;
                for(int i = 1; i <= half_range - (AVERAGING_STREAK >> 1); ++i)
                {
                    float current_range_averaged = average_range(half_range + i);
                    if(current_range_averaged >= target_min && current_range_averaged <= range_max)
                    {
                        target_index = half_range + i;
                        break;
                    }
                    current_range_averaged = average_range(half_range - i);
                    if(current_range_averaged >= target_min && current_range_averaged <= range_max)
                    {
                        target_index = half_range - i;
                        break;
                    }
                }
                if(-1 == target_index)
                {
                    // then trying the extremes first
                    
                    
                    // finally turning 180 degrees since the robot got stuck
                    if(times_forced > MAX_TIMES_FORCED)
                    {
                        ROS_ERROR("Error: exceeded max no. %d of 180 degrees forced turns. Quitting...", MAX_TIMES_FORCED);
                        return 1;
                    }
                    to_turn = PI/2.;
                    int rotational_limit = (int)(SECONDS_TOTURN * (double) LOOP_RATE);
                    for(int i = 0; i < rotational_limit && ros::ok(); ++i)
                    {
                        odometryCommand.pose.pose.orientation.w = 1.;
                        odometryCommand.twist.twist.linear.x    = 0.;
                        odometryCommand.twist.twist.linear.y    = 0.;
                        odometryCommand.twist.twist.linear.z    = 0.;
                        odometryCommand.twist.twist.angular.x   = 0.;
                        odometryCommand.twist.twist.angular.y   = 0.;
                        odometryCommand.twist.twist.angular.z   = to_turn;

                        pubOdometry.publish(odometryCommand);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    ROS_INFO("forced to turn 180 degrees | time no. %d", times_forced++);
                }
                else
                {
                    to_turn = (double) (target_index - half_range) * angle_increment;
                }
                // start braking (apply. speed 0.0 if critical safety distance is not ensured any more)
                double speed = SPEED;
                if(range_averaged < CRITICAL_SAFE_DIST_WEIGHT * range_max)
                {
                    speed = 0.;
                    AVERAGING_STREAK = DEFAULT_CRITICAL_AVERAGING_STREAK;
                }
                else
                {
                    AVERAGING_STREAK = DEFAULT_AVERAGING_STREAK;
                }

                // turning
                for(int i = 0; i < ROTATIONAL_ITERATIONS && ros::ok(); ++i)
                {
                    odometryCommand.pose.pose.orientation.w = 1.;
                    odometryCommand.twist.twist.linear.x    = speed;
                    odometryCommand.twist.twist.linear.y    = 0.;
                    odometryCommand.twist.twist.linear.z    = 0.;
                    odometryCommand.twist.twist.angular.x   = 0.;
                    odometryCommand.twist.twist.angular.y   = 0.;
                    odometryCommand.twist.twist.angular.z   = to_turn;

                    pubOdometry.publish(odometryCommand);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            ROS_INFO("threshold range = %f | threshold critical = %f | range found = %f | range best index = %d", SAFE_DIST_WEIGHT * range_max, CRITICAL_SAFE_DIST_WEIGHT * range_max, ranges[half_range], target_index);
        }
        else
        {
            ROS_WARN("Ranges not present. Skipping...");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
