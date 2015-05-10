/**
 * 
 * 
 * @file path_planner_straight_heuristic.cpp
 * 
 * @brief node to implement the "go straight && avoid with min cost" heuristic (idea no.3 in the exploration task solutions paper)
 * @author Petre Munteanu, clasianvmk (at) gmail (dot) com
 * 
 * 
 * Note: It is assumed that the exploration scenario starts with the robot inside the 2D map to be explored
 */

#include "ros/ros.h"
#include "constants.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

/* Pose-and-OccupancyGrid-related relevant variables */
//boost::mutex data_mutex; // boost mutex for scoped protection of relevant variables | Note: ros nevertheless executes all routines sequentially
nav_msgs::OccupancyGrid::ConstPtr occMap; // the full-size incoming datastructure
std::vector<signed char> mapData; // incoming char data[] in an OccupancyGrid instance
unsigned int width; // known 2D width of incoming vector data
unsigned int height; // known 2D height of incoming vector data
double posx; // 3D robot position - x-component
double posy; // 3D robot position - y-component
double posz; // 3D robot position - z-component

double orix; // 3D robot orientation quaternion - x component
double oriy; // 3D robot orientation quaternion - y component
double oriz; // 3D robot orientation quaternion - z component
double oriw; // 3D robot orientation quaternion - w component

bool init = false; // true if the occupancyGrid map dimensions and coordinates have been initialized; false otherwise
bool poseInit = false; // true if the robot Pose has been received; false otherwise

/* LaserScans-related relevant variables */
float angle_min, angle_max, angle_increment, range_min = 0., range_max = 0.; // variables to cache one by one the contents of incoming LaserScans
std::vector <float> ranges; 
int beams; // no. of beams per LaserScan computed out of the (range_max-range_min) difference over the angular density/resolution
int times_forced = 0; // global no. of times the robot was forced to perform a full in-place turn

/* OccupancyGrid digesting callback - gets called during ros::spinOnce() calls whenever a new OccupancyGrid message is available */
void digestOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    ROS_INFO("new OccupancyGrid received");
    init = true;
    occMap = map;
    geometry_msgs::Pose origin = map->info.origin;
//    boost::mutex::scoped_lock scoped_lock(data_mutex);
    width  = map->info.width;
    height = map->info.height;
    mapData.clear();
    mapData = map->data;
}

/* Pose digesting callback - gets called during ros::spinOnce() calls whenever a new Odometry message (containing the updated robot pose) is available */
void digestPoseOffOdometry(const nav_msgs::Odometry::ConstPtr &odometry)
{
    ROS_INFO("new robot Pose received");
    poseInit = true;
    const geometry_msgs::Pose pose = odometry->pose.pose;
    posx = pose.position.x;
    posy = pose.position.y;
    posz = pose.position.z;
    
    orix = pose.orientation.x;
    oriy = pose.orientation.y;
    oriz = pose.orientation.z;
    oriw = pose.orientation.w;
}

/* LaserScans digesting callback - gets called during ros::spinOnce() calls whenever a new LaserScan message is available */
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

/**
 * @brief evaluates distance to first obstacle in the direction of the laser beam whose ID is being provided
 * @param index "beam ID"; takes values from 0 to ranges.size()-1 (i.e. ranges vector from a LaserScan)
 * @return smoothed range value
 */
float average_range(int index)
{
    // checking indexing bounds to avoid accessing unallocated memory
    if(index - (AVERAGING_STREAK >> 1) < 0 || index + (AVERAGING_STREAK >> 1) >= ranges.size())
    {
        ROS_ERROR("Error: for index = %d the %d-long-averanging cannot be applied (note: max range size = %lu)", index, AVERAGING_STREAK, ranges.size());
        return 0.;
    }
    float sum = ranges[index];
    if(SIMPLE_STREAK)
    {
        // if the simple streak flag is set, it simply delivers the self value (no smoothing!)
        return sum;
    }
    // performing smoothing
    int limit = (AVERAGING_STREAK >> 1);
    for(int i = 1; i <= limit; ++i)
    {
        sum += ranges[index - i] + ranges[index + i];
    }
    return sum / AVERAGING_STREAK;
}

ros::Publisher pubOdometry; // to be initialized and registered (e.g. in main()) before calling the following function:
/**
 * @brief 
 * @param r ros loop rate
 * @param loop_rate dictates the no. of iterations i.e. controls the value of the to-turn angle
 * @param forced boolean flag indicating whether a forced rotation is being performed | defaults to true
 * @param speed in-turn linear speed | defaults to 0.0
 * @param sgn direction of rotation; -1 -> left; 1 -> right; 0 -> no rotation 
 */
void turn_inplace(ros::Rate r, double loop_rate, bool leap = false, bool forced = true, float speed = 0., float sgn = 1.)
{
    if(forced)
    {   // checking if full-rotation was forced already too many times
        if(++times_forced > MAX_TIMES_FORCED)
        {
            ROS_ERROR("Error: exceeded max no. %d of 180 degrees forced turns. Quitting...", MAX_TIMES_FORCED);
            ros::shutdown();
        }
    }
    // constructing Odometry command
    nav_msgs::Odometry odometryCommand;
    odometryCommand.pose.pose.orientation.w = 1.;
    odometryCommand.twist.twist.linear.x    = speed;
    odometryCommand.twist.twist.linear.y    = 0.;
    odometryCommand.twist.twist.linear.z    = 0.;
    odometryCommand.twist.twist.angular.x   = 0.;
    odometryCommand.twist.twist.angular.y   = 0.;
    odometryCommand.twist.twist.angular.z   = sgn * PI/2.;

    ROS_INFO("forced full turn no. %d | no. of complete-spin iterations SECONDS_TOTURN * LOOP_RATE = %d", times_forced, (int)(SECONDS_TOTURN * (double)LOOP_RATE));
    // iterating for turn
    for(int i = (int) (SECONDS_TOTURN * loop_rate); i > 0 && ros::ok(); --i)
    {
        pubOdometry.publish(odometryCommand);
        ros::spinOnce();
        r.sleep();
    }
    if(leap)
    {
        odometryCommand.twist.twist.linear.x    = MAX_TURN_SPEED;
        odometryCommand.twist.twist.angular.z   = 0.;
        for(int i = (int) (SECONDS_TOLEAP * loop_rate); i > 0 && ros::ok(); --i)
        {
            pubOdometry.publish(odometryCommand);
            ros::spinOnce();
            r.sleep();
        }
    }
}

bool outOfMapBounds()
{
    int robotx, roboty; // robot current location indices in the occupancy grid map (to be computed)
    worldToMapCell(occMap, posx, posy, robotx, roboty);
    if(robotx < 0 || roboty < 0 || robotx >= width || roboty >= height)
    {
        return true;
    }
    return false;
}

/// The robot avoids obstacles with a minimum effort and keeps on going with no stop performing 2D exploration
/// NOTE: it is assumed that the robot is initially aligned with the to-be-explored 2D plane
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_straight_heuristic");
    ros::NodeHandle n;
    pubOdometry = n.advertise<nav_msgs::Odometry>("/dataNavigator_G500RAUVI", 1032);
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber subOccupancyGrid = n.subscribe("/grid_mapping/costmap/costmap", 1032, digestOccupancyGrid);
    ros::Subscriber subPoseStamped   = n.subscribe("/uwsim/girona500_odom_RAUVI", 1032, digestPoseOffOdometry);
    ros::Subscriber subLaserScan     = n.subscribe("/girona500_RAUVI/multibeam", 1032, digestLaserScan);

    double to_turn; // angle to command the in-place turning
    while (ros::ok())
    {
        // checking the occupancyGrid map dimensions and coordinates to have been initialized
        // also checking for valid data in the LaserScans to be already present
        if(init && poseInit && ranges.size())
        {
            // checking for LaserScan data to be consistent with the computed no. of beams
            if(ranges.size() != beams)
            {
                ROS_WARN("Skipped one laser scan due to inconsistent ranges[] size %lu with beams no. %d", ranges.size(), beams);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            // before doing anything else, checking if the robot is not out of map bounds
            if(outOfMapBounds())
            {
                // robot reached out of the map: turning in-place (non-forcefully)
                turn_inplace(loop_rate, (double)LOOP_RATE, true, false);
                continue; // the spinOnce() and sleep() methods are being called by the in-place turning routine
            }
            nav_msgs::Odometry odometryCommand;
            int half_range = beams >> 1;
            int target_index = -1;
            float range_averaged = average_range(half_range);
            // going straight until next obstacle
            if(range_averaged >= SAFE_DIST_WEIGHT * range_max && range_averaged <= range_max)
            {
                odometryCommand.pose.pose.orientation.w = 1.;
                odometryCommand.twist.twist.linear.x    = MAX_SPEED;
                odometryCommand.twist.twist.linear.y    = 0.;
                odometryCommand.twist.twist.linear.z    = 0.;
                odometryCommand.twist.twist.angular.x   = 0.;
                odometryCommand.twist.twist.angular.y   = 0.;
                odometryCommand.twist.twist.angular.z   = 0.;
                
                pubOdometry.publish(odometryCommand);
            }
            else
            {
                // looking for the closest beam with convenient ranges[] value
                double target_min = SAFE_DIST_WEIGHT * range_max;
                to_turn = 0.;
                target_index = -1;
                float current_range_averaged;
                for(int i = 1 + BEAMS_SKIP; i <= half_range - (AVERAGING_STREAK >> 1); ++i)
                {
                    current_range_averaged = average_range(half_range + i);
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
                    // robot is stuck: turning in-place
                    turn_inplace(loop_rate, (double)LOOP_RATE);
                    continue; // the spinOnce() and sleep() methods are being called by the in-place turning routine
                }
                else
                {
                    // turning still in-place (with or without partial linear speed)
                    float speed = range_averaged < CRITICAL_SAFE_DIST_WEIGHT * range_max ? 0. : DEFAULT_MAX_TURN_SPEED;
                    to_turn = (double) (target_index - half_range) * angle_increment;
                    turn_inplace(loop_rate, abs(2. * to_turn / PI * (double)LOOP_RATE), false, false, speed, (float)sgn(to_turn));
                    ROS_INFO("turning in-place %lf percent of PI/2 to beam ID %d", 2. * to_turn / PI, target_index);
                    continue; // the spinOnce() and sleep() methods are being called by the in-place turning routine
                }
            }
            ROS_INFO("threshold range = %f | threshold critical = %f | range found = %f", SAFE_DIST_WEIGHT * range_max, CRITICAL_SAFE_DIST_WEIGHT * range_max, ranges[half_range]);
        }
        else
        {
            // confirming and communicating the responsible and prevailing error message
            if(!init)
            {
                ROS_WARN("OccupancyGrid map dimensions and coordinates have not been initialized. Skipping...");
            }
            else if(!poseInit)
            {
                ROS_WARN("No Robot Pose has been delivered yet. Skipping...");
            }
            else
            {
                ROS_WARN("Ranges not present. Skipping...");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
