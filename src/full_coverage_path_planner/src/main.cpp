#include "full_coverage_path_planner/boustrophedon_stc.h"

#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace cv;

using PoseStamped = geometry_msgs::PoseStamped;

static double euDist2D(const PoseStamped& p1, const PoseStamped& p2)
{
    return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
}

static PoseStamped parametricInterp1(const PoseStamped& p1, const PoseStamped& p2, double alpha)
{
    PoseStamped p;
    p.header.frame_id = p1.header.frame_id;
    p.pose.orientation = p1.pose.orientation;
    p.pose.position.x = p1.pose.position.x + alpha * (p2.pose.orientation.x - p1.pose.orientation.x);
    p.pose.position.y = p1.pose.position.y + alpha * (p2.pose.orientation.y - p1.pose.orientation.y);
    return p;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    //  4 4 0 0 0  robot
    std::string start_pose, robotNamespace;
    // 0.25
    float robotRadius;

    // Load Parameters
    ros::param::get("~start_pose", start_pose);
    ros::param::get("~robot_radius", robotRadius);
    ros::param::get("~robot_namespace", robotNamespace);

    // wait map topic loading 
    auto occ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");

    std::vector<std::vector<bool>> grid;  // Binary matrix for path planning

    // can use farther object --- there use son object
    full_coverage_path_planner::BoustrophedonSTC planner;

    Point_t scaled;    // This will hold the index of the start location in binary matrix
    PoseStamped pose;  // This is the point from which path planning starts
    {
        // 起始位姿信息
        std::stringstream ss(start_pose);
        ss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z;   // 4 4 0
        double roll, pitch, yaw;
        ss >> roll >> pitch >> yaw;
#if(0)
        ROS_INFO(" start pose infomation : position x = %f  position y = %f position z = %f ",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z );
        ROS_INFO(" start pose infomation : roll = %f  pitch = %f yaw = %f ", roll , pitch , yaw );
#endif
        tf2::Quaternion q; 
        q.setRPY(roll, pitch, yaw);  // 欧拉角转四元数
        pose.header.frame_id = "map";
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
    }

    // 区间划分
    // Convert occupancy grid to binary matrix
    planner.parseGrid(*occ, grid, robotRadius, robotRadius, pose, scaled);
    
    // 路径规划
    // Boustrophedon path planning
    int multiple_pass_counter, visited_counter;
    std::list<Point_t> path = full_coverage_path_planner::BoustrophedonSTC::boustrophedon_stc(
        grid, scaled, multiple_pass_counter, visited_counter);


    // 坐标系转换 并发布
    // path has the indices of corner points in the path, we now convert that to real world coordinates
    std::vector<PoseStamped> plan;
    plan.reserve(path.size());
    ROS_INFO( "Path point numbers = %ld ", path.size());
    planner.parsePointlist2Plan(pose, path, plan);


    auto planPub = nh.advertise<nav_msgs::Path>("boustrophedon/path", 10, true);
    auto pathMsg = nav_msgs::Path();
    pathMsg.header.frame_id = "map";
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.poses = plan;
    planPub.publish(pathMsg);

    
    // Inorder to divide the path among agents, we need more points in between the corner points of the path.
    // We just perform a linear parametric up-sampling
    std::vector<PoseStamped> upSampled;

    ROS_INFO("Path computed. Up-sampling...");
    for (size_t i = 0; i < plan.size() - 1; ++i)
    {
        double mul = 0.5 * euDist2D(plan[i], plan[i + 1]) / floor(euDist2D(plan[i], plan[i + 1]) / robotRadius);
        double a = 0;
        while (a < 1)
        {
            upSampled.push_back(parametricInterp1(plan[i], plan[i + 1], a));
            a += mul;
        }
        upSampled.push_back(plan[i + 1]);
    }

    ROS_INFO("Up-sampling complete");
    
    ROS_INFO("Waiting for number of agents");

    // Prepare publishers
    // size_t nAgents = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;
    size_t nAgents = 1;
    // size_t nAgents = 1;
    ROS_INFO(" ================================ nAgents = %ld ",nAgents);
    std::vector<ros::Publisher> waypointPublishers;
    waypointPublishers.reserve(nAgents);
    for (auto i = 0; i < nAgents; ++i)
    {
        // std::stringstream ss;
        // ss << robotNamespace << "_" << i << "/waypoints";
        // waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));
        waypointPublishers.push_back(nh.advertise<nav_msgs::Path>("waypoints", 100, true));
    }

    // Now divide the path approximately equally for each agent
    std::vector<nav_msgs::Path> agentPaths(nAgents);

    size_t length = upSampled.size() / nAgents;
    size_t leftover = upSampled.size() % nAgents;
    size_t begin = 0, end = 0;

    ROS_INFO("Publishing paths");
    for (size_t i = 0; i < std::min(nAgents, upSampled.size()); ++i)
    {
        end += leftover > 0 ? (length + !!(leftover--)) : length;

        agentPaths[i].header.frame_id = "map";
        agentPaths[i].header.stamp = ros::Time::now();
        agentPaths[i].poses.reserve(end - begin);

        for (size_t j = begin; j < end; ++j)
        {
            agentPaths[i].poses.push_back(upSampled[j]);
        }
        waypointPublishers[i].publish(agentPaths[i]);
        begin = end;
    }
    ROS_ERROR_STREAM(upSampled.size());
    size_t s = 0;
    for (size_t i = 0; i < nAgents; ++i)
    {
        s += agentPaths[i].poses.size();
    }
    ROS_ERROR_STREAM(s);
    ros::spin();
    return EXIT_SUCCESS;
}
