#include "full_coverage_path_planner/boustrophedon_stc.h"
#include "full_coverage_path_planner/bspline.h"
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
#include <iostream>
#include <fstream>

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

struct Point {
    double x;
    double y;
};

double  dot_product(Point_t a, Point_t b){
    return a.x * b.x + a.y * b.y;
}

double  length(Point_t v){
    return sqrt(v.x * v.x + v.y * v.y);
}

double angle(Point_t a, Point_t b){
    double dot = dot_product(a,b);
    double  len_a = length(a);
    double len_b = length(b);
    return acos(dot/(len_a *len_b));
}


std::vector<Point> densify(const std::vector<Point>& points, int factor) {
    std::vector<Point> densified_points;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        densified_points.push_back(points[i]);

        for (int j = 1; j < factor; ++j) {
            double t = static_cast<double>(j) / factor;
            Point interpolated_point;
            interpolated_point.x = points[i].x + t * (points[i + 1].x - points[i].x);
            interpolated_point.y = points[i].y + t * (points[i + 1].y - points[i].y);
            densified_points.push_back(interpolated_point);
        }
    }

    densified_points.push_back(points.back());
    return densified_points;
}

double distance(PoseStamped a, PoseStamped b){
    return sqrt(pow(a.pose.position.x - b.pose.position.x,2) + pow(a.pose.position.y -b.pose.position.y,2));
} 


int main(int argc, char** argv)
{
    ofstream   location_out;
    ofstream   location_updated_out;
    ofstream   location_increase;
    ofstream   location_get_path;
    location_out.open("/home/metoak/Downloads/test_path_figure-main/src/location_out.txt",std::ios::out);
    location_updated_out.open("/home/metoak/Downloads/test_path_figure-main/src/location_updated_out.txt",std::ios::out);
    location_increase.open("/home/metoak/Downloads/test_path_figure-main/src/location_increase.txt",std::ios::out);
    location_get_path.open("/home/metoak/Downloads/test_path_figure-main/src/location_get_path.txt",std::ios::out);
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    std::string start_pose, robotNamespace;
    float robotRadius;

    // Load Parameters
    ros::param::get("~start_pose", start_pose);
    ros::param::get("~robot_radius", robotRadius);
    ros::param::get("~robot_namespace", robotNamespace);

    auto occ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");

    std::vector<std::vector<bool>> grid;  // Binary matrix for path planning
    full_coverage_path_planner::BoustrophedonSTC planner;
    Point_t scaled;    // This will hold the index of the start location in binary matrix
    PoseStamped pose;  // This is the point from which path planning starts
    {
        std::stringstream ss(start_pose);
        ss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z;
        double roll, pitch, yaw;
        ss >> roll >> pitch >> yaw;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.header.frame_id = "map";
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
    }
    // Convert occupancy grid to binary matrix
    planner.parseGrid(*occ, grid, robotRadius, robotRadius, pose, scaled);

    // Boustrophedon path planning
    int multiple_pass_counter, visited_counter;
    std::list<Point_t> path = full_coverage_path_planner::BoustrophedonSTC::boustrophedon_stc(
        grid, scaled, multiple_pass_counter, visited_counter);



    for(auto i = path.begin(); i != path.end();i++){
        location_out  << (*i).x << " ";
    }
    location_out  << endl;

    for(auto i = path.begin(); i != path.end();i++){
        location_out  << (*i).y << " ";
    }
    location_out  << endl;

    // path has the indices of corner points in the path, we now convert that to real world coordinates
    // std::vector<PoseStamped> plan;
    // plan.reserve(path.size());
    // planner.parsePointlist2Plan(pose, path, plan);
    std::list<Point_t>   get_path;

  // path has the indices of corner points in the path, we now convert that to real world coordinates
    std::vector<PoseStamped> plan;
    // plan.reserve(path.size());
    // planner.parsePointlist2Plan(pose, path, plan);

    // 先对数据进行增密处理
     vector<Point>   transfer_path;
     for(auto it= path.begin();it != path.end();it++){
           Point   temp_point;
           temp_point.x = (*it).x ;
           temp_point.y = (*it).y ;
           transfer_path.push_back(temp_point);
     }
    vector<Point>   result_path;
    int factor = 5;
    result_path = densify(transfer_path,factor);
    for(auto it : result_path){
        ROS_INFO_STREAM("reslut paht size is :"<< it.x    << " " << it.y);
    }
   


   //增加增密后的点的显示
   for(auto i  = 0; i < result_path.size();i++){
         location_increase << result_path[i].x  << " ";
   }
   location_increase << endl;

   for(auto i = 0; i < result_path.size();i++){
       location_increase << result_path[i].y  << " ";
   }
   location_increase  << endl;


    //对坐标进行转换数据格式
    auto  count = result_path.size();
    auto  m_count = ceil(count /20);
    std::vector<std::vector<double>>   x_set,y_set;
    int  m_temp = 0;
    std::vector<double> m_x_set,m_y_set;
    for(int i = 0 ;i < count;i++){
        m_temp+=1;
        if(m_temp%20 == 0){
            x_set.push_back(m_x_set);
            y_set.push_back(m_y_set);
            m_x_set.clear();
            m_y_set.clear();
            m_temp = 0;
        }
        m_x_set.push_back(result_path[i].x);
        m_y_set.push_back(result_path[i].y);
    }

   

    //处理平滑
    // plan.clear();
    Points points;
    PoseStamped  first_point,end_point;
    
    for(int i  = 0; i < x_set.size();i++){
        ROS_INFO_STREAM("x set size is :" << x_set[i].size());
        points = B_Spline(x_set[i],y_set[i]);
        //验证使用
        for(auto m = 0;m < x_set[i].size();m++){
            ROS_INFO_STREAM("the value is " << x_set[i][m]   << "y" << y_set[i][m]); 
        }
        for(auto j = 0; j <  points.points_x.size();j++){
            PoseStamped  pose;
            pose.pose.position.x =  points.points_x[j];
            pose.pose.position.y =  points.points_y[j];
            if(pose.pose.position.x == 0){
                continue;
            }
            plan.push_back(pose);
            // Point_t  point;
            // point.x = points.points_x[j];
            // point.y = points.points_y[j];
            // get_path.push_back(point);
        }
    }

    //过滤无用的点
    std::vector<PoseStamped>   dress_path;
    double threshold =0.25;
    dress_path.push_back(plan[0]);
    for(int i = 1;i < plan.size();i++){
        //构造两个向量用来做内积判断
        Point_t  vector_1,vector_2;
        vector_1.x = plan[i+1].pose.position.x - plan[i].pose.position.x;
        vector_1.y = plan[i+1].pose.position.y - plan[i].pose.position.y;
        vector_2.x = plan[i+2].pose.position.x - plan[i+1].pose.position.x;
        vector_2.y = plan[i+2].pose.position.x - plan[i+1].pose.position.x;
        double angle_value = angle(vector_1,vector_2);
        ROS_INFO_STREAM("the value is :" << angle_value);
        if(distance(plan[i],dress_path.back()) > threshold ){
            // if((plan[i -1].pose.position.y == plan[i].pose.position.y) && (plan[i + 1].pose.position.y == plan[i].pose.position.y)){
            //     continue;
            // }
            dress_path.push_back(plan[i]);
        }
    }
    dress_path.push_back(plan[plan.size()-1]);


     //平滑后的数据可视化
    for(auto i = 0; i < dress_path.size();i++){
    location_get_path  << dress_path[i].pose.position.x  << " ";
    }
    location_get_path  << endl;

    for(auto i = 0; i < dress_path.size();i++){
        location_get_path  << dress_path[i].pose.position.y << " ";
    }
    location_get_path  << endl;


    
    //平滑后的数据可视化
    for(auto i = plan.begin(); i != plan.end();i++){
    location_updated_out  << (*i).pose.position.x  << " ";
    }
    location_updated_out  << endl;

    for(auto i = plan.begin(); i != plan.end();i++){
        location_updated_out  << (*i).pose.position.y << " ";
    }
    location_updated_out  << endl;
    
   
    auto planPub = nh.advertise<nav_msgs::Path>("boustrophedon/path", 10, true);
    auto pathMsg = nav_msgs::Path();
    pathMsg.header.frame_id = "map";
    pathMsg.header.stamp = ros::Time::now();
    // pathMsg.poses = plan;
    pathMsg.poses = dress_path;
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
    // location_out.close();
    // location_updated_out.close();
    // Prepare publishers
    // size_t nAgents = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;
    size_t nAgents = 1;
    std::vector<ros::Publisher> waypointPublishers;
    waypointPublishers.reserve(nAgents);
    for (auto i = 0; i < nAgents; ++i)
    {
        std::stringstream ss;
        ss << robotNamespace << "_" << i << "/waypoints";
        waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));
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
