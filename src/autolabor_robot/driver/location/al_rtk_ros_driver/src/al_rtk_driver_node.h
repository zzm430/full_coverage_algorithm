//
// Created by autolabor on 2022/3/17.
//

#ifndef AL_RTK_DRIVER_AL_RTK_DRIVER_NODE_H
#define AL_RTK_DRIVER_AL_RTK_DRIVER_NODE_H
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include "nmea_msgs/Gpgga.h"
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include "configor/json.hpp"
#include "al_rtk_raw_driver.h"

using namespace configor;

struct wgs84_pos
{
    double latitude;
    double longtitude;
    double altitude;
};

struct enu_pos
{
    double e; //东
    double n; //北
    double u; //上
};

al_rtk::RTKDriver *rtk;

wgs84_pos enu_origin; // 东北天原点对应的 WGS84 参考点

geometry_msgs::PointStamped current_pos;

std::string map_frame;
ros::Publisher gpgga_pub;
ros::Publisher local_pos_pub;
bool publish_pos_fix_only;
bool auto_use_first_fix_as_enu_origin;
bool is_origin_set;
std::string origin_config_file;
std::string log_dir;
json node_config;


void publish_gpgga(nmea_msgs::Gpgga * gpgga);
int decode_gpgga(std::string gpgga_str,nmea_msgs::Gpgga * gpgga);
void on_receive_gpgga_str(std::string gpgga_str);
void set_enu_origin(wgs84_pos *source, bool save_to_config);
void convert_WGS84_to_enu(wgs84_pos * reference, wgs84_pos * source, enu_pos * target);
void sigint_handler(int sig);
void publish_local_position(enu_pos * pos);

#endif //AL_RTK_DRIVER_AL_RTK_DRIVER_NODE_H


