//
// Created by autolabor on 2022/3/17.
//

#include "al_rtk_driver_node.h"

#include <signal.h>


void test_call_back(std::string data) {

    std::cout << "call back:" << data << std::endl;

}


void on_receive_gpgga_str(std::string gpgga_str) {

    std::cout <<"received gpgga "<< gpgga_str;

    if (rtk->is_gpgga_empty(gpgga_str))
    {
        return;
    }
    nmea_msgs::Gpgga gpgga;

    decode_gpgga(gpgga_str, &gpgga);

    publish_gpgga(&gpgga);

    wgs84_pos wgs84pos;

    wgs84pos.latitude = gpgga.lat;

    wgs84pos.longtitude = gpgga.lon;

    wgs84pos.altitude = gpgga.alt;

    if (gpgga.gps_qual == 4 && is_origin_set == false && auto_use_first_fix_as_enu_origin) { //按照配置设置enu坐标系原点

        set_enu_origin(&wgs84pos,true);
    }

    if (publish_pos_fix_only && gpgga.gps_qual != 4) {

        return;
    }

    if (is_origin_set == false) {

        return;
    }

    enu_pos local_pos;

    convert_WGS84_to_enu(&enu_origin,&wgs84pos, &local_pos);

    publish_local_position(&local_pos);


}

int decode_gpgga(std::string gpgga_str, nmea_msgs::Gpgga *gpgga) {


    std::stringstream ss(gpgga_str);

    std::vector<std::string> items;

    while (ss.good()) {

        std::string substr;

        getline(ss, substr, ',');

        items.push_back(substr);
    }



    float t_hours = std::stof(items.at(1).substr(0, 2));
    float t_minutes = std::stof(items.at(1).substr(2, 2));
    float t_seconds = std::stof(items.at(1).substr(4, items.at(1).length() - 4));

    gpgga->utc_seconds = t_hours * 3600 + t_minutes * 60 + t_seconds;

    std::string lat_degrees = items.at(2).substr(0, 2);

    std::string lat_minutes = items.at(2).substr(2, items.at(2).length() - 2);

    gpgga->lat = std::stod(lat_degrees) + std::stod(lat_minutes) / 60.0;

    if (0 == items.at(3).compare("S")) {
        gpgga->lat = -gpgga->lat;
    }
    gpgga->lat_dir = items.at(3);

    std::string lon_degrees = items.at(4).substr(0, 3);
    std::string lon_minutes = items.at(4).substr(3, items.at(4).length() - 3);

    gpgga->lon = std::stod(lon_degrees) + std::stod(lon_minutes) / 60.0;

    if (0 == items.at(5).compare("W")) {
        gpgga->lon = -gpgga->lon;
    }
    gpgga->lon_dir = items.at(5);
    gpgga->gps_qual = std::stoi(items.at(6));
    gpgga->num_sats = std::stoi(items.at(7));
    gpgga->hdop = std::stod(items.at(8));
    gpgga->alt = std::stod(items.at(9));
    gpgga->altitude_units = "m";

    return 0;
}

void publish_gpgga(nmea_msgs::Gpgga *gpgga) {

    gpgga_pub.publish(*gpgga);

}

void publish_local_position(enu_pos *pos) {

    current_pos.header.stamp = ros::Time::now();
    current_pos.header.frame_id = map_frame;
    current_pos.point.x = pos->e;
    current_pos.point.y = pos->n;
    current_pos.point.z = pos->u;
    local_pos_pub.publish(current_pos);
}


void set_enu_origin(wgs84_pos *source, bool save_to_config) {

    is_origin_set = true;

    enu_origin.latitude = source->latitude;
    enu_origin.longtitude = source->longtitude;
    enu_origin.altitude = source->altitude;

    if (save_to_config) {

        node_config["enu_origin"]["latitude"] = std::to_string(enu_origin.latitude);

        node_config["enu_origin"]["longtitude"] = std::to_string(enu_origin.longtitude);

        node_config["enu_origin"]["altitude"] = std::to_string(enu_origin.altitude);

        std::string current_path = ros::package::getPath("al_rtk_ros_driver");

        std::ofstream ofs_config(origin_config_file);

        ofs_config << std::setw(4) << node_config << std::endl;


    }
}

    void convert_WGS84_to_enu(wgs84_pos *reference, wgs84_pos *source, enu_pos *target) {
//    toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

#ifndef M_PI
        constexpr double M_PI = 3.141592653589793;
#endif
        constexpr double DEG_TO_RAD{M_PI / 180.0};
        constexpr double HALF_PI{M_PI / 2.0};
        constexpr double EPSILON10{1.0e-10};
        constexpr double EPSILON12{1.0e-12};

        constexpr double EQUATOR_RADIUS{6378137.0};
        constexpr double FLATTENING{1.0 / 298.257223563};
        constexpr double SQUARED_ECCENTRICITY{2.0 * FLATTENING - FLATTENING * FLATTENING};
        constexpr double SQUARE_ROOT_ONE_MINUS_ECCENTRICITY{0.996647189335};
        constexpr double POLE_RADIUS{EQUATOR_RADIUS * SQUARE_ROOT_ONE_MINUS_ECCENTRICITY};

        constexpr double C00{1.0};
        constexpr double C02{0.25};
        constexpr double C04{0.046875};
        constexpr double C06{0.01953125};
        constexpr double C08{0.01068115234375};
        constexpr double C22{0.75};
        constexpr double C44{0.46875};
        constexpr double C46{0.01302083333333333333};
        constexpr double C48{0.00712076822916666666};
        constexpr double C66{0.36458333333333333333};
        constexpr double C68{0.00569661458333333333};
        constexpr double C88{0.3076171875};

        constexpr double R0{C00 - SQUARED_ECCENTRICITY * (C02 + SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY *
                                                                                              (C06 +
                                                                                               SQUARED_ECCENTRICITY *
                                                                                               C08)))};
        constexpr double R1{SQUARED_ECCENTRICITY * (C22 - SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY * (C06 +
                                                                                                                SQUARED_ECCENTRICITY *
                                                                                                                C08)))};
        constexpr double R2T{SQUARED_ECCENTRICITY * SQUARED_ECCENTRICITY};
        constexpr double R2{R2T * (C44 - SQUARED_ECCENTRICITY * (C46 + SQUARED_ECCENTRICITY * C48))};
        constexpr double R3T{R2T * SQUARED_ECCENTRICITY};
        constexpr double R3{R3T * (C66 - SQUARED_ECCENTRICITY * C68)};
        constexpr double R4{R3T * SQUARED_ECCENTRICITY * C88};

        auto mlfn = [&](const double &lat) {
            const double sin_phi{std::sin(lat)};
            const double cos_phi{std::cos(lat) * sin_phi};
            const double squared_sin_phi = sin_phi * sin_phi;
            return (R0 * lat - cos_phi * (R1 + squared_sin_phi * (R2 + squared_sin_phi * (R3 + squared_sin_phi * R4))));
        };


        const double ML0{mlfn(reference->latitude * DEG_TO_RAD)};

        auto msfn = [&](const double &sinPhi, const double &cosPhi, const double &es) { return (cosPhi / std::sqrt(
                1.0 - es * sinPhi * sinPhi));
        };

        auto project = [&](double lat, double lon) {
            std::array<double, 2> retVal{lon, -1.0 * ML0};
            if (!(std::abs(lat) < EPSILON10)) {
                const double ms{(std::abs(std::sin(lat)) > EPSILON10) ?
                                msfn(std::sin(lat), std::cos(lat), SQUARED_ECCENTRICITY) / std::sin(lat) : 0.0};
                retVal[0] = ms * std::sin(lon *= std::sin(lat));
                retVal[1] = (mlfn(lat) - ML0) + ms * (1.0 - std::cos(lon));
            }
            return retVal;
        };

        auto fwd = [&](double lat, double lon) {
            const double D = std::abs(lat) - HALF_PI;
            if ((D > EPSILON12) || (std::abs(lon) > 10.0)) {
                return std::array<double, 2>{0.0, 0.0};
            }
            if (std::abs(D) < EPSILON12) {
                lat = (lat < 0.0) ? -1.0 * HALF_PI : HALF_PI;
            }
            lon -= reference->longtitude * DEG_TO_RAD;
            const auto projectedRetVal{project(lat, lon)};
            return std::array<double, 2>{EQUATOR_RADIUS * projectedRetVal[0], EQUATOR_RADIUS * projectedRetVal[1]};
        };

        std::array<double, 2> result = fwd(source->latitude * DEG_TO_RAD, source->longtitude * DEG_TO_RAD);

        target->e = result[0];
        target->n = result[1];
        target->u = source->altitude - reference->altitude;
    }



    int main(int argc, char **argv) {

        ros::init(argc, argv, "al_rtk");

        ros::NodeHandle node_handle("~");

        signal(SIGINT, sigint_handler);

        std::string device;
        int baudrate;
        std::string account;
        std::string password;
        std::string domain;
        int stderrthreshold;

        node_handle.param<std::string>("map_frame", map_frame, std::string("map"));

        node_handle.param<std::string>("serial_device", device, "");

        node_handle.param<int>("baudrate", baudrate, 115200);

        node_handle.param<std::string>("domain", domain, "rtk.ntrip.qxwz.com");

        node_handle.param<std::string>("account", account, "");

        node_handle.param<std::string>("password", password, "");

        if (account.empty()||password.empty())
        {
            ROS_ERROR("al_rtk_ros_driver: Please setup ntrip service  account and password in launch file ");
            ROS_ERROR("al_rtk_ros_driver: 请设置千寻服务的账号密码， 位于launch 文件中 al_rtk_ros_driver 节点");
        }

        node_handle.param<std::string>("origin_config_file", origin_config_file, "origin_config.json");

        node_handle.param<bool>("publish_pos_fix_only", publish_pos_fix_only, true);

        node_handle.param<bool>("auto_use_first_fix_as_enu_origin", auto_use_first_fix_as_enu_origin, true);

        node_handle.param<std::string>("log_dir", log_dir, "");

        node_handle.param<int>("stderrthreshold", stderrthreshold, 2);

        if (log_dir.empty())
        {
            ROS_ERROR("al_rtk_ros_driver:请设置日志文件输出目录,参数名称 log_dir");
        }


        is_origin_set = false;


        std::ifstream ifs_config(origin_config_file);

        if (ifs_config.good()) {

            ifs_config >> node_config;

            wgs84_pos config_origin;

            config_origin.latitude = std::stod(node_config["enu_origin"]["latitude"].as_string());
            config_origin.longtitude = std::stod(node_config["enu_origin"]["longtitude"].as_string());
            config_origin.altitude = std::stod(node_config["enu_origin"]["altitude"].as_string());

            ROS_INFO("load enu origin from configure file.");

            set_enu_origin(&config_origin, false);


        }


        gpgga_pub = node_handle.advertise<nmea_msgs::Gpgga>("gpgga", 10);

        local_pos_pub = node_handle.advertise<geometry_msgs::PointStamped>("location_pos", 10);

        rtk = new al_rtk::RTKDriver();

        rtk->register_gpgga_callback(on_receive_gpgga_str);

        rtk->start_rtk(device, baudrate, domain, account, password,log_dir,stderrthreshold);

        return 0;

    }

    void sigint_handler(int sig) {

        ROS_INFO("shuttding down\r\n");

        rtk->exit_rtk();

        ros::shutdown();

    }
