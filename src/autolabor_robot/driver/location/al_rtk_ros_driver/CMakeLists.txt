cmake_minimum_required(VERSION 3.16)
project(al_rtk_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_subdirectory(al_rtk_raw_driver)
add_subdirectory(configor)
add_subdirectory(nmea_msgs)
#add_library(nmea_msgs STATIC nmea_msgs)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nmea_msgs
        roslib
        message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES al_rtk_ros_driver
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  al_rtk_raw_driver/include
  configor/include

)

add_executable(${PROJECT_NAME} src/al_rtk_driver_node.cpp)
add_dependencies(${PROJECT_NAME} nmea_msgs_gencpp)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} al_rtk_raw_driver
 )
