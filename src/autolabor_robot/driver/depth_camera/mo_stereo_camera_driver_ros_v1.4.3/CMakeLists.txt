cmake_minimum_required(VERSION 3.0.2)
project(moak_camera)

set(MO_TARGET_NAME usb_asic_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moak_camera
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  lib/mo_stereo_camera_driver/include/mo_stereo_camera
)

link_directories(
    ${PCL_LIBRARY_DIRS} 
    ${OpenCV_LIBRARY_DIRS}
    lib/mo_stereo_camera_driver/lib
)

add_executable(${MO_TARGET_NAME} 
    src/usb_asic_cam.cpp 
)

target_link_libraries(${MO_TARGET_NAME} 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS}
    libmoStereoCameraDriver.so
)

