cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-Wall)
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
#========================
# Dependencies Setup
#========================
#ROS#
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)
  add_definitions(-DROS_FOUND)
  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})
endif(roscpp_FOUND)

#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS2_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED QUIET)
  find_package(std_msgs REQUIRED QUIET)
  find_package(sensor_msgs REQUIRED QUIET)
  find_package(rslidar_msg REQUIRED QUIET)
  find_package(pcl_conversions REQUIRED QUIET)
  include_directories(${rclcpp_INCLUDE_DIRS})

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

#Protobuf#
find_package(Protobuf QUIET)
find_program(PROTOC protoc)
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)

  add_definitions(-DPROTO_FOUND)

  include_directories(${PROTOBUF_INCLUDE_DIRS})
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/src/msg/proto_msg)
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  foreach(proto_file ${PROTOBUF_FILELIST})

    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    --proto_path=${PROTO_FILE_PATH}
                    --cpp_out=${PROTO_FILE_PATH}
                    ${proto_file})
  endforeach()
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)

#Others#
find_package(yaml-cpp REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Catkin#
if(${COMPILE_METHOD} STREQUAL "CATKIN")
  add_definitions(-DRUN_IN_ROS_WORKSPACE)
  find_package(catkin REQUIRED QUIET
    COMPONENTS
      roscpp
      sensor_msgs
      roslib
      pcl_ros
      pcl_conversions
  )

  catkin_package(
    CATKIN_DEPENDS
      roscpp
      sensor_msgs
      roslib
      pcl_conversions
  )
endif(${COMPILE_METHOD} STREQUAL "CATKIN")

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED QUIET)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
add_definitions(-DPOINT_TYPE_XYZIRT)
endif()

#========================
# Build Setup
#========================
#Protobuf#
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              src/manager/adapter_manager.cpp
              ${PROTO_FILE_PATH}/packet.pb.cc
              ${PROTO_FILE_PATH}/scan.pb.cc
              ${PROTO_FILE_PATH}/point_cloud.pb.cc
              )
  target_link_libraries(rslidar_sdk_node
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${PROTOBUF_LIBRARY}
                      ${rs_driver_LIBRARIES}
                      )
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              src/manager/adapter_manager.cpp
              )
  target_link_libraries(rslidar_sdk_node
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES}
                      )
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)

#Ros#
if(roscpp_FOUND)
  target_link_libraries(rslidar_sdk_node ${ROS_LIBS})
  if(${COMPILE_METHOD} STREQUAL "CATKIN")
    install(TARGETS rslidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)
    install(DIRECTORY config/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
            PATTERN ".svn" EXCLUDE)
    install(DIRECTORY rviz/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
            PATTERN ".svn" EXCLUDE)
  endif()
endif(roscpp_FOUND)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  ament_target_dependencies(rslidar_sdk_node rclcpp sensor_msgs std_msgs rslidar_msg)
  install(TARGETS
    rslidar_sdk_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
