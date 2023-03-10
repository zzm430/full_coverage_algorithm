cmake_minimum_required(VERSION 3.16)
project(al_rtk_raw_driver)

set(CMAKE_CXX_STANDARD 11)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(glog)

find_library(LIBEVENT_LIBRARY event)

include_directories(include)

#add_executable(al_rtk_raw_driver src/example.cpp src/al_rtk_raw_driver.cpp src/base64.cpp)

ADD_LIBRARY(al_rtk_raw_driver STATIC
        src/al_rtk_raw_driver.cpp
        src/base64.cpp
        )

#include_directories(/usr/local/include)

target_link_libraries(al_rtk_raw_driver ${LIBEVENT_LIBRARY} glog::glog)