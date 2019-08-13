cmake_minimum_required(VERSION 3.5)
project(boat)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#add_definitions(-Wall)

#add_definitions(-D DEBUG)

include_directories(geo_to_utm/datum)
include_directories(geo_to_utm/utm)
include_directories(kalman_filter)
include_directories(navigation)
include_directories(3rdparty/tiny_xml)
#include_directories(3rdparty/Eigen)
include_directories(3rdparty/nlohmann)
include_directories(3rdparty)
include_directories(serial_communication)
include_directories(socket_communication)


add_subdirectory(navigation)
add_subdirectory(geo_to_utm)
add_subdirectory(serial_communication)
add_subdirectory(socket_communication)

add_executable(Ship
        main.cc
        kalman_filter/KalmanFilter.h
        3rdparty/tiny_xml/tinystr.cpp
        3rdparty/tiny_xml/tinystr.h
        3rdparty/tiny_xml/tinyxml.cpp
        3rdparty/tiny_xml/tinyxml.h
        3rdparty/tiny_xml/tinyxmlerror.cpp
        3rdparty/tiny_xml/tinyxmlparser.cpp
        boat.cc boat.h
        boat_config.h
        )

target_link_libraries(Ship serial pthread rt crypt glog
        datum
        utm
        Navigation
        SerialCommunication
        SocketCommunication)

