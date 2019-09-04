cmake_minimum_required(VERSION 3.5)
project(boat)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#add_definitions(-Wall)

configure_file (
        "${PROJECT_SOURCE_DIR}/config.h.in"
        "${PROJECT_SOURCE_DIR}/config.h"
)
option(USE_DEBUG
        "Debug" ON)

option (USE_GLOG
        "Use google glog" ON)


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
include_directories(system_detection)
include_directories(algorithm)
include_directories(3rdparty/socket)
include_directories(3rdparty/transform)
include_directories(timer)
include_directories(ctimer)

add_subdirectory(navigation)
add_subdirectory(geo_to_utm)
add_subdirectory(serial_communication)
add_subdirectory(socket_communication)
add_subdirectory(system_detection)
add_subdirectory(algorithm)
add_subdirectory(3rdparty/transform)
add_subdirectory(timer)
add_subdirectory(ctimer)

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

target_link_libraries(Ship rt crypt
        Navigation
        SerialCommunication
        SocketCommunication
        Algorithm)

if (USE_GLOG)
    target_link_libraries(Ship glog)
endif (USE_GLOG)

add_executable(Test
        test.cc
        )

target_link_libraries(Test
        SerialCommunication
        Navigation
        SystemDetection
        Algorithm
        Transform
        Timer
        CTimer)

if (USE_GLOG)
    target_link_libraries(Test glog)
endif ()

add_executable(server c_socket/server.cc)
add_executable(client c_socket/client.cc)