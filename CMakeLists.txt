cmake_minimum_required(VERSION 3.5)
project(boat)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#add_definitions(-Wall)

add_definitions(-D DEBUG)

include_directories(geo_to_utm/datum)
include_directories(geo_to_utm/utm)
include_directories(kalman_filter)
include_directories(navigation)
include_directories(3rdparty/tiny_xml)
#include_directories(3rdparty/Eigen)
include_directories(c_socket)
#include_directories(3rdparty/json)
# include_directories(data_transmission)
include_directories(3rdparty/nlohmann)
include_directories(3rdparty)
include_directories(serial_communication)
include_directories(socket_communication)

#link_directories(3rdparty/pthread/lib 3rdparty/pthread/dll/x64)

#aux_source_directory(. DIR_SRCS)
add_subdirectory(navigation)
add_subdirectory(geo_to_utm)
add_subdirectory(serial_communication)
add_subdirectory(socket_communication)
#add_subdirectory(3rdparty/nlohmann)

add_executable(Ship
        main.cc
#        geo_to_utm/datum/datum.cpp
#        geo_to_utm/utm/utm.cpp
        kalman_filter/KalmanFilter.h
#        navigation/navigation.cc
#        navigation/navigation.h
#        navigation/navigation_config.h
        3rdparty/tiny_xml/tinystr.cpp
        3rdparty/tiny_xml/tinystr.h
        3rdparty/tiny_xml/tinyxml.cpp
        3rdparty/tiny_xml/tinyxml.h
        3rdparty/tiny_xml/tinyxmlerror.cpp
        3rdparty/tiny_xml/tinyxmlparser.cpp
        boat.cc boat.h
        boat_config.h
#        serial_communication/serial_communication.cc serial_communication/serial_communication.h
#        socket_communication/socket_communication.cc socket_communication/socket_communication.h
        )
#add_executable(serial_test
#        test.cc
#        serial_communication/serial_communication.cc
#        serial_communication/serial_communication.h
#        navigation/navigation_config.h
#        geo_to_utm/datum/datum.cpp
#        geo_to_utm/datum/datum.h
#        geo_to_utm/utm/utm.cpp
#        geo_to_utm/utm/utm.h
#        3rdparty/tiny_xml/tinystr.cpp
#        3rdparty/tiny_xml/tinystr.h
#        3rdparty/tiny_xml/tinyxml.cpp
#        3rdparty/tiny_xml/tinyxml.h
#        3rdparty/tiny_xml/tinyxmlerror.cpp
#        3rdparty/tiny_xml/tinyxmlparser.cpp)
#add_executable(client c_socket/client.c)
#add_executable(serve c_socket/server.cc)
#add_executable(json_test json_test.cc socket_communication/socket_communication.cc socket_communication/socket_communication.h)
target_link_libraries(Ship serial pthread wiringPi rt crypt glog
        datum
        utm
        Navigation
        SerialCommunication
        SocketCommunication)
#target_link_libraries(serial_test pthread serial crypt glog)
#target_link_libraries(json_test pthread crypt glog)
