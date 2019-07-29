//
// Created by wumode on 19-4-11.
//

/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <chrono>
#include <tinyxml.h>
#include <json.hpp>
#include "navigation/navigation_config.h"
#include <map>
#include <serial_communication.h>

using namespace std;

int main() {
    GpsPosition gps;
    gps.latitude = 37.5354;
    gps.longitude = 122.076;
    UtmPosition utm;
    GpsToUtm(&gps, &utm);
    std::cout<<"x: "<<utm.x<<std::endl;
    std::cout<<"y: "<<utm.y<<std::endl;
    std::cout<<"z: "<<utm.gridZone<<std::endl;
    return 0;
}
