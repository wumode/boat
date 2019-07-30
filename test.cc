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
    char ser_ptr[50];
    char s[50];
    sprintf(s, "%p", ser_ptr);
    LOG(INFO)<<"ser_ptr: "<<s;
    return 0;
}
