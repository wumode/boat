/******************************************************************************
 *
 * Copyright 2019 wumo1999@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *****************************************************************************/

//
// Created by wumo on 19-4-10.
//

#include <iostream>
#include "boat.h"

#ifdef USE_GLOG
    #include <glog/logging.h>
#endif


using namespace std;

int main(int argc, char* argv[])
{
    //bool status;
    string config_path, com;
    //unsigned int baud;
    config_path = "config.xml";
    com = "/dev/ttyUSB0";
    //baud = 460800;
#ifdef USE_GLOG
    google::InitGoogleLogging((const char *)argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "./log/log");
    LOG(INFO)<<"run";
#endif
    if(argc==2){
        config_path = (std::string)argv[1];
    } else if(argc == 3){
        config_path = (std::string)argv[1];
        com = argv[2];
    } else if(argc >= 4){
        config_path = (std::string)argv[1];
        com = (std::string)argv[2];
        //baud = (unsigned int)std::stoi(argv[3]);
    }
    navigation::boat boat(config_path);
    //status = boat.StartSerialThread(com, bound);
    FLAGS_logtostderr = false;
    boat.Control();
#ifdef USE_GLOG
    google::ShutdownGoogleLogging();
#endif
    //boat.KillSerialThread();
    return 0;
}