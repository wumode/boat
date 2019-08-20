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
// Created by wumode on 19-8-20.
//

#ifndef BOAT_SYSTEM_DETECTION_H
#define BOAT_SYSTEM_DETECTION_H

#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>

namespace system_detection{

    class CoreTemperature{
    public:
        CoreTemperature();
        CoreTemperature(const CoreTemperature& C);
        explicit CoreTemperature(char* path);
        explicit CoreTemperature(std::string& path);
        ~CoreTemperature();
        double Temperature();
        void SetTemperature(std::string& p);

    private:
        double core_temperature_;
        std::string temperature_file_path_;
        char* file_data_ = nullptr;
        std::ifstream infile;
    };

    class SystemDetection {
    public:
        SystemDetection();
        explicit SystemDetection(std::string& temperature_path);
        double Temperature();
        ~SystemDetection();

    private:
        CoreTemperature* core_temperature_;
    };
}




#endif //BOAT_SYSTEM_DETECTION_H
