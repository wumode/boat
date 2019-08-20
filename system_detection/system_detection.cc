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

#include "system_detection.h"

namespace system_detection{
    CoreTemperature::CoreTemperature() {
        core_temperature_ = 0.0;
        temperature_file_path_ = "/sys/class/thermal/thermal_zone0/temp";
        infile.open(temperature_file_path_, std::ios::in);
        if(!infile.is_open()){
            std::cout<<"Temperature file open error"<<std::endl;
        }
        file_data_ = new char[8];
        memset(file_data_, 0, 8);
    }

    CoreTemperature::CoreTemperature(const system_detection::CoreTemperature &C) {
        core_temperature_ = C.core_temperature_;
        temperature_file_path_ = C.temperature_file_path_;
        infile.open(temperature_file_path_, std::ios::in);
        if(!infile.is_open()){
            std::cout<<"Temperature file open error"<<std::endl;
        }
        std::cout<<"copy"<<std::endl;
        file_data_ = new char[8];
        memset(file_data_, 0, 8);
    }

    CoreTemperature::CoreTemperature(std::string& path) {
        temperature_file_path_ = path;
        core_temperature_ = 0.0;
        infile.open(temperature_file_path_, std::ios::in);
        if(!infile.is_open()){
            std::cout<<"Temperature file open error"<<std::endl;
        }
        file_data_ = new char[8];
        memset(file_data_, 0, 8);
    }

    CoreTemperature::CoreTemperature(char *path) {
        temperature_file_path_ = path;
        core_temperature_ = 0.0;
        infile.open(temperature_file_path_, std::ios::in);
        if(!infile.is_open()){
            std::cout<<"Temperature file open error"<<std::endl;
        }
        file_data_ = new char[8];
        memset(file_data_, 0, 8);
    }

    void CoreTemperature::SetTemperature(std::string &p) {
        temperature_file_path_ = p;
    }

    double CoreTemperature::Temperature() {
        infile.seekg(0);
        infile>>file_data_;
        try {
            core_temperature_ = std::stoi(file_data_)/1000.0;
        }
        catch (std::invalid_argument& e){
            std::cerr<<"Read temperature error"<<std::endl;
        }
        return core_temperature_;
    }

    CoreTemperature::~CoreTemperature() {
        infile.close();
        delete file_data_;
    }

    SystemDetection::SystemDetection() {
        std::string p = "/sys/class/thermal/thermal_zone0/temp";
        core_temperature_ = new CoreTemperature(p);
    }

    SystemDetection::SystemDetection(std::string &temperature_path) {
        core_temperature_ = new CoreTemperature(temperature_path);
    }

    double SystemDetection::Temperature() {
        return core_temperature_->Temperature();
    }

    SystemDetection::~SystemDetection() {
        delete core_temperature_;
    }
}