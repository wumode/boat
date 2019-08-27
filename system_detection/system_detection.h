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
#include <pwd.h>

#define VMRSS_LINE 17
#define VMSIZE_LINE 13
#define PROCESS_ITEM 14

namespace system_detection{
    typedef struct {
        unsigned long user;
        unsigned long nice;
        unsigned long system;
        unsigned long idle;
    }Total_Cpu_Occupy_t;

    typedef struct {
        unsigned int pid;
        unsigned long utime;  //user time
        unsigned long stime;  //kernel time
        unsigned long cutime; //all user time
        unsigned long cstime; //all dead time
    }Proc_Cpu_Occupy_t;

    class ProcessInformation{
    public:
        ProcessInformation();
        explicit ProcessInformation(int pid);
        explicit ProcessInformation(const char* process_name);
        ProcessInformation(const char* process_name, const char* user);
        int Pid();
        float Cpu();
        uint32_t Mem();
        uint32_t VirtualMem();

    private:
        uint16_t pid_;
        float pcpu_;     //CPU占用率
        uint32_t procmem_;   //进程占用内存
        uint32_t virtualmem_;    //进程占用虚拟内存
        const char* get_items(const char*buffer ,unsigned int item);
        unsigned long get_cpu_total_occupy();
        unsigned long get_cpu_proc_occupy(unsigned int pid);
        float get_proc_cpu(unsigned int pid);
        unsigned int get_proc_mem(unsigned int pid);
        unsigned int get_proc_virtualmem(unsigned int pid);
        int get_pid(const char* process_name, const char* user = nullptr);
        std::string User();
    };
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
