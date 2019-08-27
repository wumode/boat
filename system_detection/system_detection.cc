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
    ProcessInformation::ProcessInformation() {
        pid_ = getpid();
        pcpu_ = get_proc_cpu(pid_);
        procmem_ = get_proc_mem(pid_);
        virtualmem_ = get_proc_virtualmem(pid_);
    }

    ProcessInformation::ProcessInformation(int pid){
        pid_ = pid;
        pcpu_ = get_proc_cpu(pid_);
        procmem_ = get_proc_mem(pid_);
        virtualmem_ = get_proc_virtualmem(pid_);
    }

    ProcessInformation::ProcessInformation(const char *process_name) {
        pid_= get_pid(process_name);
        pcpu_ = get_proc_cpu(pid_);
        procmem_ = get_proc_mem(pid_);
        virtualmem_ = get_proc_virtualmem(pid_);
    }

    ProcessInformation::ProcessInformation(const char *process_name, const char *user) {
        pid_= get_pid(process_name, user);
        pcpu_ = get_proc_cpu(pid_);
        procmem_ = get_proc_mem(pid_);
        virtualmem_ = get_proc_virtualmem(pid_);
    }

    float ProcessInformation::Cpu() {
        pcpu_ = get_proc_cpu(pid_);
        return pcpu_;
    }

    uint32_t ProcessInformation::Mem() {
        procmem_ = get_proc_mem(pid_);
        return procmem_;
    }

    uint32_t ProcessInformation::VirtualMem() {
        virtualmem_ = get_proc_virtualmem(pid_);
        return virtualmem_;
    }

    int ProcessInformation::Pid() {
        return pid_;
    }

    const char* ProcessInformation::get_items(const char*buffer ,unsigned int item){
        const char *p =buffer;
        int len = strlen(buffer);
        int count = 0;
        for (int i=0; i<len;i++){
            if (' ' == *p){
                count ++;
                if(count == item -1){
                    p++;
                    break;
                }
            }
            p++;
        }
        return p;
    }


//获取总的CPU时间
    unsigned long ProcessInformation::get_cpu_total_occupy(){

        FILE *fd;
        char buff[1024]={0};
        Total_Cpu_Occupy_t t;

        fd =fopen("/proc/stat","r");
        if (nullptr == fd){
            return 0;
        }
        char* temp;
        temp = fgets(buff,sizeof(buff),fd);
        char name[64]={0};
        sscanf(buff,"%s %ld %ld %ld %ld",name,&t.user,&t.nice,&t.system,&t.idle);
        fclose(fd);

        return (t.user + t.nice + t.system + t.idle);
    }


//获取进程的CPU时间
    unsigned long ProcessInformation::get_cpu_proc_occupy(unsigned int pid){

        char file_name[64]={0};
        Proc_Cpu_Occupy_t t;
        FILE *fd;
        char line_buff[1024]={0};
        sprintf(file_name,"/proc/%d/stat",pid);

        fd = fopen(file_name,"r");
        if(nullptr == fd){
            return 0;
        }
        char* temp;
        temp = fgets(line_buff,sizeof(line_buff),fd);

        sscanf(line_buff,"%u",&t.pid);
        const char *q =get_items(line_buff,PROCESS_ITEM);
        sscanf(q,"%ld %ld %ld %ld",&t.utime,&t.stime,&t.cutime,&t.cstime);
        fclose(fd);

        return (t.utime + t.stime + t.cutime + t.cstime);
    }


//获取CPU占用率
    float ProcessInformation::get_proc_cpu(unsigned int pid){

        unsigned long totalcputime1,totalcputime2;
        unsigned long procputime1,procputime2;

        totalcputime1=get_cpu_total_occupy();
        procputime1=get_cpu_proc_occupy(pid);

        usleep(200000);

        totalcputime2=get_cpu_total_occupy();
        procputime2=get_cpu_proc_occupy(pid);

        float pcpu = 0.0;
        if(0 != totalcputime2-totalcputime1){
            pcpu=100.0 * (procputime2-procputime1)/(totalcputime2-totalcputime1);
        }

        return pcpu;
    }


    //获取进程占用内存
    unsigned int ProcessInformation::get_proc_mem(unsigned int pid){

        char file_name[64]={0};
        FILE *fd;
        char line_buff[512]={0};
        sprintf(file_name,"/proc/%d/status",pid);

        fd =fopen(file_name,"r");
        if(nullptr == fd){
            return 0;
        }

        char name[64];
        int vmrss;
        char* temp;
        for (int i=0; i<VMRSS_LINE-1;i++){
            temp = fgets(line_buff,sizeof(line_buff),fd);
        }

        temp = fgets(line_buff,sizeof(line_buff),fd);
        sscanf(line_buff,"%s %d",name,&vmrss);
        fclose(fd);

        return vmrss;
    }

    unsigned int ProcessInformation::get_proc_virtualmem(unsigned int pid){

        char file_name[64]={0};
        FILE *fd;
        char line_buff[512]={0};
        sprintf(file_name,"/proc/%d/status",pid);

        fd =fopen(file_name,"r");
        if(nullptr == fd){
            return 0;
        }

        char name[64];
        int vmsize;
        char* temp;
        for (int i=0; i<VMSIZE_LINE-1;i++){
            temp = fgets(line_buff,sizeof(line_buff),fd);
        }

        temp = fgets(line_buff,sizeof(line_buff),fd);
        sscanf(line_buff,"%s %d",name,&vmsize);
        fclose(fd);

        return vmsize;
    }

    int ProcessInformation::get_pid(const char* process_name, const char* user )
    {
        if(user == nullptr){
            user = User().c_str();
        }

        char cmd[512];
        if (user){
            sprintf(cmd, "pgrep %s -u %s", process_name, user);
        }

        FILE *pstr = popen(cmd,"r");

        if(pstr == nullptr){
            return 0;
        }

        char buff[512];
        ::memset(buff, 0, sizeof(buff));
        if(NULL == fgets(buff, 512, pstr)){
            return 0;
        }

        return atoi(buff);
    }

    std::string ProcessInformation::User()
    {
        uid_t user_id;
        struct passwd* pwd;
        user_id=getuid();
        pwd=getpwuid(user_id);
        return pwd->pw_name;
    }

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