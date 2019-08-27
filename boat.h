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
// Created by wumo on 2019/5/21.
//

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <navigation.h>
#include <json.hpp>
#include <serial_communication.h>
#include <socket_communication.h>
#include <algorithm.h>
#include "boat_config.h"
#ifndef SHIP_BOAT_H
#define SHIP_BOAT_H


namespace navigation {
    namespace mode{
        class DynamicPositioning{
        public:
            DynamicPositioning();
            //DynamicPositioning(navigation::pose::Pose& pose);
            DynamicPositioning(navigation::pose::Pose& pose, double kp, double ki, double kd, double az_upper_limit_, double az_lower_limit_);
        private:
            navigation::pose::Pose pose_;
            algorithm::PidController yaw_pid_controller_;
            double az_upper_limit_;
            double az_lower_limit_;
        };
    }
    /**
     * @class Boat: 船类
     * @param navigation_config_path: 配置文件路径
     */
    class boat: public navigation::Navigation{
    public:
        explicit boat(std::string navigation_config_path);
        static void ImuMsgsCallback(uint8_t* buffer_ptr_, void* __this);
        static void GpsMsgsCallback(uint8_t* buffer_ptr_, void* __this);
        static void RemoteControlSignalCallback(uint8_t* buffer_ptr_, void* __this);
        static void HardWareInitializationCallBack(uint8_t* buffer_ptr_, void* __this);
        static void LockingCallback(uint8_t* buffer_ptr_, void* __this);
        static void SocketReceiveCallBack(uint8_t* buffer_ptr_, void* __this);

        void Control();

    private:
        void HardWareInitialization_(const std::string& com, unsigned int bound);
        void AnalysisRemoteInfo(const RemoteChannelTrans& channel, volatile bool* stop);
        void RemoteVelocityAnalyze_(const RemoteChannelTrans& channel, VelocityData* v);
        bool LoadBoatConfig_(const std::string &config_xml_path, BoatParams& boatParams);
        void VelocityPublish_(VelocityData& velocity_data);
        void ControlPowerPublish_(ControlPowerTrans& control_power_trans);
        void StopPublish_(StopTrans& stopTrans);
        void EmpowerPublish_(EmpowerTrans& empowerTrans);
        void SocketShowPublish_();
        //volatile bool serial_thread_;   //串口线程运行的标志
        volatile bool hardware_initialized_;
        volatile uint8_t route_updated_;
        //volatile bool socket_thread_;   //socket线程运行标志
        serial_communication::SerialCommunication* ser_com_ptr_;
        socket_communication::SocketCommunication* socket_com_ptr_;
        //serial::Serial* ser_ptr_ = nullptr; //指向串口的指针
        //int* client_socket_ptr_ = nullptr;
        pthread_mutex_t* serial_measurement_mutex_ptr_ = nullptr;   //线程锁指针，保护变量 boat_measurement_vector_
        pthread_mutex_t* serial_channel_mutex_ptr_ = nullptr;   //线程锁指针，保护变量 remote_channel_info_serial_thread_
        pthread_mutex_t* route_updated_mutex_ptr_ = nullptr;
        //SerialDataInfo* serial_data_info_ptr_;
        ImuData imu_data_;
        GpsData gps_data_;
        RemoteChannelTrans remote_channel_data_;
        std::vector<GpsPosition> locus_points_main_thread_; //目标点向量
        VelocityData velocity_data_;
        BoatMode boat_mode_;    //运行模式，自主航行/遥控
        MeasurementVector boat_measurement_vector_;     //观测值向量
        RemoteChannelTrans remote_channel_data_main_thread_;
        EmpowerTrans empower_trans_;
        LockingTrans locking_trans_;
        StopTrans stop_trans_;
        ControlPowerTrans control_power_trans_;
        BoatParams boat_params_;
        bool main_thread_;
        volatile bool stop;  //停止

        ///debug
//        std::chrono::steady_clock::time_point now_call_timestamp_;
//        std::chrono::steady_clock::time_point last_call_timestamp_;
    };
}

#endif //SHIP_BOAT_H
