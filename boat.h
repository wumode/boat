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
#include <algorithm>
#include <navigation.h>
#include <json.hpp>
#include <serial_communication.h>
#include <socket_communication.h>
#include <algorithm.h>
#include <ctimer.h>
#include "boat_config.h"
#ifndef SHIP_BOAT_H
#define SHIP_BOAT_H


namespace navigation {
    typedef struct AngularVelocityLimit{
        algorithm::Limit z_limit;
        algorithm::Limit y_limit;
        algorithm::Limit x_limit;
    }AngularVelocityLimit;

    namespace mode{
        class DynamicPositioning{
        public:
            DynamicPositioning();
            //DynamicPositioning(navigation::pose::Pose& pose);
            DynamicPositioning(navigation::pose::Pose& pose, double kp, double ki, double kd, double max_distance);
            DynamicPositioning(const DynamicPositioning& D);
            int Update(const navigation::pose::Pose& target, const navigation::pose::Pose& input, navigation::Velocity& v);
            int Update(const navigation::pose::Pose& input, navigation::Velocity& v);
            void Limit(const navigation::AngularVelocityLimit& limit);
            void MaxDistance(double d);
            void TargetLinearVelocity(LinearVelocity l);
        private:
            navigation::pose::Pose target_pose_;
            algorithm::PidController yaw_pid_controller_;
            navigation::AngularVelocityLimit limit_;
            LinearVelocity target_linear_velocity_;
            double max_distance_;
        };
    }
    /**
     * @class Boat: 船类
     * @param navigation_config_path: 配置文件路径
     */
    class boat: public navigation::Navigation{
    public:
        explicit boat(std::string& navigation_config_path);
        static void ImuMsgsCallback(uint8_t* buffer_ptr_, void* __this);
        static void GpsMsgsCallback(uint8_t* buffer_ptr_, void* __this);
        static void RemoteControlSignalCallback(uint8_t* buffer_ptr_, void* __this);
        static void HardWareInitializationCallBack(uint8_t* buffer_ptr_, void* __this);
        static void SocketHandShakeCallBack(uint8_t* buffer_ptr_, void* __this);
        static void LockingCallback(uint8_t* buffer_ptr_, void* __this);
        static void SocketReceiveCallBack(uint8_t* buffer_ptr_, void* __this);
        static void SocketHandShake2CallBack(uint8_t* buffer_ptr_, void* __this);
        static void SocketOfflineReconnectedCallBack(uint8_t* buffer_ptr_, void* __this);
        void Control();

    private:
        void SocketHandShake_();
        void SocketHandShake_(void* __this);
        void HardWareInitialization_(const std::string& com, unsigned int bound);
        void AnalysisRemoteInfo(const RemoteChannelTrans& channel, volatile bool* stop);
        void RemoteVelocityAnalyze_(const RemoteChannelTrans& channel, Velocity* v);
        bool LoadBoatConfig_(const std::string &config_xml_path, BoatParams& boatParams);
        void VelocityPublish_(Velocity& velocity_data);
        void ControlPowerPublish_(ControlPowerTrans& control_power_trans);
        void StopPublish_(StopTrans& stopTrans);
        void EmpowerPublish_(EmpowerTrans& empowerTrans);
        void ModePublish_(const BoatMode& mode);
        void BehaviorPublish_(uint8_t behavior);
        void SocketShowPublish_();

        volatile bool hardware_initialized_;
        volatile bool socket_handshake_ok_;
        volatile uint8_t route_updated_;
        serial_communication::SerialCommunication* ser_com_ptr_;
        socket_communication::SocketCommunication* socket_com_ptr_;
        pthread_mutex_t* serial_measurement_mutex_ptr_ = nullptr;   //线程锁指针，保护变量 boat_measurement_vector_
        pthread_mutex_t* serial_channel_mutex_ptr_ = nullptr;   //线程锁指针，保护变量 remote_channel_info_serial_thread_
        pthread_mutex_t* route_updated_mutex_ptr_ = nullptr;
        ImuData imu_data_;
        GpsData gps_data_;
        RemoteChannelTrans remote_channel_data_;
        std::vector<GpsPosition> locus_points_main_thread_;
        Velocity velocity_data_;
        BoatMode boat_mode_;
        MeasurementVector boat_measurement_vector_;
        RemoteChannelTrans remote_channel_data_main_thread_;
        EmpowerTrans empower_trans_;
        LockingTrans locking_trans_;
        StopTrans stop_trans_;
        ControlPowerTrans control_power_trans_;
        BoatParams boat_params_;
        bool main_thread_;
        volatile bool stop;  //停止
        uint8_t behavior_;

        ///debug
//        std::chrono::steady_clock::time_point now_call_timestamp_;
//        std::chrono::steady_clock::time_point last_call_timestamp_;
    };
}

#endif //SHIP_BOAT_H
