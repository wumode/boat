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

#ifndef SHIP_NAVIGATION_H
#define SHIP_NAVIGATION_H

#include <chrono>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <bitset>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <serial/serial.h>
#include <tinyxml.h>
#include <KalmanFilter.h>
#include "navigation_config.h"

namespace navigation {
    /**
     * @class Navigation: 自主航行类
     * @param navigation_config_path: 配置文件路径
     */
    class Navigation {
    public:
        explicit Navigation(std::string config_path);
        int SetMarkPointFlag(const std::string& path);
        int ReSetMarkPointFlag();

    protected:
        void UpdateMeasurementVector(MeasurementVector& measurementVector);
        void NavigationVelocityAnalyze_(const float &navigation_yaw, VelocityData &v_data);
        void Filter();
        void NavigationCalculation();
        void MarkGpsPosition();
        bool Init_(const std::string &config_path);
        void UpdateLocusPoints_(std::vector<GpsPosition>& l_p, uint32_t num);
        void GetLocusPoints_(std::vector<GpsPosition>& l_p);

        float initial_yaw_;     //初始偏航角
        StateVector now_state_; //姿态
        float yaw;              //偏航角
        MarkPointParameter mark_point_parameter_;
        GpsPosition key_position_gps_;  //目标点gps坐标
        GpsPosition now_location_gps_;  //位置gps坐标
        GridZone utm_zone_;             //UTM分区
        Hemisphere hemisphere_;         //半球

    private:
        bool KalmanFilterInitialization_();
        bool VariableLengthCheck_();
        bool LoadConfig_(const std::string &config_xml_path);

        KalmanFilter kf_;   //卡尔曼滤波器
        UtmPosition key_position_utm_;  //目标点UTM坐标
        std::vector<GpsPosition> locus_points_; //目标点向量
        NavigationParameter navigation_parameter_;  //自主航行参数
        uint32_t key_position_gps_num_;  //目标点计数
        Flag mark_point_flag_;
        Flag stop_motor_;
        std::ofstream* point_file_ptr_ = nullptr;

        std::chrono::steady_clock::time_point now_timestamp_;   //时刻，卡尔曼滤波器更新用
        std::chrono::steady_clock::time_point last_timestamp_;
        MeasurementVector measurement_vector_;  //观测值
    };
}

#endif //SHIP_NAVIGATION_H
