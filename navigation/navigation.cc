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

#include "navigation.h"


namespace navigation
{
    /**
     * @brief Navigation
     * @param config_path
     */
     ///构造函数，调用初始化成员函数Init_
    Navigation::Navigation(std::string config_path) {
//        if (Init_(config_path)){
//            std::cout<<"Initialized success!"<<std::endl;
//        }
//        else{
//            std::cerr<<"Initialized failed!"<<std::endl;
//        }
        //initialized_ = true;
    }

    /**
     * @brief Init_
     * @param config_path
     * @return
     */
     ///初始化成员变量值
    bool Navigation::Init_(const std::string &config_path) {
        //boat_mode_ = NAVIGATION_FLAG;
        mark_point_flag_ = 0;
        mark_point_parameter_.mark_flag = 0;
        stop_motor_ = 0;
        if(!LoadConfig_(config_path)){
            exit(-1);
        }
        key_position_gps_ = locus_points_[0];
        //now_location_gps_ = locus_points_[0];
        key_position_gps_num_ = 0;

        //19-4-9
        GpsToUtm(&now_location_gps_, &measurement_vector_.position);
        GpsToUtm(&key_position_gps_, &key_position_utm_);
        now_state_.position = measurement_vector_.position;

        initial_yaw_ = CalcAngleUtm(&key_position_utm_,&now_state_.position);
        now_state_.attitude_angle = initial_yaw_;
        now_state_.angular_velocity.z = 0;
        now_state_.line_velocity.x = 0.0;
        now_state_.line_velocity.y = 0.0;

        measurement_vector_.imu_data.angle.yaw = initial_yaw_;
        measurement_vector_.imu_data.linear_acceleration.x = 0.0;
        measurement_vector_.imu_data.linear_acceleration.y = 0.0;
        measurement_vector_.imu_data.angular_velocity.z = 0.0;

        GpsToUtm(&now_location_gps_, &now_state_.position);
        GpsToUtm(&key_position_gps_, &key_position_utm_);
        now_timestamp_ = std::chrono::steady_clock::now();

        last_timestamp_ = now_timestamp_;
        return KalmanFilterInitialization_() && VariableLengthCheck_();
    }

    /**
     * @brief LoadConfig_
     * @param config_xml_path
     * @return
     */
     ///读取配置文件
    bool Navigation::LoadConfig_(const std::string &config_xml_path) {
        TiXmlDocument config_xml;
        if (!config_xml.LoadFile(config_xml_path.c_str()))
        {
            std::cerr << "Can load file: " << config_xml_path << std::endl;
            config_xml.Clear();
            return false;
        }
        TiXmlElement* xmlRoot = config_xml.RootElement();
        if (xmlRoot == nullptr)
        {
            std::cerr << "Failed to load file: No root element." << std::endl;
            return false;
        }

        TiXmlElement* gps_xml = xmlRoot->FirstChildElement("gps");
        TiXmlElement* point_xml = gps_xml->FirstChildElement("point");
        while (point_xml){
            GpsPosition gps_position_xml;
            gps_position_xml.longitude = (float)std::strtod(point_xml->FirstChildElement("longitude")->GetText(), nullptr);
            gps_position_xml.latitude = (float)std::strtod(point_xml->FirstChildElement("latitude")->GetText(), nullptr);
            locus_points_.push_back(gps_position_xml);
            point_xml = point_xml->NextSiblingElement();
        }
        TiXmlElement* mark_xml = xmlRoot->FirstChildElement("mark");
        mark_point_parameter_.mark_flag = std::stoi(mark_xml->FirstChildElement("flag")->GetText());
        //std::cout<<"mark_flag: "<<mark_flag<<std::endl;
        if(mark_point_parameter_.mark_flag){
            //TiXmlElement* mark_path_flag_xml = mark_xml->FirstChildElement("path");
            SetMarkPointFlag((std::string)mark_xml->FirstChildElement("path")->GetText());
            mark_point_parameter_.period = (float)std::strtod(mark_xml->FirstChildElement("period")->GetText(), nullptr);
        }
        TiXmlElement* parameter_xml = xmlRoot->FirstChildElement("parameter");
        navigation_parameter_.steering_coefficient =
                (float)std::strtod(parameter_xml->FirstChildElement("steering_coefficient")->GetText(), nullptr);

        navigation_parameter_.min_distance = (float)std::strtod(parameter_xml->FirstChildElement("min_distance")->GetText(),
                                                               nullptr);
        navigation_parameter_.base_velocity = (float)std::strtod(parameter_xml->FirstChildElement("base_velocity")->GetText(),
                                                               nullptr);
        navigation_parameter_.steering_deceleration_coefficient =
                (float)std::strtod(parameter_xml->FirstChildElement("steering_deceleration_coefficient")->GetText(),
                                                                nullptr);
        navigation_parameter_.corner_threshold =
                (float)std::strtod(parameter_xml->FirstChildElement("corner_threshold")->GetText(), nullptr);
        config_xml.Clear();
        return true;
    }

    /**
     * @brief VariableLengthCheck_
     * @return
     */
     ///变量类型长度检查，避免遇到奇葩的编译器，出现奇怪的问题
    bool Navigation::VariableLengthCheck_() {
//        bool res = (sizeof(SerialDataInfo)==SERIAL_DATA_LENGTH)&&(sizeof(int)==4)&&sizeof(float)==4;
//        if(!res){
//            std::cerr<<"Variable length check failed!"<<std::endl;
//        }
        return true;
    }

    /**
     * @brief UpdateMeasurementVector
     * @param measurementVector
     */
     ///传入观测值
    void Navigation::UpdateMeasurementVector(MeasurementVector& measurementVector) {
        measurement_vector_.position = measurementVector.position;
        measurement_vector_.imu_data.angle = measurementVector.imu_data.angle;
        measurement_vector_.imu_data.angular_velocity = measurementVector.imu_data.angular_velocity;
        measurement_vector_.imu_data.linear_acceleration = measurementVector.imu_data.linear_acceleration;
    }

    /**
     * @brief SetMarkPointFlag
     * @param path
     * @return
     */
     ///置位标记gps点标志
    int Navigation::SetMarkPointFlag(const std::string& path){
        point_file_ptr_ = new std::ofstream;
        if(point_file_ptr_== nullptr){
            return -1;
        }
        point_file_ptr_->open(path, std::ios::out | std::ios::trunc);
        mark_point_flag_ = 1;
        return 0;
    }

    /**
     * @brief ReSetMarkPointFlag
     * @return
     */
     ///结束记录gps点
    int Navigation::ReSetMarkPointFlag(){
        if(mark_point_flag_){
            mark_point_flag_ = 0;
            point_file_ptr_->close();
            free(point_file_ptr_);
            point_file_ptr_ = nullptr;
        }
        //std::cout<<"ReSetMarkPointFlag"<<std::endl;
        return 0;
    }


//19-4-9
    /**
     * @brief KalmanFilterInitialization_
     * @return true or false
     */
    ///卡尔曼滤波器初始化
    bool Navigation::KalmanFilterInitialization_(){
        //last_timestamp_ = now_timestamp_;
        Eigen::VectorXd x_in(6, 1);
        x_in << now_state_.position.x, now_state_.position.y, now_state_.attitude_angle,0.0, 0.0, 0.0;
        kf_.Initialization(x_in);
        //std::cout<<" x_in: "<<x_in<<" ";
        Eigen::MatrixXd P_in(6, 6);
        P_in << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 1.0;
        kf_.SetP(P_in);

        Eigen::MatrixXd Q_in(6,6);
        Q_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q_in);

        Eigen::MatrixXd H_in(3, 6);
        H_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        kf_.SetH(H_in);

        Eigen::MatrixXd R_in(3, 3);
        R_in << 0.0225, 0.0, 	0.0,
                0.0, 	0.0225, 0.0,
                0.0, 	0.0,    0.0225;
        kf_.SetR(R_in);
        return true;
    }

    /**
     * @brief NavigationVelocityAnalyze_
     * @param navigation_yaw : 偏航角
     * @param v_data
     */
     ///计算自主航行速度
    void Navigation::NavigationVelocityAnalyze_(const float &navigation_yaw, VelocityData &v_data) {
        float yaw_abs = fabsf(navigation_yaw);
        float s = navigation_yaw/yaw_abs;
        if(yaw_abs>navigation_parameter_.corner_threshold){
            v_data.velocity_angle = s*navigation_parameter_.steering_coefficient*navigation_parameter_.corner_threshold;
        } else{
            v_data.velocity_angle = navigation_parameter_.steering_coefficient*navigation_yaw;
        }
        v_data.velocity_x = navigation_parameter_.base_velocity - navigation_parameter_.steering_deceleration_coefficient
                *fabsf(v_data.velocity_angle);
        if(stop_motor_){
            std::cout<<"stop"<<std::endl;
            v_data.velocity_x = 0;
            v_data.velocity_angle = 0;
        }
    }

    /**
     * @brief Filter
     */
     ///卡尔曼滤波器调用，详见
     ///     https://blog.csdn.net/u012411498/article/details/82887417
    void Navigation::Filter(){
        now_timestamp_ = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(now_timestamp_
                -last_timestamp_);
        last_timestamp_ = now_timestamp_;
        double delta_t = time_used.count();
        //std::cout<<"delta: "<<delta_t<<std::endl;
        Eigen::MatrixXd F_in(6, 6);
        F_in << 1.0, 0.0, 0.0, delta_t, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, delta_t, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, delta_t,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        kf_.SetF(F_in);
        Eigen::VectorXd u_in(6);
        //double u_in_data[6];

        u_in(0) = measurement_vector_.imu_data.linear_acceleration.x*delta_t*delta_t/2;
        u_in(1) = measurement_vector_.imu_data.linear_acceleration.y*delta_t*delta_t/2;
        u_in(2) = 0.0;
        u_in(3) = measurement_vector_.imu_data.linear_acceleration.x*delta_t;
        u_in(4) = measurement_vector_.imu_data.linear_acceleration.y*delta_t;

        u_in(5) = 0.0;
        //u_in << 0.0, 0.0, 0.0, 0.0;
        kf_.SetU(u_in);
        kf_.Prediction();
        //std::cout<<"u_in: "<<u_in<<std::endl;
        Eigen::VectorXd z(3);

        z << measurement_vector_.position.x, measurement_vector_.position.y, measurement_vector_.imu_data.angular_velocity.z;

        kf_.MeasurementUpdate(z);
        Eigen::VectorXd x_out = kf_.GetX();

        now_state_.position.x = x_out(0);
        now_state_.position.y = x_out(1);
        now_state_.attitude_angle = x_out(2);
    }

    /**
     * @brief MarkGpsPosition
     */
     ///记录GPS点
    void Navigation::MarkGpsPosition() {
        GpsPosition gps_pos;
        UtmToGps(&measurement_vector_.position, &gps_pos);
        if(point_file_ptr_ != nullptr){
            *point_file_ptr_<<std::fixed << std::setprecision(5)<<gps_pos.latitude<<","<<gps_pos.longitude<<std::endl;
        }
    }

    /**
     * @brief NavigationCalculation
     */
     ///自主航行的所有计算在NavigationCalculation中调用
    void Navigation::NavigationCalculation(){
        float route_angle;
        std::cout<<" key: "<<key_position_gps_num_;
        float distance = CalcDistanceUtm(&key_position_utm_, &now_state_.position);
        std::cout<<" distance: "<<distance;
        if(distance<navigation_parameter_.min_distance){
            if(key_position_gps_num_ == locus_points_.size()-1)
            {
                stop_motor_ = 1;
            }
            else{
                key_position_gps_num_++;
                key_position_gps_ = locus_points_[key_position_gps_num_];
                GpsToUtm(&key_position_gps_, &key_position_utm_);
            }
        }
        route_angle = CalcAngleUtm(&key_position_utm_, &now_state_.position);
        yaw = CalcYaw(&route_angle, &now_state_.attitude_angle);
    }


    void Navigation::UpdateLocusPoints_(std::vector<GpsPosition> &l_p, uint32_t num) {
        locus_points_ = l_p;
        key_position_gps_num_ = num;
    }

    void Navigation::GetLocusPoints_(std::vector<GpsPosition> &l_p) {
        l_p = locus_points_;
    }
}
