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

#include "boat.h"

//using namespace std;


void to_json(json& j, const GpsPosition& gpsPosition){
    j = json{{"latitude", gpsPosition.latitude}, {"longitude", gpsPosition.longitude}};
}

void from_json(const json& j, GpsPosition& gpsPosition){
    gpsPosition.longitude = j.at("longitude").get<float>();
    gpsPosition.latitude = j.at("latitude").get<float>();
}

void to_json(json& j, const UtmPosition& utmPosition){
    j = json{{"x", utmPosition.x},
             {"y", utmPosition.y},
             {"GridZone", (int)utmPosition.gridZone},
             {"Hemisphere", (int)utmPosition.hemisphere}};
}

void from_json(const json& j, UtmPosition& utmPosition){
    utmPosition.x = j.at("x").get<float>();
    utmPosition.y = j.at("y").get<float>();
    utmPosition.gridZone = (GridZone)j.at("GridZone").get<int>();
    utmPosition.hemisphere = (Hemisphere)j.at("GridZone").get<int>();
}

void to_json(json& j, const LinearAcceleration& linearAcceleration){
    j = json{{"x", linearAcceleration.x},
             {"y", linearAcceleration.y}};
}

void from_json(const json& j, LinearAcceleration& linearAcceleration){
    linearAcceleration.x = j.at("x").get<float>();
    linearAcceleration.y = j.at("y").get<float>();
}

void to_json(json& j, const AngularVelocity& angular_velocity){
    j = json{{"z", angular_velocity.z}};
}

void from_json(const json& j, AngularVelocity& angular_velocity){
    angular_velocity.z =  j.at("z").get<float>();
}

void to_json(json& j, const Angle& angle){
    j = json{{"roll", angle.roll}, {"pitch", angle.pitch}, {"yaw", angle.yaw}};
}

void from_json(const json& j, Angle& angle){
    angle.yaw = j.at("yaw").get<float>();
    angle.pitch = j.at("pitch").get<float>();
    angle.roll = j.at("roll").get<float>();
}

void to_json(json& j, const ImuData& imuData){
    j = json{{"angle", imuData.angle}, {"angular_velocity", imuData.angular_velocity}, {"linear_acceleration", imuData.linear_acceleration}};
}

void from_json(const json& j, ImuData& imuData){
    imuData.angle = j.at("angle").get<Angle>();
    imuData.angular_velocity = j.at("angular_velocity").get<AngularVelocity>();
    imuData.linear_acceleration = j.at("linear_acceleration").get<LinearAcceleration>();
}

void to_json(json& j, const SocketShow& socketShow){
    j = json{{"gps_position", socketShow.gps_position},
             {"raw_gps_position", socketShow.raw_gps_position},
             {"utm_position", socketShow.utm_position},
             {"route_gps_positions", socketShow.route_gps_positions},
             {"locking", socketShow.locking},
             {"imu_data", socketShow.imu_data}};
}

void from_json(const json& j, SocketShow& socketShow){
    socketShow.imu_data = j.at("imu_data").get<ImuData>();
    socketShow.gps_position = j.at("gps_position").get<GpsPosition>();
    socketShow.raw_gps_position = j.at("raw_gps_position").get<GpsPosition>();
    socketShow.utm_position = j.at("utm_position").get<UtmPosition>();
    socketShow.route_gps_positions = j.at("route_gps_positions").get<std::vector<GpsPosition>>();
    socketShow.locking = j.at("locking").get<uint8_t >();
}

void to_json(json& j, const SocketReceive& socketReceive){
    j = json{{"mode", socketReceive.mode},
             {"empower", socketReceive.empower},
             {"route_gps_positions", socketReceive.route_gps_positions},
             {"route_updated", socketReceive.route_updated},
             {"stop", socketReceive.stop}};
}

void from_json(const json& j, SocketReceive& socketReceive){
    socketReceive.route_gps_positions = j.at("route_gps_positions").get<std::vector<GpsPosition>>();
    socketReceive.empower = j.at("empower").get<uint8_t >();
    socketReceive.mode = j.at("mode").get<uint8_t >();
    socketReceive.route_updated = j.at("route_updated").get<uint8_t >();
    socketReceive.stop = j.at("stop").get<uint8_t >();
}


namespace navigation{
    /**
     * @param navigation_config_path
     */
     ///Boat的构造函数，将配置文件路径传入父类，初始化传感器值
    boat::boat(std::string navigation_config_path) :navigation::Navigation(navigation_config_path){
        hardware_initialized_ = false;
        LoadBoatConfig_(navigation_config_path, boat_params_);
        boat_mode_ = boat_params_.boatMode;
        ///debug
        HardWareInitialization_(boat_params_.serialParams.port, boat_params_.serialParams.baud);
        Init_(navigation_config_path);

        //socket_thread_ = false;
        ser_com_ptr_ = nullptr;
        socket_com_ptr_ = nullptr;
        main_thread_ = true;
        route_updated_ = 0;
        stop = true;
        imu_data_.linear_acceleration.x = 0.0;
        imu_data_.linear_acceleration.y = 0.0;
        imu_data_.angular_velocity.z = 0.0;
        imu_data_.angle.yaw = 0.0;
        //imu_data_.angle.pitch = 0.0;
        imu_data_.angle.roll = 0.0;
        imu_data_.angle.pitch = 0.0;
        remote_channel_data_.channel_1 = 1500;
        remote_channel_data_.channel_2 = 1500;
        remote_channel_data_.channel_3 = 1500;
        remote_channel_data_.channel_4 = 1500;
        remote_channel_data_main_thread_ = remote_channel_data_;
        empower_trans_.empower = 0;
        locking_trans_.locking = 0;

        gps_data_.gps_position = now_location_gps_;
        GpsToUtm(&gps_data_.gps_position, &boat_measurement_vector_.position);
        boat_measurement_vector_.imu_data.angle.yaw = initial_yaw_;
        boat_measurement_vector_.imu_data.linear_acceleration.x = 0.0;
        boat_measurement_vector_.imu_data.linear_acceleration.y = 0.0;
        boat_measurement_vector_.imu_data.angular_velocity.z = 0.0;
        velocity_data_.velocity_x = 0.0;
        velocity_data_.velocity_angle = 0.0;

         serial_measurement_mutex_ptr_ = new pthread_mutex_t;
         serial_channel_mutex_ptr_ = new pthread_mutex_t;
         route_updated_mutex_ptr_ = new pthread_mutex_t;
         pthread_mutex_init(serial_measurement_mutex_ptr_, nullptr); //线程锁初始化
         pthread_mutex_init(serial_channel_mutex_ptr_, nullptr); //线程锁初始化
         pthread_mutex_init(route_updated_mutex_ptr_, nullptr); //线程锁初始化

        ser_com_ptr_ = new serial_communication::SerialCommunication(boat_params_.serialParams.port, boat_params_.serialParams.baud);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)GpsMsgsCallback, GPS_FLAG, this);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)ImuMsgsCallback, IMU_FLAG, this);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)RemoteControlSignalCallback, REMOTE_CHANNEL_FLAG, this);
        bool res = ser_com_ptr_->StartSerialReceiveThread();
        if(!res){
            exit(-1);
        }
        ///debug
        socket_com_ptr_ = new socket_communication::SocketCommunication(boat_params_.socketParams.host, boat_params_.socketParams.port);
        socket_com_ptr_->SetCallBackFunction((socket_communication::callBack)SocketReceiveCallBack, 1, this);
         //std::cout<<"er"<<std::endl;
        if(!socket_com_ptr_->StartSocketReceiveThread()){
             exit(-1);
        }
//        now_call_timestamp_ = std::chrono::steady_clock::now();
//        last_call_timestamp_ = std::chrono::steady_clock::now();
        std::cout<<"Initialized!"<<std::endl;
    }

    /**
     * @brief StartSerialThread
     * @param port: 串口号
     * @param baud_rate: 波特率
     * @return true/false: 串口打开是否成功
     */
    void boat::HardWareInitialization_(const std::string &com, unsigned int baud) {
        serial_communication::SerialCommunication ser(com, baud);
        ser.SetCallBackFunction((serial_communication::callBack)HardWareInitializationCallBack, GPS_FLAG, this);
        bool res = ser.StartSerialReceiveThread();
        if(!res){
            exit(-1);
        }
        while(!hardware_initialized_){
            //std::cout<<"while"<<std::endl;
            std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)500));
        }
        ser.CloseSerialReceiveThread();
    }

    void boat::HardWareInitializationCallBack(uint8_t *buffer_ptr_, void *__this) {
        auto* _this = (boat*)__this;
        //auto* gps_trans = (GpsDataTrans*)buffer_ptr_;
        GpsDataTrans gps_trans;
        memcpy(&gps_trans, buffer_ptr_, sizeof(GpsDataTrans));
        std::cout<<"initialize callback"<<std::endl;
//       gps_trans.latitude = 37.0;
//       gps_trans.longitude = 122.0;
        //std::cout<<gps_trans->latitude<<"-"<<gps_trans->longitude<<std::endl;
        if(gps_trans.latitude == 0.0 && gps_trans.longitude==0.0){
            return;
        }
        GpsPosition g_p;
        g_p.latitude = gps_trans.latitude;
        g_p.longitude = gps_trans.longitude;
        GpsToUtmPartition(&g_p, &_this->utm_zone_, &_this->hemisphere_);
        std::cout<<"utm zone: "<<_this->utm_zone_<<" hemisphere: "<<_this->hemisphere_<<std::endl;
        //std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        //auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
        //std::time_t timestamp = tmp.count();
        //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
        //std::cout<<timestamp<<std::endl;
        //exit(-1);
        _this->gps_data_.gps_position.latitude = gps_trans.latitude;
        _this->gps_data_.gps_position.longitude = gps_trans.longitude;
        _this->now_location_gps_ = _this->gps_data_.gps_position;
        _this->hardware_initialized_ = true;
    }

    bool boat::LoadBoatConfig_(const std::string &config_xml_path, BoatParams& boatParams) {
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

        TiXmlElement* boat_xml = xmlRoot->FirstChildElement("boat");
        TiXmlElement* mode_xml = boat_xml->FirstChildElement("mode");
        int mode_flag = std::stoi(mode_xml->GetText());
        switch (mode_flag){
            case (int)remote_mode:
            case (int)navigation_mode:
            case (int)track_mode:
                boatParams.boatMode = (BoatMode)mode_flag;
                break;
            default:
                std::cerr<<"Undefined mode: "<<mode_flag<<std::endl;
                break;
        }
        TiXmlElement* socket_xml = boat_xml->FirstChildElement("socket");
        TiXmlElement* host_xml = socket_xml->FirstChildElement("host");
        TiXmlElement* socket_port_xml = socket_xml->FirstChildElement("port");
        TiXmlElement* socket_send_frequency_xml = socket_xml->FirstChildElement("send_frequency");
        boatParams.socketParams.host = host_xml->GetText();
        boatParams.socketParams.port = std::stoi(socket_port_xml->GetText());
        boatParams.socketParams.send_frequency = std::stoi(socket_send_frequency_xml->GetText());

        TiXmlElement* serial_xml = boat_xml->FirstChildElement("serial");
        TiXmlElement* baud_xml = serial_xml->FirstChildElement("baud");
        TiXmlElement* serial_port_xml = serial_xml->FirstChildElement("port");
        boatParams.serialParams.port = serial_port_xml->GetText();
        boatParams.serialParams.baud = std::stoi(baud_xml->GetText());

        TiXmlElement* frequency_xml = boat_xml->FirstChildElement("frequency");
        boatParams.frequency = std::stoi(frequency_xml->GetText());
        config_xml.Clear();
    }

    void boat::SocketReceiveCallBack(uint8_t* buffer_ptr_, void* __this){
        auto* _this = (boat*)__this;
        std::string string_rec = (const char*)buffer_ptr_;
        std::cout<<"receive: "<<string_rec<<std::endl;
        json j = json::parse(string_rec);
        SocketReceive s_r = j;
        if(s_r.empower!=_this->empower_trans_.empower){
            _this->empower_trans_.empower = s_r.empower;
            _this->EmpowerPublish_(_this->empower_trans_);
        }
        if(s_r.mode!=_this->boat_mode_){
            switch (s_r.mode){
                case (int)remote_mode:
                case (int)navigation_mode:
                case (int)track_mode:
                    _this->boat_mode_ = (BoatMode)s_r.mode;
                    break;
                default:
                    std::cerr<<"Undefined mode: "<<s_r.mode<<std::endl;
                    break;
            }
//          if(s_r.mode<=3 && s_r.mode>=1){
//              _this->boat_mode_ = (BoatMode)s_r.mode;
//           }
        }
        if(s_r.stop){
            _this->stop = true;
        }
        if(s_r.route_updated){
            pthread_mutex_lock(_this->route_updated_mutex_ptr_);
            _this->route_updated_ = s_r.route_updated;
            _this->locus_points_main_thread_ = s_r.route_gps_positions;
            pthread_mutex_unlock(_this->route_updated_mutex_ptr_);
            //_this->UpdateLocusPoints_(s_r.route_gps_positions, 0);
        }
    }

    /**
     * @brief VelocityPublish_
     * @param velocity_info : 速度
     */
     ///速度发布
    void boat::VelocityPublish_(VelocityData& velocity_data){
        if(velocity_data.velocity_angle>3.0f){
            velocity_data.velocity_angle = 3.0f;
        }else if(velocity_data.velocity_angle < -3.0f){
            velocity_data.velocity_angle = -3.0f;
        }
        if(velocity_data.velocity_x>9.9f){
            velocity_data.velocity_x = 9.9f;
        }else if(velocity_data.velocity_x<-9.9f){
            velocity_data.velocity_x = -9.9f;
        }
        //std::cout<<"v_x: "<<velocity_data.velocity_x<<std::endl;
        //std::cout<<"v_a: "<<velocity_data.velocity_angle<<std::endl;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(velocity_data, VELOCITY_FLAG);
        }
    }

    void boat::ControlPowerPublish_(ControlPowerTrans& control_power_trans){
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(control_power_trans, CONTROL_POWER_FLAG);
        }
    }

    void boat::EmpowerPublish_(EmpowerTrans& empowerTrans){
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(empowerTrans, EMPOWER_FLAG);
        }
    }

    void boat::SocketShowPublish_() {
        if(socket_com_ptr_->IsOpen()){
            SocketShow s_s;
            GpsPosition gps_p;
            UtmToGps(&now_state_.position, &gps_p);
            GetLocusPoints_(s_s.route_gps_positions);
            s_s.imu_data = imu_data_;
            s_s.imu_data.angle.yaw = now_state_.attitude_angle;
            s_s.locking = locking_trans_.locking;
            s_s.utm_position = now_state_.position;
            s_s.raw_gps_position = gps_data_.gps_position;
            s_s.gps_position = gps_p;
            socket_com_ptr_->SendData(s_s, 1);
        }
    }

//    void boat::SocketDataPublish_(){
//        char c[1000];
//        GpsPosition gps_pos;
//        UtmToGps(&now_state_.position, &gps_pos);
//        sprintf(c, "{\n"
//                   "\t\"gps\": {\n"
//                   "\t\t\"latitude\": %.5f,\n"
//                   "\t\t\"longitude\": %.5f\n"
//                   "\t},\n"
//                   "\t\"attitude\": {\n"
//                   "\t\t\"yaw\": %.5f,\n"
//                   "\t\t\"pitch\": %.5f,\n"
//                   "\t\t\"roll\": %.5f\n"
//                   "\t}\n"
//                   "}",
//                   gps_pos.latitude,gps_pos.longitude,yaw,imu_data_.angle.pitch,
//                   imu_data_.angle.roll);
//    }
    /**
     * @brief ImuMsgsCallback
     * @param imu_info
     * @param __this Boat类的this指针，多线程调用时需手动传入，下同
     */
     ///IMU信息回调，由串口线程调用，将串口IMU信息转化为滤波器更新需要的观测值
    void boat::ImuMsgsCallback(uint8_t* buffer_ptr_, void* __this) {
        auto* _this = (boat*)__this;
         ImuDataTrans imu_trans_data;
         ImuDataTrans* imu_trans;
         imu_trans = &imu_trans_data;
         memcpy(imu_trans, buffer_ptr_, sizeof(ImuDataTrans));
        //auto* imu_trans = (ImuDataTrans*)buffer_ptr_;
        float ax, ay;
        //float yaw_raw = imu_trans->yaw;
        _this->imu_data_.angle.roll = imu_trans->roll;
        _this->imu_data_.angle.pitch = imu_trans->pitch;
        _this->imu_data_.angle.yaw = imu_trans->yaw;
        _this->imu_data_.angular_velocity.z = imu_trans->angular_velocity_z;
        _this->imu_data_.linear_acceleration.x = imu_trans->linear_acceleration_x;
        _this->imu_data_.linear_acceleration.y = imu_trans->linear_acceleration_y;

        ax = cos(_this->imu_data_.angle.pitch)*_this->imu_data_.linear_acceleration.x*cos(_this->now_state_.attitude_angle)-
                _this->imu_data_.linear_acceleration.y*sin(_this->now_state_.attitude_angle);
        ay = cos(_this->imu_data_.angle.pitch)*_this->imu_data_.linear_acceleration.x*sin(_this->now_state_.attitude_angle)+
                _this->imu_data_.linear_acceleration.y*cos(_this->now_state_.attitude_angle);
        //std::cout<<"imu call back"<<std::endl;
        //std::cout<<" v_z: "<<imu_trans->angular_velocity_z<<" ";
        pthread_mutex_lock(_this->serial_measurement_mutex_ptr_);
        _this->boat_measurement_vector_.imu_data.angle.yaw = _this->yaw;
        _this->boat_measurement_vector_.imu_data.linear_acceleration.x = ax;
        _this->boat_measurement_vector_.imu_data.linear_acceleration.y = ay;
        _this->boat_measurement_vector_.imu_data.angular_velocity.z = _this->imu_data_.angular_velocity.z;
        pthread_mutex_unlock(_this->serial_measurement_mutex_ptr_);

//         _this->now_call_timestamp_ = std::chrono::steady_clock::now();
//         std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(_this->now_call_timestamp_
//                 - _this->last_call_timestamp_);
//         std::cout<<time_used.count()<<std::endl;
//         _this->last_call_timestamp_ = std::chrono::steady_clock::now();
     }

    /**
     * @brief GpsMsgsCallback
     * @param gps_info
     * @param __this
     */
     ///GPS回调，由串口线程调用，将串口GPS信息转化为滤波器更新需要的观测值
    void boat::GpsMsgsCallback(uint8_t* buffer_ptr_, void* __this){
        //std::cout<<"gps call back"<<std::endl;
        auto* _this = (boat*)__this;
         GpsDataTrans gps_trans_data;
         GpsDataTrans* gps_trans;
         gps_trans = &gps_trans_data;
         memcpy(gps_trans, buffer_ptr_, sizeof(GpsDataTrans));
        //auto* gps_trans = (GpsDataTrans*)buffer_ptr_;
        _this->gps_data_.gps_position.longitude = gps_trans->longitude;
        _this->gps_data_.gps_position.latitude = gps_trans->latitude;
        pthread_mutex_lock(_this->serial_measurement_mutex_ptr_);
        GpsToUtm(&_this->gps_data_.gps_position, &_this->boat_measurement_vector_.position);
        pthread_mutex_unlock(_this->serial_measurement_mutex_ptr_);
    }
    /**
     * @brief RemoteControlSignalCallback
     * @param channel_info
     * @param __this
     */
     ///遥控信号回调，由串口线程调用
    void boat::RemoteControlSignalCallback(uint8_t* buffer_ptr_, void* __this){
        ///channel 3    up     2000
        ///             down   1000
        ///channel 4    up     1000
        ///             down   2000
        auto* _this = (boat*)__this;
         RemoteChannelTrans remote_channel_trans_data;
         RemoteChannelTrans* remote_channel_trans;
         remote_channel_trans = &remote_channel_trans_data;
         memcpy(remote_channel_trans, buffer_ptr_, sizeof(RemoteChannelTrans));
        //auto* remote_channel_trans = (RemoteChannelTrans*)buffer_ptr_;
         pthread_mutex_lock(_this->serial_channel_mutex_ptr_);
        _this->remote_channel_data_ = *remote_channel_trans;
         pthread_mutex_unlock(_this->serial_channel_mutex_ptr_);
//         std::cout<<"channel: ";
//         std::cout<<remote_channel_trans->channel_1<<" ";
//         std::cout<<remote_channel_trans->channel_2<<" ";
//         std::cout<<remote_channel_trans->channel_3<<" ";
//         std::cout<<remote_channel_trans->channel_4<<std::endl;
        //pthread_mutex_lock(serial_channel_mutex_ptr_);
        //remote_channel_info_main_thread_ = channel_info;
        //pthread_mutex_unlock(serial_channel_mutex_ptr_);
    }


//    void* boat::SocketCommunication(void* __this) {
//        auto* _this = (boat*)__this;
//        char buf[SIZE];
//        while (_this->socket_thread_){
//            memset(buf,'\0', SIZE);
//            read(*_this->client_socket_ptr_, buf, SIZE-1);
//        }
//        free(_this->client_socket_ptr_);
//        _this->client_socket_ptr_ = nullptr;
//    }
//
//    bool boat::KillSocketThread() {
//        socket_thread_ = false;
//        return true;
//    }

    /**
     * @brief AnalysisRemoteInfo
     * @param channel
     */
     ///分析遥控信号
    void boat::AnalysisRemoteInfo(const RemoteChannelTrans &channel, volatile bool* stop) {
        if(channel.channel_3>1900 && channel.channel_3<2100){
            boat_mode_ = remote_mode;
        }
        else if(channel.channel_3>900 && channel.channel_3<1100){
            boat_mode_ = navigation_mode;
        }
//        if(channel.channel_4>1900&&channel.channel_4<2100){
//            *stop = true;
//        } else if(channel.channel_4>900&&channel.channel_4<1100){
//            *stop = false;
//        }
    }
    /**
     * @brief RemoteVelocityAnalyze_
     * @param channel
     * @param v
     */
    ///分析遥控信号, 计算速度
    void boat::RemoteVelocityAnalyze_(const RemoteChannelTrans &channel, VelocityData* v) {
        //TODO
        int v_a = channel.channel_2 - 1500;
        int v_x= channel.channel_1 - 1500;
        v->velocity_x =  0.006f * v_x;
        v->velocity_angle = 0.003f * v_a;
    }

    /**
     * @brief Control
     */
     ///控制船运动
    void boat::Control() {
        std::chrono::steady_clock::time_point now_mark_timestamp;
        std::chrono::steady_clock::time_point last_mark_timestamp;
        std::chrono::steady_clock::time_point now_timestamp;
        std::chrono::steady_clock::time_point last_timestamp;
        now_mark_timestamp = std::chrono::steady_clock::now();

        last_mark_timestamp = now_mark_timestamp;
        int time_used_u;
        stop = false;
        //int times = 0;
        double period = 1.0/(double)boat_params_.frequency;
        uint32_t socket_send_count = 0;
        uint32_t count = 0;
        while(main_thread_){
            //times++;
            last_timestamp = std::chrono::steady_clock::now();
            count++;
            socket_send_count++;
            if(socket_send_count == boat_params_.frequency/boat_params_.socketParams.send_frequency){
                socket_send_count = 0;
                SocketShowPublish_();
            }
            if(ser_com_ptr_->IsOpen()){
                pthread_mutex_lock(serial_channel_mutex_ptr_);
                remote_channel_data_main_thread_ = remote_channel_data_;
                pthread_mutex_unlock(serial_channel_mutex_ptr_);
            } else{
                remote_channel_data_main_thread_ = remote_channel_data_;
            }
            if(ser_com_ptr_->IsOpen()){
                pthread_mutex_lock(serial_measurement_mutex_ptr_);
                UpdateMeasurementVector(boat_measurement_vector_);
                pthread_mutex_unlock(serial_measurement_mutex_ptr_);
            }
            AnalysisRemoteInfo(remote_channel_data_main_thread_, &stop);
            Filter();
            if(mark_point_parameter_.mark_flag){
                now_mark_timestamp = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>
                        (now_mark_timestamp-last_mark_timestamp);
                if(time_used.count()>mark_point_parameter_.period){
                    last_mark_timestamp = now_mark_timestamp;
                    MarkGpsPosition();
                }
            }
            //std::cout<<"boat_mode: "<<boat_mode_<<std::endl;
            if(boat_mode_ == navigation_mode){
                NavigationCalculation();
                //std::cout<<" yaw: "<<yaw<<std::endl;
                //std::cout<<" state_a: "<<now_state_.attitude_angle;
                if(route_updated_){
                    pthread_mutex_lock(route_updated_mutex_ptr_);
                    UpdateLocusPoints_(locus_points_main_thread_, 0);
                    pthread_mutex_unlock(route_updated_mutex_ptr_);
                    route_updated_ = 0;
                }
                NavigationVelocityAnalyze_(yaw, velocity_data_);
            }
            else if(boat_mode_ == remote_mode){
                RemoteVelocityAnalyze_(remote_channel_data_main_thread_, &velocity_data_);
            }
            //std::cout<<stop<<std::endl;
            if(stop){
                velocity_data_.velocity_angle = 0.0;
                velocity_data_.velocity_x = 0.0;
                ReSetMarkPointFlag();
            }
            //std::cout<<times<<std::endl;
            //std::cout<<"v: "<<velocity_data_.velocity_x<<std::endl;
            VelocityPublish_(velocity_data_);
            now_timestamp = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(now_timestamp-last_timestamp);
            time_used_u = (int)((period- time_used.count())*1000000);
            if(time_used_u<0){
                time_used_u = 0;
            }
            std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)time_used_u));
        }
    }
}