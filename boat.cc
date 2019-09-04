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

namespace navigation {
    void to_json(json &j, const navigation::GpsPosition &gpsPosition) {
        j = json{{"latitude",  gpsPosition.latitude},
                 {"longitude", gpsPosition.longitude}};
    }

    void from_json(const json &j, navigation::GpsPosition &gpsPosition) {
        gpsPosition.longitude = j.at("longitude").get<double>();
        gpsPosition.latitude = j.at("latitude").get<double>();
    }

    void to_json(json &j, const navigation::UtmPosition &utmPosition) {
        j = json{{"x",          utmPosition.x},
                 {"y",          utmPosition.y},
                 {"GridZone",   (int) utmPosition.gridZone},
                 {"Hemisphere", (int) utmPosition.hemisphere}};
    }

    void from_json(const json &j, navigation::UtmPosition &utmPosition) {
        utmPosition.x = j.at("x").get<double>();
        utmPosition.y = j.at("y").get<double>();
        utmPosition.gridZone = (GridZone) j.at("GridZone").get<int>();
        utmPosition.hemisphere = (Hemisphere) j.at("GridZone").get<int>();
    }

    void to_json(json &j, const navigation::LinearAcceleration &linearAcceleration) {
        j = json{{"x", linearAcceleration.x},
                 {"y", linearAcceleration.y}};
    }

    void from_json(const json &j, navigation::LinearAcceleration &linearAcceleration) {
        linearAcceleration.x = j.at("x").get<double>();
        linearAcceleration.y = j.at("y").get<double>();
    }

    void to_json(json &j, const navigation::AngularVelocity &angular_velocity) {
        j = json{{"z", angular_velocity.z}};
    }

    void from_json(const json &j, navigation::AngularVelocity &angular_velocity) {
        angular_velocity.z = j.at("z").get<double>();
    }

    void to_json(json &j, const navigation::Angle &angle) {
        j = json{{"roll",  angle.roll},
                 {"pitch", angle.pitch},
                 {"yaw",   angle.yaw}};
    }

    void from_json(const json &j, navigation::Angle &angle) {
        angle.yaw = j.at("yaw").get<double>();
        angle.pitch = j.at("pitch").get<double>();
        angle.roll = j.at("roll").get<double>();
    }

    void to_json(json &j, const navigation::ImuData &imuData) {
        j = json{{"angle",               imuData.angle},
                 {"angular_velocity",    imuData.angular_velocity},
                 {"linear_acceleration", imuData.linear_acceleration}};
    }

    void from_json(const json &j, navigation::ImuData &imuData) {
        imuData.angle = j.at("angle").get<navigation::Angle>();
        imuData.angular_velocity = j.at("angular_velocity").get<navigation::AngularVelocity>();
        imuData.linear_acceleration = j.at("linear_acceleration").get<navigation::LinearAcceleration>();
    }

    void to_json(json &j, const SocketShow &socketShow) {
        j = json{{"gps_position",        socketShow.gps_position},
                 {"raw_gps_position",    socketShow.raw_gps_position},
                 {"utm_position",        socketShow.utm_position},
                 {"route_gps_positions", socketShow.route_gps_positions},
                 {"locking",             socketShow.locking},
                 {"imu_data",            socketShow.imu_data},
                 {"speed",               socketShow.speed},
                 {"id",                  socketShow.id},
                 {"receiver_id",         socketShow.receiver_id}};
    }

    void from_json(const json &j, SocketShow &socketShow) {
        socketShow.imu_data = j.at("imu_data").get<navigation::ImuData>();
        socketShow.gps_position = j.at("gps_position").get<navigation::GpsPosition>();
        socketShow.raw_gps_position = j.at("raw_gps_position").get<navigation::GpsPosition>();
        socketShow.utm_position = j.at("utm_position").get<navigation::UtmPosition>();
        socketShow.route_gps_positions = j.at("route_gps_positions").get<std::vector<navigation::GpsPosition>>();
        socketShow.locking = j.at("locking").get<uint8_t>();
        socketShow.speed = j.at("speed").get<double>();
        socketShow.id = j.at("id").get<uint8_t>();
        socketShow.receiver_id = j.at("receiver_id").get<std::vector<uint8_t >>();
    }

    void to_json(json &j, const SocketReceive &socketReceive) {
        j = json{{"mode",                socketReceive.mode},
                 {"empower",             socketReceive.empower},
                 {"route_gps_positions", socketReceive.route_gps_positions},
                 {"route_updated",       socketReceive.route_updated},
                 {"stop",                socketReceive.stop},
                 {"behavior",            socketReceive.behavior},
                 {"receiver_id",         socketReceive.receiver_id}};
    }

    void from_json(const json &j, SocketReceive &socketReceive) {
        socketReceive.route_gps_positions = j.at("route_gps_positions").get<std::vector<navigation::GpsPosition>>();
        socketReceive.empower = j.at("empower").get<uint8_t>();
        socketReceive.mode = j.at("mode").get<uint8_t>();
        socketReceive.route_updated = j.at("route_updated").get<uint8_t>();
        socketReceive.stop = j.at("stop").get<uint8_t>();
        socketReceive.behavior = j.at("behavior").get<uint8_t>();
        socketReceive.receiver_id = j.at("receiver_id").get<std::vector<uint8_t>>();
    }

    void to_json(json& j, const SocketHandShake& socketHandShake){
        j = json{{"identity", socketHandShake.identity},
                 {"id",       socketHandShake.id},
                 {"ok",       socketHandShake.ok}};
    }
    void from_json(const json& j, SocketHandShake& socketHandShake){
        socketHandShake.identity = j.at("identity").get<uint8_t>();
        socketHandShake.ok = j.at("ok").get<uint8_t>();
        socketHandShake.id = j.at("id").get<uint8_t>();
    }

    void to_json(json& j, const SocketHandShake2& socketHandShake2){
        j = json{{"identity",   socketHandShake2.identity},
                 {"error_type", socketHandShake2.error_type}};
    }

    void from_json(const json& j, SocketHandShake2& socketHandShake2){
        socketHandShake2.identity = j.at("identity").get<uint8_t>();
        socketHandShake2.error_type = j.at("error_type").get<uint8_t>();
    }

    void to_json(json& j, SocketHandShake3& socketHandShake3){
        j = json{{"identity",   socketHandShake3.identity},
                 {"id_list",    socketHandShake3.id_list}};
    }

    void from_json(const json& j, SocketHandShake3& socketHandShake3){
        socketHandShake3.id_list = j.at("id_list").get<std::vector<uint8_t>>();
        socketHandShake3.identity = j.at("identity").get<uint8_t>();
    }
}
namespace navigation{
    namespace mode{
        DynamicPositioning::DynamicPositioning() {
            max_distance_ = 3.0;
        }
//        DynamicPositioning::DynamicPositioning(navigation::pose::Pose &pose) {
//            pose_ = pose;
//        }

        DynamicPositioning::DynamicPositioning(navigation::pose::Pose &pose, double kp, double ki, double kd,
                double max_distance) {

            target_pose_ = pose;
            yaw_pid_controller_ = algorithm::PidController(kp, ki, kd);
            max_distance_ = max_distance;
            yaw_pid_controller_.Target() = 0.0;
        }

        DynamicPositioning::DynamicPositioning(const navigation::mode::DynamicPositioning &D) {
            limit_.y_limit = D.limit_.y_limit;
            limit_.x_limit = D.limit_.x_limit;
            limit_.z_limit = D.limit_.z_limit;
            yaw_pid_controller_ = D.yaw_pid_controller_;
            target_pose_ = D.target_pose_;
            max_distance_ = D.max_distance_;
            target_linear_velocity_ = D.target_linear_velocity_;
        }

        void DynamicPositioning::Limit(const navigation::AngularVelocityLimit &limit) {
            limit_.x_limit = limit.x_limit;
            limit_.y_limit = limit.y_limit;
            limit_.z_limit = limit.z_limit;
            yaw_pid_controller_.SetLimit(limit_.z_limit);
        }

        void DynamicPositioning::MaxDistance(double d) {
            max_distance_ = d;
        }

        void DynamicPositioning::TargetLinearVelocity(navigation::LinearVelocity l) {
            target_linear_velocity_ = l;
        }

        int DynamicPositioning::Update(const navigation::pose::Pose& target, const navigation::pose::Pose& input,
                navigation::Velocity& v) {
            double distance = navigation::point::Distance(target.Position(), input.Position());
            if(distance>max_distance_){
                return -1;
            }
            target_pose_ = target;
            double yaw = CalcYaw(target_pose_.Attitude().yaw, input.Attitude().yaw);
            v.angular_velocity.z = yaw_pid_controller_.Update(yaw);
            v.linear_velocity = target_linear_velocity_;
            return 0;
        }

        int DynamicPositioning::Update(const navigation::pose::Pose &input, navigation::Velocity &v) {
            return Update(target_pose_, input, v);
        }
    }

    /**
     * @param navigation_config_path
     */
     ///Boat的构造函数，将配置文件路径传入父类，初始化传感器值
    boat::boat(std::string& navigation_config_path) :navigation::Navigation(navigation_config_path){
        hardware_initialized_ = false;
        socket_handshake_ok_ = false;
        LoadBoatConfig_(navigation_config_path, boat_params_);
        boat_mode_ = boat_params_.boatMode;
        behavior_ = 0;
        ///debug
        if(boat_params_.gpsInitialization.hardware_flag){
            HardWareInitialization_(boat_params_.serialParams.port, boat_params_.serialParams.baud);
        }
        else{
            gps_data_.gps_position = boat_params_.gpsInitialization.gpsPosition;
            now_location_gps_ = gps_data_.gps_position;
        }
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
        stop_trans_.stop = 0;
        control_power_trans_.host = 1;

        gps_data_.gps_position = now_location_gps_;
        gps_data_.speed = 0.0f;
        UtmPosition utmPosition;

        LOG(INFO)<<"Init gps --- longitude: "<<gps_data_.gps_position.longitude<<" latitude: "<<gps_data_.gps_position.latitude;
        GpsToUtm(&gps_data_.gps_position, &utmPosition);
        boat_measurement_vector_.position = utmPosition;

        LOG(INFO)<<"Init utm --- x: "<<boat_measurement_vector_.position.x<<" y: "<<boat_measurement_vector_.position.y<<" z: "<<boat_measurement_vector_.position.gridZone;
        boat_measurement_vector_.imu_data.angle.yaw = initial_yaw_;
        boat_measurement_vector_.imu_data.linear_acceleration.x = 0.0;
        boat_measurement_vector_.imu_data.linear_acceleration.y = 0.0;
        boat_measurement_vector_.imu_data.angular_velocity.z = 0.0;

        serial_measurement_mutex_ptr_ = new pthread_mutex_t;
        serial_channel_mutex_ptr_ = new pthread_mutex_t;
        route_updated_mutex_ptr_ = new pthread_mutex_t;
        pthread_mutex_init(serial_measurement_mutex_ptr_, nullptr); //线程锁初始化
        pthread_mutex_init(serial_channel_mutex_ptr_, nullptr); //线程锁初始化
        pthread_mutex_init(route_updated_mutex_ptr_, nullptr); //线程锁初始化

        ser_com_ptr_ = new serial_communication::SerialCommunication(boat_params_.serialParams.port, boat_params_.serialParams.baud);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)GpsMsgsCallback, kGPS_FLAG, this);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)ImuMsgsCallback, kIMU_FLAG, this);
        ser_com_ptr_ ->SetCallBackFunction((serial_communication::callBack)RemoteControlSignalCallback, kREMOTE_CHANNEL_FLAG, this);
        ser_com_ptr_->SetCallBackFunction((serial_communication::callBack)LockingCallback, kLOCKING_FLAG, this);
        bool res = ser_com_ptr_->StartSerialReceiveThread();
        if(!res){
            exit(-1);
        }
        socket_com_ptr_ = new socket_communication::SocketCommunication(boat_params_.socketParams.host, boat_params_.socketParams.port);
         if(!socket_com_ptr_->StartSocketReceiveThread()){
             exit(-1);
         }
        SocketHandShake_();
        socket_com_ptr_->SetCallBackFunction((socket_communication::callBack)SocketReceiveCallBack, kNormalData, this);
        socket_com_ptr_->SetCallBackFunction((socket_communication::callBack)SocketHandShake2CallBack, kHandShake2, this);
        socket_com_ptr_->SetSignalCallBackFunction((socket_communication::callBack)SocketOfflineReconnectedCallBack,
                socket_communication::kSocketOfflineReconnected, this);
         //std::cout<<"er"<<std::endl;
//        now_call_timestamp_ = std::chrono::steady_clock::now();
//        last_call_timestamp_ = std::chrono::steady_clock::now();
        std::cout<<"Initialized!"<<std::endl;
         LOG(INFO)<<"Initialized!"<<std::endl;
    }

    void boat::SocketHandShake_() {
        SocketHandShake_(this);
    }

    void boat::SocketHandShake_(void *__this) {
        auto* _this = (boat*)__this;
        _this->socket_com_ptr_->SetCallBackFunction((socket_communication::callBack)SocketHandShakeCallBack, kHandShake1, _this);
        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)500));
        SocketHandShake s;
        s.ok = 1;
        s.identity = 1;
        s.id = boat_params_.id;
        std::cout<<"Wait for socket handshake"<<std::endl;
#ifdef USE_GLOG
        LOG(INFO)<<"Wait for socket handshake"<<std::endl;
#endif
        int n = 0;
        while(!_this->socket_handshake_ok_){
            if(n % 10 == 0){
                _this->socket_com_ptr_->SendData(s, kHandShake1);
            }
            std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)5000));
            n++;
        }
        std::cout<<"Socket handshake ok"<<std::endl;
#ifdef USE_GLOG
        LOG(INFO)<<"Socket handshake ok"<<std::endl;
#endif
        _this->socket_com_ptr_->RemoveCallBackFunction(kHandShake1);
    }

    void boat::SocketHandShake2CallBack(uint8_t *buffer_ptr_, void *__this) {
        std::cout<<"error callback"<<std::endl;
        auto* _this = (boat*)__this;
        std::string string_rec = (const char*)buffer_ptr_;
        json j;
        try {
            j = json::parse(string_rec);
        }
        catch (nlohmann::detail::parse_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"SocketHandShake2 parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"SocketHandShake2 parse error, data: "<<string_rec<<std::endl;
            return;
        }
        SocketHandShake2 s_h2;
        try {
            s_h2 = j;
        }
        catch (nlohmann::detail::type_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"SocketHandShake2 parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"SocketHandShake2 parse error, data: "<<string_rec<<std::endl;
            return;
        }
        if (s_h2.identity == 2){
            switch(s_h2.error_type){
                case (uint8_t)kMissHandShake:
                    _this->socket_handshake_ok_ = false;
                    _this->SocketHandShake_(_this);
                    break;
                default:
                    std::cerr<<"Undefined socket error type: "<<s_h2.error_type<<std::endl;
#ifdef USE_GLOG
                    LOG(ERROR) <<"Undefined socket error type: "<<s_h2.error_type<<std::endl;
#endif
                    break;
            }
        }
    }

    /**
     * @brief StartSerialThread
     * @param port: 串口号
     * @param baud_rate: 波特率
     * @return true/false: 串口打开是否成功
     */
    void boat::HardWareInitialization_(const std::string &com, unsigned int baud) {
        serial_communication::SerialCommunication ser(com, baud);
        ser.SetCallBackFunction((serial_communication::callBack)HardWareInitializationCallBack, kGPS_FLAG, this);
        bool res = ser.StartSerialReceiveThread();
        if(!res){
            exit(-1);
        }
        std::cout<<"Wait for hardware initialized"<<std::endl;
#ifdef USE_GLOG
        LOG(INFO)<<"Wait for hardware initialized"<<std::endl;
#endif
        while(!hardware_initialized_){
            std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)500));
        }
        std::cout<<"Hardware initialized"<<std::endl;
#ifdef USE_GLOG
        LOG(INFO)<<"Hardware initialized"<<std::endl;
#endif
        ser.CloseSerialReceiveThread();
    }

    void boat::SocketHandShakeCallBack(uint8_t *buffer_ptr_, void *__this) {
        auto* _this = (boat*)__this;
        std::cout<<"Socket handshake callback"<<std::endl;
        std::string string_rec = (const char*)buffer_ptr_;
        json j;
        try {
            j = json::parse(string_rec);
        }
        catch (nlohmann::detail::parse_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"SocketHandShake parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"SocketHandShake parse error, data: "<<string_rec<<std::endl;
            return;
        }
        SocketHandShake s_r;
        try {
            s_r = j;
        }
        catch (nlohmann::detail::type_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"SocketHandShake parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"SocketHandShake parse error, data: "<<string_rec<<std::endl;
            return;
        }
        if(s_r.identity == 2 && s_r.ok){
            _this->socket_handshake_ok_ = true;
        } else if(s_r.identity == 2 && s_r.ok == 0){
#ifdef USE_GLOG
            LOG(ERROR)<<"socket_handshake_ok_: "<<(int)s_r.ok<<std::endl;
#endif
            std::cerr<<"socket_handshake_ok_: "<<(int)s_r.ok<<std::endl;
            exit(-2);
        }
    }

    void boat::HardWareInitializationCallBack(uint8_t *buffer_ptr_, void *__this) {
        auto* _this = (boat*)__this;
        GpsDataTrans gps_trans;
        memcpy(&gps_trans, buffer_ptr_, sizeof(GpsDataTrans));
#ifdef USE_GLOG
        LOG(INFO)<<"initialize callback"<<std::endl;
#endif
        if(gps_trans.latitude == 0.0 && gps_trans.longitude==0.0){
            return;
        }
        GpsPosition g_p;
        g_p.latitude = gps_trans.latitude;
        g_p.longitude = gps_trans.longitude;
        GpsToUtmPartition(&g_p, &_this->utm_zone_, &_this->hemisphere_);
#ifdef USE_GLOG
        LOG(INFO)<<"utm zone: "<<_this->utm_zone_<<" hemisphere: "<<_this->hemisphere_<<std::endl;
        LOG(INFO)<<"initialize call back -- longitude: "<<gps_trans.longitude<<" latitude: "<<gps_trans.latitude;
#endif
        _this->gps_data_.gps_position.latitude = gps_trans.latitude;
        _this->gps_data_.gps_position.longitude = gps_trans.longitude;
        _this->now_location_gps_ = _this->gps_data_.gps_position;
        _this->hardware_initialized_ = true;
    }

    bool boat::LoadBoatConfig_(const std::string &config_xml_path, BoatParams& boatParams) {
        TiXmlDocument config_xml;
        if (!config_xml.LoadFile(config_xml_path.c_str()))
        {
            LOG(ERROR)  << "Can not load file: " << config_xml_path << std::endl;
            config_xml.Clear();
            return false;
        }
        TiXmlElement* xmlRoot = config_xml.RootElement();
        if (xmlRoot == nullptr)
        {
            LOG(ERROR)  << "Failed to load file: No root element." << std::endl;
            return false;
        }

        TiXmlElement* boat_xml = xmlRoot->FirstChildElement("boat");
        TiXmlElement* mode_xml = boat_xml->FirstChildElement("mode");
        int mode_flag = std::stoi(mode_xml->GetText());
        switch (mode_flag){
            case (int)remote_mode:
            case (int)navigation_mode:
            case (int)track_mode:
            case (int)attack_mode:
            case (int)kDynamicPositioningMode:
                boatParams.boatMode = (BoatMode)mode_flag;
                break;
            default:
                std::cerr<<"Undefined mode: "<<mode_flag<<std::endl;
#ifdef USE_GLOG
                LOG(ERROR) <<"Undefined mode: "<<mode_flag<<std::endl;
#endif
                break;
        }
        TiXmlElement* socket_xml = boat_xml->FirstChildElement("socket");
        TiXmlElement* host_xml = socket_xml->FirstChildElement("host");
        TiXmlElement* socket_port_xml = socket_xml->FirstChildElement("port");
        TiXmlElement* socket_send_frequency_xml = socket_xml->FirstChildElement("send_frequency");
        boatParams.socketParams.host = host_xml->GetText();
        boatParams.socketParams.port = (uint16_t)std::stoi(socket_port_xml->GetText());
        boatParams.socketParams.send_frequency = (uint32_t)std::stoi(socket_send_frequency_xml->GetText());

        TiXmlElement* serial_xml = boat_xml->FirstChildElement("serial");
        TiXmlElement* baud_xml = serial_xml->FirstChildElement("baud");
        TiXmlElement* serial_port_xml = serial_xml->FirstChildElement("port");
        TiXmlElement* serial_send_frequency_xml = serial_xml->FirstChildElement("send_frequency");
        boatParams.serialParams.port = serial_port_xml->GetText();
        boatParams.serialParams.baud = (uint32_t)std::stoi(baud_xml->GetText());
        boatParams.serialParams.send_frequency = (uint32_t)std::stoi(serial_send_frequency_xml->GetText());

        TiXmlElement* gps_initialization_xml = boat_xml->FirstChildElement("gps_initialization");
        TiXmlElement* hardware_flag_xml = gps_initialization_xml->FirstChildElement("hardware_flag");
        TiXmlElement* gps_initialization_point_xml = gps_initialization_xml->FirstChildElement("point");
        boatParams.gpsInitialization.hardware_flag = (uint8_t )std::stoi(hardware_flag_xml->GetText());
        boatParams.gpsInitialization.gpsPosition.latitude = (float)std::strtod(gps_initialization_point_xml->FirstChildElement("latitude")->GetText(), nullptr);
        boatParams.gpsInitialization.gpsPosition.longitude = (float)std::strtod(gps_initialization_point_xml->FirstChildElement("longitude")->GetText(), nullptr);

        TiXmlElement* frequency_xml = boat_xml->FirstChildElement("frequency");
        TiXmlElement* id_xml = boat_xml->FirstChildElement("id");
        boatParams.frequency = (uint32_t)std::stoi(frequency_xml->GetText());
        boatParams.id = (uint8_t)std::stoi(id_xml->GetText());
        config_xml.Clear();
        return true;
    }

    void boat::SocketOfflineReconnectedCallBack(uint8_t *buffer_ptr_, void *__this) {
        auto* _this = (boat*)__this;
        if(_this->socket_com_ptr_->OfflineReconnection()){
            _this->socket_handshake_ok_ = false;
            _this->SocketHandShake_(_this);
            _this->socket_com_ptr_->ResetOfflineFlag();
        }
    }

    void boat::SocketReceiveCallBack(uint8_t* buffer_ptr_, void* __this) {
        auto* _this = (boat*)__this;
        std::string string_rec = (const char*)buffer_ptr_;
#ifdef USE_GLOG
        LOG(INFO)<<"socket receive: "<<string_rec<<std::endl;
#endif
        std::cout<<"socket receive: "<<string_rec<<std::endl;
        json j;
        try {
            j = json::parse(string_rec);
        }
        catch (nlohmann::detail::parse_error& e) {
#ifdef USE_GLOG
            LOG(ERROR)<<"Dara parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"Dara parse error, data: "<<string_rec<<std::endl;
            return;
        }
        SocketReceive s_r;
        try {
            s_r = j;
        }
        catch (nlohmann::detail::type_error& e) {
#ifdef USE_GLOG
            LOG(ERROR)<<"Dara parse error, data: "<<string_rec<<std::endl;
#endif
            std::cerr<<"Dara parse error, data: "<<string_rec<<std::endl;
            return;
        }
        std::vector<uint8_t>::iterator iter;
        iter = std::find(s_r.receiver_id.begin(), s_r.receiver_id.end(), _this->boat_params_.id);
        if(iter == s_r.receiver_id.end()){
            return;
        }
//        if(s_r.receiver_id != _this->boat_params_.id) {
//            return;
//        }
        if(s_r.empower!=_this->empower_trans_.empower) {
            _this->empower_trans_.empower = s_r.empower;
        }
        if(s_r.mode!=_this->boat_mode_) {
            switch (s_r.mode) {
                case (int)remote_mode:
                case (int)navigation_mode:
                case (int)track_mode:
                case (int)attack_mode:
                case (int)kDynamicPositioningMode:
                    _this->boat_mode_ = (BoatMode)s_r.mode;
                    LOG(INFO)<<"switch boat mode: "<<_this->boat_mode_;
                    break;
                default:
                    std::cerr<<"Undefined mode: "<<s_r.mode<<std::endl;
                    LOG(INFO)<<"Undefined mode: "<<s_r.mode<<std::endl;
                    break;
            }
        }
        if(_this->stop!=s_r.stop){
            _this->stop = (bool)s_r.stop;
            _this->stop_trans_.stop = s_r.stop;
            _this->StopPublish_(_this->stop_trans_);
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
    void boat::VelocityPublish_(Velocity& velocity){
        VelocityDataTrans velocity_data;
        velocity_data.velocity_x = (float)velocity.linear_velocity.x;
        velocity_data.velocity_angle = (float)velocity.angular_velocity.z;
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
        LOG(INFO)<<"Velocity publish v_x: "<<velocity_data.velocity_x<<" v_a: "<<velocity_data.velocity_angle<<std::endl;
        //std::cout<<"Velocity publish v_x: "<<velocity_data.velocity_x<<" v_a: "<<velocity_data.velocity_angle<<std::endl;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(velocity_data, kVELOCITY_FLAG);
        }
    }

    void boat::ControlPowerPublish_(ControlPowerTrans& control_power_trans){
        LOG(INFO)<<"Control power publish: "<<(int)control_power_trans.host;
        //std::cout<<"Control power publish: "<<(int)control_power_trans.host<<std::endl;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(control_power_trans, kCONTROL_POWER_FLAG);
        }
    }

    void boat::EmpowerPublish_(EmpowerTrans& empowerTrans){
        LOG(INFO)<<"Empower publish: "<<(int)empowerTrans.empower;
        //std::cout<<"Empower publish: "<<(int)empowerTrans.empower<<std::endl;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(empowerTrans, kEMPOWER_FLAG);
        }
    }

    void boat::StopPublish_(StopTrans& stopTrans){
        LOG(INFO)<<"Stop publish: "<<(int)stopTrans.stop;
        //std::cout<<"Stop publish: "<<(int)stopTrans.stop<<std::endl;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(stopTrans, kSTOP_FLAG);
        }
    }

    void boat::ModePublish_(const BoatMode& mode) {
        LOG(INFO)<<"Mode publish: "<<(int)mode;
        ModeTrans m;
        m.mode = mode;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(m, kMODE_FLAG);
        }
    }

    void boat::BehaviorPublish_(uint8_t behavior) {
        LOG(INFO)<<"Mode publish: "<<(int)behavior;
        BehaviorTrans b;
        b.behavior = behavior;
        if(ser_com_ptr_){
            ser_com_ptr_->SendData(b, kBEHAVIOR_FLAG);
        }
    }

    void boat::SocketShowPublish_() {

        if(socket_com_ptr_->IsOpen()){
            //std::cout<<"offline flag: "<<socket_com_ptr_->OfflineReconnection()<<std::endl;
            SocketShow s_s;
            GpsPosition gps_p;
            UtmToGps(&now_state_.position.utm_position, &gps_p);
            //LOG(INFO)<<"socket send --- "<<"utm x: "<<now_state_.position.x<<"utm y: "<<now_state_.position.y<<std::endl;
            GetLocusPoints_(s_s.route_gps_positions);
            s_s.imu_data = imu_data_;
            s_s.imu_data.angle.yaw = now_state_.angle.yaw;
            s_s.locking = locking_trans_.locking;
            s_s.utm_position = now_state_.position.utm_position;
            s_s.raw_gps_position = gps_data_.gps_position;
            s_s.gps_position = gps_p;
            s_s.speed = gps_data_.speed;
            s_s.id = boat_params_.id;
            s_s.receiver_id.push_back(1) ;
            socket_com_ptr_->SendData(s_s, kNormalData);
        }
    }

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
        LOG(INFO)<<"Imu call back --"
            <<" pitch: "<<imu_trans->pitch
            <<" roll: "<<imu_trans->roll
            <<" angular_velocity_z:"<<imu_trans->angular_velocity_z
            <<" linear_acceleration_x: "<<imu_trans->linear_acceleration_x
            <<" linear_acceleration_y: "<<imu_trans->linear_acceleration_y;
        //auto* imu_trans = (ImuDataTrans*)buffer_ptr_;
        double ax, ay;
        if(fabsf(imu_trans->roll) > 10000*M_PI){
            imu_trans->roll = 0.0;
        }
        if(fabsf(imu_trans->pitch) > 10000*M_PI){
            imu_trans->pitch = 0.0;
        }
        if(fabsf(imu_trans->angular_velocity_z)>10000*M_PI){
            imu_trans->angular_velocity_z = 0.0;
        }
        if(fabsf(imu_trans->linear_acceleration_y)>10000*M_PI){
            imu_trans->linear_acceleration_y = 0.0;
        }
        if(fabsf(imu_trans->linear_acceleration_x)>10000*M_PI){
            imu_trans->linear_acceleration_x = 0.0;
        }
        //float yaw_raw = imu_trans->yaw;
        _this->imu_data_.angle.roll = imu_trans->roll;
        _this->imu_data_.angle.pitch = imu_trans->pitch;
        _this->imu_data_.angle.yaw = imu_trans->yaw;
        _this->imu_data_.angular_velocity.z = imu_trans->angular_velocity_z;
        _this->imu_data_.linear_acceleration.x = imu_trans->linear_acceleration_x;
        _this->imu_data_.linear_acceleration.y = imu_trans->linear_acceleration_y;

        double a_a = _this->now_state_.angle.yaw;
        ax = cos(_this->imu_data_.angle.pitch)*_this->imu_data_.linear_acceleration.x*cos(a_a)-
                _this->imu_data_.linear_acceleration.y*sin(a_a);
        ay = cos(_this->imu_data_.angle.pitch)*_this->imu_data_.linear_acceleration.x*sin(a_a)+
                _this->imu_data_.linear_acceleration.y*cos(a_a);
        //std::cout<<"imu call back"<<std::endl;
        //std::cout<<" v_z: "<<imu_trans->angular_velocity_z<<" ";
        pthread_mutex_lock(_this->serial_measurement_mutex_ptr_);
        _this->boat_measurement_vector_.imu_data.angle.yaw = _this->yaw;
        _this->boat_measurement_vector_.imu_data.linear_acceleration.x = ax;
        _this->boat_measurement_vector_.imu_data.linear_acceleration.y = ay;
        _this->boat_measurement_vector_.imu_data.angular_velocity.z = _this->imu_data_.angular_velocity.z;
        pthread_mutex_unlock(_this->serial_measurement_mutex_ptr_);
    }

    /**
     * @brief GpsMsgsCallback
     * @param gps_info
     * @param __this
     */
     ///GPS回调，由串口线程调用，将串口GPS信息转化为滤波器更新需要的观测值
    void boat::GpsMsgsCallback(uint8_t* buffer_ptr_, void* __this){
        auto* _this = (boat*)__this;
        GpsDataTrans gps_trans_data;
        GpsDataTrans* gps_trans;
        gps_trans = &gps_trans_data;
        memcpy(gps_trans, buffer_ptr_, sizeof(GpsDataTrans));
        LOG(INFO)<<"Gps call back -- longitude: "<<gps_trans->longitude<<" latitude: "<<gps_trans->latitude;
        //std::cout<<"Gps call back -- longitude: "<<gps_trans->longitude<<" latitude: "<<gps_trans->latitude <<std::endl;
        //auto* gps_trans = (GpsDataTrans*)buffer_ptr_;
        if(gps_trans->longitude>360.0 || gps_trans->longitude<0.0 || gps_trans->latitude>90.0 || gps_trans->latitude< -90.0){
            LOG(ERROR)<<"Gps transmission error";
            return;
        }
        _this->gps_data_.gps_position.longitude = gps_trans->longitude;
        _this->gps_data_.gps_position.latitude = gps_trans->latitude;
        _this->gps_data_.speed = gps_trans->speed;
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
         _this = static_cast<boat*>(__this);
         RemoteChannelTrans remote_channel_trans_data;
         RemoteChannelTrans* remote_channel_trans;
         remote_channel_trans = &remote_channel_trans_data;
         memcpy(remote_channel_trans, buffer_ptr_, sizeof(RemoteChannelTrans));
        //auto* remote_channel_trans = (RemoteChannelTrans*)buffer_ptr_;
         pthread_mutex_lock(_this->serial_channel_mutex_ptr_);
        _this->remote_channel_data_ = *remote_channel_trans;
         pthread_mutex_unlock(_this->serial_channel_mutex_ptr_);
#ifdef USE_GLOG
         LOG(INFO)<<"Channel call back: "
         <<remote_channel_trans->channel_1 <<" "
         <<remote_channel_trans->channel_2<<" "
         <<remote_channel_trans->channel_3<<" "
         <<remote_channel_trans->channel_4<<std::endl;
#endif
        //pthread_mutex_lock(serial_channel_mutex_ptr_);
        //remote_channel_info_main_thread_ = channel_info;
        //pthread_mutex_unlock(serial_channel_mutex_ptr_);
    }

    void boat::LockingCallback(uint8_t* buffer_ptr_, void* __this){
        auto* _this = (boat*)__this;
        LockingTrans locking_trans_data;
        LockingTrans* locking_trans;
        locking_trans = &locking_trans_data;
        memcpy(locking_trans, buffer_ptr_, sizeof(StopTrans));
        _this->locking_trans_.locking = locking_trans->locking;
        LOG(INFO)<<"Locking call back: "<<locking_trans->locking;
    }

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
    void boat::RemoteVelocityAnalyze_(const RemoteChannelTrans &channel, Velocity* v) {
        int v_a = channel.channel_2 - 1500;
        int v_x = channel.channel_1 - 1500;
        int fre = 125;
        int v_a_s = v_a/fre;
        int v_x_s = v_x/fre;
        v->linear_velocity.x =  0.006 * v_x_s*fre;
        v->angular_velocity.z = 0.003 * v_a_s*fre;
        //std::cout<<"pub: v_x: "<<v->velocity_x<<"v_a: "<<v->velocity_angle<<std::endl;
        //LOG(INFO)<<"pub: v_x: "<<v->velocity_x<<"v_a: "<<v->velocity_angle<<std::endl;
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
#ifdef USE_DEBUG
        std::chrono::steady_clock::time_point debug_now_timestamp;
        std::chrono::steady_clock::time_point debug_last_timestamp;
        debug_now_timestamp = std::chrono::steady_clock::now();
        debug_last_timestamp = std::chrono::steady_clock::now();
#endif
        last_mark_timestamp = now_mark_timestamp;
        int time_used_u;
        stop = true;
        int times = 0;
        double period = 1.0/(double)boat_params_.frequency;
        uint32_t socket_send_count = 0;
        uint32_t serial_send_count = 0;
        uint32_t count = 0;
        while(main_thread_){
#ifdef DEBUG
            debug_now_timestamp = std::chrono::steady_clock::now();
            std::chrono::duration<double> debug_time_used = std::chrono::duration_cast<std::chrono::duration<double>>(debug_now_timestamp-debug_last_timestamp);
            debug_last_timestamp = debug_now_timestamp;
            double debug_time_count = debug_time_used.count();
            if(debug_time_count>0.0065){
                std::cout<<"time used: "<<debug_time_count<<std::endl;
                LOG(WARNING)<<"time used: "<<debug_time_count<<std::endl;
            }
#endif
            last_timestamp = std::chrono::steady_clock::now();
            count++;
            socket_send_count++;
            serial_send_count++;
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
            //AnalysisRemoteInfo(remote_channel_data_main_thread_, &stop);
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
            //LOG(INFO)<<"boat_mode: "<<boat_mode_<<std::endl;
            if(boat_mode_ == navigation_mode){
                control_power_trans_.host = 1;
                //LOG(INFO)<<"Navi calc before "<<now_state_.attitude_angle;
                NavigationCalculation();
                NavigationVelocityAnalyze_(yaw, velocity_data_);
                //std::cout<<" yaw: "<<yaw<<std::endl;
                //LOG(INFO)<<"Navi calc after "<<now_state_.attitude_angle;
                if(route_updated_){
                    pthread_mutex_lock(route_updated_mutex_ptr_);
                    UpdateLocusPoints_(locus_points_main_thread_, 0);
                    pthread_mutex_unlock(route_updated_mutex_ptr_);
                    route_updated_ = 0;
                }
                //LOG(INFO)<<"Navi anal";
            }
            else if(boat_mode_ == remote_mode){
                RemoteVelocityAnalyze_(remote_channel_data_main_thread_, &velocity_data_);
            }
            else if(boat_mode_ == track_mode){
            }
//            else if(boat_mode_ == attack_mode){
//                RemoteVelocityAnalyze_(remote_channel_data_main_thread_, &velocity_data_);
//                control_power_trans_.host = 3;
//            }
            if(behavior_ & kAttack){
                RemoteVelocityAnalyze_(remote_channel_data_main_thread_, &velocity_data_);
                //control_power_trans_.host = 3;
            }
            if(stop){
                velocity_data_.angular_velocity.x = 0.0;
                velocity_data_.angular_velocity.y = 0.0;
                velocity_data_.angular_velocity.z = 0.0;
                velocity_data_.linear_velocity.x = 0.0;
                velocity_data_.linear_velocity.y = 0.0;
                velocity_data_.linear_velocity.z = 0.0;
                //ReSetMarkPointFlag();
            }
            if(serial_send_count == boat_params_.frequency/boat_params_.serialParams.send_frequency){
                serial_send_count = 0;
                VelocityPublish_(velocity_data_);
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)50));
                ControlPowerPublish_(control_power_trans_);
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)50));
                EmpowerPublish_(empower_trans_);
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)50));
                ModePublish_(boat_mode_);
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)50));
                BehaviorPublish_(behavior_);
            }
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