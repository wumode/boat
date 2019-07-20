//
// Created by wumode on 19-7-16.
//

#include <iostream>
#include <json.hpp>
#include <socket_communication.h>

using nlohmann::json;

typedef struct GpsPosition{
    float latitude;
    float longitude;
}GpsPosition;

typedef struct UtmPosition{
    float x;
    float y;
}UtmPosition;

typedef struct LinearAcceleration{
    volatile float x;
    volatile float y;
    //float z;
}LinearAcceleration;

typedef struct LinearVelocity{
    float x;
    float y;
    //float z;
}LinearVelocity;

typedef struct AngularVelocity{
    //float x;
    //float y;
    volatile float z;
}AngularVelocity;

typedef struct Angle{
    volatile float roll;
    volatile float pitch;
    volatile float yaw;
}Angle;

typedef struct ImuData{
    Angle angle;
    AngularVelocity angular_velocity;
    LinearAcceleration linear_acceleration;
}ImuData;


struct SocketShow{
    std::vector<GpsPosition> route_gps_positions;
    GpsPosition gps_position;
    UtmPosition utm_position;
    uint8_t locking;
    ImuData imu_data;
};


struct SocketReceive{
    std::vector<GpsPosition> route_gps_positions;
    uint8_t mode;
    uint8_t empower;
    uint8_t route_updated;
    uint8_t stop;
};


void to_json(json& j, const GpsPosition& gpsPosition){
    j = json{{"latitude", gpsPosition.latitude}, {"longitude", gpsPosition.longitude}};
}

void from_json(const json& j, GpsPosition& gpsPosition){
    gpsPosition.longitude = j.at("longitude").get<float>();
    gpsPosition.latitude = j.at("latitude").get<float>();
}

void to_json(json& j, const UtmPosition& utmPosition){
    j = json{{"x", utmPosition.x}, {"y", utmPosition.y}};
}

void from_json(const json& j, UtmPosition& utmPosition){
    utmPosition.x = j.at("x").get<float>();
    utmPosition.y = j.at("y").get<float>();
}

void to_json(json& j, const LinearAcceleration& linearAcceleration){
    j = json{{"x", linearAcceleration.x}, {"y", linearAcceleration.y}};
}

void from_json(const json& j, LinearAcceleration& linearAcceleration){
    linearAcceleration.x = j.at("x").get<float>();
    linearAcceleration.y = j.at("y").get<float>();
}

void to_json(json& j, const AngularVelocity& angular_velocity){
    j = json{"z", angular_velocity.z};
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
             {"utm_position", socketShow.utm_position},
             {"route_gps_positions", socketShow.route_gps_positions},
             {"locking", socketShow.locking},
             {"imu_data", socketShow.imu_data}};
}

void from_json(const json& j, SocketShow& socketShow){
    socketShow.imu_data = j.at("imu_data").get<ImuData>();
    socketShow.gps_position = j.at("gps_position").get<GpsPosition>();
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



std::string j_r_= R"({
  "empower": 0,
  "mode": 1,
  "stop": 0,
  "route_updated": 0,
  "route_gps_positions": [
    {
      "latitude": 0.0,
      "longitude": 0.0
    },
    {
      "latitude": 0.0,
      "longitude": 0.0
    }
  ]
}
)";

class SocketTest{
public:
    SocketTest();
    static void TestCallback(uint8_t* buffer_ptr_, void* __this);

private:
    void SendTest();

    socket_communication::SocketCommunication* socket_com_test_ptr_;
    std::string j_s_;
};

void SocketTest::TestCallback(uint8_t *buffer_ptr_, void *__this) {
    auto* _this = (SocketTest*)__this;
    std::string string_rec = (const char*)buffer_ptr_;
    json j = json::parse(string_rec);
    SocketReceive ps = j;
    std::cout<<ps.route_gps_positions.size()<<std::endl;
}


SocketTest::SocketTest() {
    //std::string host = "localhost";
    socket_com_test_ptr_ = new socket_communication::SocketCommunication("localhost", 8888);
    socket_com_test_ptr_->SetCallBackFunction((socket_communication::callBack)TestCallback, 1, this);
    if(!socket_com_test_ptr_->StartSocketReceiveThread()){
        exit(-1);
    }
    SendTest();
}

void SocketTest::SendTest() {
    while(1){
        std::string j_s_ = R"({
  "gps_position": {
    "latitude": 1.0,
    "longitude": 0.0
  },
  "imu_data": {
    "angle": {
      "pitch": 20.0,
      "roll": 3.0,
      "yaw": 20.0
    },
    "angular_velocity": {
      "z": 0.0
    },
    "linear_acceleration": {
      "x": 0.0,
      "y": 0.0
    }
  },
  "locking": 2,
  "route_gps_positions": [
    {
      "latitude": 3.0,
      "longitude": 0.0
    },
    {
      "latitude": 4.0,
      "longitude": 0.0
    }
  ],
  "utm_position": {
    "x": 0.0,
    "y": 2.0
  }
}
)";
        json j = json::parse(j_s_);
        SocketShow s_show;
        s_show = j;
        socket_com_test_ptr_->SendData(s_show, 0x01);
        std::cout<<"send"<<std::endl;
        std::this_thread::sleep_for(std::chrono:: microseconds (50000));
    }
}

std::string j_l_ = R"({
  "gps_position": {
    "latitude": 1.0,
    "longitude": 0.0
  },
  "imu_data": {
    "angle": {
      "pitch": 20.0,
      "roll": 3.0,
      "yaw": 20.0
    },
    "angular_velocity": {
      "z": 0.0
    },
    "linear_acceleration": {
      "x": 0.0,
      "y": 0.0
    }
  },
  "locking": 2,
  "route_gps_positions": [
    {
      "latitude": 3.0,
      "longitude": 0.0
    },
    {
      "latitude": 4.0,
      "longitude": 0.0
    }
  ],
  "utm_position": {
    "x": 0.0,
    "y": 2.0
  }
}
)";

enum test_emum{
    test1 = 1,
    test2,
    test3
};

int main(int argc, char *args[])
{
    std::cout<<"hello world"<<std::endl;
    test_emum Tt;
    Tt = test1;
    int a = 2;
    switch (a){
        case (int)test1:
        case (int)test2:
        case (int)test3:
            Tt = (test_emum)a;
            break;
        default:
            std::cout<<"err"<<std::endl;
            break;
    }
    std::cout<<Tt<<std::endl;
    //json j = json::parse(j_l_);
    //SocketShow ps = j;
    //SocketTest s_t;
    return 0;
}