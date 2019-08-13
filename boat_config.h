//
// Created by wumode on 19-6-19.
//

#ifndef SHIP_BOAT_CONFIG_H
#define SHIP_BOAT_CONFIG_H

#ifndef __SYSTEM_STM32F10X_H
typedef unsigned char u8;
#endif


//IMU数据
typedef struct ImuDataTrans{
    volatile float roll;
    volatile float pitch;
    volatile float yaw;
    volatile float linear_acceleration_x;
    volatile float linear_acceleration_y;
    volatile float angular_velocity_z;
}ImuDataTrans;
//IMU功能字
#define IMU_FLAG 0xf1

//GPS数据
typedef struct GpsDataTrans {
    volatile float latitude;
    volatile float longitude;
    volatile float speed;
}GpsDataTrans;
//GPS功能字
#define GPS_FLAG 0xf2

//速度数据
typedef struct VelocityDataTrans {
    volatile float velocity_x;
    volatile float velocity_angle;
}VelocityDataTrans;
//速度功能字
#define VELOCITY_FLAG 0xf3
//速度数据长度

//控制权数据
typedef struct ControlPowerTrans {
    uint8_t host;    //1 树莓派, 2 OpenMV
}ControlPowerTrans;
//控制权功能字
#define CONTROL_POWER_FLAG 0xf4

typedef struct RemoteChannelTrans{
    uint16_t channel_1;
    uint16_t channel_2;
    uint16_t channel_3;
    uint16_t channel_4;
}RemoteChannelTrans;
#define REMOTE_CHANNEL_FLAG 0xf5

typedef struct EmpowerTrans{
    volatile uint8_t empower;
}EmpowerTrans;
#define EMPOWER_FLAG 0xf6

typedef struct LockingTrans{
    uint8_t locking;
}LockingTrans;
#define LOCKING_FLAG 0xf7

typedef struct InitializedTrans{
    uint8_t initialized;
}InitializedTrans;
#define INITIALIZED_FLAG 0xf8

typedef struct StopTrans{
    volatile uint8_t stop;
}StopTrans;
#define STOP_FLAG 0xf9

typedef struct SocketShow{
    std::vector<GpsPosition> route_gps_positions;
    GpsPosition gps_position;
    GpsPosition raw_gps_position;
    UtmPosition utm_position;
    uint8_t locking;
    ImuData imu_data;
    float speed;
}SocketShow;


typedef struct SocketReceive{
    std::vector<GpsPosition> route_gps_positions;
    uint8_t mode;
    uint8_t empower;
    uint8_t route_updated;
    uint8_t stop;
}SocketReceive;

typedef struct SerialParams{
    std::string port;
    uint32_t baud;
    uint32_t send_frequency;
}SerialParams;

typedef struct SocketParams{
    std::string host;
    uint16_t port;
    uint32_t send_frequency;
}SocketParams;

typedef struct GpsInitialization{
    uint8_t hardware_flag;
    GpsPosition gpsPosition;
}GpsInitialization;

typedef enum BoatMode{
    remote_mode = 1,
    navigation_mode,
    track_mode,
    attack_mode
}BoatMode;

typedef struct BoatParams{
    SerialParams serialParams;
    SocketParams socketParams;
    BoatMode boatMode;
    uint32_t frequency;
    GpsInitialization gpsInitialization;
}BoatParams;

#endif //SHIP_BOAT_CONFIG_H
