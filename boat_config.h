//
// Created by wumode on 19-6-19.
//

#ifndef SHIP_BOAT_CONFIG_H
#define SHIP_BOAT_CONFIG_H

#include "config.h"

//IMU数据
namespace navigation{
    typedef enum SerialDataType{
        kIMU_FLAG = 0xf1,
        kGPS_FLAG = 0xf2,
        kVELOCITY_FLAG = 0xf3,
        kCONTROL_POWER_FLAG = 0xf4,
        kREMOTE_CHANNEL_FLAG = 0xf5,
        kEMPOWER_FLAG = 0xf6,
        kLOCKING_FLAG = 0xf7,
        kINITIALIZED_FLAG = 0xf8,
        kSTOP_FLAG = 0xf9,
        kMODE_FLAG = 0xfa,
        kBEHAVIOR_FLAG = 0xfb
    }SerialDataType;
    typedef struct ImuDataTrans{
        volatile float roll;
        volatile float pitch;
        volatile float yaw;
        volatile float linear_acceleration_x;
        volatile float linear_acceleration_y;
        volatile float angular_velocity_z;
    }ImuDataTrans;
//IMU功能字
    typedef struct GpsDataTrans {
        volatile float latitude;
        volatile float longitude;
        volatile float speed;
    }GpsDataTrans;

    typedef struct VelocityDataTrans {
        volatile float velocity_x;
        volatile float velocity_angle;
    }VelocityDataTrans;

    typedef struct ControlPowerTrans {
        uint8_t host;    //1 树莓派, 2 OpenMV
    }ControlPowerTrans;
//控制权功能字

    typedef struct RemoteChannelTrans{
        uint16_t channel_1;
        uint16_t channel_2;
        uint16_t channel_3;
        uint16_t channel_4;
    }RemoteChannelTrans;

    typedef struct EmpowerTrans{
        volatile uint8_t empower;
    }EmpowerTrans;

    typedef struct LockingTrans{
        uint8_t locking;
    }LockingTrans;

    typedef struct InitializedTrans{
        uint8_t initialized;
    }InitializedTrans;


    typedef struct StopTrans{
        volatile uint8_t stop;
    }StopTrans;

    typedef struct ModeTrans{
        uint8_t mode;
    }ModeTrans;

    typedef struct BehaviorTrans{
        uint8_t behavior;
    }BehaviorTrans;

    typedef struct SocketShow{
        std::vector<navigation::GpsPosition> route_gps_positions;
        navigation::GpsPosition gps_position;
        navigation::GpsPosition raw_gps_position;
        navigation::UtmPosition utm_position;
        uint8_t locking;
        navigation::ImuData imu_data;
        double speed;
        uint8_t id;
        std::vector<uint8_t> receiver_id;
    }SocketShow;

    typedef struct SocketReceive{
        std::vector<navigation::GpsPosition> route_gps_positions;
        uint8_t mode;
        uint8_t empower;
        uint8_t route_updated;
        uint8_t stop;
        uint8_t behavior;
        std::vector<uint8_t> receiver_id;
    }SocketReceive;

    typedef enum SocketHeader{
        kHandShake1 = 1,
        kHandShake2 = 2,
        kHandShake3 = 3,
        kNormalData = 4
    }SocketHeader;

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
        navigation::GpsPosition gpsPosition;
    }GpsInitialization;

    typedef enum BoatMode{
        remote_mode = 1,
        navigation_mode,
        track_mode,
        attack_mode,
        kDynamicPositioningMode
    }BoatMode;

    typedef struct BoatParams{
        uint8_t id;
        SerialParams serialParams;
        SocketParams socketParams;
        BoatMode boatMode;
        uint32_t frequency;
        GpsInitialization gpsInitialization;
    }BoatParams;

    typedef struct SocketHandShake{
        uint8_t identity;   /// 1 boat 2 server 3 user
        uint8_t id;
        uint8_t ok;
    }SocketHandShake;

    typedef struct SocketHandShake2{
        uint8_t identity;
        uint8_t error_type;
    }SocketHandShake2;

    typedef struct SocketHandShake3{
        uint8_t identity;
        std::vector<uint8_t> id_list;
    }SocketHandShake3;

    typedef enum SocketErrorType{
        kErrorFree = 0,
        kMissHandShake = 1
    }SocketErrorType;

    typedef enum Behavior{
        kNoBehavior = 0,
        kAttack = 1
    }Behavior;
}
#endif //SHIP_BOAT_CONFIG_H
