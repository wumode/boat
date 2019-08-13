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

#ifndef SHIP_NAVIGATION_CONFIG_H
#define SHIP_NAVIGATION_CONFIG_H

#include <cmath>
#include <cstdio>
#include <utm.h>
#include <datum.h>
#include <glog/logging.h>

#ifndef M_PI
#define M_PI 3.1415926535f
#endif


typedef uint32_t Flag;

typedef struct LinearAcceleration{
    volatile float x;
    volatile float y;
    //float z;
}LinearAcceleration;

typedef struct LinearVelocity{
    volatile float x;
    volatile float y;
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

typedef struct GpsPosition{
    volatile float latitude;
    volatile float longitude;
}GpsPosition;

typedef struct UtmPosition{
    float x;
    float y;
    GridZone gridZone;
    Hemisphere hemisphere;
}UtmPosition;

typedef struct GpsData
{
    GpsPosition gps_position;
    volatile float speed;
}GpsData;

typedef struct StateVector{
    UtmPosition position;                //UTM坐标
    float attitude_angle;               //姿态角
    LinearVelocity line_velocity;       //线速度
    AngularVelocity angular_velocity;   //角速度
}StateVector;

typedef struct MeasurementVector
{
    UtmPosition position;               //UTM坐标
    ImuData imu_data;                   //IMU数据
    //float attitude_angle;
}MeasurementVector;

typedef struct VelocityData{
    volatile float velocity_x;
    volatile float velocity_angle;
}VelocityData;


typedef struct NavigationParameter{
    float steering_coefficient;
    float corner_threshold;
    float min_distance;
    float base_velocity;
    float steering_deceleration_coefficient;    //转向减速系数
}NavigationParameter;

typedef struct MarkPointParameter{
    int mark_flag;
    float period;
}MarkPointParameter;

inline void GpsToUtm(const GpsPosition* gps_position, UtmPosition* utm_position){
    //LOG(INFO)<<"gps to utm latitude: "<<gps_position->latitude<<" longitude: "<<gps_position->longitude<<std::endl;
    double lat = gps_position->latitude/180.0*M_PI;
    double lon = gps_position->longitude/180.0*M_PI;
    double N, E;
    //GridZone zone = GRID_AUTO;
    //Hemisphere hemi = HEMI_AUTO;
    const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
    geographic_to_grid(e->a, e->e2, lat, lon, &utm_position->gridZone, &utm_position->hemisphere, &N, &E);
    utm_position->x = (float)E;
    utm_position->y = (float)N;
    //LOG(INFO)<<"gps to utm x: "<<utm_position->x<<" y: "<<utm_position->y<<" zone: "<<utm_position->gridZone<<" hemi: "<<utm_position->hemisphere;
}

inline void UtmToGps(const UtmPosition* utm_position, GpsPosition* gps_position){
    double N = utm_position->y;
    double E = utm_position->x;
    double lat;
    double lon;
    const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
    grid_to_geographic(e->a, e->e2, utm_position->gridZone, utm_position->hemisphere, N, E, &lat, &lon);
    gps_position->latitude = (float)(lat/M_PI*180.0f);
    gps_position->longitude = (float)(lon/M_PI*180.0f);
}

inline void GpsToUtmPartition(const GpsPosition* gpsPosition, GridZone* gridZone, Hemisphere* hemisphere){
    double lat = gpsPosition->latitude/180.0*M_PI;
    double lon = gpsPosition->longitude/180.0*M_PI;
    double N, E;
    const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
    geographic_to_grid(e->a, e->e2, lat, lon, gridZone, hemisphere, &N, &E);
}

inline float CalcDistanceUtm(UtmPosition* key_position,UtmPosition* now_position){
    float a,b;
    a = key_position->x - now_position->x;
    b = key_position->y - now_position->y;
    return sqrt(a*a+b*b);
}

inline float CalcAngleUtm(UtmPosition* key_position, UtmPosition* now_position){
    float angle;
    angle = atan2(key_position->y-now_position->y, key_position->x-now_position->x);
    if(angle<0){
        angle += 2*M_PI;
    }
    return angle;
}

inline float CalcYaw(const float* route_angle, const float* attitude_angle){
    float attitude = *attitude_angle;
    if(fabs(attitude)>1024*M_PI){
        return 0.0;
    }
    while((attitude)<0){
        (attitude) += 2*M_PI;
    }
    while((attitude)>2*M_PI){
        (attitude) -= 2*M_PI;
    }
    float yaw;
    yaw = *route_angle - attitude;
    if(yaw>M_PI){
        yaw -= 2 * M_PI;
    } else if(yaw<-M_PI){
        yaw += 2 * M_PI;
    }
    //printf("yaw: %0.5f\n", yaw);
    return yaw;
}
#endif //SHIP_NAVIGATION_CONFIG_H
