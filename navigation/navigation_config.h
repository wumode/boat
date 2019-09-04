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
#include "point/point.h"


namespace navigation{

    typedef struct LinearAcceleration{
        double x;
        double y;
        double z;
    }LinearAcceleration;

    typedef struct LinearVelocity{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    }LinearVelocity;

    typedef struct AngularVelocity{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    }AngularVelocity;

    typedef struct fLinearAcceleration{
        float x;
        float y;
        float z;
    }fLinearAcceleration;

    typedef struct fLinearVelocity{
        float x;
        float y;
        float z;
    }fLinearVelocity;

    typedef struct fAngularVelocity{
        float x;
        float y;
        float z;
    }fAngularVelocity;

    typedef struct Velocity{
        AngularVelocity angular_velocity;
        LinearVelocity linear_velocity;
    }Velocity;

    typedef struct ImuData{
        navigation::Angle angle;
        navigation::AngularVelocity angular_velocity;
        navigation::LinearAcceleration linear_acceleration;
    }ImuData;

    typedef struct fGpsPosition{
        volatile float latitude = 0.0;
        volatile float longitude = 0.0;
    }fGpsPosition;

    typedef struct GpsData
    {
        navigation::GpsPosition gps_position;
        volatile double speed;
    }GpsData;

    typedef struct StateVector{
        //navigation::UtmPosition position;                //UTM坐标
        navigation::Position position;
        Angle angle;              //姿态角
        LinearVelocity line_velocity;       //线速度
        AngularVelocity angular_velocity;   //角速度
    }StateVector;

typedef struct MeasurementVector
{
    navigation::UtmPosition position;               //UTM坐标
    ImuData imu_data;                   //IMU数据
    //float attitude_angle;
}MeasurementVector;


typedef struct NavigationParameter{
    double steering_coefficient;
    double corner_threshold;
    double min_distance;
    double base_velocity;
    double steering_deceleration_coefficient;    //转向减速系数
}NavigationParameter;

typedef struct MarkPointParameter{
    int mark_flag;
    double period;
}MarkPointParameter;

inline void GpsToUtmPartition(const GpsPosition* gpsPosition, GridZone* gridZone, Hemisphere* hemisphere){
    double lat = gpsPosition->latitude/180.0*M_PI;
    double lon = gpsPosition->longitude/180.0*M_PI;
    double N, E;
    const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
    geographic_to_grid(e->a, e->e2, lat, lon, gridZone, hemisphere, &N, &E);
}

inline double CalcDistanceUtm(UtmPosition* key_position,UtmPosition* now_position){
    double a,b;
    a = key_position->x - now_position->x;
    b = key_position->y - now_position->y;
    return sqrt(a*a+b*b);
}


inline double CalcYaw(const double& route_angle, const double& attitude_angle){
    double attitude = attitude_angle;
    if(fabs(attitude)>10000*M_PI){
        return 0.0;
    }
    while((attitude)<0){
        (attitude) += 2*M_PI;
    }
    while((attitude)>2*M_PI){
        (attitude) -= 2*M_PI;
    }
    double yaw;
    yaw = route_angle - attitude;
    if(yaw>M_PI){
        yaw -= 2 * M_PI;
    } else if(yaw<-M_PI){
        yaw += 2 * M_PI;
    }
    return yaw;
}
}
#endif //SHIP_NAVIGATION_CONFIG_H
