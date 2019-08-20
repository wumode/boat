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
// Created by wumode on 19-8-19.
//

#ifndef BOAT_POINT_H
#define BOAT_POINT_H
#include <utm.h>
#include <datum.h>
#include <cmath>
#include <iostream>
#include <iomanip>


namespace navigation
{
    typedef struct GpsPosition{
        volatile double latitude = 0.0;
        volatile double longitude = 0.0;
    }GpsPosition;

    typedef struct UtmPosition{
        double x = 0.0;
        double y = 0.0;
        GridZone gridZone = GRID_AUTO;
        Hemisphere hemisphere = HEMI_AUTO;
    }UtmPosition;

    inline void GpsToUtm(const GpsPosition* gps_position, UtmPosition* utm_position){
        double lat = gps_position->latitude/180.0*M_PI;
        double lon = gps_position->longitude/180.0*M_PI;
        double N, E;

        const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
        geographic_to_grid(e->a, e->e2, lat, lon, &utm_position->gridZone, &utm_position->hemisphere, &N, &E);
        utm_position->x = E;
        utm_position->y = N;
    }

    inline void UtmToGps(const UtmPosition* utm_position, GpsPosition* gps_position){
        double N = utm_position->y;
        double E = utm_position->x;
        double lat;
        double lon;
        const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
        grid_to_geographic(e->a, e->e2, utm_position->gridZone, utm_position->hemisphere, N, E, &lat, &lon);
        gps_position->latitude = lat/M_PI*180.0;
        gps_position->longitude = lon/M_PI*180.0;
    }


    inline double CalcAngleUtm(UtmPosition* key_position, UtmPosition* now_position){
        float angle;
        angle = atan2(key_position->y-now_position->y, key_position->x-now_position->x);
        if(angle<0){
            angle += 2*M_PI;
        }
        return angle;
    }

    namespace point{

        enum class CoordinateSystem{
            kWgs84Coordinate = 1,
            kUtmCoordinate = 2
        };

        class Point {
        public:
            Point();
            explicit Point(const navigation::GpsPosition& gpsPosition);
            explicit Point(const navigation::UtmPosition& utmPosition);
            Point(double longitude, double latitude);
            Point(double x, double y, GridZone zone, Hemisphere hemisphere);
            Point(const Point &P);
            ~Point();
            Point& operator=(const Point &P);
            Point& operator=(const UtmPosition& utmPosition);
            Point& operator=(const GpsPosition& gpsPosition);
            friend std::ostream &operator<<(std::ostream &output, const Point &D );
            UtmPosition Utm () const;
            GpsPosition Gps() const;
            bool Initialized() const;
        private:
            bool initialized_;
            GpsPosition gps_;
            UtmPosition utm_;
            void GpsToUtm();
            void UtmToGps();
        };

        double Distance(Point* point1, Point* point2);
        double CalcAngle(Point* end_point, Point* starting_point);
    }
}

#endif //BOAT_POINT_H
