//
// Created by wumode on 19-8-19.
//

#include "point.h"


namespace navigation{
    namespace point{
        double Distance(Point* point1, Point* point2){
            double a,b;
            UtmPosition u1, u2;
            u1 = point1->Utm();
            u2 = point2->Utm();
            a = u1.x - u2.x;
            b = u1.y - u2.y;
            return sqrt(a*a+b*b);
        }

        double CalcAngle(Point* end_point, Point* starting_point){
            double angle;
            UtmPosition u1 = starting_point->Utm();
            UtmPosition u2 = end_point->Utm();
            angle = atan2(u2.y-u1.y, u2.x-u1.x);
            if(angle<0){
                angle += 2*M_PI;
            }
            return angle;
        }


        Point::Point() {
            utm_.x = 0.0;
            utm_.y = 0.0;
            gps_.latitude = 0.0;
            gps_.longitude = 0.0;
            initialized_ = false;
        }
        Point::Point(const navigation::GpsPosition& gpsPosition) {
            gps_ = gpsPosition;
            navigation::GpsToUtm(&gps_, &utm_);
            initialized_ = true;
        }

        Point::Point(const navigation::UtmPosition& utmPosition) {
            utm_ = utmPosition;
            navigation::UtmToGps(&utm_, &gps_);
            initialized_ = true;
        }

        Point::Point(double longitude, double latitude){
            gps_.longitude = longitude;
            gps_.latitude = latitude;
            navigation::GpsToUtm(&gps_, &utm_);
            initialized_ = true;
        }

        Point::Point(double x, double y, GridZone zone, Hemisphere hemisphere) {
            utm_.y = y;
            utm_.x = x;
            utm_.hemisphere = hemisphere;
            utm_.gridZone = zone;
            navigation::UtmToGps(&utm_, &gps_);
            initialized_ = true;
        }

        Point::Point(const navigation::point::Point &P) {
            utm_ = P.Utm();
            gps_ = P.Gps();
            initialized_ = P.Initialized();
            //std::cout<<"copy"<<std::endl;
        }

        UtmPosition Point::Utm() const {
            return utm_;
        }

         GpsPosition Point::Gps() const {
            return gps_;
        }

        bool Point::Initialized() const {
            return initialized_;
        }

        Point& Point::operator=(const Point &P) {
            gps_ = P.Gps();
            utm_ = P.Utm();
            initialized_ = P.Initialized();
            //std::cout<<"operator ="<<std::endl;
            return *this;
        }

        Point& Point::operator=(const UtmPosition &utmPosition) {
            utm_ = utmPosition;
            UtmToGps();
            initialized_ = true;
            return *this;
        }

        Point& Point::operator=(const GpsPosition& gpsPosition) {
            gps_ = gpsPosition;
            GpsToUtm();
            initialized_ = true;
            return *this;
        }

        std::ostream &operator<<(std::ostream &output, const Point &P) {
            output<<"gps longitude: "<<std::fixed << std::setprecision(7)<<P.gps_.longitude<<" latitude: "<<P.gps_.latitude<<std::endl;
            output<<"utm x: "<<P.utm_.x<<" y: "<<P.utm_.x<<" grid zone: "<<P.utm_.gridZone<<" hemisphere: "<<P.utm_.hemisphere;
            return output;
        }

        void Point::GpsToUtm() {
            double lat = gps_.latitude/180.0*M_PI;
            double lon = gps_.longitude/180.0*M_PI;
            const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
            geographic_to_grid(e->a, e->e2, lat, lon, &utm_.gridZone, &utm_.hemisphere, &utm_.y, &utm_.x);
        }

        void Point::UtmToGps() {
            double lat;
            double lon;
            const Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
            grid_to_geographic(e->a, e->e2, utm_.gridZone, utm_.hemisphere, utm_.y, utm_.x, &lat, &lon);
            gps_.latitude = lat/M_PI*180.0;
            gps_.longitude = lon/M_PI*180.0;
        }

        Point::~Point()=default;
    }
}
