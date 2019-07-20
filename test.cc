//
// Created by wumode on 19-4-11.
//

/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <chrono>
#include <tinyxml.h>
#include <json.hpp>
#include "navigation/navigation_config.h"

#include <map>
#include <serial_communication.h>
using namespace std;

typedef struct VelocityDataTrans {
    volatile float velocity_x;
    volatile float velocity_angle;
}VelocityDataTrans;

using json = nlohmann::json;


class main_c{
public:
    main_c();
    serial_communication::SerialCommunication s_c;
    static int onmyevent(uint8_t* p,void* aa); //被回调函数
    void* return_this();
    void setCallback();
    void SendDataToSelf();
    template <typename T> void template_test(const T& t);
    VelocityDataTrans v_t;
};

template <typename T>
void main_c::template_test(const T &t) {
    std::cout<< sizeof(T)<<std::endl;
}

main_c::main_c() {
    v_t.velocity_angle = 1.2;
    v_t.velocity_x= 2.2;
    s_c.SetComPort("/dev/ttyUSB0");
    s_c.SetBaudRate(921600);
    s_c.StartSerialReceiveThread();
}

void main_c::SendDataToSelf() {
    int i = 0;
    while(i<5){
        s_c.SendData(v_t, 0xf1);
        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)5000));
        i++;
    }
    s_c.CloseSerialReceiveThread();
}

int main_c::onmyevent(uint8_t * p,void* aa)
{
    // 将回调回来的指针强制转换为类指针，然后调用类的成员函数
    auto* _this = (main_c*)aa;
    auto* v_t_ptr_ = (VelocityDataTrans*)p;
    _this->v_t.velocity_x = v_t_ptr_->velocity_x;
    _this->v_t.velocity_angle = v_t_ptr_->velocity_angle;
    std::cout<<_this->v_t.velocity_angle<<std::endl;
    return 0;
}

void main_c::setCallback() {
    //callBack pf = (callBack)(onmyevent);
    s_c.SetCallBackFunction((serial_communication::callBack)(onmyevent), 0xf1, this);

    //s_c.SetCallBackFunction((callBack)(onmyevent), 0xff, this);
    //std::cout<<s_c.callback_function_directory_.size()<<std::endl;
    //s_c.setCallBackThis((callBack)(onmyevent), this);
}

void* main_c::return_this() {
    return (void*) this;
}


void load_xml_test1(){
    TiXmlDocument config_xml;
    std::string config_xml_path = "config.xml";
    if (!config_xml.LoadFile(config_xml_path.c_str()))
    {
        std::cerr << "Can load file: " << config_xml_path << std::endl;
        config_xml.Clear();
        return;
    }
    TiXmlElement* xmlRoot = config_xml.RootElement();
    if (xmlRoot == nullptr)
    {
        std::cerr << "Failed to load file: No root element." << std::endl;
        return;
    }
    TiXmlElement* mode_xml = xmlRoot->FirstChildElement("mode");
    int mode_flag = std::stoi(mode_xml->GetText());
    std::cout<<mode_flag<<std::endl;
}

void load_xml_test2(){
    TiXmlDocument config_xml;
    std::string config_xml_path = "config.xml";
    if (!config_xml.LoadFile(config_xml_path.c_str()))
    {
        std::cerr << "Can load file: " << config_xml_path << std::endl;
        config_xml.Clear();
        return;
    }
    TiXmlElement* xmlRoot = config_xml.RootElement();
    if (xmlRoot == nullptr)
    {
        std::cerr << "Failed to load file: No root element." << std::endl;
        return;
    }
    TiXmlElement* mode_xml = xmlRoot->FirstChildElement("mode");
    int mode_flag = std::stoi(mode_xml->GetText());
    std::cout<<mode_flag<<std::endl;
}

int main() {
    load_xml_test1();
    load_xml_test2();
//    serial_communication::SerialCommunication ser("/dev/ttyUSB0", 921600);
//    main_c m_c;
//    m_c.setCallback();
//    GpsPosition gg;
//    gg.latitude = 30.9007846;
//    gg.longitude = 121.9371962;
//    UtmPosition uu;
//    GpsToUtm(&gg, &uu);
//    std::cout<<uu.gridZone<<std::endl;
//    while (true){
//        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)5000));
//    }
    //m_c.SendDataToSelf();
    //VelocityDataTrans mm;
    //m_c.template_test(mm);
    //m_c.SendDataToSelf();
    //m_c.mm = 48;
    //m_c.s_c.ser_recv();
    //double num_a;
    //m_c.s_c.SendData(num_a);
    //return 0;

    // 创建一个json对象(null)

    return 0;
}
