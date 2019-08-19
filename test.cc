//
// Created by wumode on 19-8-19.
//

#include "serial_communication.h"

#include <iostream>
#include <chrono>
//#include <glog/logging.h>

typedef struct TestMsg{
    uint32_t a;
    char b;
}TestMsg;

static void TestCallback(uint8_t* buffer_ptr_, void* __this){
    std::cout<<"call back"<<std::endl;
    TestMsg* testMsg;
    testMsg = (TestMsg*)__this;
    memcpy(testMsg, buffer_ptr_, sizeof(TestMsg));
    std::cout<<"rec: "<<testMsg->a<<" "<<(int)testMsg->b<<std::endl;
}


int main(int argc, char* argv[]){
    TestMsg t_m_s;
    TestMsg t_m_r;
    t_m_s.a = 15;
    t_m_s.b = 16;
    uint8_t flag = 0xf1;
    //google::InitGoogleLogging((const char *)argv[0]);
    //google::SetLogDestination(google::GLOG_INFO, "./log/log");
    std::cout<<"a"<<std::endl;
    serial_communication::SerialCommunication ser;
    ser.SetCallBackFunction((serial_communication::callBack)TestCallback, flag, (void*)&t_m_r);
    ser.StartSerialReceiveThread("/dev/ttyUSB0", 460800);

    while(1){
        ser.SendData(t_m_s, flag);
        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)500));
    }

//    uint8_t serial_data_buffer[SERIAL_SIZE];
//    bool serial_thread_ = true;
//    int size_read;
//    serial::Serial* ser_ptr_;
//    ser_ptr_ = new serial::Serial;
//    std::string port = "/dev/ttyUSB0";
//    uint32_t baud_rate = 460800;
//    ser_ptr_->setPort(port);
//    ser_ptr_->setBaudrate(baud_rate);
//    serial::Timeout to = serial::Timeout::simpleTimeout(serial::Timeout::max());
//    ser_ptr_->setTimeout(to);
//    ser_ptr_->setParity(serial::parity_none);
//    ser_ptr_->setStopbits(serial::stopbits_one);
//    ser_ptr_->open();
//    try {
//        while (serial_thread_){
//            size_read = ser_ptr_->read(serial_data_buffer,1);
//            if(size_read){
//                std::cout<<(int)serial_data_buffer[0];
//            }
//
//            else{
//                ser_ptr_->flushInput();
//            }
//            std::cout<<"while"<<std::endl;
//        }
//    }
//    catch (serial::SerialException& e){
//        std::cerr<<"err"<<std::endl;
//    }
    //google::ShutdownGoogleLogging();
}