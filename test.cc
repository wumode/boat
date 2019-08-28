////
//// Created by wumode on 19-8-19.
////
//
//#include "serial_communication.h"
//
#include <iostream>
#include <algorithm.h>
//#include <chrono>
//#include <unistd.h>
//#include "point/point.h"
//#include "system_detection.h"
//#include "config.h"
//#ifdef USE_GLOG
//    #include <glog/logging.h>
//#endif
//
//typedef struct TestMsg{
//    uint32_t a;
//    char b;
//}TestMsg;
//
//static void TestCallback(uint8_t* buffer_ptr_, void* __this){
//    std::cout<<"call back"<<std::endl;
//    TestMsg* testMsg;
//    testMsg = (TestMsg*)__this;
//    memcpy(testMsg, buffer_ptr_, sizeof(TestMsg));
//    std::cout<<"rec: "<<testMsg->a<<" "<<(int)testMsg->b<<std::endl;
//}
//
//TestMsg test_return_fun(TestMsg& t) {
//    return t;
//}
//
//int& test_and(int& a){
//    return a;
//}
//
//int main(int argc, char* argv[]){
//#ifdef USE_GLOG
//    google::InitGoogleLogging((const char *)argv[0]);
//    google::SetLogDestination(google::GLOG_INFO, "./log/log");
//    LOG(INFO)<<"run";
//#endif
////    system_detection::CoreTemperature c1;
////    int i = 0;
////    for (int i = 0; i < 10; ++i) {
////        std::cout<<c1.Temperature()<<std::endl;
////        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)5000));
////    }
//
//    system_detection::SystemDetection s1;
//    std::cout<<s1.Temperature()<<std::endl;
//    printf("now pid is %d \n", getpid());
//    return 0;
////    TestMsg t_m_s;
////    TestMsg t_m_r;
////    t_m_s.a = 15;
////    t_m_s.b = 16;
////    uint8_t flag = 0xf1;
////    //google::InitGoogleLogging((const char *)argv[0]);
////    //google::SetLogDestination(google::GLOG_INFO, "./log/log");
////    std::cout<<"a"<<std::endl;
////    serial_communication::SerialCommunication ser;
////    ser.SetCallBackFunction((serial_communication::callBack)TestCallback, flag, (void*)&t_m_r);
////    ser.StartSerialReceiveThread("/dev/ttyUSB0", 460800);
////
////    while(1){
////        ser.SendData(t_m_s, flag);
////        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)500));
////    }
//    navigation::point::Point p1;
//    navigation::point::Point p2;
//    //p2 = p1;
//    navigation::UtmPosition u1, u2;
//    navigation::GpsPosition g1, g2;
//    g2.latitude = 37.0;
//    g2.longitude = 122.0;
//    g1.latitude = 37.001;
//    g1.longitude = 122.001;
//    p1 = g1;
//    p2 = g2;
//    u1 = p1.Utm();
//    u2 = p2.Utm();
//    u1.x = 10.0;
//    u2.x = 0.0;
//    u1.y = 10.0;
//    u2.y = 0.0;
//    p2 = u2;
//    p1 = u1;
//    std::cout<<p1<<std::endl;
//    std::cout<<p2<<std::endl;
//    std::cout<<navigation::point::Distance(&p1, &p2)<<std::endl;
//    std::cout<<navigation::point::CalcAngle(&p1, &p2)*180.0/M_PI<<std::endl;
//#ifdef USE_GLOG
//    google::ShutdownGoogleLogging();
//#endif
//}




int main(int argc, char *argv[])
{
    double p = 0;
    double i = 2.5;
    double d = 0.00;
    double up = 40.0;
    double low = -40.0;
    algorithm::PidController p1(p, i, d);
    p1.SetLimit(up, low);
    uint32_t n = 0;
    double me;
    double in;
    in = 0.0;
    me = 0.0;
    p1.SocketShow("127.0.0.1", 6800);
    //outfile.open("test.dat",  std::ios::out | std::ios::trunc);
    double target = 3.0;
    char str1[100];
    while(n<200){
        me = 10 * sin(0.05 * in);
        in = p1.Update(me, target);
        //sprintf(str1, "%.4f, %.4f, %.4f\n", in, me, target);
        //outfile << str1;
        std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)50000));
        n++;
        std::cout<<n<<std::endl;
    }
    p1.CloseSocketShow();
    //outfile.close();
    // 向文件写入用户输入的数据
}