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
// Created by wumode on 19-7-16.
//

#ifndef SHIP_SOCKETCOMMUNICATION_H
#define SHIP_SOCKETCOMMUNICATION_H

#include <string>
#include <iostream>
#include <thread>
#include <map>
#include <json.hpp>
#include <climits>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <glog/logging.h>

#define SOCKET_SIZE 2048

using nlohmann::json;

namespace socket_communication {
    typedef int (*callBack)(uint8_t *, void *);

    class CallBackFunction{
    public:
        CallBackFunction();
        CallBackFunction(callBack callBack1, uint8_t flag, void* this_);
        CallBackFunction(const CallBackFunction &obj);
        callBack function_ptr_;
        void* this_ptr_;
    private:
        uint8_t data_flag;
    };

    class SocketCommunication {
    public:
        SocketCommunication();
        SocketCommunication(const std::string &host, uint16_t port);
        ~SocketCommunication();
        static void* SocketPortReceive(void* __this);
        bool StartSocketReceiveThread();
        bool StartSocketReceiveThread(const std::string& host, uint16_t port);
        bool StartSocketReceiveThread(const std::string& host, uint16_t port, void* __this);
        void CloseSocketReceiveThread();
        void SetCallBackFunction(callBack callBack1, uint8_t flag, void* this_);
        template <typename T> void SendData(const T& data, uint8_t flag);
        void SetHost(const std::string& host);
        void SetPort(uint16_t port);
        bool IsOpen();
    private:
        //void DataReceiveAnalysis(const uint8_t *data_buf, uint8_t num, void* __this);
        void CallFunction(uint8_t* data, void* __this);
        //bool WriteData();
        volatile bool socket_thread_;   //socket线程运行的标志
        volatile bool is_sending_;
        //serial::Serial* ser_ptr_; //指向串口的指针
        int* client_socket_ptr_;
        std::string host_;
        uint16_t port_;
        //uint32_t baud_rate_;
        volatile bool is_open_;
        uint8_t rx_buffer_[SOCKET_SIZE];
        uint8_t tx_buffer_[SOCKET_SIZE];
        //uint8_t _data_len, _data_cnt;
        //uint8_t serial_parse_state_;
        //uint8_t receive_head_high_, receive_head_low_;
        uint8_t send_head_high_, send_head_low_;
        std::map<uint8_t, CallBackFunction> callback_function_directory_;
    };

    template <typename T>
    inline void SocketCommunication::SendData(const T& data, uint8_t flag){
        if(is_open_){
            json j = data;
            std::string string_send;
            const uint8_t * str_send;
            string_send = j.dump();
            str_send = (const uint8_t *)string_send.c_str();
            //LOG(INFO) << "socket send: "<<str_send << std::endl;
            while(is_sending_){
                std::this_thread::sleep_for(std::chrono:: microseconds (500));
            }
            is_sending_ = true;
            char temp;
            temp = write(*client_socket_ptr_, str_send, strlen((const char*)str_send));
            //std::cout<<"write"<<std::endl;
//            ser_ptr_->write((const uint8_t*)tx_buffer_, data_length+5);
            is_sending_ = false;
        } else{
            std::cerr<<"Failed to send data, socket has closed"<<std::endl;
        }
    };
}


#endif //SHIP_SOCKETCOMMUNICATION_H
