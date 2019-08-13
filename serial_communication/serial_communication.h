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
// Created by wumode on 19-6-29.
//

#ifndef SHIP_SERIAL_COMMUNICATION_H
#define SHIP_SERIAL_COMMUNICATION_H

#include <string>
#include <serial/serial.h>
#include <iostream>
#include <thread>
#include <map>
#include <climits>
#include <glog/logging.h>

#define SERIAL_SIZE 256

namespace serial_communication {
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


    class SerialCommunication {
    public:
        SerialCommunication();
        SerialCommunication(const std::string &com, uint32_t baud_rate);
        ~SerialCommunication();
        static void* SerialPortReceive(void* __this);
        bool StartSerialReceiveThread();
        bool StartSerialReceiveThread(const std::string& port, uint32_t baud_rate);
        void CloseSerialReceiveThread();
        void SetCallBackFunction(callBack callBack1, uint8_t flag, void* this_);
        template <typename T> void SendData(const T& data, uint8_t flag);
        void SetBaudRate(uint32_t baud_rate);
        void SetComPort(const std::string& port);
        bool IsOpen();
    private:
        void DataReceiveAnalysis(const uint8_t *data_buf, uint8_t num, void* __this);
        void DataReceivePrepare(uint8_t data, void* __this);
        //bool WriteData();

        volatile bool serial_thread_;   //串口线程运行的标志
        volatile bool is_sending_;
        serial::Serial* ser_ptr_; //指向串口的指针
        std::string port_;
        uint32_t baud_rate_;
        volatile bool is_open_;
        uint8_t rx_buffer_[SERIAL_SIZE];
        uint8_t tx_buffer_[SERIAL_SIZE];
        uint8_t _data_len, _data_cnt;
        uint8_t serial_parse_state_;
        uint8_t receive_head_high_, receive_head_low_;
        uint8_t send_head_high_, send_head_low_;
        std::map<uint8_t, CallBackFunction> callback_function_directory_;
    };

    template <typename T>
    inline void SerialCommunication::SendData(const T& data, uint8_t flag){
        if(is_open_){
            uint8_t data_length = sizeof(T);
            tx_buffer_[0] = send_head_high_;
            tx_buffer_[1] = send_head_low_;
            tx_buffer_[2] = flag;
            tx_buffer_[3] = sizeof(T);
            memcpy(tx_buffer_+4, (void*)&data, data_length);
            uint8_t sum = 0;
            for (int i = 0; i < data_length+4; i++) {
                sum += tx_buffer_[i];
            }
            tx_buffer_[data_length+4] = sum;
            while(is_sending_){
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)100));
            }
            is_sending_ = true;
            try {
                ser_ptr_->write((const uint8_t*)tx_buffer_, data_length+5);
            }
            catch (serial::SerialException& e){
                char s[32];
                sprintf(s, "%p", ser_ptr_);
                LOG(ERROR)<<"ser_ptr: "<<s;
                LOG(ERROR)<<"serial::SerialException error in send, data length: "<<data_length+5;
                CloseSerialReceiveThread();
                StartSerialReceiveThread();
            }
            is_sending_ = false;
        }
    }

}
#endif //SHIP_SERIAL_COMMUNICATION_H
