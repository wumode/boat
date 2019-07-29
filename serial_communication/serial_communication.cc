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

#include "serial_communication.h"

namespace serial_communication{
    CallBackFunction::CallBackFunction() {}
    CallBackFunction::CallBackFunction(callBack callBack1, uint8_t flag, void *this_):
            function_ptr_(callBack1),data_flag(flag), this_ptr_(this_){}
    CallBackFunction::CallBackFunction(const serial_communication::CallBackFunction &obj) {

    }


    SerialCommunication::SerialCommunication() {
        ser_ptr_ = nullptr;
        serial_thread_ = false;
        is_open_ = false;
        is_sending_= false;
        //port_ = com;
        //baud_rate_ = baud_rate;
        _data_len = 0;
        _data_cnt = 0;
        memset(rx_buffer_,'\0', 256);
        memset(tx_buffer_, '\0', 256);
        serial_parse_state_ = 0;
        receive_head_high_ = 0xAA;
        receive_head_low_ = 0xAF;
        send_head_high_ = 0xAA;
        send_head_low_ = 0xAF;
        //StartSerialThread(port_, baud_rate_);
    }

    SerialCommunication::SerialCommunication(const std::string& com, uint32_t baud_rate) {
        ser_ptr_ = nullptr;
        serial_thread_ = false;
        is_open_ = false;
        is_sending_= false;
        port_ = com;
        baud_rate_ = baud_rate;
        _data_len = 0;
        _data_cnt = 0;
        memset(rx_buffer_,'\0', 256);
        memset(tx_buffer_, '\0', 256);
        serial_parse_state_ = 0;
        receive_head_high_ = 0xAA;
        receive_head_low_ = 0xAF;
        send_head_high_ = 0xAA;
        send_head_low_ = 0xAF;
        //StartSerialThread(port_, baud_rate_);
    }

    SerialCommunication::~SerialCommunication(){
        if(ser_ptr_){
            CloseSerialReceiveThread();
            free(ser_ptr_);
        }
        callback_function_directory_.clear();
    };

    bool SerialCommunication::StartSerialReceiveThread() {
        ser_ptr_ = new serial::Serial;
        if(ser_ptr_ == nullptr){
            return false;
        }
        try
        {
            ser_ptr_->setPort(port_);
            ser_ptr_->setBaudrate(baud_rate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(0);
            ser_ptr_->setTimeout(to);
            ser_ptr_->setParity(serial::parity_none);
            ser_ptr_->setStopbits(serial::stopbits_one);
            ser_ptr_->open();
        }
        catch (serial::IOException& e)
        {
            LOG(ERROR)<<"Unable to open port "<<port_<<" without params"<<std::endl;
            return false;
        }
        serial_thread_ = true;
        is_open_ = true;
        pthread_t pth;
        pthread_create(&pth, nullptr,SerialPortReceive, (void*)this);
        return true;
    }

    bool SerialCommunication::StartSerialReceiveThread(const std::string &port, uint32_t baud_rate) {
        ser_ptr_ = new serial::Serial;
        if(ser_ptr_ == nullptr){
            return false;
        }
        try
        {
            //设置串口属性，并打开串口
            ser_ptr_->setPort(port);
            ser_ptr_->setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(0);
            ser_ptr_->setTimeout(to);
            ser_ptr_->setParity(serial::parity_none);
            ser_ptr_->setStopbits(serial::stopbits_one);
            ser_ptr_->open();
        }
        catch (serial::IOException& e)
        {
            LOG(ERROR)<<"Unable to open port "<<port<<std::endl;
            return false;
        }
        port_ = port;
        baud_rate_ = baud_rate;
        serial_thread_ = true;
        is_open_ = true;
        pthread_t pth;
        pthread_create(&pth, nullptr,SerialPortReceive, (void*)this);
        return true;
    }

    void SerialCommunication::SetBaudRate(uint32_t baud_rate) {
        baud_rate_ = baud_rate;
    }

    void SerialCommunication::SetComPort(const std::string &port) {
        port_ = port;
    }

    void SerialCommunication::CloseSerialReceiveThread() {
        serial_thread_ = false;
        while(ser_ptr_!= nullptr);
    }

    void* SerialCommunication::SerialPortReceive(void* __this) {
        auto* _this = (SerialCommunication*)__this;
        uint8_t serial_data_buffer[256];
        int size_read;
        while (_this->serial_thread_){
            size_read = _this->ser_ptr_->read(serial_data_buffer,1);
            //std::cout<<std::hex;
            //std::cout<<*serial_data_buffer;
            if(size_read){
                _this->DataReceivePrepare(*serial_data_buffer, __this);
            }

            else{
                _this->ser_ptr_->flushInput();
            }
        }
        _this->ser_ptr_->close();
        _this->is_open_ = false;
        free(_this->ser_ptr_);
        _this->ser_ptr_ = nullptr;
        return nullptr;
    }

    void SerialCommunication::DataReceivePrepare(uint8_t data, void* __this)
    {
        auto* _this = (SerialCommunication*)__this;
        /* 帧头1  一个数据帧中第一个数据并且判断是否与宏定义帧头1相等*/
        if (_this->serial_parse_state_ == 0 && data == _this->receive_head_high_)
        {
            //std::cout<<"data_state = 1"<<std::endl;
            _this->serial_parse_state_ = 1;
            _this->rx_buffer_[0] = data;

        }
            /* 帧头2 一个数据帧中第二个数据并且判断是否与宏定义帧头2相等*/
        else if (_this->serial_parse_state_ == 1 && data == _this->receive_head_low_)
        {
            _this->serial_parse_state_ = 2;
            _this->rx_buffer_[1] = data;
        }
            /* 功能字 */
        else if (_this->serial_parse_state_ == 2 && data>0xf0)
        {
            //std::cout<<"parse state: 2"<<std::endl;
            _this->serial_parse_state_ = 3;
            _this->rx_buffer_[2] = data;
        }
            /* 长度 */
        else if (_this->serial_parse_state_ == 3 && data<100)
        {
            //std::cout<<"parse state: 3"<<std::endl;
            _this->serial_parse_state_ = 4;
            _this->rx_buffer_[3] = data;
            _this->_data_len = data;
            _this->_data_cnt = 0;
        }
            /* 接收数据组*/
        else if (_this->serial_parse_state_ == 4 && _this->_data_len>0)
        {
            //std::cout<<"parse state: 4"<<std::endl;
            _this->_data_len--;
            _this->rx_buffer_[4 + _this->_data_cnt++] = data;
            if (_this->_data_len == 0)
                _this->serial_parse_state_ = 5;
        }
            /* 校验累加和 */
        else if (_this->serial_parse_state_ == 5)
        {
            //std::cout<<"parse state: 5"<<std::endl;
            _this->serial_parse_state_ = 0;
            _this->rx_buffer_[4 + _data_cnt] = data;
            //std::cout<<"data_state = 5"<<std::endl;
            _this->DataReceiveAnalysis(rx_buffer_, _data_cnt + 5, __this);  //调用数据分析函数,总长比索引+1
        }
            /* 若有错误重新等待接收帧头 */
        else{
            _this->serial_parse_state_ = 0;
        }
    }


    void SerialCommunication::DataReceiveAnalysis(const uint8_t *data_buf, uint8_t num, void* __this)
    {
        auto* _this = (SerialCommunication*)__this;
        uint8_t sum = 0;
        /* 首先计算校验累加和 */
        for (uint8_t i = 0; i<(num - 1); i++){
            sum += *(data_buf + i);
        }
        /* 判断校验累加和 若不同则舍弃*/
        if (!(sum == *(data_buf + num - 1))) {
            return;
        }
        /* 判断帧头 */
        if (!(*(data_buf) == _this->receive_head_high_ && *(data_buf + 1) == _this->receive_head_low_))
        {
            return;
        }
        /* 判断功能字：*/
        uint8_t serial_data_flag = *(data_buf+2);
        int flags = serial_data_flag;
        //std::cout<<"flag: "<<flags<<std::endl;
        std::map<uint8_t , CallBackFunction>::iterator iter;
        iter = _this->callback_function_directory_.find(serial_data_flag);
        if(iter != _this->callback_function_directory_.end()){
            _this->callback_function_directory_[serial_data_flag].function_ptr_((uint8_t*)data_buf + 4, _this->callback_function_directory_[serial_data_flag].this_ptr_);
        }
    }

    void SerialCommunication::SetCallBackFunction(serial_communication::callBack callBack1, uint8_t flag,
                                                  void *this_) {
        //CallBackFunction c_f(callBack1, flag, this_);
        std::map<uint8_t , CallBackFunction>::iterator iter;
        iter = callback_function_directory_.find(flag);
        if(iter != callback_function_directory_.end()){
            callback_function_directory_.insert(std::pair<uint8_t, CallBackFunction>(flag, CallBackFunction(callBack1, flag, this_)));
        }
        else{
            callback_function_directory_[flag] = CallBackFunction(callBack1, flag, this_);
        }
    }

    bool SerialCommunication::IsOpen() {
        return is_open_;
    }
}