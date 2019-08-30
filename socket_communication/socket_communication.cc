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

#include <FreeImage.h>
#include "socket_communication.h"

namespace socket_communication{
    void to_json(json &j, const SocketTransmission &socketTransmission) {
        j = json{{"header",     socketTransmission.header},
                 {"data",       socketTransmission.data},
                 {"time_stamp", socketTransmission.time_stamp}};
    }

    void from_json(const json &j, SocketTransmission &socketTransmission) {
        socketTransmission.header = j.at("header").get<uint8_t >();
        socketTransmission.data = j.at("data").get<std::string >();
        socketTransmission.time_stamp = j.at("time_stamp").get<uint64_t>();
    }

    CallBackFunction::CallBackFunction() {}
    CallBackFunction::CallBackFunction(callBack callBack1, uint8_t flag, void *this_):
            function_ptr_(callBack1), this_ptr_(this_), data_flag(flag){}
    CallBackFunction::CallBackFunction(const socket_communication::CallBackFunction &obj) {
    }

    SocketCommunication::SocketCommunication() {
        client_socket_ptr_ = nullptr;
        socket_thread_ = false;
        is_open_ = false;
        is_sending_ = false;
        offline_reconnection_ = false;
        memset(rx_buffer_,'\0', SOCKET_SIZE);
        memset(tx_buffer_, '\0', SOCKET_SIZE);
    }

    SocketCommunication::SocketCommunication(const std::string &host, uint16_t port) {
        client_socket_ptr_ = nullptr;
        socket_thread_ = false;
        is_open_ = false;
        is_sending_ = false;
        offline_reconnection_ = false;
        memset(rx_buffer_,'\0', SOCKET_SIZE);
        memset(tx_buffer_, '\0', SOCKET_SIZE);
        host_ = host;
        port_ = port;
    }

    SocketCommunication::~SocketCommunication() {
        if(client_socket_ptr_){
            CloseSocketReceiveThread();
            delete client_socket_ptr_;
        }
        callback_function_directory_.clear();
    }

    bool SocketCommunication::StartSocketReceiveThread() {
        return StartSocketReceiveThread(host_, port_);
    }

    bool SocketCommunication::StartSocketReceiveThread(const std::string &host, uint16_t port) {
        port_ = port;
        host_ = host;
        return StartSocketReceiveThread(host_, port_, this);;
    }

    bool SocketCommunication::StartSocketReceiveThread(const std::string &host, uint16_t port, void *__this) {
        auto* _this = (SocketCommunication*)__this;
        _this->client_socket_ptr_ = new int;
        *_this->client_socket_ptr_ = socket(AF_INET, SOCK_STREAM, 0);
        if (*_this->client_socket_ptr_ == -1) {
            //perror("socket");
            //LOG(ERROR)<<"socket error\n";
            delete _this->client_socket_ptr_;
            _this->client_socket_ptr_ = nullptr;
            std::cerr<<"Start socket error"<<std::endl;
            return false;
        }
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(_this->port_);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        inet_aton(_this->host_.c_str(), &(addr.sin_addr));
        int addrlen = sizeof(addr);
        BOOL bReuseaddr = TRUE;
        setsockopt(*_this->client_socket_ptr_, SOL_SOCKET, SO_REUSEADDR, (const char*)&bReuseaddr,sizeof(BOOL));
        int listen_socket = connect(*_this->client_socket_ptr_, (struct sockaddr *)&addr, addrlen);
        if (listen_socket == -1) {
            //LOG(ERROR)<<"connect "<<host_<<":"<<port_<<" error";
            delete _this->client_socket_ptr_;
            _this->client_socket_ptr_ = nullptr;
            std::cerr<<"connect "<<_this->host_<<":"<<_this->port_<<" error"<<std::endl;
            return false;
        }
        struct timeval timeout={0,1000};
        int ret=setsockopt(*_this->client_socket_ptr_,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout,sizeof(timeout));
        if(ret == -1){
            std::cerr<<"Failed to set socket timeout"<<std::endl;
        }
        _this->is_open_ = true;
        _this->socket_thread_ = true;
        pthread_t pth;
        pthread_create(&pth, nullptr,SocketPortReceive, (void*)this);
        _this->port_ = port;
        _this->host_ = host;
        std::cout<<"Socket started"<<std::endl;
        return true;
    }

    void SocketCommunication::SetHost(const std::string& host) {
        host_ = host;
    }

    void SocketCommunication::SetPort(uint16_t port) {
        port_ = port;
    }

    void SocketCommunication::SetCallBackFunction(socket_communication::callBack callBack1, uint8_t flag,
                                                  void *this_) {
        std::map<uint8_t , CallBackFunction>::iterator iter;
        iter = callback_function_directory_.find(flag);
        if(iter != callback_function_directory_.end()){
            callback_function_directory_.insert(std::pair<uint8_t, CallBackFunction>(flag, CallBackFunction(callBack1, flag, this_)));
        }
        else{
            callback_function_directory_[flag] = CallBackFunction(callBack1, flag, this_);
        }
    }

    int SocketCommunication::RemoveCallBackFunction(uint8_t flag) {
        return callback_function_directory_.erase(flag);
    }

    bool SocketCommunication::IsOpen() {
        return is_open_;
    }

    void* SocketCommunication::SocketPortReceive(void *__this) {
        auto* _this = (SocketCommunication*)__this;
        int socket_closed = 0;
        while (_this->socket_thread_){
            memset(_this->rx_buffer_,'\0', SOCKET_SIZE);
            char temp;
            temp = read(*_this->client_socket_ptr_, _this->rx_buffer_, SOCKET_SIZE-1);
            if(temp == 0)
            {
                std::cerr<<"Socket closed"<<std::endl;
                socket_closed = 1;
                break;
            }
            _this->CallFunction(_this->rx_buffer_, (void*)_this);
        }
        if(_this->client_socket_ptr_){
            close(*_this->client_socket_ptr_);
            delete _this->client_socket_ptr_;
            _this->client_socket_ptr_ = nullptr;
        }
        //_this->socket_thread_ = false;
        _this->is_open_ = false;
        if(socket_closed){
            while (_this->socket_thread_){
                if(_this->StartSocketReceiveThread(_this->host_, _this->port_, _this)){
                    _this->offline_reconnection_ = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)1000000));
            }
        }
        return nullptr;
    }

    void SocketCommunication::CloseSocketReceiveThread() {
        socket_thread_ = false;
        uint32_t n = 0;
        while(client_socket_ptr_!= nullptr){
            std::this_thread::sleep_for(std::chrono:: microseconds ((unsigned int)200));
            if(n>9){
                if(client_socket_ptr_){
                    //close(*client_socket_ptr_);
                    shutdown(*client_socket_ptr_, SHUT_RDWR);
                    delete client_socket_ptr_;
                    client_socket_ptr_ = nullptr;
                }
            }
            n++;
        }
        is_open_ = false;
    }

    void SocketCommunication::ResetOfflineFlag() {
        offline_reconnection_ = false;
    }

    bool SocketCommunication::OfflineReconnection() {
        return offline_reconnection_;
    }

    void SocketCommunication::CallFunction(uint8_t* data, void *__this) {
        auto* _this = (SocketCommunication*)__this;
        uint8_t socket_data_flag;
        std::string data_receive = (const char*)data;
        json j;
        try {
            j = json::parse(data_receive);
        }
        catch (nlohmann::detail::parse_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"Dara parse error, data: "<<data_receive<<std::endl;
#endif
            std::cerr<<"Dara parse error, data: "<<data_receive<<std::endl;
            return;
        }
        SocketTransmission s_t;
        try {
            s_t = j;
        }
        catch (nlohmann::detail::type_error& e){
#ifdef USE_GLOG
            LOG(ERROR)<<"Dara parse error, data: "<<data_receive<<std::endl;
#endif
            std::cerr<<"Dara parse error, data: "<<data_receive<<std::endl;
            return;
        }

        socket_data_flag = s_t.header;
        std::map<uint8_t , CallBackFunction>::iterator iter;
        iter = _this->callback_function_directory_.find(socket_data_flag);
        if(iter != _this->callback_function_directory_.end()){
            _this->callback_function_directory_[socket_data_flag].function_ptr_((uint8_t*)s_t.data.c_str(), _this->callback_function_directory_[socket_data_flag].this_ptr_);
        }
    }
}