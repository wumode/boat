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

#include "socket_communication.h"

namespace socket_communication{
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
        memset(rx_buffer_,'\0', SOCKET_SIZE);
        memset(tx_buffer_, '\0', SOCKET_SIZE);
    }

    SocketCommunication::SocketCommunication(const std::string &host, uint16_t port) {
        client_socket_ptr_ = nullptr;
        socket_thread_ = false;
        is_open_ = false;
        is_sending_ = false;
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
        client_socket_ptr_ = new int;
        *client_socket_ptr_ = socket(AF_INET, SOCK_STREAM, 0);
        if (*client_socket_ptr_ == -1) {
            //perror("socket");
            LOG(ERROR)<<"socket error\n";
            return false;
        }
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        inet_aton(host_.c_str(), &(addr.sin_addr));
        int addrlen = sizeof(addr);
        int listen_socket = connect(*client_socket_ptr_, (struct sockaddr *)&addr, addrlen);
        if (listen_socket == -1) {
            LOG(ERROR)<<"connect "<<host_<<":"<<port_<<" error\n";
            return false;
        }
        is_open_ = true;
        socket_thread_ = true;
        pthread_t pth;
        pthread_create(&pth, nullptr,SocketPortReceive, (void*)this);
        return true;
    }

    bool SocketCommunication::StartSocketReceiveThread(const std::string &host, uint16_t port) {
        client_socket_ptr_ = new int;
        *client_socket_ptr_ = socket(AF_INET, SOCK_STREAM, 0);
        if (*client_socket_ptr_ == -1) {
            //perror("socket");
            LOG(ERROR)<<"socket error\n";
            return false;
        }
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        inet_aton(host.c_str(), &(addr.sin_addr));

        int addrlen = sizeof(addr);
        int listen_socket = connect(*client_socket_ptr_, (struct sockaddr *)&addr, addrlen);
        if (listen_socket == -1) {
            LOG(ERROR)<<"connect "<<host_<<":"<<port_<<" error\n";
            return false;
        }
        host_ = host;
        port_ = port;
        socket_thread_ = true;
        is_open_ = true;
        pthread_t pth;
        pthread_create(&pth, nullptr,SocketPortReceive, (void*)this);
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

    bool SocketCommunication::IsOpen() {
        return is_open_;
    }

    void* SocketCommunication::SocketPortReceive(void *__this) {
        auto* _this = (SocketCommunication*)__this;
        //char buf[SIZE];
        while (_this->socket_thread_){
            memset(_this->rx_buffer_,'\0', SOCKET_SIZE);
            read(*_this->client_socket_ptr_, _this->rx_buffer_, SOCKET_SIZE-1);
            _this->CallFunction(_this->rx_buffer_, (void*)_this);
        }
        delete _this->client_socket_ptr_;
        _this->client_socket_ptr_ = nullptr;
        return nullptr;
    }

    void SocketCommunication::CloseSocketReceiveThread() {
        socket_thread_ = false;
        while(client_socket_ptr_!= nullptr);
        is_open_ = false;
    }

    void SocketCommunication::CallFunction(uint8_t* data, void *__this) {
        auto* _this = (SocketCommunication*)__this;
        uint8_t socket_data_flag = 1;
        std::map<uint8_t , CallBackFunction>::iterator iter;
        iter = _this->callback_function_directory_.find(socket_data_flag);
        if(iter != _this->callback_function_directory_.end()){
            _this->callback_function_directory_[socket_data_flag].function_ptr_((uint8_t*)data, _this->callback_function_directory_[socket_data_flag].this_ptr_);
        }
    }
}