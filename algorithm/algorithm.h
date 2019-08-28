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
// Created by wumode on 19-8-22.
//
#ifndef BOAT_ALGORITHM_H
#define BOAT_ALGORITHM_H
#include <socket_communication.h>
#include <chrono>

namespace algorithm {
    typedef struct Limit{
        double upper_limit = 0.0;
        double lower_limit = 0.0;
    }Limit;
    typedef struct PidStatus{
        double P;
        double I;
        double D;
        double target;
        double input;
        double output;
        double time;
        int end;
    }PidStatus;

    class PidController{
    public:
        PidController();
        PidController(double kp, double ki, double kd);
        PidController(double kp, double ki, double kd, double upper_limit, double lower_limit);
        PidController(double kp, double ki, double kd, const Limit& limit);
        PidController(const PidController& p);
        double& P();
        double& I();
        double& D();
        double& Target();
        double Output();
        double Update(double& input);
        double Update(double& input, double& target);
        void SetLimit(double upper_limit, double lower_limit);
        void SetLimit(const Limit& limit);
        void SocketShow(const std::string &host, uint16_t port);
        void CloseSocketShow();
        static void SocketReceiveCallBack(uint8_t* buffer_ptr_, void* __this);

    private:
        void SocketShowPublish_();
        double kp_;
        double ki_;
        double kd_;
        double error_;
        double last_error_;
        double last_last_error_;
        double output_;
        double input_;
        double target_;
        Limit limit_;
        bool is_limit_;
        bool debug_;
        int end_;
        socket_communication::SocketCommunication* socket_com_ptr_;
        std::chrono::steady_clock::time_point now_timestamp_;
        std::chrono::steady_clock::time_point start_timestamp_;
    };
}

#endif //BOAT_ALGORITHM_H
