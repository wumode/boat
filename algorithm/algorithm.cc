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

#include "algorithm.h"

namespace algorithm{
    void to_json(json &j, const algorithm::PidStatus &pidStatus) {
        j = json{{"P", pidStatus.P},
                 {"I", pidStatus.I},
                 {"D", pidStatus.D},
                 {"target", pidStatus.target},
                 {"input", pidStatus.input},
                 {"output", pidStatus.output},
                 {"time", pidStatus.time},
                 {"end", pidStatus.end}};
    }

    void from_json(const json &j, algorithm::PidStatus &pidStatus) {
        pidStatus.P = j.at("P").get<double>();
        pidStatus.I = j.at("P").get<double>();
        pidStatus.D = j.at("D").get<double>();
        pidStatus.target = j.at("target").get<double>();
        pidStatus.input = j.at("input").get<double>();
        pidStatus.output = j.at("output").get<double>();
        pidStatus.time = j.at("time").get<double>();
        pidStatus.end = j.at("end").get<int>();
    }

    PidController::PidController() {
        kd_ = 0.0;
        ki_ = 0.0;
        kp_ = 0.0;
        error_ = 0.0;
        last_error_ = 0.0;
        last_last_error_ = 0.0;
        output_ = 0.0;
        input_ = 0.0;
        target_ = 0.0;
        limit_ = false;
        upper_limit_ = 0.0;
        lower_limit_ = 0.0;
        debug_ = false;
        socket_com_ptr_ = nullptr;
        start_timestamp_ = std::chrono::steady_clock::now();
        now_timestamp_ = start_timestamp_;
        end_ = 0;
    }

    PidController::PidController(double kp, double ki, double kd) :kp_(kp),ki_(ki), kd_(kd){
        error_ = 0.0;
        last_error_ = 0.0;
        last_last_error_ = 0.0;
        output_ = 0.0;
        input_ = 0.0;
        target_ = 0.0;
        limit_ = false;
        upper_limit_ = 0.0;
        lower_limit_ = 0.0;
        debug_ = false;
        socket_com_ptr_ = nullptr;
        start_timestamp_ = std::chrono::steady_clock::now();
        now_timestamp_ = start_timestamp_;
        end_ = 0;
    }

    PidController::PidController(double kp, double ki, double kd, double upper_limit,
                                 double lower_limit) :kp_(kp),
                                                      ki_(ki),
                                                      kd_(kd),
                                                      upper_limit_(upper_limit),
                                                      lower_limit_(lower_limit) {
        error_ = 0.0;
        last_error_ = 0.0;
        last_last_error_ = 0.0;
        output_ = 0.0;
        input_ = 0.0;
        target_ = 0.0;
        limit_ = true;
        debug_ = false;
        socket_com_ptr_ = nullptr;
        start_timestamp_ = std::chrono::steady_clock::now();
        now_timestamp_ = start_timestamp_;
        end_ = 0;
    }

    PidController::PidController(const algorithm::PidController &p) {
        error_ = p.error_;
        last_error_ = p.last_error_;
        last_last_error_ = p.last_last_error_;
        output_ = p.output_;
        input_ = p.input_;
        target_ = p.target_;
        limit_ = p.limit_;
        debug_ = p.debug_;
        socket_com_ptr_ = p.socket_com_ptr_;
        start_timestamp_ = p.start_timestamp_;
        now_timestamp_ = p.now_timestamp_;
        end_ = p.end_;
        kd_ = p.kd_;
        ki_ = p.ki_;
        kp_ = p.kp_;
        upper_limit_ = p.upper_limit_;
        lower_limit_ = p.lower_limit_;
    }

    double& PidController::P() {
        return kp_;
    }

    double& PidController::I() {
        return ki_;
    }

    double& PidController::D() {
        return kd_;
    }

    double& PidController::Target() {
        return target_;
    }

    double PidController::Output() {
        return output_;
    }

    double PidController::Update(double &input) {
        input_ = input;
        error_ = target_ - input_;
        output_ += kp_*(error_ - last_error_) + ki_*error_ + kd_ * (error_ - 2*last_error_ + last_last_error_);
        if(limit_){
            if(output_<lower_limit_){
                output_ = lower_limit_;
            } else if (output_>upper_limit_){
                output_ = upper_limit_;
            }
        }
        last_last_error_ = last_error_;
        last_error_ = error_;
        if(debug_){
            SocketShowPublish_();
        }
        return output_;
    }

    double PidController::Update(double &input, double &target) {
        target_ = target;
        return Update(input);
    }

    void PidController::SetLimit(double upper_limit, double lower_limit) {
        limit_ = true;
        upper_limit_ = upper_limit;
        lower_limit_ = lower_limit;
    }

    void PidController::SocketShow(const std::string &host, uint16_t port) {
        socket_com_ptr_ = new socket_communication::SocketCommunication(host, port);
        socket_com_ptr_->SetCallBackFunction((socket_communication::callBack)SocketReceiveCallBack, 1, this);
        //std::cout<<"er"<<std::endl;
        if(!socket_com_ptr_->StartSocketReceiveThread()){
            exit(-1);
        }
        start_timestamp_ = std::chrono::steady_clock::now();
        end_ = 0;
        debug_ = true;
    }

    void PidController::SocketReceiveCallBack(uint8_t *buffer_ptr_, void *__this) {

    }

    void PidController::SocketShowPublish_(){
        now_timestamp_ = std::chrono::steady_clock::now();
        if(socket_com_ptr_->IsOpen()){
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(now_timestamp_
                                                                                                                -start_timestamp_);
            PidStatus p;
            p.P = kp_;
            p.I = ki_;
            p.D = kd_;
            p.output = output_;
            p.input = input_;
            p.target = target_;
            p.time = time_used.count();
            p.end = end_;
            socket_com_ptr_->SendData(p, 1);
        }
    }

    void PidController::CloseSocketShow() {
        debug_ = false;
        end_ = 1;
        SocketShowPublish_();
        socket_com_ptr_->CloseSocketReceiveThread();
        delete socket_com_ptr_;
        socket_com_ptr_ = nullptr;
    }
}
