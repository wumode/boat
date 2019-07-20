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
// Created by wumode on 19-4-9.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter() {
        is_initialized_ = false;
    }

    //~KalmanFilter() {};

    Eigen::VectorXd GetX() {
        return x_;
    }

    bool IsInitialized() {
        return is_initialized_;
    }

    void Initialization(Eigen::VectorXd x_in) {
        x_ = x_in;
        is_initialized_ = true;
    };

    void SetF(Eigen::MatrixXd F_in) {
        F_ = F_in;
    }

    void SetP(Eigen::MatrixXd P_in) {
        P_ = P_in;
    }

    void SetQ(Eigen::MatrixXd Q_in) {
        Q_ = Q_in;
    }

    void SetH(Eigen::MatrixXd H_in) {
        H_ = H_in;
    }

    void SetR(Eigen::MatrixXd R_in) {
        R_ = R_in;
    }

    void SetU(Eigen::VectorXd u_in) {
        u_ = u_in;
    }

    void Prediction() {
        x_ = F_ * x_ + u_;
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
    }

    void MeasurementUpdate(const Eigen::VectorXd &z) {
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + (K*y);
        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K * H_)*P_;
    }

private:
    bool is_initialized_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd u_;
};


#endif //KALMANFILTER_KALMANFILTER_H
