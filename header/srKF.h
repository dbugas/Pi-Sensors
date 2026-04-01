#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <random>
#include <complex>
#include <string>
#include <chrono>
#include <algorithm>

class srKF {
public:
	    srKF(const Eigen::Matrix4d& sigmax, const double sigmaV_, const Eigen::Matrix4d& sigmaW_, const Eigen::Vector4d& xhat0,double dt);
	void KF_predict_alt(const double meas_az);
	void KF_update_alt(const double zmeas, const double temp);
	void SageHusa_update_alt();

 	Eigen::Vector4d xhat;
    Eigen::Matrix4d sigmaX;
private:
 	Eigen::Matrix4d Bhat;
    Eigen::Matrix4d Ahat;
    Eigen::Matrix<double,1,4> Chat;   // 1×4 row vector
    Eigen::Matrix<double,1,1> Dhat;

    double zhat;
    Eigen::Matrix4d I;
    double sigmaV;
    Eigen::Matrix4d sigmaW;
    double sigmaZ;
    Eigen::Vector4d L;                // 4×1 = Vector4d

    double nu;

	double T = 1.0; // accel Sampling time
	double p0 = 101325.0;
	double d_R = 0.95;
};
