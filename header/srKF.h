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
	srKF(Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0, double dt);
	void KF_step(const Eigen::Ref<Eigen::VectorXd> ztrue);
	void KF_predict_alt(const double meas_az);
	void KF_update_alt(const Eigen::VectorXd zmeas, const double temp);
	void SageHusa_update_alt();

	Eigen::VectorXd xhat;
	Eigen::MatrixXd sigmaX;
	int counter = 0;
private:
	Eigen::MatrixXd Bhat;
	Eigen::MatrixXd Ahat;
	Eigen::MatrixXd Chat;
	Eigen::MatrixXd Dhat;

	Eigen::VectorXd zhat;
	Eigen::MatrixXd I;
	Eigen::MatrixXd sigmaV; 
	Eigen::MatrixXd sigmaW; 
	Eigen::MatrixXd sigmaZ;
	Eigen::MatrixXd L;

	Eigen::VectorXd nu;

	double T = 1.0; // accel Sampling time
	double p0 = 101325.0;
	double d_R = 0.9;
};