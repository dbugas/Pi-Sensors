#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <cmath>
#include <random>
#include <algorithm>
#include <string>
#include <chrono>

class KF {
public:
   KF(Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0, double dt);
   void srKF_step(const Eigen::Ref<Eigen::VectorXd> ztrue);
   void KF_step(const Eigen::Ref < Eigen::Vector<double, 2>> ztrue);
   void KF_predict_alt(const double meas_az);
   void KF_update_alt(const Eigen::VectorXd zmeas);
   void SageHusa_update_alt();
   void srSageHusa_update_alt();

   Eigen::VectorXd xhat;
   Eigen::MatrixXd sigmaX;
   int counter = 0;
private:
   Eigen::LLT<Eigen::MatrixXd> chol;

   Eigen::MatrixXd Bhat;
   Eigen::MatrixXd Ahat;
   Eigen::MatrixXd Chat;
   Eigen::MatrixXd Dhat;

   Eigen::VectorXd zhat;
   Eigen::MatrixXd I;
   Eigen::MatrixXd sigmaV; 
   Eigen::MatrixXd sigmaW; 
   Eigen::MatrixXd sigmaW_save;
   Eigen::MatrixXd sigmaZ;
   Eigen::MatrixXd L;

   Eigen::VectorXd nu;

   void Model();
   void Model_output();

   double T = 1.0; // Sampling time
   double d_R = 0.9;
   double d_W = 0.999;

   Eigen::MatrixXd sigx_temp;
   Eigen::MatrixXd sigz_temp;
   Eigen::Index Nx;
   Eigen::Index Nw;
   Eigen::Index Nv;

   double dt; // Added 'dt' as a private member variable
};