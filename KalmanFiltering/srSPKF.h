#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <complex>
#include <string>
#include <chrono>

class srSPKF {
public:
    srSPKF(int Nx_, int Nxa_, int Nz_, Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0);
    void srSPKF_step(int t);

    Eigen::MatrixXd getTrueStates()    const { return xtrue; }
    Eigen::MatrixXd getTrueMeasurements() const { return ztrue; }

    Eigen::MatrixXd xtrue;
    Eigen::VectorXd xhat;
    Eigen::MatrixXd sigmaX;
    int counter = 0;
private:
    Eigen::VectorXd Model(const Eigen::VectorXd X);
    Eigen::VectorXd Model_output(const Eigen::VectorXd X);
    void GetTrueValues();

    int Nx;
    int Nxa;
    int Nz;
    int Nw;
    int Nv;
    double h = std::sqrt(3);
    int maxIter = 1000;

    Eigen::VectorXd zhat;
    Eigen::MatrixXd ztrue;

    Eigen::Vector2d Wmx;
    Eigen::Vector2d Wcx;
    Eigen::VectorXd Wmxz;

    // Cholesky decomposition assumed 
    Eigen::MatrixXd sigmaV;
    Eigen::MatrixXd sigmaW;
    Eigen::Matrix<double, 5, 3> B;

    std::mt19937 gen;
    std::normal_distribution<double> randn;

    Eigen::MatrixXd X; // Sigma points
    Eigen::MatrixXd Xx; // states at sigma points
    Eigen::MatrixXd Z; // output states at sigma points
    Eigen::MatrixXd Xs;
    Eigen::MatrixXd Zs;
    Eigen::MatrixXcd sigmaX_;
    Eigen::VectorXd xhata;  // [xhat; zeros] states and process and sensor noise mean
    std::complex<double> srWcx;

    Eigen::HouseholderQR<Eigen::MatrixXcd> qr;
};