#include "srSPKF.h"

srSPKF::srSPKF(int Nx_, int Nxa_, int Nz_, Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0) :
    Nx(Nx_), Nxa(Nxa_), Nz(Nz_), sigmaX(sigmax), sigmaV(sigmaV_), sigmaW(sigmaW_), xhat(xhat0),
    randn(0, 1)
{
    Nw = sigmaW.cols();
    Nv = sigmaV.cols();
    zhat = Eigen::VectorXd::Zero(Nz);
    std::random_device rd;
    gen.seed(rd());

    B << 0.5, 0, 0,
        1.0, 0, 0,
        0, 0.5, 0,
        0, 1.0, 0,
        0, 0, 1.0;

    Wmx(0) = (h * h - (double)Nxa) / (h * h);
    Wmx(1) = 1.0 / (2.0 * h * h);
    Wcx = Wmx;
    Wmxz.resize(1 + 2 * Nxa);
    Wmxz.segment(1, 2 * Nxa).setConstant(Wmx(1));
    Wmxz(0) = Wmx(0);

    xtrue.resize(Nx, maxIter);
    ztrue.resize(Nz, maxIter);

    // Following is for simulation purposes 
    Eigen::Vector3d vec;
    vec << sigmaW(0, 0) * randn(gen), sigmaW(1, 1)* randn(gen), sigmaW(2, 2)* randn(gen);

    xtrue.col(0) = xhat0 + B * vec;
    xhat = xhat0;
    GetTrueValues();

    X = Eigen::MatrixXd::Zero(Nxa, 2 * Nxa + 1);
    Xx = Eigen::MatrixXd::Zero(Nx, 2 * Nxa + 1);
    Z = Eigen::MatrixXd::Zero(Nz, 2 * Nxa + 1);
    xhata = Eigen::VectorXd::Zero(Nxa);
    srWcx = sqrt(std::complex<double>(Wcx(0)));
    sigmaX_ = Eigen::MatrixXcd::Zero(Nx, 1 + 2 * Nxa);
}
void srSPKF::srSPKF_step(int t) {

    xhata << xhat, Eigen::VectorXd::Zero(Nw + Nv);
    // Construct block diagonal matrix: sPxa = blkdiag(sigmaX, sigmaW, sigmaV)
    Eigen::MatrixXd sPxa = Eigen::MatrixXd::Zero(Nxa, Nxa);
    sPxa.topLeftCorner(Nx, Nx) = sigmaX;
    sPxa.block(Nx, Nx, Nw, Nw) = sigmaW;
    sPxa.block(Nx + Nw, Nx + Nw, Nv, Nv) = sigmaV;

    // --- 1a-iii: Sigma point calculation ---
    X.col(0) = xhata;
    sPxa = h * sPxa;

    // Calculate scaled Cholesky factors for positive and negative sigma points
    X.block(0, 1, Nxa, Nxa) = xhata.replicate(1, Nxa) + sPxa;
    X.block(0, Nxa + 1, Nxa, Nxa) = xhata.replicate(1, Nxa) - sPxa;
    // 1a - iv: Calculate state equation for every sigma point
    for (int i = 0; i < 1 + 2 * Nxa; i++) {
        Xx.col(i) = Model(X.col(i));
    }
    xhat = Xx * Wmxz;

    // SPKF Step 1b: Covariance of prediction
    Eigen::MatrixXd Xs = (Xx.block(0, 1, Nx, 2 * Nxa) - xhat.replicate(1, 2 * Nxa)) * sqrt(Wcx(1));
    Eigen::MatrixXd Xs1 = Xx.col(0) - xhat;

    sigmaX_ << srWcx * Xs1, Xs;
    Eigen::MatrixXcd srSigmaX_temp = qr.compute(sigmaX_.transpose()).matrixQR().triangularView<Eigen::Upper>(); // get R matrix
    Eigen::MatrixXcd temp = srSigmaX_temp.block(0, 0, Nx, Nx);
    Eigen::MatrixXcd srSigmaXm = temp.transpose();

    // SPKF Step 1c: Create output estimate
    for (int i = 0; i < 1 + 2 * Nxa; i++) {
        Z.col(i) = Model_output(X.col(i));
    }
    zhat = Z * Wmxz;

    // -----------------------------------
    // SPKF Step 2a: Estimator gain matrix
    // -----------------------------------

    Eigen::MatrixXd Zs = (Z.block(0, 1, Nz, 2 * Nxa) - zhat.replicate(1, 2 * Nxa)) * sqrt(Wcx(1));
    Eigen::MatrixXd Zs1 = Z.col(0) - zhat;
    Eigen::MatrixXcd SigmaXZ = Xs * Zs.transpose() + Wcx(0) * Xs1 * Zs1.transpose();

    Eigen::MatrixXcd SigmaZ = Eigen::MatrixXcd::Zero(Nz, 1 + 2 * Nxa);
    SigmaZ << srWcx * Zs1, Zs;

    Eigen::MatrixXcd srSigmaZ_temp = qr.compute(SigmaZ.transpose()).matrixQR().triangularView<Eigen::Upper>(); // get R matrix
    temp = srSigmaZ_temp.block(0, 0, Nz, Nz);
    Eigen::MatrixXcd srSigmaZ = temp.transpose();

    // compute Lx = (SigmaXZ*srSigmaZ^(-T))*srSigmaZ^(-1)
    Eigen::MatrixXcd intermediate_Lx = srSigmaZ.transpose().colPivHouseholderQr().solve(SigmaXZ.transpose());
    Eigen::MatrixXcd Lx = srSigmaZ.colPivHouseholderQr().solve(intermediate_Lx).transpose();

    // SPKF Step 2b: Measurement state update
    xhat = xhat + Lx.real() * (ztrue.col(t) - zhat); // update prediction to estimate

    //SPKF Step 2c: Measurement covariance update
    Eigen::LLT<Eigen::MatrixXcd> chol(srSigmaXm);

    intermediate_Lx.transposeInPlace();
    for (int i = 0; i < Nz; i++) {
        chol.rankUpdate((intermediate_Lx).col(i), -1.0);
    }

    srSigmaXm = chol.matrixL();
    sigmaX = srSigmaXm.real();
}

Eigen::VectorXd srSPKF::Model(const Eigen::VectorXd X) {
    Eigen::Matrix<double, 5, 5> A;
    if (fabs(X(4)) < 1e-8) {

        A << 1.0, 1.0, 0, 0, 0,
            0, 1.0, 0, 0, 0,
            0, 0, 1.0, 1.0, 0,
            0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 1.0;
    }
    else {
        double w = X(4), sw = std::sin(X(4)), cw = std::cos(X(4));
        A << 1.0, sw / w, 0, (cw - 1.0) / w, 0,
            0, cw, 0, -sw, 0,
            0, (1.0 - cw) / w, 1.0, sw / w, 0,
            0, sw, 0, cw, 0,
            0, 0, 0, 0, 1.0;
    }
    Eigen::MatrixXd x = X.segment(0, Nx);
    Eigen::MatrixXd u = X.segment(Nx, 3);

    return A * x + B * u;
}

Eigen::VectorXd srSPKF::Model_output(const Eigen::VectorXd X) {

    Eigen::VectorXd Z = Eigen::VectorXd::Zero(2, 1);

    Z(0) = sqrt(X(0) * X(0) + X(2) * X(2)) + X(8); // radius + measurement noise
    Z(1) = std::atan2(X(2), X(0)) + X(9); // angle + measurment noise
    return Z;
}

void srSPKF::GetTrueValues() {

    Eigen::VectorXd X = Eigen::VectorXd::Zero(Nxa);
    Eigen::VectorXd x_prev = xtrue.col(0);
    for (int i = 0; i < maxIter; i++) {
        Eigen::Vector3d vec1;
        vec1 << randn(gen), randn(gen), randn(gen);

        Eigen::Vector2d vec2;
        vec2 << randn(gen), randn(gen);

        X.segment(0, Nx) = x_prev;
        X.segment(Nx, Nw) = sigmaW * vec1;
        X.segment(Nx + Nw, Nv) = sigmaV * vec2;

        xtrue.col(i) = Model(X);
        ztrue.col(i) = Model_output(X);
        x_prev = xtrue.col(i);
    }
}