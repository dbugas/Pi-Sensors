#include "KF.h"

KF::KF(Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0, double dt) : T(dt)
{
	sigmaX = sigmax;
	sigmaV = sigmaV_;
	sigmaW = sigmaW_;
	sigmaW_save = sigmaW_;
	xhat = xhat0;
	zhat = Eigen::VectorXd::Zero(sigmaV_.cols());

	Ahat = Eigen::MatrixXd::Zero(sigmax.rows(), sigmax.cols());
	Chat = Eigen::MatrixXd::Zero(sigmaV_.cols(), sigmax.rows());
	I = Eigen::MatrixXd::Identity(sigmax.rows(), sigmax.rows());
    Dhat = Eigen::MatrixXd::Identity(sigmaV_.cols(), sigmaV_.cols());

    Nx = Ahat.rows();
    Nw = sigmaW.cols();
	Nv = sigmaV.cols();
    sigx_temp = Eigen::MatrixXd::Zero(Nx + Nw, Nx);
	sigz_temp = Eigen::MatrixXd::Zero(Nx + Nv, Nv);
	
    /*
    Bhat = Eigen::MatrixXd::Zero(Nx, Nw);
    Bhat << 0.5, 0, 0,
        1.0, 0, 0,
        0, 0.5, 0,
        0, 1.0, 0,
        0, 0, 1.0;
*/
    
	Dhat = Eigen::MatrixXd::Identity(sigmaV_.cols(), sigmaV_.cols());
    Eigen::VectorXd temp(4);
    temp << 0.5 * dt * dt,
        dt,
        1.0,
        1.0;
    Bhat = temp.asDiagonal();
    Eigen::Matrix<double, 4, 4> A;
    A << 1.0, dt, 1.0 / 2.0 * dt * dt, 1.0 / 2.0 * dt * dt,
        0.0, 1.0, dt, dt,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.99501247919268232;
    Ahat = A;
    
}

void KF::srKF_step(const Eigen::Ref<Eigen::VectorXd> ztrue)
{
	// KF Prediction step: compute Ahat, Bhat, xhat
	Model();
    // KF Error covariance time update
    sigx_temp.topRows(Nx) = (Ahat * sigmaX).transpose();
    sigx_temp.bottomRows(Nw) = (Bhat * sigmaW).transpose();
    auto qr = sigx_temp.householderQr();

    Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(Nx, Nx).triangularView<Eigen::Upper>();
    sigmaX = R.transpose();
	//// KF Estimate system output: Chat, Dhat, zhat
	Model_output();

	// Compute Kalman gain matrix 
    sigz_temp.topRows(Nx) = (Chat * sigmaX).transpose();
    sigz_temp.bottomRows(Nv) = (Dhat * sigmaV).transpose();

    qr = sigz_temp.householderQr();

    R = qr.matrixQR().topLeftCorner(Nv, Nv).triangularView<Eigen::Upper>();
    sigmaZ = R.transpose();

    Eigen::MatrixXd sigmaZinv = R.triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(Nv, Nv));
    L = (sigmaX*sigmaX.transpose() * Chat.transpose() * sigmaZinv.transpose()) * sigmaZinv;
	// KF Measurement update
	xhat = xhat + L * (ztrue - zhat);
	// KF Error covariance measurement update
    Eigen::LLT<Eigen::MatrixXd> chol(sigmaX);
    Eigen::MatrixXd intermediate = L * sigmaZ;
    
    for(int i = 0; i < Nv; i++) {
        chol.rankUpdate((intermediate).col(i), -1.0);
    }
    sigmaX = chol.matrixL();
    //std::cout << "sigmaX: \n" << sigmaX << "\n";
    //std::cout << "Ahat: \n" << Ahat << "\n";
    //std::cout << "Chat: \n" << Chat << "\n";
    //std::cout << "L: \n" << L << "\n";
    //std::cout << "xhat: \n" << xhat << "\n\n";
}

void KF::KF_step(const Eigen::Ref < Eigen::Vector<double, 2>> ztrue)
{
    // KF Prediction step: compute Ahat, Bhat, xhat
    Model();
    // KF Error covariance time update
    sigmaX = Ahat * sigmaX * Ahat.transpose() + Bhat * sigmaW * Bhat.transpose();
    //// KF Estimate system output: Chat, Dhat, zhat
    Model_output();
    // Compute Kalman gain matrix 
    sigmaZ = Chat * sigmaX * Chat.transpose() + Dhat * sigmaV * Dhat.transpose();
    L = (sigmaX * Chat.transpose()) * sigmaZ.ldlt().solve(
        Eigen::MatrixXd::Identity(sigmaZ.rows(), sigmaZ.cols()));
    // KF Measurement update
    xhat = xhat + L * (ztrue - zhat);
    // KF Error covariance measurement update
    sigmaX = (I - L * Chat) * sigmaX * (I - L * Chat).transpose() + L * sigmaV * L.transpose();

    //std::cout << "sigmaX: \n" << sigmaX << "\n";
    //std::cout << "Ahat: \n" << Ahat << "\n";
    //std::cout << "Chat: \n" << Chat << "\n";
    //std::cout << "L: \n" << L << "\n";
    //std::cout << "xhat: \n" << xhat << "\n\n";
}

void KF::KF_predict_alt(const double meas_az)
{
	xhat(2) = meas_az - xhat(3);
	xhat = Ahat * xhat;

    sigmaX = Ahat * sigmaX * Ahat.transpose() + sigmaW;
	//std::cout << "sigmaX (pred): \n" << sigmaX << "\n";
	//std::cout << "xhat (pred): \n" << xhat << "\n\n";
}

void KF::KF_update_alt(const Eigen::VectorXd zmeas)
{
    double s = 29.24032374*(288.15 - 0.0065 * xhat(0));
	double zhat_alt = 101325.0 * std::exp(-xhat(0) / s);
	Chat << -(101325.0 / s) * std::exp(-xhat(0) / s), 0.0, 0.0, 0.0;

	zhat << zhat_alt;
	sigmaZ = Chat * sigmaX * Chat.transpose() + sigmaV;
    nu = zmeas - zhat;
	double neez = nu.transpose() * sigmaZ.ldlt().solve(nu);
    // KF validation gate
    if (neez > 9.8175e-06 && neez < 9.1406) {
        L = (sigmaX * Chat.transpose()) * sigmaZ.ldlt().solve(
            Eigen::MatrixXd::Identity(sigmaZ.rows(), sigmaZ.cols()));

        xhat = xhat + L * nu;
        sigmaX = ((I - L * Chat) * sigmaX * (I - L * Chat).transpose() + L * sigmaV * L.transpose());
    }
    else {
		std::cout << "Measurement rejected by validation gate. NEEZ = " << neez << "\n";
    }

	//std::cout << "sigmaX (upd): \n" << sigmaX << "\n";
	//std::cout << "xhat (upd): \n" << xhat << "\n\n";
	//std::cout << "L: \n" << L << "\n";
    //std::cout << "sigmaZ: \n" << sigmaZ << "\n";
	//std::cout << "Chat: \n" << Chat << "\n";
	//std::cout << "zhat: \n" << zhat << "\n\n";

}

void KF::SageHusa_update_alt()
{
	Eigen::MatrixXd post_residual_cov = nu * nu.transpose();
    sigmaV = d_R * sigmaV + (1.0 - d_R) * (post_residual_cov - Chat * sigmaX * Chat.transpose());
    //sigmaW = d_W * sigmaW + (1.0 - d_W) * (L * post_residual_cov * L.transpose() + sigmaX - Ahat * sigmaX * Ahat.transpose());

	//std::cout << "Updated sigmaW: \n" << sigmaW << "\n\n";

}

void KF::srSageHusa_update_alt()
{
    double alpha = std::sqrt(d_R);
    double beta = std::sqrt(1.0 - d_R);

    Eigen::VectorXd v = std::sqrt(1.0 - d_R) * nu;
    Eigen::LLT<Eigen::MatrixXd> chol(alpha*sigmaV*sigmaV.transpose());
    chol.rankUpdate(v, 1.0);

    Eigen::MatrixXd U = std::sqrt(1.0 - d_R) * Chat * sigmaX;

	for (int i = 0; i < Nv; ++i) {
		chol.rankUpdate(U.col(i), -1.0);
	}

}

void KF::Model()
{
    const double omega = xhat(4);  

    Eigen::Matrix<double, 5, 5> A;
    if(fabs(omega) < 1e-8) {
        A << 1.0, 1.0, 0, 0, 0,
            0, 1.0, 0, 0, 0,
            0, 0, 1.0, 1.0, 0,
            0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 1.0;
    }
    else {
        double w = omega, sw = std::sin(w), cw = std::cos(w);
        A << 1.0, sw / w, 0, (cw - 1.0) / w, 0,
            0, cw, 0, -sw, 0,
            0, (1.0 - cw) / w, 1.0, sw / w, 0,
            0, sw, 0, cw, 0,
            0, 0, 0, 0, 1.0;
    }

    const double c = std::cos(omega * T);
    const double s = std::sin(omega * T);
    const double oo = omega * omega;

    // f1 – f4 correction terms
    // xi, xi_dot, eta, eta_dot, omega
    double f1 = c*T*xhat(1)/omega - s*xhat(1)/oo - s*T*xhat(3)/omega - (c-1.0)*xhat(3)/oo;
    double f2 = -s*T*xhat(1) - c*T*xhat(3);
    double f3 = s*T*xhat(1)/omega - (1.0-c)*xhat(1)/oo + c*T*xhat(3)/omega - s*xhat(3)/oo;
    double f4 = c*T*xhat(1) - s*T*xhat(3);

    Ahat << 1.0, s / omega, 0.0, (c - 1.0) / omega, f1,
        0.0, c, 0.0, -s, f2,
        0.0, (1.0 - c) / omega, 1.0, s / omega, f3,
        0.0, s, 0.0, c, f4,
        0.0, 0.0, 0.0, 0.0, 1.0;

    xhat = A * xhat;
    //std::cout << "A: \n" << A << "\n";
}

void KF::Model_output()
{
	double d = sqrt(xhat(0) * xhat(0) + xhat(2) * xhat(2));
	Chat << xhat(0)/d, 0.0, xhat(2)/d, 0.0, 0.0,
		-xhat(2)/(d*d), 0.0, xhat(0)/(d*d), 0.0, 0.0;

	zhat << d, atan2(xhat(2), xhat(0));
}
