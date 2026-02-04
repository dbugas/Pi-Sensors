#include "srKF.h"

srKF::srKF(Eigen::MatrixXd sigmax, Eigen::MatrixXd sigmaV_, Eigen::MatrixXd sigmaW_, Eigen::VectorXd xhat0, double dt)
:
T(dt)
{
	sigmaX = sigmax;
	sigmaV = sigmaV_;
	sigmaW = sigmaW_;
	xhat = xhat0;
	zhat = Eigen::VectorXd::Zero(sigmaV_.cols());

	Ahat = Eigen::MatrixXd::Zero(sigmax.rows(), sigmax.cols());
	Chat = Eigen::MatrixXd::Zero(sigmaV_.cols(), sigmax.rows());
	I = Eigen::MatrixXd::Identity(sigmax.rows(), sigmax.rows());

	Dhat = Eigen::MatrixXd::Identity(sigmaV_.cols(), sigmaV_.cols());
    Bhat = Eigen::MatrixXd::Zero(sigmaW_.rows(), 1);
	Bhat << 0.5 * dt * dt,
		    dt, 
			1.0,
		    1.0;
    Eigen::Matrix<double, 4, 4> A;
    A << 1.0, dt, 1.0/2.0*dt*dt, 1.0/2.0*dt*dt, 
           0.0, 1.0, dt, dt,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 0.997; 
    Ahat = A;
}

void srKF::KF_step(const Eigen::Ref<Eigen::VectorXd> ztrue)
{
	// KF Prediction step: compute Ahat, Bhat, xhat
	//Model();
    // KF Error covariance time update
	sigmaX = Ahat * sigmaX * Ahat.transpose() + Bhat*sigmaW*Bhat.transpose();
	//// KF Estimate system output: Chat, Dhat, zhat
	//Model_output();
	// Compute Kalman gain matrix 
    sigmaZ = Chat * sigmaX * Chat.transpose() + Dhat * sigmaV * Dhat.transpose();
    L = (sigmaX * Chat.transpose()) * sigmaZ.ldlt().solve(
        Eigen::MatrixXd::Identity(sigmaZ.rows(), sigmaZ.cols()));
	// KF Measurement update
	xhat = xhat + L * (ztrue - zhat);
	// KF Error covariance measurement update
	sigmaX = (I - L * Chat) * sigmaX * (I - L * Chat).transpose() + L * sigmaV * L.transpose();
    sigmaX = 0.5 * (sigmaX + sigmaX.transpose());
}
void srKF::KF_predict_alt(const double meas_az)
{
	xhat(2) = meas_az - xhat(3);
	xhat = Ahat * xhat;

    sigmaX = Ahat * sigmaX * Ahat.transpose() + sigmaW;
	//std::cout << "sigmaX (pred): \n" << sigmaX << "\n";
	//std::cout << "xhat (pred): \n" << xhat << "\n\n";
}

void srKF::KF_update_alt(const Eigen::VectorXd zmeas, const double temp)
{
    double s = 29.24032374*temp; // hypsometric equation constants
	double zhat_alt = p0 * std::exp(-xhat(0) / s);
	Chat << -zhat_alt/s, 0.0, 0.0, 0.0;

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
    //else {
	//	std::cout << "Measurement rejected by validation gate. NEEZ = " << neez << "\n";
    //}

}

void srKF::SageHusa_update_alt()
{
	Eigen::MatrixXd post_residual_cov = nu * nu.transpose();
    sigmaV = d_R * sigmaV + (1.0 - d_R) * (post_residual_cov - Chat * sigmaX * Chat.transpose());
    //sigmaW = d_W * sigmaW + (1.0 - d_W) * (L * post_residual_cov * L.transpose() + sigmaX - Ahat * sigmaX * Ahat.transpose());

	//std::cout << "Updated sigmaV: \n" << sigmaV << "\n";

}
