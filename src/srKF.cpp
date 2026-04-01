#include "srKF.h"

srKF::srKF(const Eigen::Matrix4d& sigmax, const double sigmaV_, const Eigen::Matrix4d& sigmaW_, const Eigen::Vector4d& xhat0, double dt)
:
T(dt)
{
	sigmaX = sigmax;
	sigmaV = sigmaV_;
	sigmaW = sigmaW_;
	xhat = xhat0;
	zhat = 0.0;

	Ahat.setZero();
	Chat.setZero();
	I.setIdentity();

	Dhat.setIdentity();
    Bhat.setZero();
    Bhat(0,0) = 0.5 * dt * dt;
    Bhat(1,0) = dt;
    Bhat(2,0) = 1.0;
    Bhat(3,0) = 1.0;

    Eigen::Matrix<double, 4, 4> A;
    A << 1.0, dt, 1.0/2.0*dt*dt, 1.0/2.0*dt*dt, 
           0.0, 1.0, dt, dt,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 0.997; 
    Ahat = A;
}

void srKF::KF_predict_alt(const double meas_az)
{
	xhat(2) = meas_az - xhat(3);
	xhat = Ahat * xhat;

    sigmaX = Ahat * sigmaX * Ahat.transpose() + sigmaW;
}

void srKF::KF_update_alt(const double zmeas, const double temp)
{
    double s = 29.24032374*temp; // hypsometric equation constants
	double zhat_alt = p0 * std::exp(-xhat(0) / s);
	Chat << -zhat_alt/s, 0.0, 0.0, 0.0;

	zhat = zhat_alt;
	sigmaZ = Chat * sigmaX * Chat.transpose() + sigmaV;
    nu = zmeas - zhat;
	double neez = nu*nu / sigmaZ;
    // KF validation gate
    if (neez > 9.8175e-06 && neez < 9.1406) {
        L = (sigmaX * Chat.transpose()) / sigmaZ;

        xhat = xhat + L * nu;
        sigmaX = ((I - L * Chat) * sigmaX * (I - L * Chat).transpose() + L * sigmaV * L.transpose());
    }
    //else {
	//	std::cout << "Measurement rejected by validation gate. NEEZ = " << neez << "\n";
    //}

}

void srKF::SageHusa_update_alt()
{
	double post_residual_cov = nu * nu;
	double post_model_cov = Chat * sigmaX * Chat.transpose();
    sigmaV = d_R * sigmaV + (1.0 - d_R) * (post_residual_cov - post_model_cov);
    sigmaV = std::clamp(sigmaV,-500.0,500.0);
    //sigmaW = d_W * sigmaW + (1.0 - d_W) * (L * post_residual_cov * L.transpose() + sigmaX - Ahat * sigmaX * Ahat.transpose());

	//std::cout << "Updated sigmaV: " << sigmaV << "\n";

}
