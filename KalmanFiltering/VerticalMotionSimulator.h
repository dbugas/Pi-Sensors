#pragma once

#include <random>
#include <cmath>
#include <cstdint>
#include <chrono>

constexpr double PI = 3.141592653589793;

class VerticalMotionSimulator {
public:
    struct TrueState {
        double z = 0.0;          // altitude [m]
        double v = 0.0;          // vertical velocity [m/s]
        double a = 0.0;          // true acceleration [m/s^2]
        double pressure = 0.0;   // true pressure [Pa]
        double accel_bias = 0.0; // accelerometer bias [m/s^2]
    };

    struct Measurements {
        bool accel_valid = false;
        bool pressure_valid = false;

        double accel = 0.0;
        double pressure = 0.0;
    };

    VerticalMotionSimulator(
        double dt_truth,
        double dt_accel,
        double dt_pressure,
        double gravity,
        double p0,
        double vel_noise_std,
        double accel_noise_std,
        double pressure_noise_std,
        double accel_bias_rw_std,
        uint32_t seed = std::chrono::system_clock::now().time_since_epoch().count()
    )
        : dt_(dt_truth),
        dt_accel_(dt_accel),
        dt_pressure_(dt_pressure),
        g_(gravity),
        p0_(p0),
        vel_noise_std_(vel_noise_std),
        accel_noise_std_(accel_noise_std),
        pressure_noise_std_(pressure_noise_std),
        accel_bias_rw_std_(accel_bias_rw_std),
        rng_(seed),
        vel_noise_(0.0, vel_noise_std),
        accel_noise_(0.0, accel_noise_std),
        pressure_noise_(0.0, pressure_noise_std),
        accel_bias_rw_(0.0, accel_bias_rw_std),
        uniform_(0.0, 1.0),
        jitter_(0.0, 1.0)
    {
    }

    // ---------------- Configuration toggles ----------------

    void enableGaussMarkovBias(double tau_seconds) {
        use_gm_bias_ = true;
        bias_tau_ = tau_seconds;
        phi_bias_ = std::exp(-dt_ / bias_tau_);
    }
    void enableWindGust(double Gust_prob, double gust_magnitude_) {
		Gust_prob_ = Gust_prob;
        Gust_Magnitude_ = gust_magnitude_;
    }
    void enablePressureLag(double tau_seconds) {
        use_pressure_lag_ = true;
        pressure_tau_ = tau_seconds;
    }

    void enableTimingJitter(double accel_jitter_std,
        double pressure_jitter_std,
        double drop_probability = 0.0) {
        accel_jitter_std_ = accel_jitter_std;
        pressure_jitter_std_ = pressure_jitter_std;
        drop_prob_ = drop_probability;
    }

    // ---------------- Initialization ----------------

    void setInitialState(double z0, double v0, double accel_bias0 = 0.0) {
        state_.z = z0;
        state_.v = v0;
        state_.a = g_;
        state_.accel_bias = accel_bias0;

        updateTruePressure();
        pressure_state_ = state_.pressure;
        temp_ = 288.15 - 0.0065 * state_.z;
        t_ = 0.0;
        t_last_accel_ = 0.0;
        t_last_pressure_ = 0.0;
    }

    // ---------------- Main step ----------------

    Measurements step() {
        Measurements meas;

        // ---- Truth propagation ----

        double da = vel_noise_(rng_);
		state_.a = g_*sin(t_*PI/1.0) + da;

        // Bias evolution
        if (use_gm_bias_) {
            state_.accel_bias =
                phi_bias_ * state_.accel_bias +
                accel_bias_rw_(rng_);
        }
        else {
            state_.accel_bias += accel_bias_rw_(rng_);
        }

        state_.z += state_.v * dt_ + 0.5 * state_.a * dt_ * dt_;
        state_.v += state_.a * dt_;

        updateTruePressure();

        // Pressure lag (sensor dynamics)
        if (use_pressure_lag_) {
            double alpha = dt_ / (pressure_tau_ + dt_);
            pressure_state_ += alpha * (state_.pressure - pressure_state_);
        }
        else {
            pressure_state_ = state_.pressure;
        }

        t_ += dt_;

        // ---- Accelerometer measurement ----
        if (t_ - t_last_accel_ >= nextAccelInterval()) {
            t_last_accel_ = t_;

            if (!dropMeasurement()) {
				double ori_error = (uniform_(rng_) * 0.4 + 0.8); // Random orientation error in [0.9, 1.1]
                meas.accel_valid = true;
                meas.accel = ori_error*state_.a + state_.accel_bias + accel_noise_(rng_);
            }
        }

        // ---- Pressure measurement ----
        if (t_ - t_last_pressure_ >= nextPressureInterval()) {
            t_last_pressure_ = t_;

            if (!dropMeasurement()) {
                meas.pressure_valid = true;
                meas.pressure =pressure_state_ + pressure_noise_(rng_);
                if (uniform_(rng_) < Gust_prob_ && !gust_active_) {
					gust_active_ = true;
					sgn = (uniform_(rng_) < 0.5) ? -1.0 : 1.0;
					Gust_time_remaining_ = Gust_duration_* uniform_(rng_);
				}
                if (gust_active_) {
                    std::normal_distribution<double> wind_gust_rng(sgn * Gust_Magnitude_, Gust_std_);
                    double gust = wind_gust_rng(rng_);
                    meas.pressure += gust;
					Gust_time_remaining_ -= dt_;
                    if (Gust_time_remaining_ <= 0.0) {
                        gust_active_ = false;
                        //Gust_prob_ = 0.0;
                    }
                }
            }
        }

        return meas;
    }

    const TrueState& trueState() const {
        return state_;
    }

    // ---------------- Helpers ----------------

    void updateTruePressure() {
        temp_ = 288.15 - 0.0065 * state_.z;
        s_ = 29.24032374 * temp_;
        state_.pressure = p0_ * std::exp(-state_.z / s_);
    }

    bool dropMeasurement() {
        return uniform_(rng_) < drop_prob_;
    }

    double nextAccelInterval() {
        return dt_accel_ + accel_jitter_std_ * jitter_(rng_);
    }

    double nextPressureInterval() {
        return dt_pressure_ + pressure_jitter_std_ * jitter_(rng_);
    }

private:
    // Time
    double dt_;
    double dt_accel_;
    double dt_pressure_;
    double t_ = 0.0;
    double t_last_accel_ = 0.0;
    double t_last_pressure_ = 0.0;

    // Physics
    double g_;
    double p0_;
    double temp_;
    double s_;

    // Noise
    double vel_noise_std_;
    double accel_noise_std_;
    double pressure_noise_std_;
    double accel_bias_rw_std_;

    // Bias model
    bool use_gm_bias_ = false;
    double bias_tau_ = 1.0;
    double phi_bias_ = 1.0;

    // Pressure dynamics
    bool use_pressure_lag_ = false;
    double pressure_tau_ = 0.0;
    double pressure_state_ = 0.0;

    // Timing realism
    double accel_jitter_std_ = 0.0;
    double pressure_jitter_std_ = 0.0;
    double drop_prob_ = 0.0;

    // Wind Gust
	double Gust_prob_ = 0.0;
	double Gust_Magnitude_ = 0.0;
    double Gust_std_ = 10.0;
	double Gust_duration_ = 1.0;
	double Gust_time_remaining_ = 0.0;
	bool gust_active_ = false;
    double sgn = 0.0;

    // RNG
    std::mt19937 rng_;
    std::normal_distribution<double> vel_noise_;
    std::normal_distribution<double> accel_noise_;
    std::normal_distribution<double> pressure_noise_;
    std::normal_distribution<double> accel_bias_rw_;
    std::normal_distribution<double> jitter_;
    std::uniform_real_distribution<double> uniform_;

    // State
    TrueState state_;
};
