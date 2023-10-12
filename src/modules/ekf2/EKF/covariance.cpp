/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/predict_covariance.h>
#include <ekf_derivation/generated/rot_var_ned_to_lower_triangular_quat_cov.h>

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	resetQuatCov();

	// velocity
#if defined(CONFIG_EKF2_GNSS)
	const float vel_var = sq(fmaxf(_params.gps_vel_noise, 0.01f));
#else
	const float vel_var = sq(0.5f);
#endif
	P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(vel_var, vel_var, sq(1.5f) * vel_var));

	// position
	float z_pos_var = sq(fmaxf(_params.baro_noise, 0.01f));
#if defined(CONFIG_EKF2_GNSS)
	const float xy_pos_var = sq(fmaxf(_params.gps_pos_noise, 0.01f));

	if (_control_status.flags.gps_hgt) {
		z_pos_var = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));
	}
#else
	const float xy_pos_var = sq(fmaxf(_params.pos_noaid_noise, 0.01f));
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		z_pos_var = sq(fmaxf(_params.range_noise, 0.01f));
	}
#endif // CONFIG_EKF2_RANGE_FINDER

	P.uncorrelateCovarianceSetVariance<State::pos.dof>(State::pos.idx, Vector3f(xy_pos_var, xy_pos_var, z_pos_var));

	// gyro bias
	P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, sq(_params.switch_on_gyro_bias));

	// accel bias
	P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, sq(_params.switch_on_accel_bias));

#if defined(CONFIG_EKF2_MAGNETOMETER)
	resetMagCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// wind
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
#endif // CONFIG_EKF2_WIND
}

void Ekf::predictCovariance(const imuSample &imu_delayed)
{
	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = _dt_ekf_avg;

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(imu_delayed.delta_ang.norm() / imu_delayed.delta_ang_dt, beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(imu_delayed.delta_vel.norm() / imu_delayed.delta_vel_dt, beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * imu_delayed.delta_vel / imu_delayed.delta_vel_dt + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					     || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	// gyro bias inhibit
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias))
					      || is_manoeuvre_level_high;

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		// TODO: gyro bias conditions
		bool is_bias_observable = true;
		_gyro_bias_inhibit[index] = do_inhibit_all_gyro_axes || !is_bias_observable;
	}

	// accel bias inhibit
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					       || is_manoeuvre_level_high
					       || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {

		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		_accel_bias_inhibit[index] = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;
	}

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float d_ang_var = sq(imu_delayed.delta_ang_dt * gyro_noise);

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	Vector3f d_vel_var;

	for (unsigned i = 0; i < 3; i++) {
		if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			d_vel_var(i) = sq(imu_delayed.delta_vel_dt * BADACC_BIAS_PNOISE);

		} else {
			d_vel_var(i) = sq(imu_delayed.delta_vel_dt * accel_noise);
		}
	}

	// predict the covariance
	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	P = sym::PredictCovariance(_state.vector(), P,
				   imu_delayed.delta_vel, imu_delayed.delta_vel_dt, d_vel_var,
				   imu_delayed.delta_ang, imu_delayed.delta_ang_dt, d_ang_var);

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// gyro bias: add process noise unless state is inhibited
	const float gyro_bias_sig = dt * math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	const float gyro_bias_process_noise = sq(gyro_bias_sig);

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		const unsigned i = State::gyro_bias.idx + index;

		if (!_gyro_bias_inhibit[index]) {
			P(i, i) += gyro_bias_process_noise;
		}
	}

	// accel bias: add process noise unless state is inhibited
	const float accel_bias_sig = dt * math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);
	const float accel_bias_process_noise = sq(accel_bias_sig);

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		const unsigned i = State::accel_bias.idx + index;

		if (!_accel_bias_inhibit[index]) {
			P(i, i) += accel_bias_process_noise;
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag) {
		// mag_I: add process noise
		float mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.f, 1.f);
		P.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) += sq(mag_I_sig);

		// mag_B: add process noise
		float mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.f, 1.f);
		P.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) += sq(mag_B_sig);
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	if (_control_status.flags.wind) {
		// wind vel: add process noise
		float wind_vel_nsd_scaled = math::constrain(_params.wind_vel_nsd, 0.f, 1.f) * (1.f + _params.wind_vel_nsd_scaler * fabsf(_height_rate_lpf));
		P.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) += sq(wind_vel_nsd_scaled) * dt;
	}
#endif // CONFIG_EKF2_WIND

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < row; column++) {
			P(row, column) = P(column, row);
		}
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);
}

void Ekf::fixCovarianceErrors(bool force_symmetry)
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)

	constrainStateVar(State::quat_nominal, 1e-7f, 1.f);
	constrainStateVar(State::vel, 1e-6f, 1e6f);
	constrainStateVar(State::pos, 1e-6f, 1e6f);

	// gyro_bias: the ratio of a max and min variance must not exceed 100
	const float gyro_bias_var_max = math::constrain(getStateVariance<State::gyro_bias>().max(), 1e-7f, 1.f);
	const float gyro_bias_var_min = gyro_bias_var_max * 0.01f; // the ratio of a max and min variance must not exceed 100
	constrainStateVar(State::gyro_bias, gyro_bias_var_min, gyro_bias_var_max);

	// accel_bias: the ratio of a max and min variance must not exceed 100
	const float accel_bias_var_max = math::constrain(getStateVariance<State::accel_bias>().max(), 1e-7f, 1.f);
	const float accel_bias_var_min = accel_bias_var_max * 0.01f; // the ratio of a max and min variance must not exceed 100
	constrainStateVar(State::accel_bias, accel_bias_var_min, accel_bias_var_max);

	if (!_accel_bias_inhibit[0] || !_accel_bias_inhibit[1] || !_accel_bias_inhibit[2]) {
		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const Vector3f delta_vel_bias = _state.accel_bias * _dt_ekf_avg;
		const float down_dvel_bias = delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = false;
		if (fabsf(down_dvel_bias) > dVel_bias_lim) {
#if defined(CONFIG_EKF2_GNSS)
			bool bad_vz_gps = _control_status.flags.gps    && (down_dvel_bias * _aid_src_gnss_vel.innovation[2] < 0.0f);
#else
			bool bad_vz_gps = false;
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
			bool bad_vz_ev  = _control_status.flags.ev_vel && (down_dvel_bias * _aid_src_ev_vel.innovation[2] < 0.0f);
#else
			bool bad_vz_ev  = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

			if (bad_vz_gps || bad_vz_ev) {
#if defined(CONFIG_EKF2_BAROMETER)
				bool bad_z_baro = _control_status.flags.baro_hgt && (down_dvel_bias * _aid_src_baro_hgt.innovation < 0.0f);
#else
				bool bad_z_baro = false;
#endif
#if defined(CONFIG_EKF2_GNSS)
				bool bad_z_gps  = _control_status.flags.gps_hgt  && (down_dvel_bias * _aid_src_gnss_hgt.innovation < 0.0f);
#else
				bool bad_z_gps  = false;
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
				bool bad_z_rng  = _control_status.flags.rng_hgt  && (down_dvel_bias * _aid_src_rng_hgt.innovation  < 0.0f);
#else
				bool bad_z_rng  = false;
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				bool bad_z_ev   = _control_status.flags.ev_hgt   && (down_dvel_bias * _aid_src_ev_hgt.innovation   < 0.0f);
#else
				bool bad_z_ev   = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

				if (bad_z_baro || bad_z_gps || bad_z_rng || bad_z_ev) {
					bad_acc_bias = true;
				}
			}
		}

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_delayed_us;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<State::accel_bias.dof>(State::accel_bias.idx);

			_time_acc_bias_check = _time_delayed_us;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_I
	if (_control_status.flags.mag) {
		// mag_I: the ratio of a max and min variance must not exceed 100
		const float var_max = math::constrain(getStateVariance<State::mag_I>().max(), 1e-6f, 1.f);
		const float var_min = var_max * 0.01f;
		constrainStateVar(State::mag_I, var_min, var_max);
	}

	if (force_symmetry) {
		P.makeRowColSymmetric<State::mag_I.dof>(State::mag_I.idx);
	}

	// mag_B
	if (_control_status.flags.mag) {
		// mag_B: the ratio of a max and min variance must not exceed 100
		const float var_max = math::constrain(getStateVariance<State::mag_B>().max(), 1e-6f, 1.f);
		const float var_min = var_max * 0.01f;
		constrainStateVar(State::mag_B, var_min, var_max);
	}

	if (force_symmetry) {
		P.makeRowColSymmetric<State::mag_B.dof>(State::mag_B.idx);
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// wind_vel
	if (_control_status.flags.wind) {
		constrainStateVar(State::wind_vel, 0.f, 1.f);
	}

	if (force_symmetry) {
		P.makeRowColSymmetric<State::wind_vel.dof>(State::wind_vel.idx);
	}
#endif // CONFIG_EKF2_WIND

	if (force_symmetry) {
		P.makeRowColSymmetric<State::quat_nominal.dof>(State::quat_nominal.idx);
		P.makeRowColSymmetric<State::vel.dof>(State::vel.idx);
		P.makeRowColSymmetric<State::pos.dof>(State::pos.idx);
		P.makeRowColSymmetric<State::gyro_bias.dof>(State::gyro_bias.idx);
		P.makeRowColSymmetric<State::accel_bias.dof>(State::accel_bias.idx);
	}
}

void Ekf::constrainStateVar(const IdxDof &state, float min, float max)
{
	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		P(i, i) = math::constrain(P(i, i), min, max);
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrixState &KHP)
{
	bool healthy = true;

	for (int i = 0; i < State::size; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}

	return healthy;
}

void Ekf::resetQuatCov(const float yaw_noise)
{
	const float tilt_var = sq(math::max(_params.initial_tilt_err, 0.01f));
	float yaw_var = sq(0.01f);

	// update the yaw angle variance using the variance of the measurement
	if (PX4_ISFINITE(yaw_noise)) {
		// using magnetic heading tuning parameter
		yaw_var = math::max(sq(yaw_noise), yaw_var);
	}

	resetQuatCov(Vector3f(tilt_var, tilt_var, yaw_var));
}

void Ekf::resetQuatCov(const Vector3f &rot_var_ned)
{
	matrix::SquareMatrix<float, State::quat_nominal.dof> q_cov;
	sym::RotVarNedToLowerTriangularQuatCov(_state.vector(), rot_var_ned, &q_cov);
	q_cov.copyLowerToUpperTriangle();
	resetStateCovariance<State::quat_nominal>(q_cov);
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void Ekf::resetMagCov()
{
	if (_mag_decl_cov_reset) {
		ECL_INFO("reset mag covariance");
		_mag_decl_cov_reset = false;
	}

	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.mag_noise));
}
#endif // CONFIG_EKF2_MAGNETOMETER

void Ekf::resetGyroBiasZCov()
{
	P.uncorrelateCovarianceSetVariance<1>(State::gyro_bias.idx + 2, sq(_params.switch_on_gyro_bias));
}
