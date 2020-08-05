/****************************************************************************
 *
 *   Copyright (c) 2020 Jonathan Cacace <jonathan.cacace@gmail.com>
 *	 All rights reserved.
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

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <lib/flight_tasks/FlightTasks.hpp>
#include <drivers/drv_pwm_output.h>



class LeeController : public ModuleBase<LeeController>, public ModuleParams, public px4::WorkItem
{
public:
	LeeController();
	~LeeController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Get the landing gear state based on the manual control switch position
	 * @return vehicle_attitude_setpoint_s::LANDING_GEAR_UP or vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN
	 */
	float		get_landing_gear_state();

	void start_flight_task();
	void poll_subscriptions();
	void send_vehicle_cmd_do(uint8_t nav_state);
	void check_failure(bool task_failure, uint8_t nav_state);
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
	uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _motor_limits_sub{ORB_ID(multirotor_motor_limits)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};				/**< vehicle attitude */


	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	//uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	//uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _home_pos_sub{ORB_ID(home_position)}; 			/**< home position */
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};	/**< vehicle local position */

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	//uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */
	uORB::Publication<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};	/**< controller status publication */
	uORB::Publication<landing_gear_s>		_landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */
	//uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs), ORB_PRIO_DEFAULT};
	uORB::Publication<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs)};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};


	landing_gear_s 			_landing_gear{};
	manual_control_setpoint_s	_manual_control_sp{};
	vehicle_control_mode_s		_v_control_mode{};
	vehicle_status_s		_vehicle_status{};
	home_position_s	_home_pos{};			/**< home position */
	vehicle_local_position_s _local_pos{};			/**< vehicle local position */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */

	vehicle_land_detected_s _vehicle_land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};


	bool _actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */
	bool _landed{true};
	bool _maybe_landed{true};

	float _battery_status_scale{0.0f};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	bool _gear_state_initialized{false};		/**< true if the gear state has been initialized */

	hrt_abstime _last_run{0};

	uORB::PublicationQueued<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	 /**< vehicle command publication */

	static constexpr int NUM_FAILURE_TRIES = 10;

	Vector3f _ddp;
	Vector3f _ep;
	Vector3f _edp;
	Vector3f _kp;
	Vector3f _kdp;
	float _mass;
	float _gravity;

	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P>) _param_mpc_xy_vel_p,
		(ParamFloat<px4::params::MPC_XY_VEL_I>) _param_mpc_xy_vel_i,
		(ParamFloat<px4::params::MPC_XY_VEL_D>) _param_mpc_xy_vel_d,
		(ParamFloat<px4::params::MPC_Z_VEL_P>) _param_mpc_z_vel_p,
		(ParamFloat<px4::params::MPC_Z_VEL_I>) _param_mpc_z_vel_i,
		(ParamFloat<px4::params::MPC_Z_VEL_D>) _param_mpc_z_vel_d,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>) _param_mpc_use_hte,

		// Takeoff / Land
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>) _param_mpc_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t, /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode,
		(ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _param_mpc_tiltmax_lnd, /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,


		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	)

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

	vehicle_angular_velocity_s _angular_velocity;
	Vector3f _normalized_attitude_gain;
	Vector3f _normalized_angular_rate_gain;

	matrix::SquareMatrix<float, 4> _allocation_M; //TODO: find a set the dim dynamically
	matrix::SquareMatrix<float, 4> _wd2rpm; //TODO: find a set the dim dynamically



	FlightTasks _flight_tasks; /**< class generating position controller setpoints depending on vehicle task */
	int _task_failure_count{0};         /**< counter for task failures */
	vehicle_control_mode_s	_control_mode{};		/**< vehicle control mode */


};
		