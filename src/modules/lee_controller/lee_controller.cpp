/****************************************************************************
 *
 *   Copyright (c) 2020 Jonathan Cacace <jonathan.cacace@gmail.com>
 *   All rights reserved.
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


#include "lee_controller.hpp"

using namespace matrix;

LeeController::LeeController() : SuperBlock(nullptr, "LeeMPC"), ModuleParams( nullptr ), 
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_loop_perf( perf_alloc( PC_ELAPSED, MODULE_NAME": cycle time")) {

}

LeeController::~LeeController() {
	perf_free(_loop_perf);
}

void
LeeController::poll_subscriptions()
{
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_control_mode_sub.update(&_control_mode);
	_home_pos_sub.update(&_home_pos);

	if (_att_sub.updated()) {
		vehicle_attitude_s att;

		if (_att_sub.copy(&att) && PX4_ISFINITE(att.q[0])) {
			_states.yaw = Eulerf(Quatf(att.q)).psi();
		}
	}
}

void LeeController::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void LeeController::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	_pub_vehicle_command.publish(command);
}


void
LeeController::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	if (_vehicle_status.in_transition_mode) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}
	
	// offboard
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_control_mode.flag_control_altitude_enabled ||
		_control_mode.flag_control_position_enabled ||
		_control_mode.flag_control_climb_rate_enabled ||
		_control_mode.flag_control_velocity_enabled ||
		_control_mode.flag_control_acceleration_enabled)) {

		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Descend activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	}

	// manual position control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;
		error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
		
		//switch (_param_mpc_pos_mode.get()) {
		//case 1:
		//	error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
		//	break;

		//case 3:
		//	error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
		//	break;

		//default:
		//	error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
		//	break;
		//}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;
		error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
	

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				printf("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
	
}



void LeeController::Run() {

	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_local_pos_sub.update(&_local_pos)) {

		//Local pose: NED Frame
		printf("Local pos: %f %f %f\n", (double)_local_pos.x, (double)_local_pos.y, (double)_local_pos.z);
	
	
		poll_subscriptions();
		//parameters_update(false);
		// set _dt in controllib Block - the time difference since the last loop iteration in seconds
		const hrt_abstime time_stamp_now = hrt_absolute_time();
		setDt((time_stamp_now - _time_stamp_last_loop) / 1e6f);
		_time_stamp_last_loop = time_stamp_now;
		//const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;
		// switch to the required flighttask
		start_flight_task();
		
		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {

			// setpoints and constraints for the position controller from flighttask or failsafe
			vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
			vehicle_constraints_s constraints = FlightTask::empty_constraints;


			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				//failsafe(setpoint, _states, false, !was_in_failsafe);

			} else {


				setpoint = _flight_tasks.getPositionSetpoint(); //p_des, dp_des, yaw_des (ddp_des?)


				//printf("Setpoint  task: %f %f %f %f %f %f %f\n", (double)setpoint.x, (double)setpoint.y, (double)setpoint.z, (double)setpoint.yaw, (double)setpoint.vx, (double)setpoint.vy, (double)setpoint.vz);
				constraints = _flight_tasks.getConstraints();

				/*

					WRITE HERE YOUR CONTROLLER?!


				*/


				//Publish PWM at the end!
				_pwm_out.timestamp = hrt_absolute_time();
				_pwm_out.noutputs = 4;
				_pwm_out.pwm_signal[0] = 1000;
				_pwm_out.pwm_signal[1] = 1000;
				_pwm_out.pwm_signal[2] = 1000;
				_pwm_out.pwm_signal[3] = 1000;
				_pwm_pub.publish( _pwm_out );

				//_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);
			}









		}
		//else printf("No task active \n");





	}

	perf_end(_loop_perf);

}

bool LeeController::init() {

	/*
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}
	*/

	//TODO check this
	// limit to every other vehicle_local_position update (~62.5 Hz)
	_local_pos_sub.set_interval_us(16_ms);

	_time_stamp_last_loop = hrt_absolute_time();

	return true;
}

int LeeController::task_spawn(int argc, char *argv[]) {
	LeeController *instance = new LeeController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;	
		}

	}
	else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}



int LeeController::print_usage(const char *reason)
{
	return 0;
}

int LeeController::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int LeeController::print_flight_task_usage(const char *reason) {
	return 0;
}

int lee_controller_main(int argc, char *argv[]) {

	return LeeController::main(argc, argv);
	return 0;
}
