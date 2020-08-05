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
 
#include "lee_controller.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

LeeController::LeeController() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")) {
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

LeeController::~LeeController() {
	perf_free(_loop_perf);
}

bool
LeeController::init() {
	//if (!_vehicle_angular_velocity_sub.registerCallback()) {
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}
	return true;
}



void LeeController::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}


Matrix3f QuatToMat( float qw, float  qx, float  qy, float  qz ){

	Matrix3f Rot;
	float s = qw;
	float x = qx;
	float y = qy;
	float z = qz;

	Rot(0,0) = 1-2*(y*y+z*z);
	Rot(0,1) = 2*(x*y-s*z);
	Rot(0,2) = 2*(x*z+s*y);

	Rot(1,0) = 2*(x*y+s*z);
	Rot(1,1) = 1-2*(x*x+z*z);
	Rot(1,2) = 2*(y*z-s*x);

	Rot(2,0) = 2*(x*z-s*y);
	Rot(2,1) = 2*(y*z+s*x);
	Rot(2,2) = 1-2*(x*x+y*y);

	return Rot;
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
	FlightTaskError error;
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	if (_vehicle_status.in_transition_mode) {
		should_disable_task = false;
		error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
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
		error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}


	if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		error = FlightTaskError::NoError;
		error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Descend activation failed with error: %s", _flight_tasks.errorToString(error));
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
		error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} 
		else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}
	
	// manual altitude control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} 
		else {
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
		error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} 
	else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
	
}

void
LeeController::parameters_updated()
{
	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);


	/*
	I: 
	0.0347563         0         0         0
		0 0.0458929         0         0
		0         0    0.0977         0
		0         0         0         1
	Iinv: 28.7718      -0       0      -0
		-0 21.7899      -0       0
		0      -0 10.2354      -0
		-0       0      -0       1
	_attitude_gain:    2
	3
	0.15
	_angular_rate_gain:  0.4
	0.52
	0.18
	normalized_attitude_gain: 57.5435
	65.3696
	1.53531
	normalized_angular_rate_gain: 11.5087
	11.3307
	1.84237
	*/

    SquareMatrix<float, 4> T_i;
	T_i(0,0) = 0.03475;
	T_i(1,1) = 0.04589;
	T_i(2,2) = 0.0977;
	T_i(3,3) = 1.0;
	
    SquareMatrix<float, 3> I;
	I(0,0) = 0.03475;
	I(1,1) = 0.04589;
	I(2,2) = 0.0977;
	


	//Vector3f attitude_gain = Vector3f(2.0, 3.0, 0.15);
	//Vector3f angular_rate_gain = Vector3f(0.4, 0.52, 0.18);

	//for(int i=0; i<3; i++ ) {
	//	for( int j=0; j<3; j++ ) {
	//		printf("M[%d,%d]: %f - ", i, j, (double)I.I()(i,j));
	//	}
	//}

	Matrix<float, 1, 3> attitude_gain;
	attitude_gain(0,0) = 2.0;
	attitude_gain(0,1) = 3.0;
	attitude_gain(0,2) = 0.15;
	Matrix<float, 1, 3> normalized_attitude_gain = attitude_gain*I.I(); //  I.I();

	_normalized_attitude_gain(0) = normalized_attitude_gain(0,0);
	_normalized_attitude_gain(1) = normalized_attitude_gain(0,1);
	_normalized_attitude_gain(2) = normalized_attitude_gain(0,2);


	Matrix<float, 1, 3> angular_rate_gain;
	angular_rate_gain(0,0) = 0.4;
	angular_rate_gain(0,1) = 0.52;
	angular_rate_gain(0,2) = 0.18;
	Matrix<float, 1, 3> normalized_angular_rate_gain = angular_rate_gain*I.I();

	_normalized_angular_rate_gain(0) = normalized_angular_rate_gain(0,0);
	_normalized_angular_rate_gain(1) = normalized_angular_rate_gain(0,1);
	_normalized_angular_rate_gain(2) = normalized_angular_rate_gain(0,2);


    int i=0;
    double rotor_angle = -0.533708;
    double arm_length = 0.255;
    double force_k = 8.54858e-06;
    double moment_k = 1.6e-2;
    double direction = 1.0;

    _allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    _allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    _allocation_M(2, i) = direction * force_k * moment_k;
    _allocation_M(3, i) = -force_k;

    i++;
    rotor_angle = 2.565;
    arm_length = 0.238;
    direction = 1.0;
    _allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    _allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    _allocation_M(2, i) = direction * force_k * moment_k;
    _allocation_M(3, i) = -force_k;


    i++;
    rotor_angle = 0.533708;
    arm_length = 0.255;
    direction = -1.0;
    _allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    _allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    _allocation_M(2, i) = direction * force_k * moment_k;
    _allocation_M(3, i) = -force_k;


    i++; 
    rotor_angle = -2.565;
    arm_length = 0.238;
    direction = -1.0;
    _allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    _allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    _allocation_M(2, i) = direction * force_k * moment_k;
    _allocation_M(3, i) = -force_k;


	matrix::SquareMatrix<float, 4> allocation_transpose;


  	//_wd2rpm = _allocation_M.transpose() * (allocation_M*allocation_M.transpose()).inverse()*_I;    

	allocation_transpose = _allocation_M*_allocation_M.transpose()* T_i;
	//allocation_transpose = _allocation_M.transpose() * allocation_transpose * T_i;
	//_wd2rpm = allocation_transpose;

	printf("_allocation_M\n");	
	for(int k=0; k<4; k++ ) {
		for( int j=0; j<4; j++ ) 
			printf("%f ", (double)_allocation_M(k,j));
		printf("\n");
	}
			


	//_wd2rpm = _allocation_M.transpose() * (_allocation_M*_allocation_M.transpose()).I()*T_i;    
	_wd2rpm (0,0) = -7834.56; 
	_wd2rpm (0,1) = 6405.41; 
	_wd2rpm (0,2) = 178593;
	_wd2rpm (0,3) = -27847.8;
 
 	_wd2rpm (1,0) = 7834.56; 
	_wd2rpm (1,1) = -6405.41;   
	_wd2rpm (1,2) = 178557; 
	_wd2rpm (1,3) = -30641.5;
 
 	_wd2rpm (2,0) = 7834.56;  
	_wd2rpm (2,1) = 6405.41;  
	_wd2rpm (2,2) = -178593; 
	_wd2rpm (2,3) = -27847.8;

	_wd2rpm (3,0) = -7834.56; 
	_wd2rpm (3,1) = -6405.41;  
	_wd2rpm (3,2) = -178557; 
	_wd2rpm (3,3) = -30641.5;

	
} //TODO: add parameters

float
LeeController::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
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
			//TODO ??? _states.yaw = Eulerf(Quatf(att.q)).psi();
		}
	}

	/*
	if (_param_mpc_use_hte.get()) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			_control.updateHoverThrust(hte.hover_thrust);
		}
	}
	*/

}


void
LeeController::Run() {

	if (should_exit()) {
		//Unregister from Callbacks!
		//_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	poll_subscriptions();


	//TODO:   - rimuovere mixer - DONE
	//        - Pubblicare PWM -> su actuators control (questo permette di continuare il flow con simulator mavlink  (in simu, da capire come in reale)) - DONE 

	//External loop: position controller
	//if (_local_pos_sub.update(&_local_pos)) {
	//}

	// switch to the required flighttask
	start_flight_task();

	//if (_flight_tasks.isAnyTaskActive()) 
	//	PX4_INFO("Running, lee_controller] flight task: %i", _flight_tasks.getActiveTask());

	// check if any task is active
	if (_flight_tasks.isAnyTaskActive()) {

		if (!_flight_tasks.update()) {
			// FAILSAFE
			// Task was not able to update correctly. Do Failsafe.
			//failsafe(setpoint, _states, false, !was_in_failsafe);
			//TBA 
		} 
		else {
			vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
			setpoint = _flight_tasks.getPositionSetpoint();
			//printf("Goal setpoint: %f %f %f\n", (double)setpoint.x, (double)setpoint.y, (double)setpoint.z);
			//Internal loop: attitude controllers	
			float des_yaw = 1.57;
			Matrix3f R_des;

			//Local pose - Outer loop
			if (_local_pos_sub.update(&_local_pos)) {
			
				_kp = Vector3f( 0.1, 0.1, 0.1 );
				_kdp = Vector3f( 0.19, 0.19, 0.19 );
				_mass = 0.5;
				_gravity = 9.81;

				//Position error 
				_ep = Vector3f( _local_pos.x, _local_pos.y, _local_pos.z) - Vector3f( setpoint.x, setpoint.y, setpoint.z);
				_edp = Vector3f( _local_pos.vx, _local_pos.vy, _local_pos.vz) - Vector3f( setpoint.vx, setpoint.vy, setpoint.vz );

				printf("[lee controller]: ep %f %f %f\n", (double)_ep(0), (double)_ep(1), (double)_ep(2));				
				
				
				Vector3f kp;
				kp = Vector3f(1.0f, 1.0f, 1.0f);

				_ddp =  -( _ep.emult( _kp ) + _edp.emult( _kdp ) ) / _mass - _gravity * Vector3f(0.0, 0.0, 1.0) - Vector3f( setpoint.acceleration[0], setpoint.acceleration[1], setpoint.acceleration[2] );
					
				printf("[lee controller]: ddp: %f %f %f\n", (double)_ddp(0), (double)_ddp(1), (double)_ddp(2));
			}

			
			if (_vehicle_attitude_sub.update(&_v_att)) {

				Dcmf R( Quatf(_v_att.q) );
				Vector3f b1_des;
				b1_des(0) = cos( des_yaw ); 
				b1_des(1) = sin( des_yaw ); 
				b1_des(2) = 0.0;

				Vector3f b3_des;
				b3_des = -_ddp / _ddp.norm();

				Vector3f b2_des;
    			b2_des = b3_des.cross(b1_des);
    			b2_des.normalize();


				R_des.col(0) = b2_des.cross(b3_des);
				R_des.col(1) = b2_des;
				R_des.col(2) = b3_des;


				// Angle error according to lee et al.
				Matrix3f angle_error_matrix = 0.5f * (R_des.transpose() * R - R.transpose() * R_des);
				Vector3f angle_error;
				angle_error(0) = angle_error_matrix(2, 1);
				angle_error(1) = angle_error_matrix(0, 2);
				angle_error(2) = angle_error_matrix(1, 0);

				/*
				printf("Rdes: \n");
				for(int i=0; i<3; i++ ) {
					for( int j=0; j<3; j++ ) {
						printf("%f ", (double)R_des(i,j));
					}
					printf("\n");
				}
				printf("\n");
				
				printf("R: \n");
				for(int i=0; i<3; i++ ) {
					for( int j=0; j<3; j++ ) {
						printf("%f ", (double)R(i,j));
					}
					printf("\n");
				}
				printf("\n");
				*/
				printf("Angle error: %f %f %f\n", (double)angle_error(0), (double)angle_error(1), (double)angle_error(2));
				
    			Vector3f angular_rate_des(Vector3f( 0.0, 0.0, 0.0) );
				//angular_rate_des(2) = 0.1f* des_yaw;

				//printf("Rotation Matrix: \n");
				//printf("%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", (double)dcm(0,0), (double)dcm(0,1),(double)dcm(0,2), (double)dcm(1,0), (double)dcm(1,1), (double)dcm(1,2), (double)dcm(2,0), (double)dcm(2,1), (double)dcm(2,2));
				//printf("Yaw: %f Roll: %f Pitch: %f\n", (double)yaw, (double)roll, (double)pitch);
				
				//printf("Quaternion: %f %f %f %f\n", (double)_v_att.q[0], (double)_v_att.q[1], (double)_v_att.q[2], (double)_v_att.q[3] );
				//Vector3f angular_acceleration;
				//Matrix3f R = QuatToMat( 1.0f, 0.0f , 0.0f , 0.0f );
				//Vector3f b1_des;

			//}

			//if (_vehicle_angular_velocity_sub.update(&_angular_velocity)) {
				_vehicle_angular_velocity_sub.update(&_angular_velocity);
				Vector3f mes_w = Vector3f( _angular_velocity.xyz[0], _angular_velocity.xyz[1], _angular_velocity.xyz[2]);







		    	Vector3f angular_rate_error = mes_w - R_des.transpose() * R * angular_rate_des;
				//angular_rate_error(0) = 0.0;
				//angular_rate_error(1) = 0.0;
				//angular_rate_error(2) = 0.0;

				//printf("angular_rate_error: %f %f %f\n", (double)angular_rate_error(0), (double)angular_rate_error(1), (double)angular_rate_error(2));
				printf("angular_rate_des: %f %f %f\n", (double)angular_rate_des(0), (double)angular_rate_des(1), (double)angular_rate_des(2));

				
				Vector3f angular_acceleration;


				angular_acceleration = -1.0f * angle_error.emult( _normalized_attitude_gain) - angular_rate_error.emult( _normalized_angular_rate_gain) 
				+ mes_w.cross( mes_w );
				
				
				
				_mass = 0.5;
				float thrust = _mass * _ddp.dot( R.col(2) );
				//printf("Thrust: %f\n", (double)thrust);
			
				matrix::Vector<float, 4> angular_acceleration_thrust; // angular_acceleration_thrust;
				angular_acceleration_thrust(0) = angular_acceleration(0);
				angular_acceleration_thrust(1) = angular_acceleration(1);
				angular_acceleration_thrust(2) = angular_acceleration(2);
				angular_acceleration_thrust(3) = thrust;


				printf("angular_acceleration_thrust: %f %f %f %f\n", (double)angular_acceleration_thrust(0), (double)angular_acceleration_thrust(1), (double)angular_acceleration_thrust(2), (double)angular_acceleration_thrust(3));

				matrix::Vector<float, 4> rotor_velocities;
				//for(int i=0; i<4; i++ ) {
				//	for( int j=0; j<4; j++ ) 
				//		printf("%f ", (double)_wd2rpm(i,j));
				//	printf("\n");
				//}
				//printf("\n");
				rotor_velocities = _wd2rpm * angular_acceleration_thrust;

				rotor_velocities(0) = fmax(rotor_velocities(0), 0.0);
				rotor_velocities(1) = fmax(rotor_velocities(1), 0.0);
				rotor_velocities(2) = fmax(rotor_velocities(2), 0.0);
				rotor_velocities(3) = fmax(rotor_velocities(3), 0.0);

				rotor_velocities(0) = sqrt(rotor_velocities(0));
				rotor_velocities(1) = sqrt(rotor_velocities(1));
				rotor_velocities(2) = sqrt(rotor_velocities(2));
				rotor_velocities(3) = sqrt(rotor_velocities(3));


				//printf("Rotors vel: %f %f %f %f\n", (double)rotor_velocities(0), (double)rotor_velocities(1), (double)rotor_velocities(2), (double)rotor_velocities(3));

				matrix::Vector<float, 4> c;
				
				c(0) = ( rotor_velocities(0)  - 100.0f ) / 1000.0f;
				c(1) = ( rotor_velocities(1)  - 100.0f ) / 1000.0f;
				c(2) = ( rotor_velocities(2)  - 100.0f ) / 1000.0f;
				c(3) = ( rotor_velocities(3)  - 100.0f ) / 1000.0f;

				matrix::Vector<float, 4> pwm;
				float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				pwm(0) = ( c(0)*( PWM_DEFAULT_MAX - PWM_DEFAULT_MIN ) + 2.0f*pwm_center) / 2.0f;
				pwm(1) = ( c(1)*( PWM_DEFAULT_MAX - PWM_DEFAULT_MIN ) + 2.0f*pwm_center) / 2.0f;
				pwm(2) = ( c(2)*( PWM_DEFAULT_MAX - PWM_DEFAULT_MIN ) + 2.0f*pwm_center) / 2.0f;
				pwm(3) = ( c(3)*( PWM_DEFAULT_MAX - PWM_DEFAULT_MIN ) + 2.0f*pwm_center) / 2.0f;

				printf("c: %f %f %f %f\n", (double)c(0), (double)c(1), (double)c(2), (double)c(3) );
				
				//printf("PWM: %f %f %f %f\n", (double)pwm(0), (double)pwm(1), (double)pwm(2), (double)pwm(3) );


				//Rotors vel to PWM
				//float vmax = 1100.0;
				//float vmin = 0.0;
				//float k = 0.0009f;

				//const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				//float m = ( PWM_DEFAULT_MAX - vmax ) / ( PWM_DEFAULT_MIN - vmin ); 
				//matrix::Vector<float, 4> pwm;
				//for(int i=0; i<4; i++) {
				//	pwm(i) = m*rotor_velocities(i) + PWM_DEFAULT_MIN;
				//} 

				//printf("PWM: %f %f %f %f\n", (double)pwm(0), (double)pwm(1), (double)pwm(2), (double)pwm(3) );

				
				//angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
				//angular_acceleration_thrust(3) = thrust;
				//*rotor_velocities = _wd2rpm * angular_acceleration_thrust;
				//*rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
				//*rotor_velocities = rotor_velocities->cwiseSqrt();
				/*
				//---PWM output
				actuator_outputs_s actuator_outputs{};
				actuator_outputs.noutputs = 4;
				actuator_outputs.output[0] = pwm(0);
				actuator_outputs.output[1] = pwm(1);
				actuator_outputs.output[2] = pwm(2);
				actuator_outputs.output[3] = pwm(3);
				actuator_outputs.timestamp = hrt_absolute_time();
				_outputs_pub.publish(actuator_outputs);
				//---












				*/
			}
			//Low level control (output: PWM)




		}





	}
	/*else { 
		//printf("No F task\n");


	}
	*/
	//Rate controller


	




	//actuator_controls_s actuators{};
	//actuators.timestamp = hrt_absolute_time();
	//actuators.control[actuator_controls_s::INDEX_ROLL] 	 	= 1.5;
	//actuators.control[actuator_controls_s::INDEX_PITCH]	 	= 1.5;
	//actuators.control[actuator_controls_s::INDEX_YAW]	 	= 1.5;
	//actuators.control[actuator_controls_s::INDEX_THROTTLE]  = 1.5;
	//_actuators_0_pub.publish(actuators);
	//printf("Want to publish a control data: %f %f %f %f\n", (double)actuators.control[actuator_controls_s::INDEX_ROLL], 
	//														(double)actuators.control[actuator_controls_s::INDEX_YAW], 
	//														(double)actuators.control[actuator_controls_s::INDEX_THROTTLE], 
	//														(double)actuators.control[actuator_controls_s::INDEX_ROLL]);


	perf_end(_loop_perf);
}

int LeeController::task_spawn(int argc, char *argv[])
{
	LeeController *instance = new LeeController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int LeeController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LeeController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int lee_controller_main(int argc, char *argv[])
{
	return LeeController::main(argc, argv);
}
