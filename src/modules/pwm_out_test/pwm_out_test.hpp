#pragma once


#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/flight_tasks/FlightTasks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/weather_vane/WeatherVane.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/hover_thrust_estimate.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_pwm_output.h>
#include <vtol_att_control/vtol_type.h>
#include <px4_log.h>

extern "C" __EXPORT int pwm_out_test_main(int argc, char *argv[]);

static constexpr int NUM_FAILURE_TRIES = 10;
struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

class PwmOutTest: public ModuleBase<PwmOutTest>,  public control::SuperBlock, public ModuleParams,
    public px4::WorkItem {

        public:
            PwmOutTest();
            ~PwmOutTest() override;
	        static int print_usage(const char *reason = nullptr);
			int print_flight_task_usage(const char *reason);
            static int custom_command(int argc, char *argv[]);
        	static int task_spawn(int argc, char *argv[]);
        	bool init();
			void start_flight_task();
			void check_failure(bool task_failure, uint8_t nav_state);
			void send_vehicle_cmd_do(uint8_t nav_state);

        private:
            void Run() override;
            void parameters_updated();
			void poll_subscriptions();


			//---Subscribers

			uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
			uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
			uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};				/**< vehicle attitude */
			uORB::Subscription _home_pos_sub{ORB_ID(home_position)}; 			/**< home position */
			uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

            uORB::Subscription _v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint subscription */
            uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint subscription */
            uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
            uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
            uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
            uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */

			uORB::PublicationQueued<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	 
			uORB::Publication<vehicle_pwm_output_s> _pwm_pub{ORB_ID(vehicle_pwm_output)};			/**< trajectory setpoints publication */

	        uORB::Subscription _params_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	        //---
			
			uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	
			//---Input data
            struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
            struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
            struct vehicle_rates_setpoint_s		_v_rates_sp {};		/**< vehicle rates setpoint */
            struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
            struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
            struct vehicle_status_s			_vehicle_status {};	/**< vehicle status */
			struct vehicle_pwm_output_s     _pwm_out;			/**< desired PWM value for motor control */
			vehicle_local_position_s _local_pos{};			/**< vehicle local position */
	
			//---

          	perf_counter_t	_loop_perf;			/**< loop duration performance counter */


			hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */

			int _task_failure_count{0};         /**< counter for task failures */

			FlightTasks _flight_tasks; /**< class generating position controller setpoints depending on vehicle task */
			bool _in_failsafe = false; /**< true if failsafe was entered within current cycle */

			/**< vehicle-land-detection: initialze to landed */
			vehicle_land_detected_s _vehicle_land_detected = {
				.timestamp = 0,
				.alt_max = -1.0f,
				.freefall = false,
				.ground_contact = true,
				.maybe_landed = true,
				.landed = true,
			};

			vehicle_control_mode_s	_control_mode{};		/**< vehicle control mode */
			home_position_s	_home_pos{};			/**< home position */
			landing_gear_s _landing_gear{};
			int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};
			PositionControlStates _states{}; /**< structure containing vehicle state information for position control */

    };


