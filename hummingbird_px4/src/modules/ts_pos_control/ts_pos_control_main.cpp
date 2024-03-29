/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Tailsitter position controller.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for Tailsitter orientation) and thrust module (i.e. Tailsitter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <float.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_Tailsitter	0.9f
#define ONE_G	9.8066f

/**
 * Tailsitter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int ts_pos_control_main(int argc, char *argv[]);

class TailsitterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	TailsitterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~TailsitterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	orb_id_t _attitude_setpoint_id;

	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;
		param_t pos_tc_x;
		param_t pos_tc_y;
		param_t pos_tc_z;
		param_t pos_dr_x;
		param_t pos_dr_y;
		param_t pos_dr_z;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;


		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
		math::Vector<3> pos_tc;
		math::Vector<3> pos_dr;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _do_reset_alt_pos_flag;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control;
	bool _run_alt_control;

	bool _reset_int_z = true;
	bool _reset_int_xy = true;
	bool _reset_int_z_manual = false;
	bool _reset_yaw_sp = true;

	bool _hold_offboard_xy = false;
	bool _hold_offboard_z = false;

	math::Vector<3> _thrust_int;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
	uint8_t _heading_reset_counter;

	matrix::Dcmf _R_setpoint;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	void control_non_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	void control_position(float dt);

	void limit_acceleration(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	void update_velocity_derivative();

	void do_control(float dt);

	void generate_attitude_setpoint(float dt);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace ts_pos_control
{

TailsitterPositionControl	*g_control;
}

TailsitterPositionControl::TailsitterPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(0),
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_do_reset_alt_pos_flag(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	control_vel_enabled_prev(false),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_vz_reset_counter(0),
	_vxy_reset_counter(0),
	_heading_reset_counter(0)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();

	_R.identity();

	_R_setpoint.identity();

	_thrust_int.zero();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");
	_params_handles.pos_tc_x = param_find("TS_POS_TC_X");
	_params_handles.pos_tc_y = param_find("TS_POS_TC_Y");
	_params_handles.pos_tc_z = param_find("TS_POS_TC_Z");
	_params_handles.pos_dr_x = param_find("TS_POS_DR_X");
	_params_handles.pos_dr_y = param_find("TS_POS_DR_Y");
	_params_handles.pos_dr_z = param_find("TS_POS_DR_Z");

	/* fetch initial parameter values */
	parameters_update(true);
}

TailsitterPositionControl::~TailsitterPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	ts_pos_control::g_control = nullptr;
}

int
TailsitterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now */
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		param_get(_params_handles.pos_tc_x, &v);
		_params.pos_tc(0) = v;
		param_get(_params_handles.pos_tc_y, &v);
		_params.pos_tc(1) = v;
		param_get(_params_handles.pos_tc_z, &v);
		_params.pos_tc(2) = v;

		param_get(_params_handles.pos_dr_x, &v);
		_params.pos_dr(0) = 2*v;
		param_get(_params_handles.pos_dr_y, &v);
		_params.pos_dr(1) = 2*v;
		param_get(_params_handles.pos_dr_z, &v);
		_params.pos_dr(2) = 2*v;
		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1 s from full speed is feasible
		 */
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max);
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void
TailsitterPositionControl::poll_subscriptions()
{

	bool updated;

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

//	orb_check(_ctrl_state_sub, &updated);
//
//	if (updated) {
//		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
//
//		/* get current rotation matrix and euler angles from control state quaternions */
//		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
//		_R = q_att.to_dcm();
//		math::Vector<3> euler_angles;
//		euler_angles = _R.to_euler();
//		_yaw = euler_angles(2);
//
//		if (_control_mode.flag_control_manual_enabled) {
//			if (_heading_reset_counter != _ctrl_state.quat_reset_counter) {
//				_heading_reset_counter = _ctrl_state.quat_reset_counter;
//				math::Quaternion delta_q(_ctrl_state.delta_q_reset[0], _ctrl_state.delta_q_reset[1], _ctrl_state.delta_q_reset[2],
//							 _ctrl_state.delta_q_reset[3]);
//
//				// we only extract the heading change from the delta quaternion
//				math::Vector<3> delta_euler = delta_q.to_euler();
//				_att_sp.yaw_body += delta_euler(2);
//			}
//		}
//
//	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);

	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) {
			if (_z_reset_counter != _local_pos.z_reset_counter) {
				_pos_sp(2) += _local_pos.delta_z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) += _local_pos.delta_xy[0];
				_pos_sp(1) += _local_pos.delta_xy[1];
			}

			if (_vz_reset_counter != _local_pos.vz_reset_counter) {
				_vel_sp(2) += _local_pos.delta_vz;
				_vel_sp_prev(2) +=  _local_pos.delta_vz;
			}

			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) {
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
				_vel_sp_prev(0) += _local_pos.delta_vxy[0];
				_vel_sp_prev(1) += _local_pos.delta_vxy[1];
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
		_vz_reset_counter = _local_pos.vz_reset_counter;
		_vxy_reset_counter = _local_pos.vxy_reset_counter;
	}

	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;

		}

	}
}

float
TailsitterPositionControl::scale_control(float ctl, float end, float dz, float dy)
{
	if (ctl > dz) {
		return dy + (ctl - dz) * (1.0f - dy) / (end - dz);

	} else if (ctl < -dz) {
		return -dy + (ctl + dz) * (1.0f - dy) / (end - dz);

	} else {
		return ctl * (dy / dz);
	}
}

float
TailsitterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;

	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

void
TailsitterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	ts_pos_control::g_control->task_main();
}

void
TailsitterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
TailsitterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;

		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void
TailsitterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		_pos_sp(2) = _pos(2);
	}
}

void
TailsitterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
TailsitterPositionControl::control_manual(float dt)
{
	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	if (_mode_auto) {
		_mode_auto = false;

		/* Reset alt pos flags if resetting is enabled */
		if (_do_reset_alt_pos_flag) {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}
	}

	math::Vector<3> req_vel_sp; // X,Y in local frame and Z in global (D), in [-1,1] normalized range
	req_vel_sp.zero();

	if (_control_mode.flag_control_altitude_enabled) {
		/* set vertical velocity setpoint with throttle stick */
		req_vel_sp(2) = -scale_control(_manual.z - 0.5f, 0.5f, _params.alt_ctl_dz, _params.alt_ctl_dy); // D
	}

	if (_control_mode.flag_control_position_enabled) {
		/* set horizontal velocity setpoint with roll/pitch stick */
		req_vel_sp(0) = _manual.x;
		req_vel_sp(1) = _manual.y;
	}

	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	/* limit velocity setpoint */
	float req_vel_sp_norm = req_vel_sp.length();

	if (req_vel_sp_norm > 1.0f) {
		req_vel_sp /= req_vel_sp_norm;
	}

	/* _req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw */
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
	math::Vector<3> req_vel_sp_scaled = R_yaw_sp * req_vel_sp.emult(
			_params.vel_cruise); // in NED and scaled to actual velocity

	/*
	 * assisted velocity mode: user controls velocity, but if	velocity is small enough, position
	 * hold is activated for the corresponding axis
	 */

	/* horizontal axes */
	if (_control_mode.flag_control_position_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(0)) < _params.hold_xy_dz && fabsf(req_vel_sp(1)) < _params.hold_xy_dz) {
			if (!_pos_hold_engaged) {

				float vel_xy_mag = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));

				if (_params.hold_max_xy < FLT_EPSILON || vel_xy_mag < _params.hold_max_xy) {
					/* reset position setpoint to have smooth transition from velocity control to position control */
					_pos_hold_engaged = true;
					_pos_sp(0) = _pos(0);
					_pos_sp(1) = _pos(1);

				} else {
					_pos_hold_engaged = false;
				}
			}

		} else {
			_pos_hold_engaged = false;
		}

		/* set requested velocity setpoint */
		if (!_pos_hold_engaged) {
			_pos_sp(0) = _pos(0);
			_pos_sp(1) = _pos(1);
			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
			_vel_sp(0) = req_vel_sp_scaled(0);
			_vel_sp(1) = req_vel_sp_scaled(1);
		}
	}

	/* vertical axis */
	if (_control_mode.flag_control_altitude_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(2)) < FLT_EPSILON) {
			if (!_alt_hold_engaged) {
				if (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z) {
					/* reset position setpoint to have smooth transition from velocity control to position control */
					_alt_hold_engaged = true;
					_pos_sp(2) = _pos(2);

				} else {
					_alt_hold_engaged = false;
				}
			}

		} else {
			_alt_hold_engaged = false;
			_pos_sp(2) = _pos(2);
		}

		/* set requested velocity setpoint */
		if (!_alt_hold_engaged) {
			_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
			_vel_sp(2) = req_vel_sp_scaled(2);
		}
	}

	if (_vehicle_land_detected.landed) {
		/* don't run controller when landed */
		_reset_pos_sp = true;
		_reset_alt_sp = true;
		_mode_auto = false;
		_reset_int_z = true;
		_reset_int_xy = true;

		_R_setpoint.identity();

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
		_att_sp.yaw_body = _yaw;
		_att_sp.thrust = 0.0f;

		_att_sp.timestamp = hrt_absolute_time();

		/* publish attitude setpoint */
		if (_att_sp_pub != nullptr) {
			orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

		} else if (_attitude_setpoint_id) {
			_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
		}

	} else {
		control_position(dt);
	}

}

void
TailsitterPositionControl::control_non_manual(float dt)
{
	/* select control source */
	if (_control_mode.flag_control_offboard_enabled) {
		/* offboard control */
		control_offboard(dt);
		_mode_auto = false;

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;

		/* AUTO */

		control_auto(dt);
	}

	/* weather-vane mode for vtol: disable yaw control */
	if (_pos_sp_triplet.current.disable_mc_yaw_control == true) {
		_att_sp.disable_mc_yaw_control = true;

	} else {
		/* reset in case of setpoint updates */
		_att_sp.disable_mc_yaw_control = false;
	}

	// guard against any bad velocity values

	bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
			      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
			      _pos_sp_triplet.current.velocity_valid;

	// do not go slower than the follow target velocity when position tracking is active (set to valid)

	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
	    velocity_valid &&
	    _pos_sp_triplet.current.position_valid) {

		math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);

		float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());

		// only override velocity set points when uav is traveling in same direction as target and vector component
		// is greater than calculated position set point velocity component

		if (cos_ratio > 0) {
			ft_vel *= (cos_ratio);
			// min speed a little faster than target vel
			ft_vel += ft_vel.normalized() * 1.5f;

		} else {
			ft_vel.zero();
		}

		_vel_sp(0) = fabsf(ft_vel(0)) > fabsf(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
		_vel_sp(1) = fabsf(ft_vel(1)) > fabsf(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);

		// track target using velocity only

	} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
		   velocity_valid) {

		_vel_sp(0) = _pos_sp_triplet.current.vx;
		_vel_sp(1) = _pos_sp_triplet.current.vy;
	}

	/* use constant descend rate when landing, ignore altitude setpoint */
	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		_vel_sp(2) = _params.land_speed;
		_run_alt_control = false;
	}

	/* special thrust setpoint generation for takeoff from ground */
	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
	    && _control_mode.flag_armed) {


		// check if we are not already in air.
		// if yes then we don't need a jumped takeoff anymore
		if (!_takeoff_jumped && !_vehicle_land_detected.landed && fabsf(_takeoff_thrust_sp) < FLT_EPSILON) {
			_takeoff_jumped = true;
		}

		if (!_takeoff_jumped) {
			printf("taking off");
			// ramp thrust setpoint up
			if (_vel(2) > -(_params.tko_speed / 2.0f)) {
				_takeoff_thrust_sp += 0.5f * dt;
				_vel_sp.zero();
				_vel_prev.zero();

			} else {
				// copter has reached our takeoff speed. split the thrust setpoint up
				// into an integral part and into a P part
				_thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
				_thrust_int(2) = -math::constrain(_thrust_int(2), _params.thr_min, _params.thr_max);
				_vel_sp_prev(2) = -_params.tko_speed;
				_takeoff_jumped = true;
				_reset_int_z = false;
			}
		}

		if (_takeoff_jumped) {
			_vel_sp(2) = 0;//-_params.tko_speed;
		}

	} else {
		_takeoff_jumped = false;
		_takeoff_thrust_sp = 0.0f;
	}

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		/* idle state, don't run controller and set zero thrust */
		_R_setpoint.identity();


		matrix::Quatf qd = _R_setpoint;
		memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
		_att_sp.q_d_valid = true;

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
		_att_sp.yaw_body = _yaw;
		_att_sp.thrust = 0.0f;

		_att_sp.timestamp = hrt_absolute_time();

		/* publish attitude setpoint */
		if (_att_sp_pub != nullptr) {
			orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

		} else if (_attitude_setpoint_id) {
			_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
		}

	} else {
		control_position(dt);
	}
}

void
TailsitterPositionControl::control_offboard(float dt)
{
	if (_pos_sp_triplet.current.valid) {

		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_pos_control = true;

			_hold_offboard_xy = false;

			if(_pos_sp_triplet.current.velocity_valid){
				if (_pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_LOCAL_NED) {
							/* set position setpoint move rate */
							_vel_sp(0) = _pos_sp_triplet.current.vx;
							_vel_sp(1) = _pos_sp_triplet.current.vy;
							_vel_sp(2) = _pos_sp_triplet.current.vz;
							_run_alt_control = true;

				} else if (_pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
							// Transform velocity command from body frame to NED frame
							_vel_sp(0) = cosf(_yaw) * _pos_sp_triplet.current.vx - sinf(_yaw) * _pos_sp_triplet.current.vy;
							_vel_sp(1) = sinf(_yaw) * _pos_sp_triplet.current.vx + cosf(_yaw) * _pos_sp_triplet.current.vy;

				} else {
					PX4_WARN("Unknown velocity offboard coordinate frame");
				}
			}else{
				_vel_sp(0) = 0.f;
				_vel_sp(1) = 0.f;
			}

		}else{
			reset_pos_sp();
			_vel_sp(0) = 0.f;
			_vel_sp(1) = 0.f;
			_vel_sp(2) = 0.f;
		}

		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;

		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;
		reset_pos_sp();
		reset_alt_sp();
		_vel_sp(0) = 0.f;
		_vel_sp(1) = 0.f;
		_vel_sp(2) = 0.f;
	}
}

void
TailsitterPositionControl::limit_acceleration(float dt)
{
	// limit total horizontal acceleration
	math::Vector<2> acc_hor;
	acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
	acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;

	if (acc_hor.length() > _params.acc_hor_max) {
		acc_hor.normalize();
		acc_hor *= _params.acc_hor_max;
		math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
		math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
		_vel_sp(0) = vel_sp_hor(0);
		_vel_sp(1) = vel_sp_hor(1);
	}

	// limit vertical acceleration
	float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;

	// TODO: vertical acceleration is not just 2 * horizontal acceleration
	if (fabsf(acc_v) > 2 * _params.acc_hor_max) {
		acc_v /= fabsf(acc_v);
		_vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
	}

}

bool
TailsitterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
		const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		/* we have triangle CDX with known CD and CX = R, find DX */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			/* target waypoint is already behind us */
			res = line_b;

		} else {
			/* target is in front of us */
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {
		/* have no roots, return D */
		res = d; /* go directly to line */

		/* previous waypoint is still in front of us */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		/* target waypoint is already behind us */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void TailsitterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto) {
		if (!_mode_auto) {
			_mode_auto = true;
		}

		_reset_pos_sp = true;
		_reset_alt_sp = true;
	}

	// Always check reset state of altitude and position control flags in auto
	reset_pos_sp();
	reset_alt_sp();

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;

	math::Vector<3> prev_sp;
	math::Vector<3> curr_sp;

	if (_pos_sp_triplet.current.valid) {

		/* project setpoint to local frame */
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		if (PX4_ISFINITE(curr_sp(0)) &&
		    PX4_ISFINITE(curr_sp(1)) &&
		    PX4_ISFINITE(curr_sp(2))) {
			current_setpoint_valid = true;
		}
	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp.data[0], &prev_sp.data[1]);
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			previous_setpoint_valid = true;
		}
	}

	if (current_setpoint_valid &&
	    (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		/* scaled space: 1 == position error resulting max allowed speed */

		math::Vector<3> cruising_speed = _params.vel_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			cruising_speed(0) = _pos_sp_triplet.current.cruising_speed;
			cruising_speed(1) = _pos_sp_triplet.current.cruising_speed;
		}

		math::Vector<3> scale = _params.pos_p.edivide(cruising_speed);

		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s;

		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) &&
		    previous_setpoint_valid) {

			/* follow "previous - current" line */

			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of unit sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();

				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than unit radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet.next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
								       _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
								       &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();

								/* if curr - next distance is larger than unit radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

					if (!near) {
						/* we're far away from trajectory, pos_sp_s is set to the nearest point on the trajectory */
						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}

		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();

		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);


		/* update yaw setpoint if needed */

		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_do_reset_alt_pos_flag = false;

			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */

		} else {
			_do_reset_alt_pos_flag = true;
		}

		// During a mission or in loiter it's safe to retract the landing gear.
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed) {
			_att_sp.landing_gear = 1.0f;

			// During takeoff and landing, we better put it down again.

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			_att_sp.landing_gear = -1.0f;

		} else {
			// For the rest of the setpoint types, just leave it as is.
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

void
TailsitterPositionControl::update_velocity_derivative()
{

	/* Update velocity derivative,
	 * independent of the current flight mode
	 */
	if (_local_pos.timestamp == 0) {
		return;
	}

	// TODO: this logic should be in the estimator, not the controller!

	if (PX4_ISFINITE(_local_pos.x) &&
	    PX4_ISFINITE(_local_pos.y) &&
	    PX4_ISFINITE(_local_pos.z)) {

		_pos(0) = _local_pos.x;
		_pos(1) = _local_pos.y;


		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
			_pos(2) = -_local_pos.dist_bottom;

		} else {
			_pos(2) = _local_pos.z;
		}
	}

	if (PX4_ISFINITE(_local_pos.vx) &&
	    PX4_ISFINITE(_local_pos.vy) &&
	    PX4_ISFINITE(_local_pos.vz)) {

		_vel(0) = _local_pos.vx;
		_vel(1) = _local_pos.vy;

		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
			_vel(2) = -_local_pos.dist_bottom_rate;

		} else {
			_vel(2) = _local_pos.vz;
		}
	}

	_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
	_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
	_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
}

void
TailsitterPositionControl::do_control(float dt)
{

	_vel_ff.zero();

	/* by default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	_run_pos_control = true;
	_run_alt_control = true;

	if (_control_mode.flag_control_manual_enabled) {
		/* manual control */
		control_manual(dt);
		_mode_auto = false;

		_hold_offboard_xy = false;
		_hold_offboard_z = false;

	} else {
		control_non_manual(dt);
	}

}

void
TailsitterPositionControl::control_position(float dt)
{

	/* make sure velocity setpoint is saturated in xy*/
	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
				  _vel_sp(1) * _vel_sp(1));

	if (vel_norm_xy > _params.vel_max(0)) {
		/* note assumes vel_max(0) == vel_max(1) */
		_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
		_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
	}

	/* make sure velocity setpoint is saturated in z*/
	if (_vel_sp(2) < -1.0f * _params.vel_max_up) {
		_vel_sp(2) = -1.0f * _params.vel_max_up;
	}

	if (_vel_sp(2) >  _params.vel_max_down) {
		_vel_sp(2) = _params.vel_max_down;
	}

	if (!_control_mode.flag_control_position_enabled) {
		_reset_pos_sp = true;
	}

	if (!_control_mode.flag_control_altitude_enabled) {
		_reset_alt_sp = true;
	}

	if (!_control_mode.flag_control_velocity_enabled) {
		_vel_sp_prev(0) = _vel(0);
		_vel_sp_prev(1) = _vel(1);
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
		control_vel_enabled_prev = false;
	}

	if (!_control_mode.flag_control_climb_rate_enabled) {
		_vel_sp(2) = 0.0f;
	}

	/* TODO: remove this is a pathetic leftover, it's here just to make sure that
	 * _takeoff_jumped flags are reset */
	if (_control_mode.flag_control_manual_enabled || !_pos_sp_triplet.current.valid
	    || _pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF
	    || !_control_mode.flag_armed) {

		_takeoff_jumped = false;
		_takeoff_thrust_sp = 0.0f;
	}

	limit_acceleration(dt);

	_vel_sp_prev = _vel_sp;

	_global_vel_sp.vx = _vel_sp(0);
	_global_vel_sp.vy = _vel_sp(1);
	_global_vel_sp.vz = _vel_sp(2);

	/* publish velocity setpoint */
	if (_global_vel_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

	} else {
		_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
	}

	if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
	    _control_mode.flag_control_acceleration_enabled) {
		/* reset integrals if needed */
		if (_control_mode.flag_control_climb_rate_enabled) {
			if (_reset_int_z) {
				_reset_int_z = false;
				float i = _params.thr_min;

				if (_reset_int_z_manual) {
					i = math::constrain(_params.thr_hover, _params.thr_min, _params.thr_max);
				}

				_thrust_int(2) = -i;
			}

		} else {
			_reset_int_z = true;
		}

		if (_control_mode.flag_control_velocity_enabled) {
			if (_reset_int_xy) {
				_reset_int_xy = false;
				_thrust_int(0) = 0.0f;
				_thrust_int(1) = 0.0f;
			}

		} else {
			_reset_int_xy = true;
		}

		/* velocity error */
//		math::Vector<3> vel_err = _vel_sp - _vel;
//
//		// check if we have switched from a non-velocity controlled mode into a velocity controlled mode
//		// if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
//		if (!control_vel_enabled_prev && _control_mode.flag_control_velocity_enabled) {
//
//			matrix::Dcmf Rb = matrix::Quatf(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);
//
//			// choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
//			// given by the last attitude setpoint
//			_vel_sp(0) = _vel(0) + (-Rb(0,
//						    2) * _att_sp.thrust - _thrust_int(0) - _vel_err_d(0) * _params.vel_d(0)) / _params.vel_p(0);
//			_vel_sp(1) = _vel(1) + (-Rb(1,
//						    2) * _att_sp.thrust - _thrust_int(1) - _vel_err_d(1) * _params.vel_d(1)) / _params.vel_p(1);
//			_vel_sp(2) = _vel(2) + (-Rb(2,
//						    2) * _att_sp.thrust - _thrust_int(2) - _vel_err_d(2) * _params.vel_d(2)) / _params.vel_p(2);
//			_vel_sp_prev = _vel_sp;
//			control_vel_enabled_prev = true;
//
//			// compute updated velocity error
//			vel_err = _vel_sp - _vel;
//		}

		/* Acceleration in NED frame */
		math::Vector<3> thrust_sp;


		if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
			thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

		} else {
			thrust_sp = ((_pos_sp - _pos).edivide(_params.pos_tc)).edivide(_params.pos_tc) +
						((_vel_sp - _vel).edivide(_params.pos_tc)).emult(_params.pos_dr);
			//warnx("Thrust, %f, %f, %f", (double) thrust_sp(0),(double)thrust_sp(1),(double)thrust_sp(2));

		}

		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		    && !_takeoff_jumped && !_control_mode.flag_control_manual_enabled) {
			// for jumped takeoffs use special thrust setpoint calculated above
//			thrust_sp.zero();
//			thrust_sp(2) = -_takeoff_thrust_sp;
			//warnx("Cleared");
		}

//		if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
//			thrust_sp(0) = 0.0f;
//			thrust_sp(1) = 0.0f;
//		}
//
//		if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
//			thrust_sp(2) = 0.0f;
//		}

		math::Vector<3> gravity(0, 0, 0.53f * ONE_G);
		math::Vector<3> f_prop = thrust_sp * 0.53f - gravity;
		//warnx("Before Before Thrust setpoint: %f, %f, %f\n", (double)thrust_sp(0),(double)thrust_sp(1),(double)thrust_sp(2));
		thrust_sp = f_prop;
		//warnx("Before Thrust setpoint: %f, %f, %f\n", (double)thrust_sp(0),(double)thrust_sp(1),(double)thrust_sp(2));


		/* limit min lift */
		float thr_min = _params.thr_min;

		if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
			/* don't allow downside thrust direction in manual attitude mode */
			thr_min = 0.0f;
		}

		float thrust_abs = thrust_sp.length();
		float tilt_max = _params.tilt_max_air;
		float thr_max = _params.thr_max;
		/* filter vel_z over 1/8sec */
		_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
		/* filter vel_z change over 1/8sec */
		float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
		_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

		// We can only run the control if we're already in-air, have a takeoff setpoint,
		// or if we're in offboard control.
		// Otherwise, we should just bail out
		const bool got_takeoff_setpoint = (_pos_sp_triplet.current.valid &&
						   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) ||
						  _control_mode.flag_control_offboard_enabled;

		if (_vehicle_land_detected.landed && !got_takeoff_setpoint) {
			// Keep throttle low while still on ground.

			thr_max = 0.0f;

		} else if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			/* adjust limits for landing mode */
			/* limit max tilt and min lift when landing */
			tilt_max = _params.tilt_max_land;

			if (thr_min < 0.0f) {
				thr_min = 0.0f;
			}

			/* descend stabilized, we're landing */
			if (!_in_landing && !_lnd_reached_ground
			    && (float)fabsf(_acc_z_lp) < 0.1f
			    && _vel_z_lp > 0.5f * _params.land_speed) {
				_in_landing = true;
			}

			/* assume ground, cut thrust */
			if (_in_landing
			    && _vel_z_lp < 0.1f) {
				thr_max = 0.0f;
				_in_landing = false;
				_lnd_reached_ground = true;
			}

			/* once we assumed to have reached the ground always cut the thrust.
				Only free fall detection below can revoke this
			*/
			if (!_in_landing && _lnd_reached_ground) {
				thr_max = 0.0f;
			}

			/* if we suddenly fall, reset landing logic and remove thrust limit */
			if (_lnd_reached_ground
			    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
			    && (_acc_z_lp > 4.0f
				|| _vel_z_lp > 2.0f * _params.land_speed)) {
				thr_max = _params.thr_max;
				_in_landing = true;
				_lnd_reached_ground = false;
			}

		} else {
			_in_landing = false;
			_lnd_reached_ground = false;
		}

		/* limit min lift */
		if (-thrust_sp(2) < thr_min) {
			thrust_sp(2) = -thr_min;
		}

		if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

			/* limit max tilt */
			if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
				/* absolute horizontal thrust */
				float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

				if (thrust_sp_xy_len > 0.01f) {
					/* max horizontal thrust for given vertical thrust*/
					float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

					if (thrust_sp_xy_len > thrust_xy_max) {
						float k = thrust_xy_max / thrust_sp_xy_len;
						thrust_sp(0) *= k;
						thrust_sp(1) *= k;
					}
				}
			}
		}

		if (_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_velocity_enabled) {
			/* thrust compensation when vertical velocity but not horizontal velocity is controlled */
			float att_comp;

			if (_R(2, 2) > TILT_COS_MAX) {
				att_comp = 1.0f / _R(2, 2);

			} else if (_R(2, 2) > 0.0f) {
				att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;

			} else {
				att_comp = 1.0f;
			}

			thrust_sp(2) *= att_comp;
		}

		/* limit max thrust */
		thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */

		if (thrust_abs > thr_max) {
			if (thrust_sp(2) < 0.0f) {
				if (-thrust_sp(2) > thr_max) {
					/* thrust Z component is too large, limit it */
					thrust_sp(0) = 0.0f;
					thrust_sp(1) = 0.0f;
					thrust_sp(2) = -thr_max;

				} else {
					/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
					float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
					float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
					float k = thrust_xy_max / thrust_xy_abs;
					thrust_sp(0) *= k;
					thrust_sp(1) *= k;
				}

			} else {
				/* Z component is negative, going down, simply limit thrust vector */
				float k = thr_max / thrust_abs;
				thrust_sp *= k;
			}

			thrust_abs = thr_max;
		}


		/* save thrust setpoint for logging */
		_local_pos_sp.acc_x = thrust_sp(0);
		_local_pos_sp.acc_y = thrust_sp(1);
		_local_pos_sp.acc_z = thrust_sp(2);
		//warnx("Thrust setpoint: %f, %f, %f\n", (double)thrust_sp(0),(double)thrust_sp(1),(double)thrust_sp(2));


		/* calculate attitude setpoint from thrust vector */
		if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
			/* desired body_z axis = -normalize(thrust_vector) */
			math::Vector<3> body_x;
			math::Vector<3> body_y;
			math::Vector<3> body_z;

			if (thrust_abs > SIGMA) {
				body_z = -thrust_sp / thrust_abs;

			} else {
				/* no thrust, set Z axis to safe value */
				body_z.zero();
				body_z(2) = 1.0f;
			}

			/* Set Yaw s.t. thrust lies on x-z plane */
			float l = thrust_sp(0) * thrust_sp(0) + thrust_sp(1) * thrust_sp(1);
			l = pow(l,0.5f);
			float psi = asin(thrust_sp(0) / l);
			psi = _att_sp.yaw_body;
			math::Vector<3> y_C(-1*sin(psi), cos(psi), 0.0f);
			if(!_pos_sp_triplet.current.yaw_valid || std::isnan(psi)){
				y_C.zero();
				y_C(1) = 1.0f;
				//warnx("Yaw %f \n", (double) psi);

			}


			if (fabsf(body_z(2)) > SIGMA) {
				/* desired body_x axis, orthogonal to body_z */
				body_x =  y_C % body_z ;

				/* keep nose to front while inverted upside down */
				if (body_z(2) < 0.0f) {
					body_x.zero();
					body_x(0) =1.0f;
				}

				body_x.normalize();

			} else {
				/* desired thrust is in XY plane, set X downside to construct correct matrix,
				 * but yaw component will not be used actually */
				body_z.zero();
				body_z(2) = 1.0f;
			}

			/* desired body_y axis */
			body_y = body_z % body_x;

			/* fill rotation matrix */
			for (int i = 0; i < 3; i++) {
				_R_setpoint(i, 0) = body_x(i);
				_R_setpoint(i, 1) = body_y(i);
				_R_setpoint(i, 2) = body_z(i);
			}



			/* copy quaternion setpoint to attitude setpoint topic */
			matrix::Quatf q_sp = _R_setpoint;
			memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
			_att_sp.q_d_valid = true;

			/* calculate euler angles, for logging only, must not be used for control */
			matrix::Eulerf euler = _R_setpoint;
			_att_sp.roll_body = euler(0);
			_att_sp.pitch_body = euler(1);
			/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

		} else if (!_control_mode.flag_control_manual_enabled) {
			/* autonomous altitude control without position control (failsafe landing),
			 * force level attitude, don't change yaw */
			_R_setpoint = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);

			/* copy quaternion setpoint to attitude setpoint topic */
			matrix::Quatf q_sp = _R_setpoint;
			memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));

			_att_sp.q_d_valid = true;

			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;
		}



		_att_sp.thrust = (thrust_abs) / 2.0f;



		_att_sp.timestamp = hrt_absolute_time();


	} else {
		_reset_int_z = true;
	}
}

void
TailsitterPositionControl::generate_attitude_setpoint(float dt)
{
	/* reset yaw setpoint to current position if needed */
	if (_reset_yaw_sp) {
		_reset_yaw_sp = false;
		_att_sp.yaw_body = _yaw;
	}

	/* do not move yaw while sitting on the ground */
	else if (!_vehicle_land_detected.landed &&
		 !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {

		/* we want to know the real constraint, and global overrides manual */
		const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
					   _params.global_yaw_max;
		const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

		_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
		float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
		float yaw_offs = _wrap_pi(yaw_target - _yaw);

		// If the yaw offset became too big for the system to track stop
		// shifting it, only allow if it would make the offset smaller again.
		if (fabsf(yaw_offs) < yaw_offset_max ||
		    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
		    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
			_att_sp.yaw_body = yaw_target;
		}
	}

	/* control throttle directly if no climb rate controller is active */
	if (!_control_mode.flag_control_climb_rate_enabled) {
		float thr_val = throttle_curve(_manual.z, _params.thr_hover);
		_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

		/* enforce minimum throttle if not landed */
		if (!_vehicle_land_detected.landed) {
			_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
		}
	}

	/* control roll and pitch directly if no aiding velocity controller is active */
	if (!_control_mode.flag_control_velocity_enabled) {
		_att_sp.roll_body = _manual.y * _params.man_roll_max;
		_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

		/* only if optimal recovery is not used, modify roll/pitch */
		if (_params.opt_recover <= 0) {
			// construct attitude setpoint rotation matrix. modify the setpoints for roll
			// and pitch such that they reflect the user's intention even if a yaw error
			// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
			// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
			// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
			// heading of the vehicle.

			// calculate our current yaw error
			float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

			// compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			math::Vector<3> zB = {0, 0, 1};
			math::Matrix<3, 3> R_sp_roll_pitch;
			R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
			math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;


			// transform the vector into a new frame which is rotated around the z axis
			// by the current yaw error. this vector defines the desired tilt when we look
			// into the direction of the desired heading
			math::Matrix<3, 3> R_yaw_correction;
			R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
			z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

			// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
			// R_tilt is computed from_euler; only true if cos(roll) not equal zero
			// -> valid if roll is not +-pi/2;
			_att_sp.roll_body = -asinf(z_roll_pitch_sp(1));
			_att_sp.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
		}

		/* copy quaternion setpoint to attitude setpoint topic */
		matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
		memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
		_att_sp.q_d_valid = true;
	}

	if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
	    !_vehicle_land_detected.landed) {
		_att_sp.landing_gear = 1.0f;

	} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		_att_sp.landing_gear = -1.0f;
	}

	_att_sp.timestamp = hrt_absolute_time();
}

void
TailsitterPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

	parameters_update(true);

	_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool was_armed = false;

	hrt_abstime t_prev = 0;

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = -1.0f;


	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.0f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_vel_sp_prev.zero();
			_reset_int_z = true;
			_reset_int_xy = true;
			_reset_yaw_sp = true;
		}


		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		update_ref();


		update_velocity_derivative();

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) {

			do_control(dt);

			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			_local_pos_sp.vx = _vel_sp(0);
			_local_pos_sp.vy = _vel_sp(1);
			_local_pos_sp.vz = _vel_sp(2);

			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			_reset_int_z = true;
			_reset_int_xy = true;
			control_vel_enabled_prev = false;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			generate_attitude_setpoint(dt);

		} else {
			_reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}

		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;

		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity/accel control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app. Also do not publish
		 * if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
		 * attitude setpoints for the transition).
		 */
		if (!(_control_mode.flag_control_offboard_enabled &&
		      !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) {

			if (_att_sp_pub != nullptr) {

				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

			} else if (_attitude_setpoint_id) {
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		_reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled
				      && !_control_mode.flag_control_climb_rate_enabled;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int
TailsitterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&TailsitterPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int ts_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ts_pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		ts_pos_control::g_control = new TailsitterPositionControl;

		if (ts_pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != ts_pos_control::g_control->start()) {
			delete ts_pos_control::g_control;
			ts_pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (ts_pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete ts_pos_control::g_control;
		ts_pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (ts_pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
