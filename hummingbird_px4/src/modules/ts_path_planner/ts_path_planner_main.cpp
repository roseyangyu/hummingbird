/**
 * @file ts_att_control_main.cpp
 * Tailsitter attitude controller.
 *
 * @author Xintong Du	<xintong.du@mail.utoronto.ca>
 *
 */
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <systemlib/px4_macros.h>
#include "systemlib/param/param.h"
#include <lib/mathlib/mathlib.h>
#include <platforms/px4_workqueue.h>
#include "ts_path_planner.h"

/**
 * Tailsitter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int ts_path_planner_main(int argc, char *argv[]);

namespace ts_path_planner
{
TailsitterPathPlanner	*g_planner;
}

TailsitterPathPlanner::TailsitterPathPlanner():
		_task_should_exit(false),
		_planner_task(-1),
		_setpoint_updated(false),
		_control_mode_updated(false),
		_v_control_mode_pub(nullptr),
		_position_setpoint_pub(nullptr),
		_params_sub(-1),
		_local_pos_sub(-1),
		_position_setpoint_step_sub(-1),
		_control_mode{},
		_pos_sp_triplet{},
		_pos_sp_triplet_step{},
		_local_pos{},
		_work{},
		_looptimer(2e4)
{
	_params.cruise_speed_max.zero();
	_params.cruise_speed = 0;
	_param_handles.z_cruise_speed = param_find("TS_CRUISE_MAX_Z");
	_param_handles.xy_cruise_speed = param_find("TS_CRUISE_MAX_XY");
	_param_handles.cruise_speed = param_find("TS_CRUISE_SPEED");
	params_update(true);

	_waypoint.start_time = hrt_absolute_time();
	_waypoint.end_point.zero();
	_waypoint.direction.zero();
	_waypoint.start_point.zero();
	_waypoint.velocity.zero();
	_waypoint.yaw = 0;

}

TailsitterPathPlanner::~TailsitterPathPlanner()
{
	if (_planner_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_planner_task);
				break;
			}
		} while (_planner_task != -1);
	}

	ts_path_planner::g_planner = nullptr;
}


int
TailsitterPathPlanner::start(){

	ASSERT(_planner_task == -1);

	_planner_task = px4_task_spawn_cmd("ts_path_planner",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1000,
					   (px4_main_t)&TailsitterPathPlanner::task_main_trampoline,
					   nullptr);

	if (_planner_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
TailsitterPathPlanner::task_main_trampoline(int argc, char *argv[])
{
	ts_path_planner::g_planner->task_main();
}

void
TailsitterPathPlanner::task_main()
{
	reset_control_mode();
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_position_setpoint_step_sub = orb_subscribe(ORB_ID(position_setpoint_triplet_step));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	work_queue(HPWORK, &_work, (worker_t)&TailsitterPathPlanner::publish_control_mode_trampoline, this, 0);

	// If in 'raw' mode, we poll the setpoint and publish upon receive
	px4_pollfd_struct_t fds[] = {
		{ .fd = _position_setpoint_step_sub,   .events = POLLIN },
	};

	while(!_task_should_exit){
		
		if (this->raw_mode) {
			// wait for event for up to 1s
			int poll_ret = px4_poll(fds, 1, 1000);
			if (poll_ret > 0 && fds[0].revents & POLLIN) { 
				poll_subscriptions();
			}
		} else {
			_looptimer.wait();
			poll_subscriptions();
		}

		// Update the set point
		if (_setpoint_updated){

			math::Vector<3> next_point;
			math::Vector<3> velocity = _waypoint.velocity;

			if (this->raw_mode) {
				next_point = _waypoint.end_point;
				_setpoint_updated = false;
			} else {
				float dt = (hrt_absolute_time() - _waypoint.start_time)/1e6f;
				next_point = _waypoint.start_point + _waypoint.direction * dt * _waypoint.speed;
			}

			if(!(this->raw_mode) && (next_point - _waypoint.end_point).length() < 0.05f){
				_setpoint_updated = false;
				next_point = _waypoint.end_point;
				velocity.zero();
			}

			_pos_sp_triplet.previous = _pos_sp_triplet.current;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.current.position_valid = true;
			_pos_sp_triplet.current.velocity_valid = true;
			_pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
			_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
			_pos_sp_triplet.current.alt_valid = false;
			_pos_sp_triplet.current.yawspeed_valid = false;
			_pos_sp_triplet.current.acceleration_valid =  false;
			_pos_sp_triplet.current.yaw_valid = true;
			_pos_sp_triplet.current.x = next_point(0);
			_pos_sp_triplet.current.y = next_point(1);
			_pos_sp_triplet.current.z = next_point(2);
			_pos_sp_triplet.current.vx = velocity(0);
			_pos_sp_triplet.current.vy = velocity(1);
			_pos_sp_triplet.current.vz = velocity(2);
			_pos_sp_triplet.current.yaw = _waypoint.yaw;
			_pos_sp_triplet.current.timestamp = hrt_absolute_time();
//			printf("Next point %f, %f, %f\n", (double) next_point(0),(double) next_point(1),(double) next_point(2));
//			printf("Velocity %f, %f, %f\n", (double) velocity(0),(double) velocity(1),(double) velocity(2));

			publish_setpoint();
		}
	}

}

void
TailsitterPathPlanner::publish_setpoint()
{
	
	if (_position_setpoint_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _position_setpoint_pub, &_pos_sp_triplet);

	} else {
		_position_setpoint_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

void
TailsitterPathPlanner::publish_control_mode_trampoline(void *arg)
{
	TailsitterPathPlanner *dev = reinterpret_cast<TailsitterPathPlanner *>(arg);
	dev->publish_control_mode();
}

void
TailsitterPathPlanner::publish_control_mode()
{
	if(_v_control_mode_pub != nullptr){
		_control_mode.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(offboard_control_mode), _v_control_mode_pub, &_control_mode);
	}
	else{
		_v_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_control_mode);
	}
	work_queue(HPWORK, &_work, (worker_t)&TailsitterPathPlanner::publish_control_mode_trampoline, this, USEC2TICK(1e5));
}

void
TailsitterPathPlanner::update_pos_setpoint(int argc, char*argv[]){

	if(argc < 2){
		warnx("usage: ts_path_planner pub {att|pos}");
	}
	else{
		if(!strcmp(argv[0], "acc")){
			if (_control_mode.ignore_acceleration_force){

				_control_mode.ignore_acceleration_force = false;
				_control_mode.ignore_position = true;
			}



			_pos_sp_triplet.previous = _pos_sp_triplet.current;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.current.position_valid = false;
			_pos_sp_triplet.current.velocity_valid = false;
			_pos_sp_triplet.current.alt_valid = false;
			_pos_sp_triplet.current.yawspeed_valid = false;

			_pos_sp_triplet.current.acceleration_valid =  true;
			_pos_sp_triplet.current.yaw_valid = true;
			PX4_INFO("Command Acc: %s, %s, %s, %s", argv[1],
													argv[2],
													argv[3],
													argv[4]);
			_pos_sp_triplet.current.a_x = strtof(argv[1], 0);
			_pos_sp_triplet.current.a_y = strtof(argv[2], 0);
			_pos_sp_triplet.current.a_z = strtof(argv[3], 0);
			_pos_sp_triplet.current.yaw = strtof(argv[4], 0);
			usleep(2e6);
			//_setpoint_updated = true;
			publish_setpoint();
			//publish_setpoint();

		}

		if(!strcmp(argv[0], "pos")){
			math::Vector<3> end_point;
			math::Vector<3> velocity;
			float yaw;
			end_point(0) = strtof(argv[1], 0);
			end_point(1) = strtof(argv[2], 0);
			end_point(2) = strtof(argv[3], 0);
			velocity(0) = 0;
			velocity(1) = 0;
			velocity(2) = 0;
			yaw = strtof(argv[4], 0) / 180.f * 3.1415926f;
			set_waypoint(end_point, velocity, yaw);
		}
	}
}


void
TailsitterPathPlanner::update_control_mode(int argc, char* argv[])
{
	reset_control_mode();
	if(!strcmp(argv[0], "acc")) {
		this->raw_mode = false;
		_control_mode.ignore_acceleration_force = false;
	}
	if(!strcmp(argv[0], "pos")) {
		this->raw_mode = false;
		_control_mode.ignore_position = false;
	}
	if(!strcmp(argv[0], "raw")) {
		// raw mode will publish the setpoint as provided only once.
		this->raw_mode = true;
		PX4_INFO("Entering raw mode");
	}
}


void
TailsitterPathPlanner::reset_control_mode()
{
	_control_mode.ignore_thrust = true;
	_control_mode.ignore_attitude = true;
	_control_mode.ignore_bodyrate = true;
	_control_mode.ignore_position = false;
	_control_mode.ignore_velocity = true;
	_control_mode.ignore_acceleration_force = true;
	_control_mode.ignore_alt_hold = true;
}
void
TailsitterPathPlanner::params_update(bool force)
{
	bool updated;
		struct parameter_update_s param_upd;

		orb_check(_params_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
		}

		if (updated || force) {
			float v;
			param_get(_param_handles.z_cruise_speed, &v);
			_params.cruise_speed_max(2) = v;
			param_get(_param_handles.xy_cruise_speed, &v);
			_params.cruise_speed_max(0) = v;
			_params.cruise_speed_max(1) = v;
			param_get(_param_handles.cruise_speed, &_params.cruise_speed);

		}
}

void
TailsitterPathPlanner::set_waypoint(math::Vector<3> end_point, math::Vector<3> velocity, float yaw)
{
	if (_control_mode.ignore_position) {
		_control_mode.ignore_position = false;
		_control_mode.ignore_acceleration_force = true;
	}
	_waypoint.start_point(0) = _local_pos.x;
	_waypoint.start_point(1) = _local_pos.y;
	_waypoint.start_point(2) = _local_pos.z;
	_waypoint.end_point = end_point;
	_waypoint.direction =  _waypoint.end_point - _waypoint.start_point;
	_waypoint.direction.normalize();
	_waypoint.yaw = yaw;
	// Saturate the maximum velocity in all directions
	for (int i=0; i<3; i++){
		if (velocity(i) > _params.cruise_speed_max(i) || velocity(i) < -_params.cruise_speed_max(i)){
			PX4_INFO("Clipping velocity while setting waypoint");
			float scale = _params.cruise_speed_max(i) / velocity(i);
			scale = scale > 0 ? scale:-scale;
			velocity = velocity * scale;
		}
	}
	if (velocity.length() < 1e-6f ) { // If effectively no velocity
		PX4_INFO("Published setpoint has effectively no velocity. Setting based on default cruise speed.");
		velocity = _waypoint.direction * _params.cruise_speed;
	}
	_waypoint.velocity = velocity;
	_waypoint.speed = velocity.length();
	_waypoint.start_time = hrt_absolute_time();
	_setpoint_updated = true;
}

void
TailsitterPathPlanner::poll_subscriptions()
{
	bool updated;
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_position_setpoint_step_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet_step), _position_setpoint_step_sub, &_pos_sp_triplet_step);
		math::Vector<3> velocity;
		velocity(0) = _pos_sp_triplet_step.current.vx;
		velocity(1) = _pos_sp_triplet_step.current.vy;
		velocity(2) = _pos_sp_triplet_step.current.vz;
		math::Vector<3> end_point;
		end_point(0) = _pos_sp_triplet_step.current.x;
		end_point(1) = _pos_sp_triplet_step.current.y;
		end_point(2) = _pos_sp_triplet_step.current.z;
		float yaw = _pos_sp_triplet_step.current.yaw;
		set_waypoint(end_point, velocity, yaw);
	}
}




int ts_path_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: ts_path_planner {start|stop|status|pub|control}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ts_path_planner::g_planner != nullptr) {
			warnx("already running");
			return 1;
		}

		ts_path_planner::g_planner = new TailsitterPathPlanner;

		if (ts_path_planner::g_planner == nullptr) {
			warnx("alloc failed");
			return 1;
		}


		if (OK != ts_path_planner::g_planner->start()) {
			delete ts_path_planner::g_planner;
			ts_path_planner::g_planner = nullptr;
			warnx("start failed");
			return 1;
		}
		PX4_INFO("Planner started");

		return 0;
	}


	if (!strcmp(argv[1], "stop")) {
		if (ts_path_planner::g_planner == nullptr) {
			warnx("not running");
			return 1;
		}

		delete ts_path_planner::g_planner;
		ts_path_planner::g_planner = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (ts_path_planner::g_planner) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	if(!strcmp(argv[1], "pub")){
		if(ts_path_planner::g_planner == nullptr){
			warnx("not running");
			return 1;
		}
		else{
			ts_path_planner::g_planner->update_pos_setpoint(argc-2, argv+2);
		}
		return 0;
	}

	if(!strcmp(argv[1], "control")){
		if(ts_path_planner::g_planner == nullptr){
			warnx("not running");
			return 1;
		}
		else{
			ts_path_planner::g_planner->update_control_mode(argc-2, argv+2);

			}

		return 0;
	}

	warnx("unrecognized command");
	return 1;
}
