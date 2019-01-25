/*
 * servo_step_test.cpp
 *
 *  Created on: Jul 25, 2017
 *      Author: yilun
 */


#include "ts_loadcell_test.h"

#define SPD_TEST_LEVEL 5.f
#define NUM_AGL_DELTA 4.f
const float delta_angles[] = {5.f, 10.f, 25.f, 50.f};

int servo_step_test(int argc, char *argv[])
{
	if (!argv) errx(1, "Please provide enabled channels");
	bool left_en = false;
	bool right_en = false;
	bool reverse = false;
	//printf("%s %s", argv[0], argv[1]);
	if (!strcmp(argv[1], "left")) left_en = true;
	else if (!strcmp(argv[1], "right")) right_en = true;
	else if (!strcmp(argv[1], "both")) left_en = right_en = true;
	else if (!strcmp(argv[1], "reverse")) left_en = right_en = reverse = true;
	else{
		warnx("not recognized command exiting...");
		exit(0);
	}
	test_should_exit = false;
	orb_advert_t _actuator_test_pub = NULL;
	struct ts_actuator_controls_s msg;

	uint64_t test_finish;
	float test_elapsed = 0;

	LoopTimer loopTimer(500000);//Loop Period 0.5s
	//LoopTimer loopTimer(20000);

	/* init all actuators to zero */
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	_actuator_test_pub = orb_advertise(ORB_ID(ts_actuator_controls_0), &msg);

	/* Arm the system */
	struct actuator_armed_s aa;

	arm_disarm_construct_msg(&aa, true);

	orb_advert_t actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &aa);



	for (unsigned i = 0; i < SPD_TEST_LEVEL; i ++) {
		msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = left_en ? (i + 1) / SPD_TEST_LEVEL : 0.0f;
		msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = right_en ? (i + 1) / SPD_TEST_LEVEL : 0.0f;
		//Publish to let motor spin to desired target first
		orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
		usleep(500000);//sleep 500ms waiting for motor response
		for (unsigned j = 0; j < sizeof(delta_angles)/sizeof(delta_angles[0]); j ++) {
			float delta_angle = delta_angles[j];
			float init_angle = (j % 2) ? -50.f : 50.f;
			float angle = init_angle;
			do {
				if (j % 2) angle += delta_angle;
				else angle -= delta_angle;
				msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = left_en ? (angle) : 0.0f;
				msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = right_en ? (angle) : 0.0f;
				if (reverse) msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = - msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT];
				msg.timestamp = hrt_absolute_time();
				if (test_should_exit) goto stop;
				orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
				loopTimer.wait();
			} while (fabsf(angle) < 50.f);
		}
	}

	//Gradually ramp down the motors
	test_finish = hrt_absolute_time();


	while(test_elapsed < 5.f){ //Ramp down in 5 seconds
		loopTimer.wait();
		test_elapsed = hrt_elapsed_time(&test_finish)/1e6f;
		msg.timestamp = hrt_absolute_time();
		msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = left_en ? (1.f - (fabsf(test_elapsed) / 5.f)) : 0.f;
		msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = right_en ? (1.f - (fabsf(test_elapsed) / 5.f)) : 0.f;;
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = 0;
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = 0;
		orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	}


	/* Disarm the system after finish */
stop:
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	usleep(1000);

	arm_disarm_construct_msg(&aa, false);
	orb_publish(ORB_ID(actuator_armed), actuator_armed_pub, &aa);

	orb_unadvertise(actuator_armed_pub);
	orb_unadvertise(_actuator_test_pub);

	return 0;
}







