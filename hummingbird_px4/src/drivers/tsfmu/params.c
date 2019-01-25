/*
 * params.c
 *
 *  Created on: Nov 5, 2017
 *      Author: tracy
 */

/**
 * PWM Left Servo Trim.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_PWM_TRIM_L, 54);

/**
 * PWM Right Servo Trim.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_PWM_TRIM_R, 0);

/**
 * Motor PI controller P gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT_P, 0.065);

/**
 * Motor PI controller I gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT_I, 0.2);

/**
 * Channel 0 Motor PI controller Feedforward gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT0_KW2, 0.0000040549);

/**
 * Channel 0 Motor PI controller Feedforward gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT0_KW, 0.0045781480);

/**
 * Channel 0 Motor PI controller Feedforward gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT0_KC, 4.3063206673);

/**
 * Channel 1 Motor PI controller Feedforward gain.
 *
 * @group GPS
 */
PARAM_DEFINE_FLOAT(TS_MOT1_KW2, 0.0000034023);

/**
 * Channel 1 Motor PI controller Feedforward gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT1_KW, 0.0023399056);

/**
 * Channel 1 Motor PI controller Feedforward gain.
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(TS_MOT1_KC, 5.6532120705);
/**
 * Servo Motor Slew Rate
 *
 * @group TS
 */
PARAM_DEFINE_FLOAT(SERVO_SLEW_RATE, 100.f);

