#include <parameters/param.h>



/**
 * Pitch P gain
 *
 * Pitch Proportional gain, i.e. desired angular speed in rad/s for error 1 rad
 *
 * @unit 1/s
 *
 * @min 0.0
 *
 * @max 10.0
 *
 * @increment 0.0005
 *
 * @reboot_required true
 *
 * @group Pendulum Attitude Control
 */



PARAM_DEFINE_FLOAT(PEND_PITCH_P, 0.5f);




struct pendulum_pid
{
	float p;
	float i;
	float d;
}




