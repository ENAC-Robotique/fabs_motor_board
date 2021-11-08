#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <ch.h>
#include <hal.h>
#include "arm_math.h"


#define SPEED_CONTROL_PERIOD 0.050
#define ODOMETRY_PERIOD 0.025

void start_motor_control_pid(void);
void set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta);

/**
 * Set speed setPoint from norm and direction.
 * speed: robot speed in mm/s
 * direction: speed direction in radians, [0, 2*pi]
 * omega: rotation speed in rad/s
 */
void set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega);

void set_pid_gains(float32_t kp, float32_t ki, float32_t kd);

void clamping(arm_matrix_instance_f32* cmd, arm_matrix_instance_f32* error, float32_t *mins, float32_t *maxs, bool* clamps);
#endif // __MOTOR_CONTROL_H__
