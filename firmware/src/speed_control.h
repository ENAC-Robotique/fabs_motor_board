#pragma once

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_motor_control_pid(void);
void set_speed_setPoint(double vx, double vy, double vtheta);
void set_pid_gains(double ng, double kp, double ki, double kd);

#ifdef __cplusplus
}
#endif
