#pragma once

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

void start_motor_control_pid(void);
void set_speed_setPoint(double vx, double vy, double vtheta);
void set_pid_gains(uint32_t motor_no, double feedforward, double kp, double ki, double kd);

#ifdef __cplusplus
}
#endif
