#pragma once

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_motor_control_pid(void);
void set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta);
void set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega);
void set_pid_gains(float32_t kp, float32_t ki, float32_t kd);

#ifdef __cplusplus
}
#endif
