
#include "speed_control.h"
#include <ch.h>
#include "HolonomicControl.h"
#include "DifferentialControl.h"
#include "config.h"

#if defined(DRIVE_CONF_DIFFERENTIAL) && defined(DRIVE_CONF_HOLONOMIC)
#error "You must choose DRIVE_CONF_DIFFERENTIAL or DRIVE_CONF_HOLONOMIC (not both)."
#endif

#if !defined(DRIVE_CONF_DIFFERENTIAL) && !defined(DRIVE_CONF_HOLONOMIC)
#error "You must define DRIVE_CONF_DIFFERENTIAL or DRIVE_CONF_HOLONOMIC."
#endif


#if defined(DRIVE_CONF_HOLONOMIC)
HolonomicControl speed_ctrl;
#elif defined(DRIVE_CONF_DIFFERENTIAL)
DifferentialControl speed_ctrl;
#endif


static THD_WORKING_AREA(waSpeedControl, 500);	// declaration de la pile du thread blinker

static void spctrl(void* arg) {
  chRegSetThreadName("Speed Control");
  speed_ctrl.speed_control(arg);
}

void start_motor_control_pid() {

  chThdCreateStatic(waSpeedControl, sizeof(waSpeedControl), NORMALPRIO, &spctrl, NULL);
}



void set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta) {
  speed_ctrl.set_speed_setPoint(vx, vy, vtheta);
}

/**
 * Set speed setPoint from norm and direction.
 * speed: robot speed in mm/s
 * direction: speed direction in radians, [0, 2*pi]
 * omega: rotation speed in rad/s
 */
void set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega) {
  speed_ctrl.set_speed_setPoint_norm_dir(speed, direction, omega);
}

void set_pid_gains(float32_t kp, float32_t ki, float32_t kd) {
  speed_ctrl.set_pid_gains(kp, ki, kd);
}
