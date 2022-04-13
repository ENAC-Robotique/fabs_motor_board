
#include "speed_control.h"
#include <ch.h>
#include "HolonomicControl.h"
#include "DifferentialControl.h"
#include "OdometryDiff.h"
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


static THD_WORKING_AREA(waSpeedControl, 1000);

static void spctrl(void* arg) {
  chRegSetThreadName("Speed Control");
  odometry.init();
  speed_ctrl.init();
  speed_ctrl.speed_control(arg);
}

void start_motor_control_pid() {

  chThdCreateStatic(waSpeedControl, sizeof(waSpeedControl), NORMALPRIO, &spctrl, NULL);
}
