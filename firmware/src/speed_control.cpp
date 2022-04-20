
#include "speed_control.h"
#include <ch.h>
#include "HolonomicControl.h"
#include "DifferentialControl.h"
#include "OdometryDiff.h"
#include "OdometryHolo.h"
#include "config.h"

#if defined(DRIVE_CONF_DIFFERENTIAL) && defined(DRIVE_CONF_HOLONOMIC)
#error "You must choose DRIVE_CONF_DIFFERENTIAL or DRIVE_CONF_HOLONOMIC (not both)."
#endif

#if !defined(DRIVE_CONF_DIFFERENTIAL) && !defined(DRIVE_CONF_HOLONOMIC)
#error "You must define DRIVE_CONF_DIFFERENTIAL or DRIVE_CONF_HOLONOMIC."
#endif


static THD_WORKING_AREA(waSpeedControl, 2000);

static void diff_speed_control(void* arg) {
  chRegSetThreadName("Differential speed control");
  OdometryDiff odometry;
  DifferentialControl speed_ctrl;
  
  odometry.init();
  speed_ctrl.init();
  speed_ctrl.set_pid_gains(0, 0.14, 0.2, 0.1, 0);
  while (true)
  {
    speed_ctrl.speed_control(&odometry);
    chThdSleepMilliseconds(1);
  }
}

static void holo_speed_control(void* arg) {
  chRegSetThreadName("Holonomic speed control");
  OdometryHolo odometry;
  HolonomicControl speed_ctrl;
  
  odometry.init();
  speed_ctrl.init();
  // speed_ctrl.set_pid_gains(0, 0.14, 0.2, 0.1, 0);
  while (true)
  {
    speed_ctrl.speed_control(&odometry);
    chThdSleepMilliseconds(1);
  }
}

void start_motor_control_pid() {
#if defined(DRIVE_CONF_HOLONOMIC)
  chThdCreateStatic(waSpeedControl, sizeof(waSpeedControl), NORMALPRIO, &holo_speed_control, NULL);
#elif defined(DRIVE_CONF_DIFFERENTIAL)
  chThdCreateStatic(waSpeedControl, sizeof(waSpeedControl), NORMALPRIO, &diff_speed_control, NULL);
#endif
  
}
