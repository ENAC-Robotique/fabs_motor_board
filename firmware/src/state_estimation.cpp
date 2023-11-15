
#include "state_estimation.h"
#include <ch.h>
#include "HolonomicControl.h"
#include "OdometryHolo.h"
#include "config.h"
#include "globalVar.h"

static THD_WORKING_AREA(waEncoders, 2000);

static void encoders_thd(void* arg) {
  (void)arg;
  chRegSetThreadName("Encoders update");

  while (true)
  {
    systime_t now = chVTGetSystemTime();
    odometry.update_filters();
    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(ENCODERS_PERIOD)));
  }
}

static THD_WORKING_AREA(waOdometry, 4000);

static void odometry_thd(void* arg) {
  (void)arg;
  chRegSetThreadName("Odometry");

  while (true)
  {
    systime_t now = chVTGetSystemTime();
    odometry.update();
    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(ODOMETRY_PERIOD)));
  }
}

static THD_WORKING_AREA(waControl, 4000);

static void control_thd(void* arg) {
  (void)arg;
  chRegSetThreadName("Holonomic speed control");

  // tmp
  Eigen::Vector3d pos_cons = {0, 0, 0};
  Eigen::Vector3d speed_cons = {100, 0, 0};
  systime_t start = chVTGetSystemTime();
  
  while (true)
  {
    systime_t now = chVTGetSystemTime();

    //tmp
    time_msecs_t t = chTimeI2MS(chTimeDiffX(start, now));
    double speed_x = 200 * sin(0.1 * 6.28*t/1000.0);
    speed_cons[0] = speed_x;
    pos_cons += speed_cons * CONTROL_PERIOD/1000.0;
    control.set_setPoints(pos_cons, speed_cons);

    control.update();
    chThdSleepUntil(chTimeAddX(now, chTimeMS2I(CONTROL_PERIOD)));
  }
}



void start_state_estimation() {
  odometry.init();
  control.init();
  chThdCreateStatic(waEncoders, sizeof(waEncoders), NORMALPRIO, &encoders_thd, NULL);
  chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, &odometry_thd, NULL);
  chThdCreateStatic(waControl, sizeof(waControl), NORMALPRIO, &control_thd, NULL);
}
