
#include "motors.h"
#include <cmath>

void Motor::init() {
  if(pwmd->state == PWM_READY) {
    return; // already initialized
  }
  pwmStart(pwmd, config);
}

/**
 *  cmd : from -100 to 100
 */
void Motor::set_cmd(float cmd)
{
  float abs_cmd = fabs(cmd);

  if(abs_cmd > 100.0) {
    abs_cmd = 100.0;
  }
  pwmcnt_t width = (pwmcnt_t)((100.0 - abs_cmd)/100.0 * config->period);

    // CW : IN1 PWM, IN2 HIGH
    // CCW: IN1 HIGH, IN1 PWM
    if(cmd > 0) {
    pwmEnableChannel(pwmd, ch1, width);
    pwmEnableChannel(pwmd, ch2, config->period);
    } else {
    pwmEnableChannel(pwmd, ch1, config->period);
    pwmEnableChannel(pwmd, ch2, width);
    }
}
