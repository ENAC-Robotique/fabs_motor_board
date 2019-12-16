
#include "motor_control.h"
#include "pwm_config.h"
#include <math.h>
#include <ch.h>
#include "odometry.h"

/*
MOT1 : TIM5_CH1 TIM5_CH2

MOT2 : TIM5_CH3 TIM5_CH4

MOT3 : TIM1_CH1 TIM1_CH2
*/


// from 0 to PWM_PERDIOD at most
// CW : IN1 PWM, IN2 HIGH
// CCW: IN1 HIGH, IN1 PWM

/**
 *  speed : from -1 to 1
 *
 */
void setMot1(float speed){
  // MOT1 : TIM5_CH1 TIM5_CH2
  pwmcnt_t width = (1 - fabs(speed)) * PWM_PERIOD;
  // discimine the case where speed = 0 ?
  if(speed > 0) {
    pwmEnableChannel(&PWMD5, 0, width);
    pwmEnableChannel(&PWMD5, 1, PWM_PERIOD);
  } else {
    pwmEnableChannel(&PWMD5, 0, PWM_PERIOD);
    pwmEnableChannel(&PWMD5, 1, width);
  }
}

void setMot2(float speed){
  //MOT2 : TIM5_CH3 TIM5_CH4
  pwmcnt_t width = (1 - fabs(speed)) * PWM_PERIOD;
  // discimine the case where speed = 0 ?
  if(speed > 0) {
    pwmEnableChannel(&PWMD5, 2, width);
    pwmEnableChannel(&PWMD5, 3, PWM_PERIOD);
  } else {
    pwmEnableChannel(&PWMD5, 2, PWM_PERIOD);
    pwmEnableChannel(&PWMD5, 3, width);
  }
}

void setMot3(float speed){
  //MOT3 : TIM1_CH1 TIM1_CH2
  pwmcnt_t width = (1 - fabs(speed)) * PWM_PERIOD;
  // discimine the case where speed = 0 ?
  if(speed > 0) {
    pwmEnableChannel(&PWMD1, 0, width);
    pwmEnableChannel(&PWMD1, 1, PWM_PERIOD);
  } else {
    pwmEnableChannel(&PWMD1, 0, PWM_PERIOD);
    pwmEnableChannel(&PWMD1, 1, width);
  }
}



static THD_WORKING_AREA(waPID, 304);	// declaration de la pile du thread blinker
static void motor_control_pid (void *arg)			// fonction d'entrée du thread blinker
{
  (void)arg;
  chRegSetThreadName("PID");

  unsigned long lastTime = chVTGetSystemTime();

  while (true) {
    systime_t now = chVTGetSystemTime();
    float elapsed = ((float)(now - lastTime)) / CH_CFG_ST_FREQUENCY;
    lastTime = now;

    update_odometry(elapsed);

    chThdSleepMilliseconds(100);
  }
}


void start_motor_control_pid() {
  chThdCreateStatic(waPID, sizeof(waPID), NORMALPRIO, &motor_control_pid, NULL);
}
