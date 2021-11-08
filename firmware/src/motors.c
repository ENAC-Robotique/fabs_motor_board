
#include <ch.h>
#include <hal.h>
#include "motors.h"
#include "arm_math.h"

/*
MOT1 : TIM5_CH1 TIM5_CH2

MOT2 : TIM5_CH3 TIM5_CH4

MOT3 : TIM1_CH1 TIM1_CH2
*/

PWMConfig pwmcfg5 = {
  .frequency = PWM_FREQ,
  .period = PWM_PERIOD,
  .callback = NULL,
  .channels = {
    {                                                               //ch1
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
    {                                                               //ch2
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
    {                                                               //ch3
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
    {                                                               //ch4
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
  },
  .cr2 = 0,
};


PWMConfig pwmcfg1 = {
  .frequency = PWM_FREQ,
  .period = PWM_PERIOD,
  .callback = NULL,
  .channels = {
    {                                                               //ch1
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
    {                                                               //ch2
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .callback = NULL
    },
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}

  },
  .cr2 = 0,
};


void initPwms() {
  pwmStart(&PWMD5, &pwmcfg5);
  pwmStart(&PWMD1, &pwmcfg1);
}


pwmcnt_t getPwmCnt(float speed) {
  if(fabs(speed) > 100.0) {
    speed = 100.0;
  } else {
    speed = fabs(speed);
  }
  return (pwmcnt_t)((100.0 - speed)/100.0 * PWM_PERIOD);
}


// from 0 to PWM_PERDIOD at most
// CW : IN1 PWM, IN2 HIGH
// CCW: IN1 HIGH, IN1 PWM

/**
 *  speed : from -100 to 100
 *
 */
void setMot1(float speed){
  // MOT1 : TIM5_CH1 TIM5_CH2
  pwmcnt_t width = getPwmCnt(speed);
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
  pwmcnt_t width = getPwmCnt(speed);
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
  pwmcnt_t width = getPwmCnt(speed);
  // discimine the case where speed = 0 ?
  if(speed > 0) {
    pwmEnableChannel(&PWMD1, 0, width);
    pwmEnableChannel(&PWMD1, 1, PWM_PERIOD);
  } else {
    pwmEnableChannel(&PWMD1, 0, PWM_PERIOD);
    pwmEnableChannel(&PWMD1, 1, width);
  }
}
