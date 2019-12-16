
#include <ch.h>
#include <hal.h>
#include "pwm_config.h"


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
  //.bdtr = 0,
  //.dier = 0
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
  //.bdtr = 0,
  //.dier = 0
};


void initPwms() {
  pwmStart(&PWMD5, &pwmcfg5);
  pwmStart(&PWMD1, &pwmcfg1);
}


