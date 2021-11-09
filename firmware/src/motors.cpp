
#include "motors.h"
#include "arm_math.h"

/*
MOT1 : TIM5_CH1 TIM5_CH2

MOT2 : TIM5_CH3 TIM5_CH4

MOT3 : TIM1_CH1 TIM1_CH2
*/

constexpr uint32_t PWM_FREQ = 50000;
constexpr pwmcnt_t PWM_PERIOD = 250;


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

static pwmcnt_t getPwmCnt(float speed) {
  if(fabs(speed) > 100.0) {
    speed = 100.0;
  } else {
    speed = fabs(speed);
  }
  return (pwmcnt_t)((100.0 - speed)/100.0 * PWM_PERIOD);
}

/**
 *  speed : from -100 to 100
 *
 */
template<PWMDriver* D, pwmchannel_t CH1, pwmchannel_t CH2>
  void setMot(float speed){
      // from 0 to PWM_PERDIOD at most
    pwmcnt_t width = getPwmCnt(speed);

    // CW : IN1 PWM, IN2 HIGH
    // CCW: IN1 HIGH, IN1 PWM
    if(speed > 0) {
    pwmEnableChannel(D, CH1, width);
    pwmEnableChannel(D, CH2, PWM_PERIOD);
    } else {
    pwmEnableChannel(D, CH1, PWM_PERIOD);
    pwmEnableChannel(D, CH2, width);
    }
}



void setmot1(float speed) {setMot1(speed);}
void setmot2(float speed) {setMot2(speed);}
void setmot3(float speed) {setMot3(speed);}

