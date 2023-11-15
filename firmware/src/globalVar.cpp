#include "globalVar.h"
#include "hal.h"

/*
 * Encoders
 */
constexpr double INC_PER_MM = 17.753023791070714;
const double F = 0.8750322081937646;

Encoder enc1(STM32_TIM4, INC_PER_MM / F);
Encoder enc2(STM32_TIM8, INC_PER_MM);
Encoder enc3(STM32_TIM2, INC_PER_MM);
Encoder enc4(STM32_TIM3, INC_PER_MM);



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
  .bdtr = 0,
  .dier = 0,
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
  .bdtr = 0,
  .dier = 0,
};


Motor mot1(&PWMD5, &pwmcfg5, 0, 1);   // MOT1 : TIM5_CH1 TIM5_CH2
Motor mot2(&PWMD1, &pwmcfg1, 0, 1);   // MOT2 : TIM1_CH1 TIM1_CH2
Motor mot3(&PWMD5, &pwmcfg5, 2, 3);   // MOT3 : TIM5_CH3 TIM5_CH4


OdometryHolo odometry;
HolonomicControl control;

/*
 * USB Driver structure.
 */
#if HAL_USE_SERIAL_USB
SerialUSBDriver SDU1;
#else

#endif // HAL_USE_SERIAL_USB
