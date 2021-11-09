#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <ch.h>
#include <hal.h>

// max switching frequency : 2kHz, whatever it means
//
// max switching time = 30 µs
// lets take a minimum period of 20 µs, and 250 steps.
// 20 µs => 50 kHz
// 250 steps : 5ms => 200 Hz  Is that enough ?????
//


void initPwms(void);

#ifdef __cplusplus


template<PWMDriver* D, pwmchannel_t CH1, pwmchannel_t CH2>
  void setMot(float speed);

// MOT1 : TIM5_CH1 TIM5_CH2
constexpr auto setMot1 = setMot<&PWMD5, 0, 1>;
//MOT2 : TIM5_CH3 TIM5_CH4
constexpr auto setMot2 = setMot<&PWMD5, 2, 3>;
//MOT3 : TIM1_CH1 TIM1_CH2
constexpr auto setMot3 = setMot<&PWMD1, 0, 1>;

#endif



#ifdef __cplusplus
extern "C" {
#endif
  void setmot1(float speed);
  void setmot2(float speed);
  void setmot3(float speed);


#ifdef __cplusplus
}
#endif


#endif // __PWM_CONFIG_H__
