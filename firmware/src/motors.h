#pragma once
#include <ch.h>
#include <hal.h>
#include "pid.h"
// max switching frequency : 2kHz, whatever it means
//
// max switching time = 30 µs
// lets take a minimum period of 20 µs, and 250 steps.
// 20 µs => 50 kHz
// 250 steps : 5ms => 200 Hz  Is that enough ?????
//

#ifdef __cplusplus

class Motor {
public:
  Motor(PWMDriver* pwmd, PWMConfig* config, pwmchannel_t ch1, pwmchannel_t ch2): pwmd(pwmd), config(config), ch1(ch1), ch2(ch2) {}
  void init();
  void set_cmd(float cmd);

private:

  PWMDriver* pwmd;
  PWMConfig* config;
  pwmchannel_t ch1;
  pwmchannel_t ch2;
};

#endif

