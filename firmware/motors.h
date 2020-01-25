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
#define PWM_FREQ 50000
#define PWM_PERIOD 250 


void initPwms(void);

void setMot1(float speed);
void setMot2(float speed);
void setMot3(float speed);

#endif // __PWM_CONFIG_H__
