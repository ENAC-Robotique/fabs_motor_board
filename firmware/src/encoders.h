#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include <ch.h>
#include <hal.h>

// 16 bits
extern const volatile int32_t * const tim4CntPtr;
#define ENC1_CNT (*tim4CntPtr)
#define TIM_ENC1 STM32_TIM4

// 16 bits
extern const volatile int32_t * const tim8CntPtr;
#define ENC2_CNT (*tim8CntPtr)
#define TIM_ENC2 STM32_TIM8

// 32 bits
extern const volatile int32_t * const tim2CntPtr;
#define ENC3_CNT (*tim2CntPtr)
#define TIM_ENC3 STM32_TIM2

// 16 bits
extern const volatile int32_t * const tim3CntPtr;
#define ENC4_CNT (*tim3CntPtr)
#define TIM_ENC4 STM32_TIM3

int32_t get_delta_enc1(void);
int32_t get_delta_enc2(void);
int32_t get_delta_enc3(void);
int32_t get_delta_enc4(void);

void initEnc1 (bool invert);
void initEnc2 (bool invert);
void initEnc3 (bool invert);
void initEnc4 (bool invert);


#endif // __ENCODERS_H__
