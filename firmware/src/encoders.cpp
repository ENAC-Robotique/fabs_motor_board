
#include <ch.h>
#include <hal.h>
#include "encoders.h"
#include "utils.h"

const volatile int32_t * const tim2CntPtr = (volatile int32_t *) &(STM32_TIM2->CNT);
const volatile int32_t * const tim3CntPtr = (volatile int32_t *) &(STM32_TIM3->CNT);
const volatile int32_t * const tim4CntPtr = (volatile int32_t *) &(STM32_TIM4->CNT);
const volatile int32_t * const tim8CntPtr = (volatile int32_t *) &(STM32_TIM8->CNT);


int32_t get_delta_enc1() {
  static int32_t last_pos_enc = 0;

  int32_t pos_enc = ENC1_CNT;
  int32_t delta_pos = pos_enc - last_pos_enc;
  last_pos_enc = pos_enc;


  if(TIM_ENC1->SR & TIM_SR_UIF    //an overflow appenned 
     && ABS(delta_pos) > 32000 ) {  //ensure that it has not been back (ex: CNT at 12, goes to 665500 by underflow, then back to 14 by overflow)
    if(pos_enc < 2000) {  // counter range from 0 to 65535. if couter is < 2000, we most probably did an overflow
      delta_pos += 65535;
    } else {  // we most probably did an underflow
      delta_pos -= 65535;
    }
  }
  TIM_ENC1->SR = 0;
  return delta_pos;
}

int32_t get_delta_enc2() {
  static int32_t last_pos_enc = 0;

  int32_t pos_enc = ENC2_CNT;
  int32_t delta_pos = pos_enc - last_pos_enc;
  last_pos_enc = pos_enc;


  if(TIM_ENC2->SR & TIM_SR_UIF    //an overflow appenned 
     && ABS(delta_pos) > 32000 ) {  //ensure that it has not been back (ex: CNT at 12, goes to 665500 by underflow, then back to 14 by overflow)
    if(pos_enc < 2000) {  // counter range from 0 to 65535. if couter is < 2000, we most probably did an overflow
      delta_pos += 65535;
    } else {  // we most probably did an underflow
      delta_pos -= 65535;
    }
  }
  TIM_ENC2->SR = 0;

  return delta_pos;
}


int32_t get_delta_enc3() {
  static int32_t last_pos_enc = 0;

  int32_t pos_enc = ENC3_CNT;
  int32_t delta_pos = pos_enc - last_pos_enc;
  last_pos_enc = pos_enc;

  // This counter is 32 bits, so no need for the overflow mecanism, its auto-math-ic! :-)
  return delta_pos;
}

int32_t get_delta_enc4() {
  static int32_t last_pos_enc = 0;

  int32_t pos_enc = ENC4_CNT;
  int32_t delta_pos = pos_enc - last_pos_enc;
  last_pos_enc = pos_enc;


  if(TIM_ENC4->SR & TIM_SR_UIF){    //an overflow appenned 
    if(ABS(delta_pos) > 32000 ) {  //ensure that it has not been back (ex: CNT at 12, goes to 665500 by underflow, then back to 14 by overflow)
      if(pos_enc < 31000) {  // counter range from 0 to 65535. if couter is < 2000, we most probably did an overflow
        delta_pos += 65535;
      } else {  // we most probably did an underflow
        delta_pos -= 65535;
      }
    }
  }
  TIM_ENC4->SR = 0;
  return delta_pos;
}



void initEnc1 (bool invert) {
  uint16_t CCER = 0;       // rising edge polarity
  if(invert) {
    CCER = TIM_CCER_CC1P;  // falling edge polarity
  }
  rccEnableTIM4(NULL);
  rccResetTIM4();
  STM32_TIM4->SMCR =  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // Encoder mode 3                         (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0)
  STM32_TIM4->CCER = CCER;
  STM32_TIM4->ARR = 0xFFFFFFFF;      // count from 0-ARR or ARR-0
  STM32_TIM4->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2   // TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  STM32_TIM4->CNT = 0;           // Initialize counter
  STM32_TIM4->EGR = 1;           // generate an update event
  STM32_TIM4->CR1 = TIM_CR1_CEN;//1;           // Enable the counter
}



void initEnc2 (bool invert) {
  uint16_t CCER = 0;       // rising edge polarity
  if(invert) {
    CCER = TIM_CCER_CC1P;  // falling edge polarity
  }
  rccEnableTIM8(NULL);
  rccResetTIM8();
  STM32_TIM8->SMCR =  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // Encoder mode 3                         (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0)
  STM32_TIM8->CCER = CCER;
  STM32_TIM8->ARR = 0xFFFFFFFF;      // count from 0-ARR or ARR-0
  STM32_TIM8->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2   // TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  STM32_TIM8->CNT = 0;           // Initialize counter
  STM32_TIM8->EGR = 1;           // generate an update event
  STM32_TIM8->CR1 = TIM_CR1_CEN;//1;           // Enable the counter
}


void initEnc3 (bool invert) {
  uint16_t CCER = 0;       // rising edge polarity
  if(invert) {
    CCER = TIM_CCER_CC1P;  // falling edge polarity
  }
  rccEnableTIM2(NULL);
  rccResetTIM2();
  STM32_TIM2->SMCR =  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // Encoder mode 3                         (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0)
  STM32_TIM2->CCER = CCER;
  STM32_TIM2->ARR = 0xFFFFFFFF;      // count from 0-ARR or ARR-0
  STM32_TIM2->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2   // TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  STM32_TIM2->CNT = 0;           // Initialize counter
  STM32_TIM2->EGR = 1;           // generate an update event
  STM32_TIM2->CR1 = TIM_CR1_CEN;//1;           // Enable the counter
}


void initEnc4 (bool invert) {
  uint16_t CCER = TIM_CCER_CC1P;  // falling edge polarity
  if(invert) {
    CCER = 0;       // rising edge polarity
  }
  rccEnableTIM3(NULL);
  rccResetTIM3();
  STM32_TIM3->SMCR =  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // Encoder mode 3                         (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0)
  STM32_TIM3->CCER = CCER;
  STM32_TIM3->ARR = 0xFFFFFFFF;      // count from 0-ARR or ARR-0
  STM32_TIM3->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2   // TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  STM32_TIM3->CNT = 0;           // Initialize counter
  STM32_TIM3->EGR = 1;           // generate an update event
  STM32_TIM3->CR1 = TIM_CR1_CEN;//1;           // Enable the counter
}

