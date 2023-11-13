
#include <ch.h>
#include <hal.h>
#include "encoders.h"
#include <cstdint>
#include "stm32_tim.h"


Encoder::Encoder(stm32_tim_t* tim): tim(tim), offset(0x7FFF), lower_half(true)
{
}

void Encoder::init(bool inverted) {
  if(tim == STM32_TIM2) {
    // 32 bits
    rccEnableTIM2(NULL);
    rccResetTIM2();
  }
  else if(tim == STM32_TIM3) {
    // 32 bits
    rccEnableTIM3(NULL);
    rccResetTIM3();
  }
  else if(tim == STM32_TIM4) {
    // 32 bits
    rccEnableTIM4(NULL);
    rccResetTIM4();
  }
  else if(tim == STM32_TIM8) {
    // 32 bits
    rccEnableTIM8(NULL);
    rccResetTIM8();
  }
  else {
    chSysHalt("Timer not supported");
  }



  // Encoder mode 3: Counter counts up/down on both TI1FP1 and TI2FP2 edges
  tim->SMCR =  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  
  if(inverted) {
    tim->CCER = TIM_CCER_CC1P;
  } else {
    tim->CCER = 0;
  }
  
  // count from 0-ARR or ARR-0. Limit to 16 bits so it works the same on all timers
  tim->ARR = 0xFFFF;

  // f_DTS/16, N=8, IC1->TI1, IC2->TI2   // TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  tim->CCMR1 = 0xC1C1;

  tim->CNT = offset;      // Initialize counter
  tim->EGR = 1;           // generate an update event to trigger configuration update
  tim->CR1 = TIM_CR1_CEN; // Enable the counter
}

int32_t Encoder::get_value() {
  uint32_t counter_val = tim->CNT;

  if(tim->SR & TIM_SR_UIF) {  //an overflow/underflow happend 
    if(counter_val <= 0x7FFF && !lower_half) {
      // the counter is in the lower half, and was previously in the upper half: overflow
      offset += 0XFFFF;
    } else if(counter_val > 0x7FFF && lower_half) {
      // the counter is in the upper half, and was previously in the lower half: underflow
      offset -= 0XFFFF;
    }
    // if it's not one of the abover cases, an overflow/underflow happened,
    // but the counter did the opposite underflow/overflow,
    // so nothing needs to be done.

    tim->SR = 0;   // reset interrupt flag
  }
  
  if(counter_val <= 0x7FFF) {
    lower_half = true;
  }
  else {
    lower_half = false;
  }

  return static_cast<int32_t>(counter_val) - offset;
}
