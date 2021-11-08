#include "utils.h"
#include "ch.h"
#include "hal.h"
#include "globalVar.h"
#include "printf.h"

/**
 * Centers an angle in radians to [-pi, pi[
 */
float center_radians(float angle){
  while (angle >= M_PI){
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI){
    angle += 2 * M_PI;
  }
  return angle;
}

ADCConversionGroup adc1cfg = {
  .circular = false,
  .num_channels = 1,
  .end_cb = NULL,
  .error_cb = NULL,
  .cr1 =0,
  .cr2 = ADC_CR2_SWSTART,
  .smpr1 = 0, //channels 10 to 18 unused
  .smpr2 = ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),
  .sqr1 = ADC_SQR1_NUM_CH(1),
  .sqr2 = 0,  // sequence 7 to 12 unused
  .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN9)

};


void power_check (void *arg) {
  (void)arg;
  chRegSetThreadName("power_check");

  adcInit();
  adcStart(&ADCD1, NULL);

  adcsample_t sample;

  while (true) {
    msg_t status = adcConvert(&ADCD1, &adc1cfg, &sample, 1);
    if(status == MSG_OK) {
      float power_voltage = sample * ADC_TO_VOLTS;

      if(power_voltage > BAT_LOW) {   // power voltage good
        palClearLine(LINE_LED_ORANGE);
        palClearLine(LINE_LED_RED);
      }
      else if(power_voltage > BAT_CRITICAL) { // power voltage low, but not critical
        palToggleLine(LINE_LED_ORANGE);
        palClearLine(LINE_LED_RED);
      } else if(power_voltage < BAT_CRITICAL) { //power voltage critical
        palToggleLine(LINE_LED_RED);
        palClearLine(LINE_LED_ORANGE);
      }

      //chprintf ((BaseSequentialStream*)&SDU1, "%f\r\n", power_voltage);
    }
    chThdSleepMilliseconds(1000);  //increase to 2000 or more
  }
}
