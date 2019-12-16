#include <ch.h>
#include <hal.h>
#include "stdutil.h"		// necessaire pour initHeap
#include "ttyConsole.h"		// fichier d'entête du shell
#include "pwm_config.h"
#include "encoders.h"
#include "motor_control.h"


ioline_t leds[] = {
  LINE_LED_GREEN,
  LINE_LED_YELLOW,
  LINE_LED_ORANGE,
  LINE_LED_RED,
  LINE_LED_CAN,
  LINE_LED_SPI,
  LINE_LED_UART,
  LINE_LED_I2C,
};


static THD_WORKING_AREA(waBlinker, 304);
static void blinker (void *arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  
  const int NB_LEDS = sizeof(leds)/sizeof(leds[0]);

  while (true) {
    for(int i=0;i<NB_LEDS; i++) {
      //palClearLine(leds[i]);
      //int next = (i+1)%NB_LEDS;
      //palSetLine(leds[next]);
      palToggleLine(leds[i]);
      chThdSleepMilliseconds(100);
    }
    //palToggleLine(LINE_LED_GREEN);
    //palClearLine(LINE_LED_
    //chThdSleepMilliseconds(100);
  }
}


int main (void)
{

  halInit();
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  initPwms();
  initEnc1(false);
  initEnc2(true);
  //initEnc3(false);
  //initEnc4(false);
  setMot1(0);
  setMot2(0);
  setMot3(0);

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL); // lancement du thread 
  start_motor_control_pid();

  consoleInit();	// initialisation des objets liés au shell
  
  // cette fonction en interne fait une boucle infinie, elle ne sort jamais
  // donc tout code situé après ne sera jamais exécuté.
  consoleLaunch();  // lancement du shell
  
  // main thread does nothing
  chThdSleep(TIME_INFINITE);
}


