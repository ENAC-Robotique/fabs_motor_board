/*
  Nom(s), prénom(s) du ou des élèves : 

  QUESTION 1 : influence de la suppression du tableau prendDeLaPlaceSurLaPile ?

 */
#include <ch.h>
#include <hal.h>
#include "stdutil.h"		// necessaire pour initHeap
#include "ttyConsole.h"		// fichier d'entête du shell
#include "pwm_config.h"
#include "encoders.h"
#include "motor_control.h"

/*
  Câbler une LED sur la broche C0


  ° connecter B6 (uart1_tx) sur PROBE+SERIAL Rx AVEC UN JUMPER
  ° connecter B7 (uart1_rx) sur PROBE+SERIAL Tx AVEC UN JUMPER
  ° connecter C0 sur led0

 */

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


static THD_WORKING_AREA(waBlinker, 304);	// declaration de la pile du thread blinker
static void blinker (void *arg)			// fonction d'entrée du thread blinker
{
  (void)arg;					// on dit au compilateur que "arg" n'est pas utilisé
  chRegSetThreadName("blinker");		// on nomme le thread
  //int prendDeLaPlaceSurLaPile[40] __attribute__((unused)); // variable automatique, donc sur la pile
  
  const int NB_LEDS = sizeof(leds)/sizeof(leds[0]);
  //const int NB_LEDS = 2;

  while (true) {				// boucle infinie
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

static THD_WORKING_AREA(waMotest, 304);	// declaration de la pile du thread motest
static void motest (void *arg)			// fonction d'entrée du thread motest
{
  (void)arg;					// on dit au compilateur que "arg" n'est pas utilisé
  chRegSetThreadName("motest");		// on nomme le thread
  const int NB_STEPS = 100;
  const int t_sleep = 10;
  while(true) {
    /*for(int i=-NB_STEPS; i<=NB_STEPS;i++) {
      float speed = (float)i / NB_STEPS;
      setMot1(speed);
      chThdSleepMilliseconds(t_sleep);
    }
    for(int i=NB_STEPS; i>=-NB_STEPS;i--) {
      float speed = (float)i / NB_STEPS;
      setMot1(speed);
      chThdSleepMilliseconds(t_sleep);
    }*/
  }

}

int main (void)
{

  halInit();
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  initPwms();
  initEnc1(false);
  initEnc2(false);
  initEnc3(false);
  initEnc4(false);
  setMot1(0);
  setMot2(0);
  setMot3(0);

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL); // lancement du thread 
  chThdCreateStatic(waMotest, sizeof(waMotest), NORMALPRIO, &motest, NULL); // lancement du thread 

  consoleInit();	// initialisation des objets liés au shell
  
  // cette fonction en interne fait une boucle infinie, elle ne sort jamais
  // donc tout code situé après ne sera jamais exécuté.
  consoleLaunch();  // lancement du shell
  
  // main thread does nothing
  chThdSleep(TIME_INFINITE);
}


