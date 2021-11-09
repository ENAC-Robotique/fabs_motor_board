
#include <ch.h>
#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif
  #include "stdutil.h"		// necessaire pour initHeap
  #include "ttyConsole.h"		// fichier d'entête du shell
  #include "encoders.h"
  #include "utils.h"
#ifdef __cplusplus
}
#endif

#include "communication.h"
#include "motors.h"
#include "speed_control.h"
#include "power_check.h"

ioline_t leds[] = {
  LINE_LED_GREEN,
  //LINE_LED_YELLOW,
  //LINE_LED_ORANGE,
  //LINE_LED_RED,
  //LINE_LED_CAN,
  //LINE_LED_SPI,
  //LINE_LED_UART,
  //LINE_LED_I2C,
};


static const SerialConfig serialConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};


static THD_WORKING_AREA(waBlinker, 304);
static void blinker (void *arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  
  const int NB_LEDS = sizeof(leds)/sizeof(leds[0]);

  while (true) {
    for(int i=0;i<NB_LEDS; i++) {
      palToggleLine(leds[i]);
      chThdSleepMilliseconds(200);
    }
  }
}


int main (void)
{

  halInit();
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  sdInit();
  sdStart(&SD5, &serialConfig);

  initPwms();
  initEnc1(true);
  initEnc2(true);
  initEnc3(true);
  //initEnc4(false);
  setMot1(0);
  setMot2(0);
  setMot3(0);

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL); // lancement du thread 
  start_power_check();
  start_motor_control_pid();
  start_communication();

  consoleInit();	// initialisation des objets liés au shell
  
  // cette fonction en interne fait une boucle infinie, elle ne sort jamais
  // donc tout code situé après ne sera jamais exécuté.
  consoleLaunch();  // lancement du shell
  
  // main thread does nothing
  chThdSleep(TIME_INFINITE);
}


