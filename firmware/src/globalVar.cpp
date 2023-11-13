#include "globalVar.h"
#include "encoders.h"
#include "hal.h"

/*
 * Encoders
 */
Encoder enc1(STM32_TIM4);
Encoder enc2(STM32_TIM8);
Encoder enc3(STM32_TIM2);
Encoder enc4(STM32_TIM3);

/*
 * USB Driver structure.
 */
#if HAL_USE_SERIAL_USB
SerialUSBDriver SDU1;
#else

#endif // HAL_USE_SERIAL_USB
