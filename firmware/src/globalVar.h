#pragma once

#include "ch.h"
#include "hal.h"

#ifdef __cplusplus
#include "encoders.h"
extern Encoder enc1;
extern Encoder enc2;
extern Encoder enc3;
extern Encoder enc4;

#endif

/*
 * USB Driver structure.
 */

#if HAL_USE_SERIAL_USB
extern SerialUSBDriver SDU1;
#endif 



