#pragma once

#include "ch.h"
#include "hal.h"

#ifdef __cplusplus
#include "encoders.h"
#include "motors.h"
#include "OdometryHolo.h"
#include "HolonomicControl.h"
#include "guidance.h"

extern Encoder enc1;
extern Encoder enc2;
extern Encoder enc3;
extern Encoder enc4;

extern Motor mot1;
extern Motor mot2;
extern Motor mot3;

extern OdometryHolo odometry;

extern HolonomicControl control;

extern Guidance guidance;

#endif

/*
 * USB Driver structure.
 */

#if HAL_USE_SERIAL_USB
extern SerialUSBDriver SDU1;
#endif 



