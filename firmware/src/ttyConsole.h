#pragma once

#include <ch.h>
#include <hal.h>
#include "stdutil.h"

#ifdef __cplusplus
extern "C" {
#endif


void consoleInit (void);
void consoleLaunch (void);

#define CONSOLE_DEV_USB TRUE

#ifdef __cplusplus
}
#endif

