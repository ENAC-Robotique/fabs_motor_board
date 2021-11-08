#ifndef POWER_CHECK_H
#define POWER_CHECK_H

#define ADC_TO_VOLTS    0.0043746701846965694
#define BAT_LOW         14.0
#define BAT_CRITICAL    13.2

// read power voltage
void start_power_check(void);

#endif  // POWER_CHECK_H
