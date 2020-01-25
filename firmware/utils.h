
#ifndef UTILS_H
#define UTILS_H

#define M_PI 3.14159265358979323846

#define ADC_TO_VOLTS    0.0043746701846965694
#define BAT_LOW         14.0
#define BAT_CRITICAL    13.2

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif

#define MAKE_VECTOR3(name) float32_t name ## _data[] = {0, 0, 0};\
arm_matrix_instance_f32 name = {\
    .numRows = 3,\
    .numCols = 1,\
    .pData = name ## _data\
};


/**
 * Centers an angle in radians to [-pi, pi[
 */
float center_radians(float angle);



// read power voltage
void power_check (void *arg);

#endif  // UTILS_H


