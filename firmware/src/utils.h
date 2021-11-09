
#ifndef UTILS_H
#define UTILS_H

#define M_PI 3.14159265358979323846

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
double center_radians(double angle);

#endif  // UTILS_H


