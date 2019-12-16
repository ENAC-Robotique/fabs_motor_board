
#ifndef UTILS_H
#define UTILS_H

#define M_PI 3.14159265358979323846

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif



/**
 * Centers an angle in radians to [-pi, pi[
 */
float center_radians(float angle);


#endif  // UTILS_H


