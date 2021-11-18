#include "utils.h"

/**
 * Centers an angle in radians to [-pi, pi[
 */
double center_radians(double angle){
  while (angle >= M_PI){
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI){
    angle += 2 * M_PI;
  }
  return angle;
}


double clamp(double lo, double val, double hi) {
    if(val < lo) {return lo;}
    if(val > hi) {return hi;}
    return val;
}

