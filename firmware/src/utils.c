#include "utils.h"
#include "ch.h"
#include "hal.h"
#include "globalVar.h"
#include "printf.h"

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
