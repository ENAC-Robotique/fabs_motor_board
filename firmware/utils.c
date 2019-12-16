#include "utils.h"

/**
 * Centers an angle in radians to [-pi, pi[
 */
float center_radians(float angle){
  while (angle >= M_PI){
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI){
    angle += 2 * M_PI;
  }
  return angle;
}
