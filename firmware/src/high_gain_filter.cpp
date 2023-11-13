#include "high_gain_filter.h"

/** Init all matrix and vectors to the right value
 *
 * @param filter pointer to a filter structure
 * @param alpha 
 * @param epsilon high gain
 * @param rate data update rate
 */
void HighGainFilter::init(std::array<double, 3> alpha, double epsilon, double rate)
{
  this->alpha = alpha;
  this->epsilon = epsilon;
  this->rate = rate;
  this->hatx[0] = 0;
  this->hatx[1] = 0;
  this->hatx[2] = 0;
  this->hatx_dot_prev[0] = 0;
  this->hatx_dot_prev[1] = 0;
  this->hatx_dot_prev[2] = 0;
}


/** Process step
 *
 * hatx_dot =  A * hatx + B * (theta - hatx[0])
 * hatx += (1/rate)* (hatx_dot_prev + hatx)/2
 *
 * @param filter pointer to the filter structure
 * @param theta measurement value
 */

void HighGainFilter::process(double theta){
  double diff_theta = theta - hatx[0];
  double hatx_dot[3] = {hatx[1] + (alpha[0]/epsilon)*diff_theta,
                       hatx[2] + (alpha[1]/pow(epsilon, 2))*diff_theta,
                                         (alpha[2]/pow(epsilon, 3))*diff_theta};
  hatx[0] += (1/rate)*(hatx_dot_prev[0] + hatx_dot[0])/2;
  hatx[1] += (1/rate)*(hatx_dot_prev[1] + hatx_dot[1])/2;
  hatx[2] += (1/rate)*(hatx_dot_prev[2] + hatx_dot[2])/2;
  hatx_dot_prev[0] = hatx_dot[0];
  hatx_dot_prev[1] = hatx_dot[1];
  hatx_dot_prev[2] = hatx_dot[2];
}
