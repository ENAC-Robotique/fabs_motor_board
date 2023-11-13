/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file filters/high_gain_filter.h
 *  @brief Implementation of the high gain filter for rotary encoder
 */

#pragma once

#include <math.h>
#include <array>


/**
 *  filter struct
 */
class HighGainFilter {
public:

  /** Init all matrix and vectors to the right value
   *
   * @param filter pointer to a filter structure
   * @param alpha 
   * @param epsilon high gain
   * @param rate data update rate
   * @param init_val_theta
   * @param init_val_theta_dot
   */
  void init(std::array<double, 3> alpha, double epsilon, double rate);
  void process(double theta);

  void reset(std::array<double, 3> hatx={0, 0, 0},
             std::array<double, 3> hatx_dot_prev={0, 0, 0}){
  this->hatx = hatx;
  this->hatx_dot_prev = hatx_dot_prev;
}

  void update_alpha0(double alpha0){ alpha[0] = alpha0; }
  void update_alpha1(double alpha1){ alpha[1] = alpha1; }
  void update_alpha2(double alpha2){ alpha[2] = alpha2; }
  void update_epsilon(double epsilon){ this->epsilon = epsilon; }
  void update_rate(double rate){ this->rate = rate; }

  double get_pos(){ return hatx[0];};
  double get_speed(){ return hatx[1];};


private:
  //states
  std::array<double, 3> hatx;  // state 
  std::array<double, 3> hatx_dot_prev; // previous state

  //parameters
  std::array<double, 3> alpha;
  float epsilon;                     //
  float rate;                     ///< data update rate (in Hz)

};


/** Process step
 *
 * hatx_dot =  A * hatx + B * (theta - hatx[0])
 * hatx += (1/rate)* (hatxprev + hatx)/2
 *
 * @param filter pointer to the filter structure
 * @param theta measurement value
 */
extern void high_gain_filter_process(struct high_gain_filter *filter, float theta);

/**
 * settings handlers
 */
extern void high_gain_filter_reset(struct high_gain_filter *filter);