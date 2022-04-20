#pragma once

#include "ch.h"

#define ROBOT_RADIUS    113.0


inline double RW_to_W(double rw) {return rw / ROBOT_RADIUS;}
inline double W_to_RW(double w)  {return  w * ROBOT_RADIUS;}




class OdometryHolo {
public:
    void init();

    double get_speed(void);
    double get_omega(void);

    double get_x(void);
    double get_y(void);
    double get_theta(void);

    double get_vx(void);
    double get_vy(void);
    double get_vtheta(void);

    void update_odometry(double elapsed);

};

