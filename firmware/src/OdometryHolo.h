#pragma once

#include "ch.h"


//#define ROBOT_RADIUS    125.0


double RW_to_W(double rw);
double W_to_RW(double w);




class OdometryHolonomic {
public:
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

