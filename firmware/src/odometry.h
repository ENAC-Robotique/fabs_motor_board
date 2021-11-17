#pragma once

#include "ch.h"

#define PERIOD_ODOM_REPORT 0.5

#define INC_PER_MM 17.31
//#define ROBOT_RADIUS    125.0
#define WHEELBASE 175.5
#define CODING_WHEELBASE 245


double RW_to_W(double rw);
double W_to_RW(double w);


class OdometryDiff {
public:

    OdometryDiff(): speed(0), omega(0), _x(0), _y(0), _theta(0) {}
    
    void update_odometry(double elapsed);
    
    double get_speed(void) { return speed;}
    double get_omega(void) {return omega;}

    double get_x(void) {return _x;}
    double get_y(void) {return _y;}
    double get_theta(void) {return _theta;}

    

private:

    msg_t sendOdomReport();

    double speed;
    double omega;

    double _x;
    double _y;
    double _theta;


    // double mot_speed;
    // double mot_omega;

};



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

extern OdometryDiff odometry;