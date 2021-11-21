#pragma once

#include "ch.h"

#define PERIOD_ODOM_REPORT 0.1

#define MOTOR_INC_PER_MM 17.31
#define CODING_INC_PER_MM 52.287852
//#define ROBOT_RADIUS    125.0
#define WHEELBASE 175.5
#define CODING_WHEELBASE 246.548


double RW_to_W(double rw);
double W_to_RW(double w);


class OdometryDiff {
public:

    OdometryDiff(): speed(0), omega(0), speed_left(0), speed_right(0), _x(0), _y(0), _theta(0) {}
    
    void update_pos(double elapsed);
    void update_mot(double elapsed);

    double get_speed_left(void) {return speed_left;}
    double get_speed_right(void) {return speed_right;}
    
    double get_speed(void) { return speed;}
    double get_omega(void) {return omega;}

    double get_x(void) {return _x;}
    double get_y(void) {return _y;}
    double get_theta(void) {return _theta;}

    

private:

    msg_t sendOdomReport();

    double speed;
    double omega;

    double speed_left;
    double speed_right;

    double _x;
    double _y;
    double _theta;
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