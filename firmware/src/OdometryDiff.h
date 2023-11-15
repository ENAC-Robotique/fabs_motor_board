#pragma once

#include "ch.h"


#define PERIOD_SLIP_REPORT 100  //ms

#define SLIP_THRESHOLD 15.0

#define MOTOR_INC_PER_MM 17.31
#define CODING_INC_PER_MM 52.287852
#define WHEELBASE 175.5
#define CODING_WHEELBASE 246.548

class OdometryDiff {
public:

    OdometryDiff(): speed(0), omega(0),
        speed_left(0), speed_right(0),
        _x(0), _y(0), _theta(0),
        slip_left(0), slip_right(0) {}
    
    void init();
    
    void update_pos(double elapsed);
    void update_mot(double elapsed);

    double get_speed_left(void) {return speed_left;}
    double get_speed_right(void) {return speed_right;}
    
    double get_speed(void) { return speed;}
    double get_omega(void) {return omega;}

    double get_x(void) {return _x;}
    double get_y(void) {return _y;}
    double get_theta(void) {return _theta;}

    void set_pos(double x, double y, double theta);

    

private:

    msg_t sendOdomReport();

    double speed;
    double omega;

    double speed_left;
    double speed_right;

    double _x;
    double _y;
    double _theta;

    double slip_left;
    double slip_right;
};

//extern OdometryDiff odometry;