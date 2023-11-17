#pragma once
#include <Eigen/LU>
#include "ch.h"

class Guidance {
    enum State {
        IDLE,
        ACCELERATION,
        CRUISE,
        DECELERATION,
    };

public:

Guidance(): state(IDLE), precision(10) {}

void init(double rate);
void update();
void setTarget(Eigen::Vector3d target);

private:

enum State state;
double precision;
double dt;


Eigen::Vector3d start_point;
Eigen::Vector3d target;
systime_t start_time;
double travel_time;
Eigen::Vector3d factors;
Eigen::Vector3d travel_accel;



};
