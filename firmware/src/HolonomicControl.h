#pragma once
#include "utils.h"
#include <hal.h>
#include <Eigen/Core>
#include "pid.h"
#include "OdometryHolo.h"

#define SETPOINT_VALIDITY 1000  //ms

class HolonomicControl {
public:

    HolonomicControl() {}

    void init();

    void set_setPoints(Eigen::Vector3d pos, Eigen::Vector3d speed);
    void update();

private:

    void ramp_setpoint(double elapsed);

    systime_t setpoint_time;
    systime_t control_time;

    Eigen::Vector3d _pos_setpoint;
    Eigen::Vector3d _pos_cons;
    Eigen::Vector3d _speed_setPoint;
    Eigen::Vector3d _speed_cons;

    PID pids[MOTORS_NB];

    MUTEX_DECL(mut_set_point);
};
