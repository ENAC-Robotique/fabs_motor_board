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

    Eigen::Vector3d get_cmds() { return _cmds;}
    Eigen::Vector3d get_pos_cons() { return _pos_cons;}
    Eigen::Vector3d get_speed_cons() { return _speed_cons;}

private:

    void ramp_setpoint(double elapsed);

    systime_t setpoint_time;
    systime_t control_time;

    Eigen::Vector3d _pos_setpoint;
    Eigen::Vector3d _pos_cons;
    Eigen::Vector3d _speed_setPoint;
    Eigen::Vector3d _speed_cons;

    PID pids[MOTORS_NB];

    // keep commands for logging
    Eigen::Vector3d _cmds;

    MUTEX_DECL(mut_set_point);
};
