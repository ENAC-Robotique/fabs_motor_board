#pragma once

#include "ch.h"
#include "pid.h"

class OdometryDiff;

class DifferentialControl {
public:

    DifferentialControl(): speed_setPoint(0), omega_setPoint(0){}

    void init();

    void set_speed_setPoint(double vx, double vy, double vtheta);
    void set_pid_gains(uint32_t motor_no, double feedforward, double kp, double ki, double kd);
    void speed_control(OdometryDiff* odometry);

    msg_t sendMotorsSpeed(double v1, double v2, double v3);
    msg_t sendMotorsCmd(double cmd1, double cmd2, double cmd3);

private:
    
    double speed_setPoint;
    double omega_setPoint;

    systime_t setpoint_time;
    systime_t lastTime_odometry;
    systime_t lastTime_motors;

    PID l_pid;
    PID r_pid;

};
