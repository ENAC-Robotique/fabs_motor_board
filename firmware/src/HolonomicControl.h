#pragma once
#include "utils.h"
#include <hal.h>
#include <Eigen/Core>

typedef float float32_t;
class OdometryHolo;

#define SPEED_CONTROL_PERIOD 50 //ms
#define ODOMETRY_PERIOD 25   //ms

#define SETPOINT_VALIDITY 1000  //ms

class HolonomicControl {
public:

    HolonomicControl() : _ng(0), _kp(0), _ki(0), _kd(0) {}

    void init();

    void set_speed_setPoint(float vx, float vy, float vtheta);
    void set_speed_setPoint_norm_dir(float speed, float direction, float omega);
    void set_pid_gains(float ng, float kp, float ki, float kd);
    void speed_control(OdometryHolo* odometry);

private:

void ramp_setpoint(double elapsed);
void integrate_error(Eigen::Vector3f error, double elapsed);

Eigen::Vector3f m_maxs_cmd = {100.,  100.,  100.};

float _ng; 
float _kp;
float _ki;
float _kd;

systime_t setpoint_time;
systime_t control_time;

Eigen::Vector3f _setpoint_target;
Eigen::Vector3f _speed_setPoint;
Eigen::Vector3f _speed_integral_error;
Eigen::Vector3f _speed_prev_error;

MUTEX_DECL(mut_speed_set_point);
};
