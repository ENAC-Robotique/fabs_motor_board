#pragma once
#include "utils.h"
#include <hal.h>
#include <Eigen/Core>

/*
 *  |v1|   |-sin(O1)  cos(O1)  1|   |vx|
 *  |v2| = |-sin(O2)  cos(O2)  1| . |vy|
 *  |v3|   |-sin(O3)  cos(O3)  1|   |Rw|
 *
 *    m  =           D            .   v
 */
#define DDATA \
  { 0.0       ,  1.0, 1.0, \
    -0.8660254, -0.5, 1.0, \
    0.8660254 , -0.5, 1.0}

//Euclidean speeds into motor speeds: m = Dv

typedef float float32_t;
class OdometryHolo;

#define SPEED_CONTROL_PERIOD 50 //ms
#define ODOMETRY_PERIOD 25   //ms

#define SETPOINT_VALIDITY 1000  //ms

class HolonomicControl {
public:

    HolonomicControl() : _kp(0), _ki(0), _kd(0) {}

    void init();

    void set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta);
    void set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega);
    void set_pid_gains(float32_t kp, float32_t ki, float32_t kd);
    void speed_control(OdometryHolo* odometry);

private:

constexpr static float32_t mins_cmd[3] = {-1., -1., -1.};
constexpr static float32_t maxs_cmd[3] = {1.,  1.,  1.};

float32_t _kp;
float32_t _ki;
float32_t _kd;

systime_t setpoint_time;
systime_t control_time;

Eigen::Vector3f _speed_setPoint;
Eigen::Vector3f m_Ierr;

MUTEX_DECL(mut_speed_set_point);
};
