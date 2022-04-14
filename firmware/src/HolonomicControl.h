#pragma once
#include "arm_math.h"
#include "utils.h"
#include <hal.h>

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


#define SPEED_CONTROL_PERIOD 0.050
#define ODOMETRY_PERIOD 0.025


class HolonomicControl {
public:

    HolonomicControl() : _kp(0), _ki(0), _kd(0),
        _speed_setPoint_data{0, 0, 0},
        D_data(DDATA),
        m_Ierr_data{0, 0, 0},
        m_cmd_data{0, 0, 0}
        {}

    void init();

    void set_speed_setPoint(float32_t vx, float32_t vy, float32_t vtheta);
    void set_speed_setPoint_norm_dir(float32_t speed, float32_t direction, float32_t omega);
    void set_pid_gains(float32_t kp, float32_t ki, float32_t kd);
    void speed_control(void *arg);

private:

constexpr static float32_t mins_cmd[3] = {-1., -1., -1.};
constexpr static float32_t maxs_cmd[3] = {1.,  1.,  1.};

float32_t _kp;
float32_t _ki;
float32_t _kd;

float32_t _speed_setPoint_data[3];
arm_matrix_instance_f32 _speed_setPoint = {
    .numRows = 3, .numCols = 1,
    .pData = _speed_setPoint_data
};

float32_t D_data[9];
arm_matrix_instance_f32 D = {
  .numRows = 3,
  .numCols = 3,
  .pData = D_data
};


float32_t m_Ierr_data[3];
arm_matrix_instance_f32 m_Ierr = {
    .numRows = 3, .numCols = 1,
    .pData = m_Ierr_data
};

float32_t m_cmd_data[3];
arm_matrix_instance_f32 m_cmd = {
    .numRows = 3, .numCols = 1,
    .pData = m_cmd_data
};


MUTEX_DECL(mut_speed_set_point);


};
