#include "DifferentialControl.h"

void DifferentialControl::set_speed_setPoint(double vx, double vy, double vtheta) {
    speed_setPoint = vx;
    omega_setPoint = vtheta;
}


void DifferentialControl::set_speed_setPoint_norm_dir(double speed, double direction, double omega) {
    speed_setPoint = speed;
    omega_setPoint = omega;
}


// TODO omega ?
// message en parametre ?
void DifferentialControl::set_pid_gains(double kp, double ki, double kd) {
    KP_SPEED = kp;
    KI_SPEED = ki;
    KD_SPEED = kd;
}


void DifferentialControl::speed_control(void *arg) {



}
