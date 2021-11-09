#include "DifferentialControl.h"
#include "odometry.h"
#include "motors.h"

#include "hal.h"
extern "C" {
    #include "printf.h"
    #include "globalVar.h"
    #include "stdutil.h"
}




double clamp(double lo, double val, double hi) {
    if(val < lo) {return lo;}
    if(val > hi) {return hi;}
    return val;
}


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


#define SPEED_CONTROL_PERIOD 0.1
#define ODOMETRY_PERIOD 0.1
//#define ODOMETRY_PERIOD 0.025

void DifferentialControl::speed_control(void *arg) {
  (void)arg;

  systime_t lastTime_odometry = chVTGetSystemTime();
  systime_t lastTime = chVTGetSystemTime();

  while(true) {

    systime_t now = chVTGetSystemTime();
    double elapsed = ((double)(now - lastTime)) / CH_CFG_ST_FREQUENCY;
    double elapsed_odometry = ((double)(now - lastTime_odometry)) / CH_CFG_ST_FREQUENCY;

    if(elapsed_odometry > ODOMETRY_PERIOD) {
      odometry.update_odometry(elapsed_odometry);
      lastTime_odometry = now;
    }

    if(elapsed > SPEED_CONTROL_PERIOD) {
      double error_speed = speed_setPoint - odometry.get_speed();
      _intError_speed += error_speed;
      double delta_speed_error = error_speed - _prevError_speed;
      _prevError_speed = error_speed;

      double cmd_speed = NOMINAL_PGAIN * speed_setPoint + KP_SPEED * error_speed + KI_SPEED * _intError_speed - KD_SPEED * delta_speed_error;


      double error_omega = omega_setPoint - odometry.get_omega();
      _intError_omega += error_omega;

      _intError_omega = clamp(-50, _intError_omega + error_omega, 50);
      double delta_omega_error = error_omega - _prevError_omega;
      _prevError_omega = error_omega;

      double cmd_omega = WHEELBASE * NOMINAL_PGAIN * omega_setPoint + KP_OMEGA * error_omega + KI_OMEGA * _intError_omega + KD_OMEGA * delta_omega_error;


      double cmd_mot1 = clamp(-100, cmd_speed - cmd_omega, 100);
      double cmd_mot2 = -clamp(-100, cmd_speed + cmd_omega, 100);

      //chprintf ((BaseSequentialStream*)&SDU1, "CMD% f %f\r\n", cmd_mot1, cmd_mot2);

      setMot1(cmd_mot1);
      setMot2(cmd_mot2);

    }

  }

}
