#include "HolonomicControl.h"
#ifdef __cplusplus
extern "C" {
#endif
  
  #include <ch.h>
  #include <hal.h>
   
  #include "stdutil.h"
  #include "printf.h"
  #include "globalVar.h"
  #include "utils.h"
#ifdef __cplusplus
}
#endif

#include <cmath>
#include "OdometryHolo.h" 
#include "motors.h"
#include "BytesWriteBuffer.h"
#include "utils.h"
#include "messages.h"
#include "communication.h"


/*
 *  |v1|   |-sin(O1)  cos(O1)  R|   |vx|
 *  |v2| = |-sin(O2)  cos(O2)  R| . |vy|
 *  |v3|   |-sin(O3)  cos(O3)  R|   |w|
 *
 *    m  =           D            .   v
 */

// Euclidean speeds into motor speeds: m = Dv


using namespace protoduck;

const Eigen::Vector3f ACCEL_MAX = {2000.0, 2000.0, 20.};

void HolonomicControl::init() {

  initPwms();

  setMot1(0);
  setMot2(0);
  setMot3(0);
  _setpoint_target = {0, 0, 0};
  _speed_setPoint   = {0, 0, 0};

  set_pid_gains(0.1, 0.2, 0.8, 0.);

  control_time = chVTGetSystemTime();
  setpoint_time = chVTGetSystemTime();

  auto set_setpoint_cb = [this](Message& msg) {
    if (msg.has_speed() && msg.msg_type() == Message::MsgType::COMMAND) {
          auto vx = msg.speed().vx();
          auto vy = msg.speed().vy();
          auto vtheta = msg.speed().vtheta();
          //chprintf ((BaseSequentialStream*)&SDU1, "Speed cmd: %f, %f, %f\r\n\r\n", vx, vy, vtheta);
          set_speed_setPoint(vx, vy, vtheta);
      }
  };

  auto set_pid_gains_cb = [this](Message& msg) {
    if(msg.has_motor_pid() && msg.msg_type() == Message::MsgType::COMMAND) {
        //auto motor_no = msg.motor_pid().motor_no();
        auto feedforward = msg.motor_pid().feedforward();
        auto kp = msg.motor_pid().kp();
        auto ki = msg.motor_pid().ki();
        auto kd = msg.motor_pid().kd();
        // acquire lock ?!
        set_pid_gains(feedforward, kp, ki, kd);
      }
  };

  register_callback(set_setpoint_cb);
  register_callback(set_pid_gains_cb);
}

msg_t sendMotorReport(float m1, float m2, float m3) {
  Message msg;
  auto& motors_speed = msg.mutable_motors_speed();
  motors_speed.set_v1(m1);
  motors_speed.set_v2(m2);
  motors_speed.set_v3(m3);
  return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
}

/**
 * Set speed setPoint
 * vx: mm/s
 * vy: mm/s
 * vtheta: rad/s
 */
void HolonomicControl::set_speed_setPoint(float vx, float vy, float vtheta) {
  chMtxLock(&(mut_speed_set_point));
  _setpoint_target = {vx, vy, vtheta};
  setpoint_time = chVTGetSystemTime();
  chMtxUnlock(&(mut_speed_set_point));
}


/**
 * Set speed setPoint from norm and direction.
 * speed: robot speed in mm/s
 * direction: speed direction in radians, [0, 2*pi]
 * omega: rotation speed in rad/s
 */
void HolonomicControl::set_speed_setPoint_norm_dir(float speed, float direction, float omega) {
  chMtxLock(&(mut_speed_set_point));
  _setpoint_target = {
    speed * cos(direction),
    speed * sin(direction),
    omega
  };
  setpoint_time = chVTGetSystemTime();
  chMtxUnlock(&(mut_speed_set_point));
}

void HolonomicControl::set_pid_gains(float ng, float kp, float ki, float kd) {
  _ng = ng;
  _kp = kp;
  _ki = ki;
  _kd = kd;
  chprintf ((BaseSequentialStream*)&SDU1, "kp = %f\tki = %f\tkd = %f\r\n\r\n", kp, ki, kd);
}

void HolonomicControl::ramp_setpoint(double elapsed) {
  auto max_setpoint = _speed_setPoint + ACCEL_MAX * elapsed;
  auto min_setpoint = _speed_setPoint - ACCEL_MAX * elapsed;

  auto h = _setpoint_target.array().min(max_setpoint.array());
  _speed_setPoint = h.max(min_setpoint.array());
}

void HolonomicControl::speed_control(OdometryHolo* odometry)
{
  systime_t now = chVTGetSystemTime();

    // set speed setpoint to 0 is no speed command has been received since a while.
  if(chVTTimeElapsedSinceX(setpoint_time) > chTimeMS2I(SETPOINT_VALIDITY)) {
    set_speed_setPoint(0, 0, 0);
  }

  time_msecs_t elapsed_ms = chTimeI2MS(chVTTimeElapsedSinceX(control_time));
  if(elapsed_ms > CONTROL_PERIOD) {
    double elapsed_s = elapsed_ms/1000.0;
    odometry->update(elapsed_s);
    ramp_setpoint(elapsed_s);

    chMtxLock(&(mut_speed_set_point));
    Eigen::Vector3f speed_error = _speed_setPoint - odometry->get_speed();
    chMtxUnlock(&(mut_speed_set_point));

    integrate_error(speed_error, elapsed_s);

    Eigen::Vector3f d_err = (speed_error - _speed_prev_error) / elapsed_s;

    Eigen::Vector3f speed_cmd = _speed_setPoint * _ng + speed_error * _kp + _speed_integral_error * _ki + d_err * _kd;

    Eigen::Vector3f m_cmds = D * speed_cmd;

    setMot1(m_cmds[0]);
    setMot2(m_cmds[1]);
    setMot3(m_cmds[2]);

    //sendMotorReport(m_cmds[0], m_cmds[1], m_cmds[2]);

    _speed_prev_error = speed_error;

    control_time = now;
  }
}

void HolonomicControl::integrate_error(Eigen::Vector3f error, double elapsed) {
  if(abs(_ki) < 0.001) {
    return;
  }
  //chprintf ((BaseSequentialStream*)&SDU1, "error: %f, %f, %f\r\n", error[0], error[1], error[2]);
  _speed_integral_error += error * elapsed;
  //chprintf ((BaseSequentialStream*)&SDU1, "int raw: %f, %f, %f\r\n", _speed_integral_error[0], _speed_integral_error[1], _speed_integral_error[2]);

  Eigen::Vector3f cmd_i = D * (_speed_integral_error * _ki);
  float factor = cmd_i.array().abs().maxCoeff() / 1.0;
  if(factor > 100.0) {
    _speed_integral_error /= factor;
  }
  //chprintf ((BaseSequentialStream*)&SDU1, "int clamp: %f, %f, %f\r\n\r\n", _speed_integral_error[0], _speed_integral_error[1], _speed_integral_error[2]);
}
