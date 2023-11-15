#include "HolonomicControl.h"
#ifdef __cplusplus
extern "C" {
#endif
  
  #include <ch.h>
  #include <hal.h>
   
  #include "stdutil.h"
  #include "printf.h"
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
#include "globalVar.h"
#include "config.h"


/*
 *  |v1|   |-sin(O1)  cos(O1)  R|   |vx|
 *  |v2| = |-sin(O2)  cos(O2)  R| . |vy|
 *  |v3|   |-sin(O3)  cos(O3)  R|   |w|
 *
 *    m  =           D            .   v
 */

// Euclidean speeds into motor speeds: m = Dv


using namespace protoduck;

const Eigen::Vector3d ACCEL_MAX = {2000.0, 2000.0, 20.};

void HolonomicControl::init() {

  mot1.init();
  mot2.init();
  mot3.init();

  mot1.set_cmd(0);
  mot2.set_cmd(0);
  mot3.set_cmd(0);


  _pos_setpoint = {0, 0, 0};
  _pos_cons = {0, 0, 0};
  _speed_setPoint = {0, 0, 0};
  _speed_cons = {0, 0, 0};


  for(int i=0; i<MOTORS_NB; i++) {
    pids[i].init(CONTROL_RATE, 30);
    pids[i].set_gains(10, 1, 0);
  }


  // set_pid_gains(0.1, 0.2, 0.8, 0.);

  // control_time = chVTGetSystemTime();
  // setpoint_time = chVTGetSystemTime();

  // auto set_setpoint_cb = [this](Message& msg) {
  //   if (msg.has_speed() && msg.msg_type() == Message::MsgType::COMMAND) {
  //         auto vx = msg.speed().vx();
  //         auto vy = msg.speed().vy();
  //         auto vtheta = msg.speed().vtheta();
  //         //chprintf ((BaseSequentialStream*)&SDU1, "Speed cmd: %f, %f, %f\r\n\r\n", vx, vy, vtheta);
  //         set_speed_setPoint(vx, vy, vtheta);
  //     }
  // };

  // auto set_pid_gains_cb = [this](Message& msg) {
  //   if(msg.has_motor_pid() && msg.msg_type() == Message::MsgType::COMMAND) {
  //       //auto motor_no = msg.motor_pid().motor_no();
  //       auto feedforward = msg.motor_pid().feedforward();
  //       auto kp = msg.motor_pid().kp();
  //       auto ki = msg.motor_pid().ki();
  //       auto kd = msg.motor_pid().kd();
  //       // acquire lock ?!
  //       set_pid_gains(feedforward, kp, ki, kd);
  //     }
  // };

  // register_callback(set_setpoint_cb);
  // register_callback(set_pid_gains_cb);
}

msg_t sendMotorReport(double m1, double m2, double m3) {
  // Message msg;
  // auto& motors_speed = msg.mutable_motors_speed();
  // motors_speed.set_v1(m1);
  // motors_speed.set_v2(m2);
  // motors_speed.set_v3(m3);
  // return post_message(msg, Message::MsgType::STATUS, TIME_IMMEDIATE);
  chprintf ((BaseSequentialStream*)&SDU1, "%lf %lf %lf\r\n", m1, m2, m3);
  return MSG_OK;
}


/**
 * pos: position setpoint in robot frame
 * speed: speed setpoint in robot frame, relative to earth
*/
void HolonomicControl::set_setPoints(Eigen::Vector3d pos, Eigen::Vector3d speed)
{
    chMtxLock(&mut_set_point);
    // //tmp
    // _pos_cons = pos;
    // _speed_cons = speed;

    _pos_cons = D * pos;
    _speed_cons = D * speed;

    // _pos_setpoint = D * pos;
    // _speed_setPoint = D * speed;
    chMtxUnlock(&mut_set_point);
}


/**
 * dt in seconds
*/
void HolonomicControl::ramp_setpoint(double dt) {
  Eigen::Vector3d  motors_pos = odometry.get_motors_pos();
  Eigen::Vector3d  motors_speed = odometry.get_motors_speed();

  // compute max and min pos cons according to current position and speed, and maximum acceleration
  auto max_pos_cons = motors_pos + motors_speed*dt + ACCEL_MAX*dt*dt/2;
  auto min_pos_cons = motors_pos + motors_speed*dt - ACCEL_MAX*dt*dt/2;
  // compute max and min speed cons according to current speed and maximum acceleration
  auto max_speed_cons = motors_speed + ACCEL_MAX*dt;
  auto min_speed_cons = motors_speed - ACCEL_MAX*dt;

  chMtxLock(&mut_set_point);
  // clamp _pos_cons
  auto h_pos = _pos_setpoint.array().min(max_pos_cons.array());
  _pos_cons = h_pos.max(min_pos_cons.array());
  // clamp _speed_cons
  auto h_speed = _speed_setPoint.array().min(max_speed_cons.array());
  _speed_cons = h_speed.max(min_speed_cons.array());
  chMtxUnlock(&mut_set_point);
}

void HolonomicControl::update()
{
  static systime_t lastControlReportTime = chVTGetSystemTime();

  //ramp_setpoint(dt);    // todo rate

  Eigen::Vector3d  motors_pos = odometry.get_motors_pos();
  Eigen::Vector3d  motors_speed = odometry.get_motors_speed();

  Eigen::Vector3d pos_error = _pos_cons - motors_pos;
  Eigen::Vector3d speed_error = _speed_cons - motors_speed;

  double cmds[MOTORS_NB];

  for(int i=0; i<MOTORS_NB; i++) {
    cmds[i] = pids[i].update(pos_error[i], speed_error[i]);
  }
  
  mot1.set_cmd(cmds[0]);
  mot2.set_cmd(cmds[1]);
  mot3.set_cmd(cmds[2]);

  //sendMotorReport(m_cmds[0], m_cmds[1], m_cmds[2]);
  if(chVTTimeElapsedSinceX(lastControlReportTime) > chTimeMS2I(PERIOD_CONTROL_REPORT)) {
    sendMotorReport(cmds[0], cmds[1], cmds[2]);
    lastControlReportTime = chVTGetSystemTime();
  }
}
