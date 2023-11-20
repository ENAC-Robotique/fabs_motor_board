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

void HolonomicControl::init() {

  mot1.init();
  mot2.init();
  mot3.init();

  mot1.set_cmd(0);
  mot2.set_cmd(0);
  mot3.set_cmd(0);

  _pos_cons = {0, 0, 0};
  _speed_cons = {0, 0, 0};
  _cmds = {0, 0, 0};

  for(int i=0; i<MOTORS_NB; i++) {
    pids[i].init(CONTROL_RATE, 30);
    pids[i].set_gains(10, 1, 0);
  }

  auto set_pid_gains_cb = [this](Message& msg) {
    if(msg.has_motor_pid() && msg.msg_type() == Message::MsgType::COMMAND) {
        auto motor_no = msg.motor_pid().motor_no();
        auto kp = msg.motor_pid().kp();
        auto ki = msg.motor_pid().ki();
        auto kd = msg.motor_pid().kd();
        if(motor_no < MOTORS_NB) {
          pids[motor_no].set_gains(kp, ki, kd);
        }
      }
  };

  register_callback(set_pid_gains_cb);
}


/**
 * pos: position setpoint in robot frame
 * speed: speed setpoint in robot frame, relative to earth
*/
void HolonomicControl::set_cons(Eigen::Vector3d posRobotR, Eigen::Vector3d vRobotR)
{
    _pos_cons = (D * posRobotR) + odometry.get_motors_pos();
    _speed_cons = D * vRobotR;
}

void HolonomicControl::update()
{
  Eigen::Vector3d  motors_pos = odometry.get_motors_pos();
  Eigen::Vector3d  motors_speed = odometry.get_motors_speed();

  Eigen::Vector3d pos_error = _pos_cons - motors_pos;
  Eigen::Vector3d speed_error = _speed_cons - motors_speed;

  for(int i=0; i<MOTORS_NB; i++) {
    _cmds[i] = pids[i].update(pos_error[i], speed_error[i]);
  }
  
  mot1.set_cmd(_cmds[0]);
  mot2.set_cmd(_cmds[1]);
  mot3.set_cmd(_cmds[2]);
}
